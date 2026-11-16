// Read RS485 byte, baud 19200, (8 n 1 ???)

long bytecounter = 0;

// read a specific bit from the message and return 0 or 1
// data = data byte array
// offset = offset in bytes to start reading from
// numberOfBit = read the specific bit at position x in specified byte
int getBit(unsigned char *data, int offset, int numberOfBit) {
  return ((data[offset] >> numberOfBit) & 1);
}

// read different values from the message as int (times fFactor)
// data = data byte array
// offset = offset in bytes to start reading from
// length = read x bytes
// fFactor = used to multiply the read result
// isSigned = is it a signed or unsigned number?
float getValue(unsigned char *data, int offset, int length, float fFactor, bool isSigned) {
  int value = 0;
  for (int i = 0; i < length; i++) {
    value += data[offset + i] << ((length - i - 1) * 8);
  }
  if (isSigned && (value >= (1 << (length * 8 - 1)))) {
    value -= (1 << (length * 8));
  }
  return (value * fFactor);
}

// helper function for checksum
unsigned char CrcAdd(unsigned char crc, unsigned char numberOfByte) {
  crc = (crc << 1) | ( crc >> 7);
  crc = crc + numberOfByte + ((crc + (int)numberOfByte > 255) ? 1 : 0);
  return (crc);
}

#define FRAME_TIMEOUT_MS 250UL

enum FrameStatus {
  FRAME_NONE = 0,
  FRAME_OK = 1,
  FRAME_ERROR = 2
};

enum RxState {
  STATE_WAIT_FOR_HEADER = 1,
  STATE_READ_TYPE_OR_LEN = 2,
  STATE_READ_ID = 3,
  STATE_READ_COUNTER = 4,
  STATE_READ_PAYLOAD = 5,
  STATE_READ_STUFFED_ZERO = 6,
  STATE_READ_CHECKSUM = 7
};

long framecounter = 0;
long errorcounter = 0;

// Non-blocking RS485 frame reader.
// Processes all currently buffered serial bytes and returns immediately if a frame is still incomplete.
FrameStatus readFrame(unsigned char data[], int &msgID, int &dataLength, int &frameID, unsigned long currentMillis) {
  static RxState msgState = STATE_WAIT_FOR_HEADER;
  static unsigned char msgLength = 0, msgCRC = 0;
  static int parsedMsgID = 0, parsedDataLength = 0, parsedFrameID = 0, dataIndex = 0;
  static unsigned long lastByteMillis = 0;

  while (Serial.available() > 0) {
    unsigned char tmp = (unsigned char)Serial.read();
    bytecounter++;
    lastByteMillis = currentMillis;

    switch (msgState) {
      case STATE_WAIT_FOR_HEADER:
        if (tmp == 2) {
          msgState = STATE_READ_TYPE_OR_LEN;
          msgCRC = tmp;
        }
        break;

      case STATE_READ_TYPE_OR_LEN:
        if (tmp == 0) {
          msgState = STATE_WAIT_FOR_HEADER;
        } else if (tmp == 2) { // extended header -> SenseMessage
          msgCRC = tmp;
        } else {
          msgLength = tmp;
          msgCRC = CrcAdd(msgCRC, msgLength);
          parsedDataLength = (int)msgLength - 4 - 1; // Data Length without header and checksum

          if ((parsedDataLength < 0) || (parsedDataLength >= MSGMAXLENGTH)) {
            msgState = STATE_WAIT_FOR_HEADER;
            errorcounter++;
            msgID = 0;
            dataLength = 0;
            frameID = 0;
            return FRAME_ERROR;
          }

          memset(data, 0, MSGMAXLENGTH);
          dataIndex = 0;
          msgState = STATE_READ_ID;
        }
        break;

      case STATE_READ_ID:
        parsedMsgID = (int)tmp;
        msgCRC = CrcAdd(msgCRC, tmp);
        msgState = STATE_READ_COUNTER;
        break;

      case STATE_READ_COUNTER:
        parsedFrameID = (int)tmp;
        msgCRC = CrcAdd(msgCRC, tmp);
        msgState = (parsedDataLength == 0) ? STATE_READ_CHECKSUM : STATE_READ_PAYLOAD;
        break;

      case STATE_READ_PAYLOAD:
        data[dataIndex++] = tmp;
        msgCRC = CrcAdd(msgCRC, tmp);
        if (tmp == 2) {
          msgState = STATE_READ_STUFFED_ZERO; // "2" in data stream is followed by "0"
        } else if (dataIndex >= parsedDataLength) {
          msgState = STATE_READ_CHECKSUM;
        }
        break;

      case STATE_READ_STUFFED_ZERO:
        if (tmp == 0) {
          // Consume stuffed 0x00 byte after 0x02 without adding to CRC or payload
          msgState = (dataIndex >= parsedDataLength) ? STATE_READ_CHECKSUM : STATE_READ_PAYLOAD;
        } else if (tmp == 2) {
          // Resync directly if a new frame start arrived after a dropped byte
          msgState = STATE_READ_TYPE_OR_LEN;
          msgCRC = tmp;
        } else {
          msgState = STATE_WAIT_FOR_HEADER;
          errorcounter++;
          msgID = 0;
          dataLength = 0;
          frameID = 0;
          return FRAME_ERROR;
        }
        break;

      case STATE_READ_CHECKSUM: {
        unsigned char msgChecksum = tmp;
        msgState = STATE_WAIT_FOR_HEADER;
        msgID = parsedMsgID;
        dataLength = parsedDataLength;
        frameID = parsedFrameID;
        framecounter++;

        if ((msgChecksum == msgCRC) || ((msgChecksum == 253) && (msgCRC == 2))) {
          return FRAME_OK;
        }
        errorcounter++;
        return FRAME_ERROR;
      }
    }
  }

  if ((msgState != STATE_WAIT_FOR_HEADER) && ((currentMillis - lastByteMillis) >= FRAME_TIMEOUT_MS)) {
    msgState = STATE_WAIT_FOR_HEADER;
    errorcounter++;
    msgID = 0;
    dataLength = 0;
    frameID = 0;
    return FRAME_ERROR;
  }

  return FRAME_NONE;
}
