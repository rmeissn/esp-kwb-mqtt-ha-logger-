// Read RS485 byte, baud 19200, (8 n 1 ???)

long bytecounter = 0;

unsigned long longwaitcount = 0;
unsigned long bytecount = 0;
unsigned long waitcount = 0;

// read a byte if it is there in x ms
unsigned char readByte() {
  int wait = 3, timestamp; // wait for 3 * 5 ms

  // counter overflow
  #define MAXCOUNT 1000000000
  if ((waitcount > MAXCOUNT) || (bytecount > MAXCOUNT)) {
    waitcount = 0;
    bytecount = 0;
  }
  if (longwaitcount > MAXCOUNT)
    longwaitcount = 0;

  while (1) {
    if (Serial.available()) {
      bytecount++;
      bytecounter++;
      return (Serial.read()); // read received byte
    } else { // no input, mx timeout wait a second
      waitcount++;

      if (wait-- == 0) {
        longwaitcount++;
        return (0);
      }
      delay(5);
    }
  }
}

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
double getValue(unsigned char *data, int offset, int length, double fFactor, bool isSigned) {
  int value = 0;
  for (int i = 0; i < length; i++) {
    value += data[offset + i] << ((length - i - 1) * 8);
    // if val > 2^15 val -= 2^16
    if (isSigned && (value > (1 << (length * 8 - 1))))
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

#define STATE_WAIT_FOR_HEADER 1
#define STATE_READ_MSG 2
#define MSG_TYPE_CTRL 1
#define MSG_TYPE_SENSE 2

long framecounter = 0;
long errorcounter = 0;
// extern int MSGMAXLENGTH;

// Read RS 484 Frame
// return error reading frame = true/false
bool readFrame(unsigned char data[], int &msgID, int &dataLength, int &frameID) {
  int msgState = STATE_WAIT_FOR_HEADER;
  bool bRxFinished = false;
  unsigned char msgType = 0, msgLength = 0, msgCounter = 0, msgChecksum = 0, msgCRC  = 0, tmp;
  dataLength = 0; msgID = 0;

  memset(data, 0, MSGMAXLENGTH); // write 0s to data

  while (!bRxFinished) {
    tmp = readByte(); // read one byte

    if ((msgState == STATE_WAIT_FOR_HEADER) && (tmp == 2)) {
      msgState = STATE_READ_MSG; // header found
      msgCRC = tmp;
      msgType = MSG_TYPE_CTRL;
    } else {
      if (msgState == STATE_READ_MSG) {
        if (tmp == 0) // header invalid -> start again
          msgState = STATE_WAIT_FOR_HEADER;
        if (tmp == 2) { // extended header -> SenseMessage
          msgState = STATE_READ_MSG;
          msgType = MSG_TYPE_SENSE;
          msgCRC = tmp;
        }
        if ((tmp != 0) && (tmp != 2)) {
          msgLength = tmp; // current byte: Message Length
          msgCRC = CrcAdd(msgCRC, msgLength);
          msgID = readByte(); // next byte: Message ID
          msgCRC = CrcAdd(msgCRC, msgID);
          msgCounter = readByte(); // next byte: Message Counter
          msgCRC = CrcAdd(msgCRC, msgCounter);
          frameID=(int)msgCounter;
          dataLength = msgLength - 4 - 1; // Data Length = Message Length without the header and checksum

          if ((dataLength >= 0) && (dataLength < MSGMAXLENGTH)) {
            for (int i = 0; (i < dataLength) ; i++) {
              data[i] = readByte();
              msgCRC = CrcAdd(msgCRC, data[i]);
              if ( data[i] == 2) {
                msgChecksum = readByte(); // "2" in data stream is followed by "0" ...
                //msgCRC = CrcAdd(msgCRC, msgChecksum); 0 wird ignoriert
              }
            }
          }
          msgChecksum = readByte();
          bRxFinished = true;
        }
      }
    }
  }

  framecounter++;
  if ((msgChecksum == msgCRC) || ((msgChecksum == 253) && (msgCRC == 2)))
    return false;
  else { // statistics for faulty frames
    errorcounter++;
    return true;
  }
}
