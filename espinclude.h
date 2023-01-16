// Rs485 einlesen eines Bytes 19200 8 n 1

// io-Stats
long bytecounter = 0;
long framecounter = 0;
long errorcounter = 0;

unsigned long longwaitcount = 0;
unsigned long bytecount = 0;
unsigned long waitcount = 0;

// read a byte if it is there in x ms
unsigned char readByte() {
  unsigned char b;
  int timestamp;
  int wait = 3 ; // wait for 3 * 5 ms

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

      b = Serial.read(); // read received byte
      return (b);
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
  int value;
  value = (data[offset] >> numberOfBit) & 1;
  return value;
}

// read different values from the message as int (times fFactor)
// data = data byte array
// offset = offset in bytes to start reading from
// length = read x bytes
// fFactor = used to multiply the read result
// isSigned = is it a signed or unsigned number?
double getValue(unsigned char *data, int offset, int length, double fFactor, bool isSigned) {
  int value = 0;
  int nI;
  for (nI = 0; nI < length; nI++) {
    value += data[offset + nI] << ((length - nI - 1) * 8);
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


// Read RS 484 Frame
int readFrame(unsigned char anData[], int &nID, int &nDataLen, int &fid, int &error) {
  int nState, bRxFinished;
  unsigned char nType, nLen, nCounter, nChecksum;
  unsigned char nX;
  unsigned char nCrc;
  nState = STATE_WAIT_FOR_HEADER;
  bRxFinished = FALSE;
  nLen = 0; nCounter = 0; nType = 0; nID = 0; nChecksum = 0; nDataLen = 0; nCrc = 0;
  for (int i = 0; i < 256; i++) anData[i] = 0;

  while (bRxFinished == FALSE) {
    nX = readByte(); // read one byte

    if ((nState == STATE_WAIT_FOR_HEADER) && (nX == 2)) {
      nState = STATE_READ_MSG; // header found
      nCrc = nX;
      nType = MSG_TYPE_CTRL;
    } else {
      if (nState == STATE_READ_MSG) {
        if (nX == 0) // header invalid -> start again
          nState = STATE_WAIT_FOR_HEADER;
        if (nX == 2) { // extended header -> SenseMessage
          nState = STATE_READ_MSG;
          nType = MSG_TYPE_SENSE;
          nCrc = nX;
        }
        if ((nX != 0) && (nX != 2)) {
          nLen = nX; // current byte: Message Length
          nCrc = CrcAdd(nCrc, nLen);
          nID = readByte(); // next byte: Message ID
          nCrc = CrcAdd(nCrc, nID);
          nCounter = readByte(); // next byte: Message Counter
          nCrc = CrcAdd(nCrc, nCounter);
          fid=(int)nCounter;
          nDataLen = nLen - 4 - 1; // Data Length = Message Length without the header and checksum

          if ((nDataLen >= 0) && (nDataLen < 256)) {
            for (int i = 0; (i < nDataLen) ; i++) {
              anData[i] = readByte();
              nCrc = CrcAdd(nCrc, anData[i]);
              if ( anData[i] == 2) {
                nChecksum = readByte(); // "2" in data stream is followed by "0" ...
                //nCrc = CrcAdd(nCrc, nChecksum); 0 wird ignoriert
              }
            }
          }
          nChecksum = readByte();
          bRxFinished = TRUE;
        }
      }
    }
  }

  framecounter++;
  if ((nChecksum == nCrc) || ((nChecksum == 253) && (nCrc == 2))) {
    error=0;
    return 1;
  } else {
    // Statistik fehlerhafte Frames
    errorcounter++;
    error=1;
    return 0;
  }
}
