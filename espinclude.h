// Rs485 einlesen eines Bytes 19200 8 n 1

// io-Stats
long bytecounter = 0;
long framecounter = 0;
long errorcounter = 0;

extern unsigned long longwaitcount;

// Lese ein Byte - wenn dies innerhalb von x ms ansteht

unsigned char readbyte() {
  unsigned char b;
  int timestamp;
  int wait = 3 ; // 3 *  5 ms  warten


  // Counter Überläufe
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

      b = Serial.read();  // Read received byte
      return (b);
    } else { // kein Input , mx timeout sekunde warten
      waitcount++;

      if (wait-- == 0) {
        longwaitcount++;
        return (0);
      }
      delay(5);
    }
  }
}

// ein bestimmtes Bit auslesen
int getbit(unsigned char *data, int nOffset, int nBit) {
  int nValue;
  nValue = (data[nOffset] >> nBit) & 1;
  return ( nValue);
}

// untersch. Messwerte auslesen
double getval2(unsigned char *anData, int nOffset, int nLen, double fFactor, int bSigned) {
  int   nValue = 0;
  int nI;
  for (nI = 0; nI < nLen; nI++) {
    nValue += anData[nOffset + nI] << ((nLen - nI - 1) * 8);
    // wenn val > 2^15 val -= 2^16
    if (bSigned && (nValue > (1 << (nLen * 8 - 1))))
      nValue -= (1 << (nLen * 8));
  }
  return (nValue * fFactor);
}

// Hilfsfunktion für Prüfsumme
unsigned char CrcAdd(unsigned char crc, unsigned char nByte) {
  crc = (crc << 1) | ( crc >> 7);
  crc = crc + nByte + ((crc + (int)nByte > 255) ? 1 : 0);
  return (crc);
}


// Read RS 484 Frame
int readframe(unsigned char anData[], int &nID, int &nDataLen, int &fid, int &error) {
  int nState, bRxFinished;
  unsigned char nType, nLen, nCounter, nChecksum;
  unsigned char nX;
  unsigned char nCrc;
  nState = STATE_WAIT_FOR_HEADER;
  bRxFinished = FALSE;
  nLen = 0; nCounter = 0; nType = 0; nID = 0; nChecksum = 0; nDataLen = 0; nCrc = 0;
  for (int i = 0; i < 256; i++) anData[i] = 0;

  while (bRxFinished == FALSE) {
    nX = readbyte(); // read one byte

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
          nID = readbyte(); // next byte: Message ID
          nCrc = CrcAdd(nCrc, nID);
          nCounter = readbyte(); // next byte: Message Counter
          nCrc = CrcAdd(nCrc, nCounter);
          fid=(int)nCounter;
          nDataLen = nLen - 4 - 1; // Data Length = Message Length without the header and checksum

          if ((nDataLen >= 0) && (nDataLen < 256)) {
            for (int i = 0; (i < nDataLen) ; i++) {
              anData[i] = readbyte();
              nCrc = CrcAdd(nCrc, anData[i]);
              if ( anData[i] == 2) {
                nChecksum = readbyte(); // "2" in data stream is followed by "0" ...
                //nCrc = CrcAdd(nCrc, nChecksum); 0 wird ignoriert
              }
            }
          }
          nChecksum = readbyte();
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
