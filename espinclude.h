//###################################################################
//###################################################################
// Rs485 einlesen eines Bytes 19200 8 n 1

// io-Stats
long bytecounter = 0;
long framecounter = 0;
long errorcounter = 0;

extern unsigned long longwaitcount;

// Lese ein Byte - wenn dies innerhalb von
// x ms ansteht

unsigned char readbyte()
{
  unsigned char b;
  int timestamp;
  int wait = 3 ; // 3 *  5 ms  warten


  // Counter Überläufe
#define MAXCOUNT 1000000000
  if ((waitcount > MAXCOUNT) || (bytecount > MAXCOUNT))
  {
    waitcount = 0;
    bytecount = 0;
  }
  if (longwaitcount > MAXCOUNT)
  {
    longwaitcount = 0;
  }


  while (1)
  {

#ifdef SWSERIAL
  if (RS485Serial.available())  //Look for data from other Arduino
#else
  if (Serial.available())
#endif  
    {
      bytecount++;
      bytecounter++;
      
#ifdef SWSERIAL
     b = RS485Serial.read();  // Read received byte
#else
     b = Serial.read();  // Read received byte
#endif  
      
      
      // 
      return (b);
    }
    else
    { // kein Input , mx timeout sekunde warten

      waitcount++;

      if (wait-- == 0)
      {
        longwaitcount++;
        return (0);
      }
      delay(5);
    }
  }
}



/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
// ein bestimmtes Bit auslesen
int getbit(unsigned char *data, int nOffset, int nBit)
{
  int nValue;
  nValue = (data[nOffset] >> nBit) & 1;
  return ( nValue);
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
// untersch. Messwerte auslesen
double getval2(unsigned char *anData, int nOffset, int nLen, double fFactor, int bSigned)
{
  int   nValue = 0;
  int nI;
  for (nI = 0; nI < nLen; nI++)
  {
    nValue += anData[nOffset + nI] << ((nLen - nI - 1) * 8);

    // wenn val > 2^15 val -= 2^16
    if (bSigned && (nValue > (1 << (nLen * 8 - 1))))
    {
      nValue -= (1 << (nLen * 8));
    }
  }
  return (nValue * fFactor);
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
// Hilfsfunktion für Prüfsumme
unsigned char CrcAdd(unsigned char crc, unsigned char nByte)
{
  crc = (crc << 1) | ( crc >> 7);
  if (crc + (int)nByte > 255)
    crc = crc + nByte + 1;
  else
    crc = crc + nByte;
  return (crc);
}

//####################################################################
//####################################################################
// Reconnected den MQTT, wenn er nicht mehr verbunden ist
void mqttreconnect() {
  int i;
  i = 5;
  // loopcount until we're reconnected

  while (i-- && !client.connected()) {
    if (client.connect(MQNAME, mqtt_user, mqtt_password)) {
      client.subscribe(INTOPIC);
    } else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

char getHexDigit(byte digit)
{
  char c;
  if (digit >= 0 && digit <= 9)
    c = digit + '0';
  else if (digit >= 0xA && digit <= 0xF)
    c = digit + 'A' - 10;
  else
    c = '0';

  return c;
}

char* byteToHexString(char* msg, byte* data,  byte length)
{
  byte digit;
  char* res;
  res = msg;
  for (int i = 0; i < length; i++)
  {
    digit = data[i] / 16;
    res[0] = getHexDigit(digit);
    res++;
    digit = data[i] % 16;
    res[0] = getHexDigit(digit);
    res++;
  }
  return res;
}

void publish_rawdata(unsigned char anData[], int nID, int nDataLen, int nChecksum, int nCrc) 
{
  char msg[256];
  char* str;
  int len;
  mqttreconnect();
  sprintf(msg, "id: %d len: %d crc: %d ccrc: %d", nID, nDataLen, nChecksum, nCrc);
  client.publish("kwb/rawdataheader", msg);   
  str = msg;
  for (int i = 0; i < nDataLen; i = i + 2)
  { 
    str = byteToHexString(str, &anData[i], 2);
    str[0] = ' ';
    str++;
  }
  str[0] = '\0';
  client.publish("kwb/rawdata", msg); 
}

// Read RS 484 Frame
int readframe(unsigned char anData[], int &nID, int &nDataLen, int &fid, int &error)
{
  int nState, bRxFinished;
  unsigned char nType, nLen, nCounter, nChecksum;
  unsigned char nX;
  unsigned char nCrc;
  nState = STATE_WAIT_FOR_HEADER;
  bRxFinished = FALSE;

  nLen = 0; nCounter = 0; nType = 0; nID = 0; nChecksum = 0; nDataLen = 0; nCrc = 0;
  for (int i = 0; i < 256; i++) anData[i] = 0;

  while (bRxFinished == FALSE)
  {
    nX = readbyte();                           // # read one byte

    if ((nState == STATE_WAIT_FOR_HEADER) && (nX == 2))
    {
      nState = STATE_READ_MSG;             // # header found
      nCrc = nX;
      nType = MSG_TYPE_CTRL;
    }
    else
    {
      if (nState == STATE_READ_MSG)
      {
        if (nX == 0)                        // # header invalid -> start again
        {
          nState = STATE_WAIT_FOR_HEADER;
        }
        if (nX == 2)                       //# extended header -> SenseMessage
        {
          nState = STATE_READ_MSG;
          nType = MSG_TYPE_SENSE;
          nCrc = nX;
        }
        if ((nX != 0) && (nX != 2))
        {
          nLen = nX;                   //     # current byte: Message Length
          nCrc = CrcAdd(nCrc, nLen);
          nID = readbyte();            //     # next byte: Message ID
          nCrc = CrcAdd(nCrc, nID);
          nCounter = readbyte();       //     # next byte: Message Counter
          nCrc = CrcAdd(nCrc, nCounter);
          fid=(int)nCounter;
          nDataLen = nLen - 4 - 1;     //     Data Length = Message Length without the header and checksum

          if ((nDataLen >= 0) && (nDataLen < 256))
            for (int i = 0; (i < nDataLen) ; i++)
            {
              anData[i] = readbyte();
              nCrc = CrcAdd(nCrc, anData[i]);
              if ( anData[i] == 2)
              {
                nChecksum = readbyte();        //      # "2" in data stream is followed by "0" ...
                //nCrc = CrcAdd(nCrc, nChecksum); 0 wird ignoriert
              }
            }
          nChecksum = readbyte();
          bRxFinished = TRUE;
        }
      }
    }
  }

  framecounter++;
  if ((nChecksum == nCrc) || ((nChecksum == 253) && (nCrc == 2)))
  {
    error=0;
    return 1;
  }  
  else
  {
    // Statistik fehlerhafte Frames
    errorcounter++;
    error=1;
    publish_rawdata(anData, nID, nDataLen, nChecksum, nCrc);
    return 0; 
  }
}
  
// liefert ne 1 wenn die beiden char Blöcke unterschiedlich sind
bool messne(unsigned char * a, unsigned char * b, int sz)
{
  int i;

  for (i = 0; i < sz; i++)
    if (a[i] != b[i]) return (1);

  return (0);
}

bool tempdiff(double a, double b, double d)
{
  return (abs(a - b) >= d);
}

// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

void send_value(char* msg, double val)
{
  char valstr[20];
  mqttreconnect();
  sprintf(valstr, "%.1f", val);
  client.publish(msg, valstr);
}

// Blink n mal die Webmod onbload LED

void blink(int n)
{

#define ledPin 2 // led Pin
  pinMode(ledPin, OUTPUT);
  // Blink
#define INTERVAL 500

  while (n-- > 0)
  {
    digitalWrite(ledPin, LOW);
    delay(INTERVAL);
    digitalWrite(ledPin, HIGH);
    delay(INTERVAL);
  }
}

void relais(int n)
{
  // https://chewett.co.uk/blog/1066/pin-numbering-for-wemos-d1-mini-esp8266/
#define RELAIS 5 // d1 pin
  pinMode(RELAIS, OUTPUT);

  if (n == 1)
  {
    digitalWrite(RELAIS, HIGH);
    Serial.println("Relais an ");
  }
  else
  {

    digitalWrite(RELAIS, LOW);
    Serial.println("Relais aus ");

  }

}

//##################################################
