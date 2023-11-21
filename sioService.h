// Initial version - 15-Nov-2023 - Dean Gienger
// TODO - convert to class format to support multiple serial ports
//---------------------------------------------------------------------
//   S E R I A L   I N P U T   S E R V I C E
//---------------------------------------------------------------------
// Call the sioService() at least once per second
//
#define SIO_INPUT_MAXLEN (128)
char siobuf[SIO_INPUT_MAXLEN+4];
char siorx[SIO_INPUT_MAXLEN+4];
#define SIOPROMPT "\n>"

int siobufptr = 0;

void sioInit()
{
  siobufptr = 0;
  Serial.print(SIOPROMPT);
}

char* sioService()
{
  while (Serial.available())
  {
    char c = Serial.read() & 0x7f; // just in case
    if ((c == '\015') || (c == '\012')) // return key  CR character
    {
      siobuf[siobufptr] = '\0';
      strcpy(siorx,siobuf);
      siobufptr = 0;
      Serial.print(SIOPROMPT); // issue prompt
      return &siorx[0];
    }
    else
    {
      if (siobufptr < SIO_INPUT_MAXLEN)
      {
        Serial.print(c); // echo
        siobuf[siobufptr++] = c; // save this char 
        // TODO - handle delete key later
      }
    }
  }
  return NULL;
}
