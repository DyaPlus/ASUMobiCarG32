#include <SoftwareSerial.h>

SoftwareSerial gps (0,1);

char data  ;

void  setup ()
{
 Serial . begin (115200);            
 gps. begin (9600);
}


void  loop ()
{
  if (gps. available ())
  {
    data = gps. read ();
    Serial . print (data);
  }
}

