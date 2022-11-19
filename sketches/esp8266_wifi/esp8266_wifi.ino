#include <Arduino.h>
void setup()
{
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    Serial.println("Start");
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }
}
void loop()
{ // run over and over
    if (Serial.available())
    {
        Serial.write(Serial.read());
    }
//            Serial.println("read");
//delay(1000);
}
