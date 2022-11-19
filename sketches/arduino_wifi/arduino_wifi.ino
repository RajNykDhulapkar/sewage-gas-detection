#include <SoftwareSerial.h>
#include <Arduino.h>

// SoftwareSerial espSerial(3, 2);

void setup()
{
    Serial.begin(115200);
}
void loop()
{
    Serial.println(millis() / 1000);
    delay(1000);
}