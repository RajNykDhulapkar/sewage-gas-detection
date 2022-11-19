#include <SoftwareSerial.h>

SoftwareSerial wifiSerial(2, 3); // RX, TX for ESP8266

void setup()
{
    wifiSerial.begin(9600);
    Serial.begin(9600);
    wifiSerial.write("AT");
}

void loop()
{
    if (wifiSerial.available())
    {
        Serial.print(wifiSerial.readBytesUntil('\n'));
    }

    if (Serial.available())
    {
        wifiSerial.write(Serial.readString());
    }
}