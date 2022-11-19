#include <Arduino.h>

#include <LiquidCrystal.h>
#include <MQUnifiedsensor.h>

#include <SoftwareSerial.h>
#include <SerialESP8266wifi.h>

#define sw_serial_rx_pin 11 //  Connect this pin to TX on the esp8266
#define sw_serial_tx_pin 10 //  Connect this pin to RX on the esp8266
#define esp8266_reset_pin 9 // Connect this pin to CH_PD on the esp8266, not reset. (let reset be unconnected)

#define SERVER_PORT "8080"
#define SSID "YourSSID"
#define PASSWORD "YourPassword"

SoftwareSerial swSerial(sw_serial_rx_pin, sw_serial_tx_pin);

// the last parameter sets the local echo option for the ESP8266 module..
SerialESP8266wifi wifi(Serial, Serial, esp8266_reset_pin, swSerial);

String inputString;
boolean stringComplete = false;
unsigned long nextPing = 0;

#define MQPin (A0) // analogue input pin for sensor
#define Board ("Arduino UNO")

#define Type ("MQ-2") // MQ2
#define Voltage_Resolution (5)
#define ADC_Bit_Resolution (10) // For arduino UNO/MEGA/NANO
#define RatioMQ2CleanAir (9.83) // R1 / R0 = 9.83 ppm

#define BuzzerPin (DD3)

// variable to store sensor value
float sensorValue; // 0 - 1023
float ppm;
float threshold = 2000; // ppm

int rs = 1,
    enable = 2, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, enable, d4, d5, d6, d7);

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

void setup()
{
  inputString.reserve(20);

  Serial.begin(9600);
  swSerial.begin(9600);

  lcd.begin(16, 2);
  swSerial.println("Starting wifi");

  wifi.setTransportToTCP();       // this is also default
  wifi.endSendWithNewline(false); // Will end all transmissions with a newline and carrage return ie println.. default is true

  wifi.begin();

  // Turn on local ap and server (TCP)
  wifi.startLocalAPAndServer("MY_CONFIG_AP", "password", "5", "2121");

  wifi.connectToAP(SSID, PASSWORD);
  wifi.connectToServer("192.168.0.28", "2121");
  wifi.send(SERVER, "ESP8266 test app started");

  delay(1000);
  Serial.println("Gas sensor warming up");
  delay(10000);
  Serial.println("Gas sensor ready");

  // Set math model to calculate the PPM concentration and the value of constants
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(574.25);
  MQ2.setB(-2.222); // Configure the equation to to calculate LPG concentration

  /*
    Exponential regression:
    Gas    | a      | b
    H2     | 987.99 | -2.162
    LPG    | 574.25 | -2.222
    CO     | 36974  | -3.109
    Alcohol| 3616.1 | -2.675
    Propane| 658.71 | -2.168
  */

  MQ2.init();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0))
  {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1)
      ;
  }
  if (calcR0 == 0)
  {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1)
      ;
  }
  /*****************************  MQ CAlibration ********************************************/

  MQ2.serialDebug(true);
}

void loop()
{
  // read the sensor value
  sensorValue = analogRead(MQPin);

  static WifiConnection *connections;

  // Make sure the esp8266 is started..
  if (!wifi.isStarted())
    wifi.begin();

  Serial.print("Sensor Value is : ");
  Serial.println(sensorValue);

  MQ2.update();           // Update data, the arduino will read the voltage from the analog pin
  ppm = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  MQ2.serialDebug();      // Will print the table on the serial port

  Serial.print("Sensor Value in ppm for LPG : ");
  Serial.println(ppm);

  // Send what you typed in the arduino console to the server
  static char buf[20];
  if (stringComplete)
  {
    inputString.toCharArray(buf, sizeof buf);
    wifi.send(SERVER, buf);
    inputString = "";
    stringComplete = false;
  }

  // Send a ping once in a while..
  if (millis() > nextPing)
  {
    wifi.send(SERVER, "Ping ping..");
    nextPing = millis() + 10000;
  }

  // Listen for incoming messages and echo back, will wait until a message is received, or max 6000ms..
  WifiMessage in = wifi.listenForIncomingMessage(6000);
  if (in.hasData)
  {
    if (in.channel == SERVER)
      Serial.println("Message from the server:");
    else
      Serial.println("Message a local client:");
    Serial.println(in.message);
    // Echo back;
    wifi.send(in.channel, "Echo:", false);
    wifi.send(in.channel, in.message);
    wifi.send(in.channel, "msg", false);
    wifi.send(in.channel, String(ppm).c_str(), false);
    // nextPing = millis() + 10000;
  }

  lcd.setCursor(0, 0);
  lcd.print(String(ppm).c_str());

  if (ppm > threshold)
  {
    digitalWrite(BuzzerPin, HIGH);
  }
  else
  {
    digitalWrite(BuzzerPin, LOW);
  }

  // delay between readings
  delay(2000);
  lcd.clear();
}

// Listen for serial input from the console
void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      stringComplete = true;
    }
  }
}