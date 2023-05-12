/*
  Multi-instrument panel for navigation. It displays depth, SOG and COG in a 2 line LCD display.

  Hardware needed:
    - Arduino MEGA
    - SIM808 GPS/GSM module (EVB-V3.2)
    - JSN-SR04T waterproof ultrasound sensor
    - HD44780 2x16 LCD display with I2C interface


  By Alberto López Navarro 2023 - GPL-3 license
*/

#include <ArduinoLog.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SIM808.h>


#define triggerPin 2
#define echoPin 3

#define SIM_RX    18 ///< SIM808 RXD
#define SIM_TX    19 ///< SIM808 TXD
#define SIM_PWR   4 ///< SIM808 PWRKEY
#define SIM_RST   5 ///< SIM808 RESET
#define SIM_STATUS  6 ///< SIM808 STATUS


#define SIM808_BAUDRATE 9600    ///< Control the baudrate use to communicate with the SIM808 module
#define SERIAL_BAUDRATE 9600   ///< Controls the serial baudrate between the arduino and the computer
#define NO_FIX_GPS_DELAY 3000   ///< Delay between each GPS read when no fix is acquired
#define FIX_GPS_DELAY  10000    ///< Delay between each GPS read when a fix is acquired

#define POSITION_SIZE   128     ///< Size of the position buffer

#if defined(__AVR__)
    typedef __FlashStringHelper* __StrPtr;
#else
    typedef const char* __StrPtr;
#endif

HardwareSerial simSerial = Serial1;
#define SIM_SERIAL Serial1;
SIM808 sim808 = SIM808(SIM_RST, SIM_PWR, SIM_STATUS);
char position[POSITION_SIZE];

long duration;
float distance;
char buffer[16],dpt[6], sog[6], cog[6];
char res;
float lat=0, lon=0, spd=0, hdg=0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  lcd.init();
  lcd.setCursor(3,0);
  lcd.backlight();
  int i = 0;

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(SERIAL_BAUDRATE);
  Log.begin(LOG_LEVEL_NOTICE, &Serial);

  simSerial.begin(SIM808_BAUDRATE);
  sim808.begin(Serial1);

  Log.notice(("Powering on SIM808..." NL));
  sim808.powerOnOff(true);
  sim808.init();

  Log.notice(("Powering on SIM808s GPS..." NL));
  sim808.powerOnOffGps(true);

}

// the loop function runs over and over again forever
void loop() {
  int i = 0;

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(5);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  //distance = (duration / 1e6)* 340 / 2; // In air
  distance = (duration / 1e6)* 1500 / 2; // In water

  lcd.setCursor(0,0);
  lcd.print(" DPT   SOG  COG");

  lcd.setCursor(0,1);
  dtostrf((float)distance,4,1,dpt);
  dtostrf((float)(spd/1.8),3,1,sog);
  dtostrf((float)hdg,3,0,cog);
  sprintf(buffer, "%sm %skt %s", dpt, sog, cog);
  lcd.print(buffer); lcd.print((char)223);

  Log.notice("%s" CR, buffer);

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                        // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);

  SIM808GpsStatus status = sim808.getGpsStatus(position, POSITION_SIZE);

  if(status < SIM808GpsStatus::Fix) {
      Log.notice("No fix yet... %d" CR, status);
      delay(NO_FIX_GPS_DELAY);
  } else {
      sim808.getGpsField(position, SIM808GpsField::Latitude, &lat);
      sim808.getGpsField(position, SIM808GpsField::Longitude, &lon);
      sim808.getGpsField(position, SIM808GpsField::Speed, &spd);
      sim808.getGpsField(position, SIM808GpsField::Course, &hdg);
      Log.notice("Fix: %F %F %F %F" CR, lat, lon, spd, hdg);
  }
}
