#include <Arduino.h>
#include "pitches.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include "RTClib.h"
#include <statusLED.h>

// put function declarations here:
int myFunction(int, int);
long readVcc();
void vccCalibration();
void setupVccReader();

#define SCL (5)
#define SDA (4)

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

Adafruit_BMP280 bmp;
RTC_DS3231 rtc;
// Status LED objects
statusLED greenLED(false); // False = not inversed
statusLED redLED(false);
statusLED blueLED(false);

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(100); // wait for native usb

  setupVccReader();

  greenLED.begin(9);
  redLED.begin(10);
  blueLED.begin(11);

  redLED.on();
  delay(2000);
  greenLED.on();
  delay(1000);
  redLED.pwm(10); // PWM capable pins required!
  delay(1000);
  greenLED.pwm(50);
  delay(1000);
  redLED.off();
  delay(1000);
  greenLED.off();
  delay(1000);

  // BMP280 temp/humidity sensor
  unsigned status;
  status = bmp.begin(0x76);
  if (!status)
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // The DS3231S RTC chip's fixed I2C address is 0x68, and the EEPROM's default I2C address is 0x57
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1)
      delay(10);
  }
  if (rtc.lostPower())
  {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  // notice setup finished
  tone(4, NOTE_C1, 400);
}

void loop()
{

  redLED.flash(140, 150, 700, 5, 70); // ON, OFF, PAUSE, PULSES, (OPTIONAL DELAY FOR FIRST PASS)
  greenLED.flash(40, 350, 0, 0);
  blueLED.flash(200, 100, 0, 0);

  Serial.print("Vcc=");
  Serial.println(readVcc());

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  Serial.print("Temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");

  delay(1000);
  // no need to repeat the melody.
}

void setupVccReader()
{
  // https://www.electronicwings.com/avr-atmega/atmega1632-adc
  //  Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
}

long readVcc()
{
  long result;
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC))
    ;
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void vccCalibration()
{

  // Read "1.1V" reference against AREF
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);            // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC))
    ;
  unsigned int reading = ADC;
  // unsigned int reading = ADCL | (ADCH << 8);
  float fraction = reading / 1024.0;
  Serial.println(F("Use a meter to measure the voltage between GND and AREF pins."));
  Serial.print(F("Multiply that voltage by "));
  Serial.print(fraction, 7);
  Serial.println(F(" to get the value of the internal voltage reference."));
  Serial.println(F("Then multiply that by 1204000 to get the constant to use in readVCC()."));

  // unsigned long VCC = 1120500UL / reading;
}