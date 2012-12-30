/*
 * ArduinoNunchuk.cpp - Improved Wii Nunchuk library for Arduino
 *
 * Copyright 2011-2012 Gabriel Bianconi, http://www.gabrielbianconi.com/
 *
 * Project URL: http://www.gabrielbianconi.com/projects/arduinonunchuk/
 *
 * Based on the following projects/websites:
 *   http://www.windmeadow.com/node/42
 *   http://todbot.com/blog/2008/02/18/wiichuk-wii-nunchuk-adapter-available/
 *   http://wiibrew.org/wiki/Wiimote/Extension_Controllers
 *
 */

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <Wire.h>
#include "ArduinoNunchuk.h"

#define ADDRESS 0x52
#define LOW_PASS_ALPHA 0.15
#define LOW_PASS_INIT  -999

void ArduinoNunchuk::init(bool useMovingAverage, bool useLowPassFilter)
{
  Wire.begin();

  ArduinoNunchuk::_sendByte(0x55, 0xF0);
  ArduinoNunchuk::_sendByte(0x00, 0xFB);

  ArduinoNunchuk::useMovingAverage = useMovingAverage;
  ArduinoNunchuk::useLowPassFilter = useLowPassFilter;

  if (ArduinoNunchuk::useMovingAverage) {
    ArduinoNunchuk::movingIndex = 0;
    // initialize all readings to 0
    for (int thisField = 0; thisField < ARDUCHUCK_AVG_FIELDS; thisField++) {
      ArduinoNunchuk::movingTotals[thisField] = 0;
      ArduinoNunchuk::movingAverages[thisField] = 0;

      for (int thisReading = 0; thisReading < ARDUCHUCK_AVG_READINGS; thisReading++) {
        ArduinoNunchuk::movingReadings[thisField][thisReading] = 0;
      }
    }
  }

  if (ArduinoNunchuk::useLowPassFilter) {
    for (int thisField = 0; thisField < ARDUCHUCK_LP_FIELDS; thisField++) {
      ArduinoNunchuk::previousLowPassValues[thisField] = LOW_PASS_INIT;
    }
  }

  ArduinoNunchuk::update();
}

void ArduinoNunchuk::update()
{
  int count = 0;
  int values[6];

  Wire.requestFrom (ADDRESS, 6);

  while(Wire.available())
  {
    values[count] = Wire.read();
    count++;
  }

  ArduinoNunchuk::analogX = values[0];
  ArduinoNunchuk::analogY = values[1];
  ArduinoNunchuk::accelX  = values[2] * 2 * 2 + ((values[5] >> 2) & 1) * 2 + ((values[5] >> 3) & 1);
  ArduinoNunchuk::accelY  = values[3] * 2 * 2 + ((values[5] >> 4) & 1) * 2 + ((values[5] >> 5) & 1);
  ArduinoNunchuk::accelZ  = values[4] * 2 * 2 + ((values[5] >> 6) & 1) * 2 + ((values[5] >> 7) & 1);
  ArduinoNunchuk::zButton = !((values[5] >> 0) & 1);
  ArduinoNunchuk::cButton = !((values[5] >> 1) & 1);

  if (ArduinoNunchuk::useMovingAverage) {
    ArduinoNunchuk::_calculateMovingAverages();
  }

  if (ArduinoNunchuk::useLowPassFilter) {
    ArduinoNunchuk::_calculateLowPass();
  }

  ArduinoNunchuk::_sendByte(0x00, 0x00);
}

// calculate a moving average for the accelerometers
void ArduinoNunchuk::_calculateMovingAverages() {
  int reading;

  for (int thisField = 0; thisField < ARDUCHUCK_AVG_FIELDS; thisField++) {
    // remove previous value in current index from running total
    ArduinoNunchuk::movingTotals[thisField] = ArduinoNunchuk::movingTotals[thisField] - ArduinoNunchuk::movingReadings[thisField][ArduinoNunchuk::movingIndex];

    switch(thisField) {
    case 0:
      reading = ArduinoNunchuk::accelX;
      break;
    case 1:
      reading = ArduinoNunchuk::accelY;
      break;
    case 2:
      reading = ArduinoNunchuk::accelZ;
      break;
    }

    ArduinoNunchuk::movingReadings[thisField][ArduinoNunchuk::movingIndex] = reading;
    ArduinoNunchuk::movingTotals[thisField] = ArduinoNunchuk::movingTotals[thisField] + ArduinoNunchuk::movingReadings[thisField][ArduinoNunchuk::movingIndex];

    // calculate the average:
    ArduinoNunchuk::movingAverages[thisField] = ArduinoNunchuk::movingTotals[thisField] / ARDUCHUCK_AVG_READINGS;
  }

  // reset the accel values
  ArduinoNunchuk::accelX = ArduinoNunchuk::movingAverages[0];
  ArduinoNunchuk::accelY = ArduinoNunchuk::movingAverages[1];
  ArduinoNunchuk::accelZ = ArduinoNunchuk::movingAverages[2];

  ArduinoNunchuk::_incrementIndex();
}

void ArduinoNunchuk::_incrementIndex() {
  ArduinoNunchuk::movingIndex++;

  // if we're at the end of the array...
  if (ArduinoNunchuk::movingIndex >= ARDUCHUCK_AVG_READINGS) {
    // ...wrap around to the beginning:
    ArduinoNunchuk::movingIndex = 0;
  }
}

void ArduinoNunchuk::_calculateLowPass() {
  int reading;
  for (int i=0; i < ARDUCHUCK_LP_FIELDS; i++) {
    switch(i) {
    case 0:
      reading = ArduinoNunchuk::analogX;
      break;
    case 1:
      reading = ArduinoNunchuk::analogY;
      break;
    case 2:
      reading = ArduinoNunchuk::accelX;
      break;
    case 3:
      reading = ArduinoNunchuk::accelY;
      break;
    case 4:
      reading = ArduinoNunchuk::accelZ;
      break;
    }

    if (ArduinoNunchuk::previousLowPassValues[i] == LOW_PASS_INIT) {
      ArduinoNunchuk::previousLowPassValues[i] = (double) reading;
    } else {
      ArduinoNunchuk::previousLowPassValues[i] = ArduinoNunchuk::previousLowPassValues[i] +
        LOW_PASS_ALPHA * (((double)reading) - ArduinoNunchuk::previousLowPassValues[i]);
    }
  }

  // reset the accel values
  ArduinoNunchuk::analogX = ArduinoNunchuk::previousLowPassValues[0];
  ArduinoNunchuk::analogY = ArduinoNunchuk::previousLowPassValues[1];
  // ArduinoNunchuk::accelX  = ArduinoNunchuk::previousLowPassValues[2];
  // ArduinoNunchuk::accelY  = ArduinoNunchuk::previousLowPassValues[3];
  // ArduinoNunchuk::accelZ  = ArduinoNunchuk::previousLowPassValues[4];
}

void ArduinoNunchuk::_sendByte(byte data, byte location) {
  Wire.beginTransmission(ADDRESS);

  #if (ARDUINO >= 100)
    Wire.write(location);
    Wire.write(data);
  #else
    Wire.send(location);
    Wire.send(data);
  #endif

  Wire.endTransmission();

  delay(10);
}
