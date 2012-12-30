/*
 * ArduinoNunchuk.h - Improved Wii Nunchuk library for Arduino
 *
 * Copyright 2011-2012 Gabriel Bianconi, http://www.gabrielbianconi.com/
 *
 * Project URL: http://www.gabrielbianconi.com/projects/arduinonunchuk/
 *
 * Based on the following projects/websites:
 *   http://www.windmeadow.com/node/42
 *   http://todbot.com/blog/2008/02/18/wiichuck-wii-nunchuck-adapter-available/
 *   http://wiibrew.org/wiki/Wiimote/Extension_Controllers
 *
 */

#ifndef ArduinoNunchuk_H
#define ArduinoNunchuk_H

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#define ARDUCHUCK_AVG_READINGS 10
#define ARDUCHUCK_AVG_FIELDS 3
#define ARDUCHUCK_LP_FIELDS 5

class ArduinoNunchuk
{
  public:
    int analogX;
    int analogY;
    int accelX;
    int accelY;
    int accelZ;
    int zButton;
    int cButton;

    void init(bool, bool);
    void update();

  private:
    /// Low pass filter for accelerometer and analog readings
    bool useLowPassFilter;
    void _calculateLowPass();
    double previousLowPassValues[ARDUCHUCK_LP_FIELDS];

    /// Moving Average smoothing for accelerometer readings
    int movingReadings[ARDUCHUCK_AVG_FIELDS][ARDUCHUCK_AVG_READINGS];
    int movingIndex;            // the index of the current reading
    int movingTotals[ARDUCHUCK_AVG_FIELDS];     // the running totals
    int movingAverages[ARDUCHUCK_AVG_READINGS]; // the average value
    bool useMovingAverage;
    void _incrementIndex();
    void _calculateMovingAverages();

    void _sendByte(byte data, byte location);
};

#endif
