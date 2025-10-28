/*
  EmonLib.h - Library for Energy Monitoring
  Copyright 2025 R Franks

  Heavily based on the work of openenergymonitor
  https://github.com/openenergymonitor/EmonLib
  Created by Trystan Lea, April 27 2010
  GNU GPL
*/

#ifndef EMONLIB_H_
#define EMONLIB_H_
#include "Arduino.h"

// to enable 12-bit ADC resolution on Arduino Due,
// include the following line in main sketch inside setup() function:
//  analogReadResolution(ADC_BITS);
// otherwise will default to 10 bits, as in regular Arduino-based boards.
#if defined(__arm__)
#define ADC_BITS    12
#else
#define ADC_BITS    10
#endif

#define ADC_COUNTS  (1 << ADC_BITS)


class EnergyMonitor {
 public:
  EnergyMonitor();

  typedef int (*analogReaderMethod) (int _pin);
  analogReaderMethod analogReader;


  void voltage(uint8_t _inPinV, double _VCAL, double _PHASECAL);
  void current(uint8_t _inPinI, double _ICAL);

  void calcVI(uint16_t crossings, uint16_t timeout, uint16_t SupplyVoltage);

  // Useful value variables
  double realPower,
    apparentPower,
    powerFactor,
    Vrms,
    Irms;

 private:
  static int defaultAnalogReader(int _pin);

  // Voltage and current input pins
  uint8_t inPinV;
  uint8_t inPinI;
  // Calibration coefficients
  // These need to be set in order to obtain accurate results
  double VCAL;
  double ICAL;
  double PHASECAL;

  // Raw analog read values
  int sampleV;
  int sampleI;

  // Raw analog values less DC offset
  double lastFilteredV, filteredV;
  double filteredI;
  // Low pass filter outputs
  double offsetV;
  double offsetI;

  // Calibrated phase-shifted voltage
  double phaseShiftedV;

  // sq = squared, sum = summed, inst = instantaneous
  double sqV,
    sumV,
    sqI,
    sumI,
    instP,
    sumP;

  // Instantaneous voltage at start of sample window
  int startV;
  // Used to measure number of times threshold is crossed
  boolean lastVCross, checkVCross;
};

#endif  // EMONLIB_H_
