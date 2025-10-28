/*
  EmonLib.cpp - Library for Energy Monitoring
  Copyright 2025 R Franks

  Heavily based on the work of openenergymonitor
  https://github.com/openenergymonitor/EmonLib
  Created by Trystan Lea, April 27 2010
  GNU GPL
*/

#include "EmonLib.h"

#include "Arduino.h"

/**
 * Constructor.
 */
EnergyMonitor::EnergyMonitor() {
  this->analogReader = defaultAnalogReader;
}

/**
 * Sets the pins to be used for voltage sensor.
 *
 * @param _inPinV The voltage pin
 * @param _VCAL Voltage calibration
 * @param _PHASECAL Phase calibration
 */
void EnergyMonitor::voltage(uint8_t _inPinV,
                            double _VCAL,
                            double _PHASECAL) {
  inPinV = _inPinV;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
}

/**
 * Sets the pins to be used for current sensor.
 *
 * @param _inPinI The current pin
 * @param _ICAL Current calibration
 */
void EnergyMonitor::current(uint8_t _inPinI,
                            double _ICAL) {
  inPinI = _inPinI;
  ICAL = _ICAL;
  offsetI = ADC_COUNTS>>1;
}


/**
 * Calculates the realPower, apparentPower, powerFactor,
 * Vrms, Irms, and kWh increment from a sample window of
 * the mains AC voltage and curent
 * The sample window length is defined by the number of half
 * wavelenghts or crossings we chooose to measure.
 *
 * @param crossings The number of crossings to measure
 * @param timeout Timeout for measurements
 * @param SupplyVoltage The supply voltage against which to compare
 *                        the readings
 */
void EnergyMonitor::calcVI(uint16_t crossings,
                           uint16_t timeout,
                           uint16_t SupplyVoltage) {
  // Used to measure number of times threshold is crossed.
  uint16_t crossCount = 0;
  // This is now incremented
  uint16_t numberOfSamples = 0;

  /* 
   * 1) Wait for the waveform to be close to 'zero'
   * (mid-scale adc) part in sin curve.
   */
  uint16_t start = millis();

  while (1) {
    startV = (this->analogReader)(inPinV);
    if (
      (startV < (ADC_COUNTS*0.55)) &&
      (startV > (ADC_COUNTS*0.45))
    ) {
      break;
    }
    if ((millis() - start) > timeout) {
      break;
    }
  }

  /* 
   * 2) Main loop
   */

  // Reset timeout counter
  start = millis();

  while ((crossCount < crossings) && ((millis() - start) < timeout)) {
    // Increment number of samples measured
    numberOfSamples++;
    // Used for delay/phase compensation
    lastFilteredV = filteredV;

    /* 
     * A) Read in raw voltage and current samples
     */

    // Get raw voltage
    sampleV = (this->analogReader)(inPinV);
    // Get raw current
    sampleI = (this->analogReader)(inPinI);

    /* 
     * B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset
     *     then subtract this - signal is now centred on 0 counts
     */
    offsetV = offsetV + ((sampleV - offsetV) / 1024);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI - offsetI) / 1024);
    filteredI = sampleI - offsetI;

    /* 
     * C) Root-mean-square method voltage
     */
    sqV = filteredV * filteredV;
    sumV += sqV;

    /* 
     * D) Root-mean-square method current
     */
    sqI = filteredI * filteredI;
    sumI += sqI;

    /* 
     * E) Phase calibration
     */
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

    /* 
     * F) Instantaneous power calc
     */
    instP = phaseShiftedV * filteredI;
    sumP += instP;

    /* 
     * G) Find the number of times the voltage has crossed the initial voltage
     *    - Every 2 crosses we will have sampled 1 wavelength
     *    - so this method allows us to sample an integer number of half wavelengths
     *      which increases accuracy
     */
    lastVCross = checkVCross;

    checkVCross = (sampleV > startV);

    if (numberOfSamples == 1) {
      lastVCross = checkVCross;
    }

    if (lastVCross != checkVCross) {
      crossCount++;
    }
  }

  /* 
   * 3) Post loop calculations
   */

  // Calculation of the root of the mean of the voltage squared (rms)
  // with calibration coefficients applied.
  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  // Calculation of the root of the mean of the current squared (rms)
  // with calibration coefficients applied.
  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / numberOfSamples);

  // Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor = realPower / apparentPower;

  // Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
}

int EnergyMonitor::defaultAnalogReader(int _pin) {
  return analogRead(_pin);
}
