//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++ sensor Lib for Sensirion SGP30 VOC sensor
// wraps Adafruit SGP30 lib
// 05/2020 Stephan, CC by-nc-sa
//- -----------------------------------------------------------------------------------------------------------------------

#ifndef __SENS_SGP30_h__
#define __SENS_SGP30_h__

#include <Sensors.h>
#include <Wire.h>
#include <Adafruit_SGP30.h>

namespace as {



class Sens_SGP30 : public Sensor {
  Adafruit_SGP30 _sgp30;
  uint16_t   _tvoc;
  uint16_t   _eco2;

public:
  Sens_SGP30 ():
    _tvoc(0),
    _eco2(0)
{
}

  void init () {
    _present = _sgp30.begin();
    DPRINT(F("SGP30 "));
    if (_present) {
      DPRINTLN(F("OK"));
      // SG: values sensor specimen dependent.
      // Inital calib 24/05/20 for testing, EEPROM storage and regular recalib could be implemented...
      if (_sgp30.setIAQBaseline(0x934A, 0x9606)) {
        DPRINTLN(F("SGP30 initial baseline value calib done"));
      }
    } else {
      DPRINTLN(F("ERROR"));
    }
  }

  bool measure (float temperature, float humidity) {
    if( present() == true ) {
      _sgp30.setHumidity(getAbsoluteHumidity(temperature, humidity));
      _sgp30.IAQmeasure();
      _tvoc = _sgp30.TVOC;
      _eco2 = _sgp30.eCO2;
      DPRINT("SGP30    TVOC   : ");
      DDECLN(_tvoc);
      DPRINT("SGP30    eCO2   : ");
      DDECLN(_eco2);
      return true;
    }
    return false;
  }
  uint16_t TVOC() { return _tvoc; }
  uint16_t eCO2() { return _eco2; }


private:
  uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
  }

};

}

#endif
