#include <BME280.h>

int main (void) {
  uint8_t mode;
  uint8_t ready;

  BME280 bme280;
  
  mode = bme280.getMode();  
  ready = bme280.isMeasuring();
  while (true) {
    std::cout << "mode:      " << bme280.modeVector[mode] << std::endl;
    std::cout << "ready:     " << (ready? "true":"false") << std::endl;

    if (ready == true) {
      bme280.readAllMeasurements();
    }

    std::cout << "tempC:     " << bme280.getTempC() << std::endl;
    std::cout << "tempF:     " << bme280.getTempF() << std::endl;
    std::cout << "pressure:  " << bme280.getPressure() << std::endl;
    std::cout << "AltMeters: " << bme280.getAltMeters() << std::endl;
    std::cout << "AltFeet:   " << bme280.getAltFeet() << std::endl;
    std::cout << "humidity:  " << bme280.getHumidity() << std::endl;
    std::cout << "dewPointC: " << bme280.getDewpointC() << std::endl;
    std::cout << "dewPointF: " << bme280.getDewpointF() << std::endl;
    std::cout << std::endl;
    
    usleep(500000);
  }
  return 0;
}