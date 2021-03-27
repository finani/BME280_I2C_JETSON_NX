#include <BME280.h>

int main (void) {
  uint8_t mode;
  uint8_t ready;

  BME280 bme280;
  
  mode = bme280.getMode();
  ready = bme280.isMeasuring();
  while (mode == bme280.MODE_NORMAL) {
    printf("mode: %d! // [3] MODE_NORMAL\n", mode);
    printf("ready: %s!\n", ready? "true":"false");

    // float tempC;
    // tempC = bme280.readTempC();
    // printf("tempC: %f\n", tempC);

    // float pressure;
    // pressure = bme280.readFloatPressure();
    // printf("pressure: %f\n", pressure);

    // float AltMeters;
    // AltMeters = bme280.readFloatAltitudeMeters(pressure);
    // printf("AltMeters: %f\n", AltMeters);

    // float AltFeet;
    // AltFeet = bme280.readFloatAltitudeFeet(pressure);
    // printf("AltFeet: %f\n", AltFeet);

    // float humidity;
    // humidity = bme280.readFloatHumidity();
    // printf("humidity: %f\n", humidity);

    float dewPointC;
    dewPointC = bme280.dewPointC();
    printf("dewPointC: %f\n", dewPointC);

    float dewPointF;
    dewPointF = bme280.dewPointF();
    printf("dewPointF: %f\n", dewPointF);

    bme280.readAllMeasurements();

    printf("tempC: %f\n", bme280.getTempC());
    printf("tempF: %f\n", bme280.getTempF());
    printf("pressure: %f\n", bme280.getPressure());
    printf("AltMeters: %f\n", bme280.getAltMeters());
    printf("AltFeet: %f\n", bme280.getAltFeet());
    printf("humidity: %f\n", bme280.getHumidity());

    printf("\n");
    usleep(500000);
  }
  return 0;
}