bme280_test :
	g++ -o bme280_test bme280_test.cpp BME280.cpp -I./include

clean :
	rm bme280_test