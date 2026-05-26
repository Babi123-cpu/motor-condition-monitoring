#include <Wire.h>
	#include <math.h>
	
	#define ADXL_ADDR 0x53
	#define NTC_PIN 34
	
	const float R_FIXED  = 10000.0;
	const float ADC_VREF = 3.3;
	const float ADC_MAX  = 4095.0;
	const float BETA     = 3950.0;
	const float T0_K     = 298.15;
	const float R0       = 10000.0;
	
	void adxlWrite(uint8_t reg, uint8_t val) {
		Wire.beginTransmission(ADXL_ADDR);
		Wire.write(reg);
		Wire.write(val);
		Wire.endTransmission();
	}
	
	void adxlReadBytes(uint8_t reg, uint8_t count, uint8_t *buf) {
		Wire.beginTransmission(ADXL_ADDR);
		Wire.write(reg);
		Wire.endTransmission(false);
		Wire.requestFrom(ADXL_ADDR, count);
		
		for (uint8_t i = 0; i < count; i++) {
			if (Wire.available()) buf[i] = Wire.read();
		}
	}
	
	void adxlInit() {
		adxlWrite(0x2D, 0x08); // POWER_CTL
		adxlWrite(0x31, 0x08); // DATA_FORMAT +/-2g
	}
	
	void readADXL(float &ax, float &ay, float &az) {
		uint8_t buf[6];
		adxlReadBytes(0x32, 6, buf);
		
		int16_t rx = (int16_t)((buf[1] << 8) | buf[0]);
		int16_t ry = (int16_t)((buf[3] << 8) | buf[2]);
		int16_t rz = (int16_t)((buf[5] << 8) | buf[4]);
		
		const float scale = 0.0039; // g/LSB
		ax = rx * scale;
		ay = ry * scale;
		az = rz * scale;
	}
	
	float readNTCTempC() {
		int raw = analogRead(NTC_PIN);
		
		float vout = (raw / ADC_MAX) * ADC_VREF;
		if (vout <= 0.0001) return NAN;
		
		float rNTC = R_FIXED * (ADC_VREF / vout - 1.0);
		float invT = 1.0 / T0_K + (1.0 / BETA) * log(rNTC / R0);
		float tempK = 1.0 / invT;
		
		return tempK - 273.15;
	}
	
	const unsigned long SAMPLE_INTERVAL_MS = 10;
	unsigned long lastMillis = 0;
	
	void setup() {
		Serial.begin(115200);
		Wire.begin();
		analogReadResolution(12);
		
		adxlInit();
		delay(100);
		
		Serial.println("ax,ay,az,tempC");
	}
	
	void loop() {
		unsigned long now = millis();
		
		if (now - lastMillis >= SAMPLE_INTERVAL_MS) {
			lastMillis = now;
			
			float ax, ay, az;
			readADXL(ax, ay, az);
			float tempC = readNTCTempC();
			
			Serial.print(ax, 6); Serial.print(",");
			Serial.print(ay, 6); Serial.print(",");
			Serial.print(az, 6); Serial.print(",");
			Serial.println(tempC, 3);
		}
	}
