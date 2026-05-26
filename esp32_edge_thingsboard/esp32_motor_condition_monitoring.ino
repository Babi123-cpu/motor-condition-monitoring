#include <Batchimeg-project-1_inferencing.h>
	#include <Wire.h>
	#include <math.h>
	#include <WiFi.h>
	#include <PubSubClient.h>
	
	/* =========================================================
	Wi-Fi ба ThingsBoard тохиргоо
	========================================================= */
	
	const char* WIFI_SSID = "YOUR_WIFI_SSID";
	const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
	
	const char* TB_SERVER = "thingsboard.cloud";
	const int   TB_PORT   = 1883;
	const char* TB_TOKEN  = "YOUR_THINGSBOARD_DEVICE_TOKEN";
	
	WiFiClient espClient;
	PubSubClient mqttClient(espClient);
	
	/* =========================================================
	ADXL345 ба NTC тохиргоо
	========================================================= */
	
	#define ADXL_ADDR 0x53
	#define NTC_PIN 34
	
	const float R_FIXED  = 10000.0;
	const float ADC_VREF = 3.3;
	const float ADC_MAX  = 4095.0;
	const float BETA     = 3950.0;
	const float T0_K     = 298.15;
	const float R0       = 10000.0;
	
	static const bool debug_nn = false;
	
	float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
	
	float lastAccX = 0.0;
	float lastAccY = 0.0;
	float lastAccZ = 0.0;
	float lastTemp = 0.0;
	
	void connectWiFi() {
		if (WiFi.status() == WL_CONNECTED) {
			return;
		}
		
		WiFi.mode(WIFI_STA);
		WiFi.begin(WIFI_SSID, WIFI_PASS);
		
		while (WiFi.status() != WL_CONNECTED) {
			delay(500);
		}
	}
	
	void connectMQTT() {
		mqttClient.setServer(TB_SERVER, TB_PORT);
		
		while (!mqttClient.connected()) {
			mqttClient.connect("ESP32_Motor_Condition_Device", TB_TOKEN, NULL);
			delay(1000);
		}
	}
	
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
			if (Wire.available()) {
				buf[i] = Wire.read();
			}
		}
	}
	
	bool adxlInit() {
		Wire.beginTransmission(ADXL_ADDR);
		
		if (Wire.endTransmission() != 0) {
			return false;
		}
		
		adxlWrite(0x2D, 0x08);
		adxlWrite(0x31, 0x08);
		adxlWrite(0x2C, 0x0A);
		
		delay(100);
		return true;
	}
	
	void readADXL(float &accX, float &accY, float &accZ) {
		uint8_t buf[6];
		adxlReadBytes(0x32, 6, buf);
		
		int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
		int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
		int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);
		
		const float scale = 0.0039;
		
		accX = rawX * scale;
		accY = rawY * scale;
		accZ = rawZ * scale;
	}
	
	float readNTCTempC() {
		int raw = analogRead(NTC_PIN);
		
		float vout = (raw / ADC_MAX) * ADC_VREF;
		
		if (vout <= 0.0001) {
			return 0.0;
		}
		
		float rNTC = R_FIXED * (ADC_VREF / vout - 1.0);
		float invT = 1.0 / T0_K + (1.0 / BETA) * log(rNTC / R0);
		float tempK = 1.0 / invT;
		
		return tempK - 273.15;
	}
	
	int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
		memcpy(out_ptr, features + offset, length * sizeof(float));
		return 0;
	}
	
	void sendTelemetryToThingsBoard(
	String state,
	float normalProb,
	float faultProb,
	float temperature,
	float accX,
	float accY,
	float accZ
	) {
		connectWiFi();
		
		if (!mqttClient.connected()) {
			connectMQTT();
		}
		
		mqttClient.loop();
		
		int stateCode = 0;
		
		if (state == "fault") {
			stateCode = 1;
		}
		
		String payload = "{";
			payload += "\"state\":\"" + state + "\",";
			payload += "\"stateCode\":" + String(stateCode) + ",";
			payload += "\"normalProb\":" + String(normalProb * 100.0, 2) + ",";
			payload += "\"faultProb\":" + String(faultProb * 100.0, 2) + ",";
			payload += "\"temperature\":" + String(temperature, 2) + ",";
			payload += "\"accX\":" + String(accX, 3) + ",";
			payload += "\"accY\":" + String(accY, 3) + ",";
			payload += "\"accZ\":" + String(accZ, 3);
			payload += "}";
		
		mqttClient.publish("v1/devices/me/telemetry", payload.c_str());
	}
	
	void setup() {
		Serial.begin(115200);
		delay(1000);
		
		Wire.begin(21, 22);
		Wire.setClock(400000);
		
		analogReadResolution(12);
		
		if (!adxlInit()) {
			while (1) {
				delay(1000);
			}
		}
		
		connectWiFi();
		connectMQTT();
	}
	
	void loop() {
		delay(2000);
		
		for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
			int64_t next_tick = (int64_t)micros() + ((int64_t)EI_CLASSIFIER_INTERVAL_MS * 1000);
			
			float accX, accY, accZ;
			readADXL(accX, accY, accZ);
			
			float temp = readNTCTempC();
			
			features[ix + 0] = accX;
			features[ix + 1] = accY;
			features[ix + 2] = accZ;
			features[ix + 3] = temp;
			
			lastAccX = accX;
			lastAccY = accY;
			lastAccZ = accZ;
			lastTemp = temp;
			
			int64_t wait_time = next_tick - (int64_t)micros();
			
			if (wait_time > 0) {
				delayMicroseconds(wait_time);
			}
		}
		
		signal_t signal;
		signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
		signal.get_data = &raw_feature_get_data;
		
		ei_impulse_result_t result = { 0 };
		
		EI_IMPULSE_ERROR res = run_classifier(&signal, &result, debug_nn);
		
		if (res != EI_IMPULSE_OK) {
			return;
		}
		
		float max_value = 0.0;
		String predicted_class = "";
		
		float normalProb = 0.0;
		float faultProb = 0.0;
		
		for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
			String label = result.classification[i].label;
			float value = result.classification[i].value;
			
			if (label == "normal") {
				normalProb = value;
			}
			
			if (label == "fault") {
				faultProb = value;
			}
			
			if (value > max_value) {
				max_value = value;
				predicted_class = label;
			}
		}
		
		sendTelemetryToThingsBoard(
		predicted_class,
		normalProb,
		faultProb,
		lastTemp,
		lastAccX,
		lastAccY,
		lastAccZ
		);
	}
