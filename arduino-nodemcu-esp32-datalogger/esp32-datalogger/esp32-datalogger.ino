/**
 * ESP32 Datalogger
 *
 * Used library            Version   Pfad
 * WiFi                    2.0.0     C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.1\libraries\WiFi
 * Networking              1.0.0     C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.1\libraries\Network
 * DHT sensor library      1.4.6     C:\Users\rpede\Documents\Arduino\libraries\DHT_sensor_library
 * BME68x Sensor library   1.2.40408 C:\Users\rpede\Documents\Arduino\libraries\Bosch-BME68x-Library
 * Wire                    2.0.0     C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.1\libraries\Wire
 * SPI                     2.0.0     C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.1\libraries\SPI
 * ESP-Google-Sheet-Client 1.4.4     C:\Users\rpede\Documents\Arduino\libraries\ESP-Google-Sheet-Client
 * SPIFFS                  2.0.0     C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.1\libraries\SPIFFS
 * SD                      2.0.0     C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.1\libraries\SD
 * Adafruit Unified Sensor 1.1.14    C:\Users\rpede\Documents\Arduino\libraries\Adafruit_Unified_Sensor
 * FS                      2.0.0     C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.1\libraries\FS
 *
 * Supported Board Packages:
 * Arduino ESP32 Boards    "board": "esp32:esp32:esp32s3" (also "esp32:esp32:nodemcu-32s")
 *
 * Enter Boot mode:
 * NodeMCU-32S: press the button on the board before flashing begins in vs code.
 * Nano ESP32: 'Reset the Arduino bootloader on the Nano ESP32' to enter Boot mode.
 *
 * Some important links:
 * ESP32 Arduino Core’s documentation
 * - https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/index.html
 * ESP32 Datalogging to Google Sheets (using Google Service Account)
 * - https://RandomNerdTutorials.com/esp32-datalogging-google-sheets/
 * Arduino Google Sheet Client Library for Arduino devices
 * - https://github.com/mobizt/ESP-Google-Sheet-Client
 * Service accounts overview
 * - https://cloud.google.com/iam/docs/service-account-overview
 * Google Sheets API Overview
 * - https://developers.google.com/sheets/api/guides/concepts
 * Best practices for using service accounts
 * - https://cloud.google.com/iam/docs/best-practices-service-accounts
 * Manage Service Accounts
 * - https://cloud.google.com/iam/docs/best-practices-service-accounts#manage-service-accounts
 * Deep sleep: Use RTC memory to store data (ESP32 + Arduino series)
 * - https://www.youtube.com/watch?v=ij-hjzv6QKY
 * Fix Waveshare ESP32-S3-Zero/Nologo ESP32C3SuperMini Serial.print() not work - enable USB CDC on Boot
 * - https://www.youtube.com/watch?v=GfVfOp_5rQE
 * ESP32-S3 Flash
 * - https://www.youtube.com/watch?v=Zk8D_r_44iA
 * How to read the integrated temperature sensor in the ESP32
 * - https://www.luisllamas.es/en/esp32-built-in-temperature-sensor/
 * Reset the Arduino bootloader on the Nano ESP32
 * - https://support.arduino.cc/hc/en-us/articles/9810414060188-Reset-the-Arduino-bootloader-on-the-Nano-ESP32
 * Measuring ESP32 capabilities with Inbuilt Temperature Sensor
 * - https://www.espboards.dev/blog/esp32-inbuilt-temperature-sensor/
 * Arduino Nano ESP32
 * - https://dronebotworkshop.com/nano-esp32/
 * Arduino Nano ESP32 Cheat Sheet
 * - https://docs.arduino.cc/tutorials/nano-esp32/cheat-sheet/
 * ESP32-S3-Nano
 * - https://www.waveshare.com/wiki/ESP32-S3-Nano
 * ESP32 Change CPU Speed (Clock Frequency)
 * - https://deepbluembedded.com/esp32-change-cpu-speed-clock-frequency/
 * Cautions In Using ESP32 ADC - Makerfabs
 * - https://www.makerfabs.com/blog/post/cautions-in-using-esp32-adc-makerfabs-2
 * ESP32 SPI Communication: Set Pins, Multiple SPI Bus Interfaces, and Peripherals (Arduino IDE)
 * - https://randomnerdtutorials.com/esp32-spi-communication-arduino/
 * ESP32 Pinout Reference: Which GPIO pins should you use?
 * - https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
 */

#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <bme68xLibrary.h>
#include "time.h"
#include "defines.h" // see defines.h.template
#include "config.h"
#include <ESP_Google_Sheet_Client.h>


// for SD/SD_MMC mounting helper
#include <GS_SDHelper.h>

/// @brief Initialize DHT sensor before reading
static void	initDhtSensor();

/// @brief Initialize BME680 sensor before reading
static void	initBmeSensor();

/// @brief Read external battery voltage
static void readBatteryVoltage();

/// @brief Check wifi status
/// @return true if wifi connected, false othwerwise
static bool wifiConnected();

/// @brief Get epoch time,  it could take up-to five seconds because of getLocalTime()
/// @return seconds since epoch time for system start
static unsigned long getStartEpochTime();

/// @brief Function that gets current epoch time
/// @return seconds since epoch time
static unsigned long getTime();

/// @brief Connect to wifi
/// @return true on connected to wifi, false othwerwise
static bool connectToWiFi();

/// @brief Initializes google sheet
static void initGSheet();

/// @brief Read DHT sensor values
static void readDhtSensor();

/// @brief Start measurement in BME680
static void startBmeSensor();

/// @brief  Read BME680 sensor values
/// @return true if values were read or timed out, false otherwise
static bool readBmeSensor();

/// @brief Append sensor values to google sheet
/// @return true if all measurements were appended to Gsheet, false if pending measurements still in buffer
static bool appendSensorValuesToGSheet();

/// @brief Stores sensor values in buffer when no wifi connection
static void storeData();

/// @brief Entering esp32 in deep sleep
static void enterDeepSleepNow();

/// @brief Method to print the reason by which esp32 has been awaken from sleep
static void printWakeupReason();

/// @brief Get timestamp
/// @return timestamp with number of seconds and microseconds since the Epoch
static int64_t getTimestamp();

enum Constants
{
	MAX_GSHEET_RETRY_COUNT = 3UL,
	MAX_RETRY_COUNT = 10UL,
	SHORT_MILLIS_DELAY = 10UL,
	HALF_SECOND_DELAY = 500UL,
	ONE_SECOND_DELAY = 1000UL,
	WORK_MILLIS_TIMEOUT = 15000UL,
};

typedef enum
{
	STATE_POWER_ON = 0,
	STATE_INIT_SENSOR,
	STATE_CHECK_BATTVOLTAGE,
	STATE_READ_DHT_SENSOR,
	STATE_START_BME_SENSOR,
	STATE_READ_BME_SENSOR,
	STATE_CONNECT_WIFI,
	STATE_INIT_GSHEET,
	STATE_APPEND_GSHEET,
	STATE_STORE_DATA,
	STATE_ENTER_DEEP_SLEEP,
	STATE_IDLE,
	STATE_ERROR
} state_t;

/// R1=(75K+5,7K), R2=100K
/// measured values for R1 and R2
constexpr float	RESISTOR_R1 = 80.5f;
constexpr float	RESISTOR_R2 = 99.9f;

constexpr float CENTI_CONVERSION_FACTOR = 100.0f;

// timer variables
unsigned long epochTimeNow;
unsigned long utilTimeStamp;

// sensor variables
float dhtTemperature = 0.0;
float dhtHeatIndex = 0.0;
float dhtHumidity = 0.0;
float bmeTemperature = 0.0;
float bmePressure = 0.0;
float bmeHumidity = 0.0;
float bmeGasResistance = 0.0;
float esp32Temperature = 0.0;
int battMillivolts = 0;

// ntp server to request epoch time
const char *ntpServer = "pool.ntp.org";

// esp32-datalogger version
const char *VERSION = "1.0.6-beta";

// main machine state variable
state_t state;

// buffers used when no wifi connection (SRAM RTC SLOW MEMORY)
RTC_DATA_ATTR static int dhtTemperatureBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR static int dhtHeatIndexBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR static int dhtHumidityBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR static int bmeTemperatureBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR static int bmePressureBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR static int bmeHumidityBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR static int bmeGasResistanceBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR static int esp32TemperatureBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR static int battVoltsBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR static unsigned int esp32WorkTimeBuffer[BUFFER_MAX_SIZE] = {0};
RTC_DATA_ATTR unsigned char rdBufferIdx = 0;
RTC_DATA_ATTR unsigned char wrBufferIdx = 0; // index to write next value
RTC_DATA_ATTR unsigned char noWifiCycles = 0;

// DHT-22 sensor
DHT dht(DHTPIN, DHTTYPE);

// BME680 sensor
Bme68x bme680;

void setup()
{
	// turn all onboard LEDs off
	TURN_OFF_ALL_ESP32_LEDs();

	// DHT sensor data pin needs a pullup
	DHT_DATA_PULLUP();

	// starts serial interface
	DBGOUT_INIT();

	// some prints at startup
	DBGOUT_START();

	// check indexes consistency
	if (rdBufferIdx >= BUFFER_MAX_SIZE)
	{
		rdBufferIdx = rdBufferIdx % BUFFER_MAX_SIZE;
	}
	if (wrBufferIdx >= BUFFER_MAX_SIZE)
	{
		wrBufferIdx = wrBufferIdx % BUFFER_MAX_SIZE;
	}
	if (noWifiCycles)
	{
		noWifiCycles = (noWifiCycles - 1) % NO_WIFI_CYCLES_MAX;
	}

	//set the resolution to 12 bits (0-4095)
	analogReadResolution(12);

	// next state
	state = STATE_POWER_ON;
}

void loop()
{
	switch (state)
	{
		case STATE_POWER_ON:
			state = STATE_INIT_SENSOR;
			break;

		case STATE_INIT_SENSOR:
			initDhtSensor();
			initBmeSensor();
			state = STATE_CHECK_BATTVOLTAGE;
			break;

		case STATE_CHECK_BATTVOLTAGE:
			readBatteryVoltage();
			state = STATE_READ_DHT_SENSOR;
			break;

		case STATE_READ_DHT_SENSOR:
			readDhtSensor();
			state = STATE_START_BME_SENSOR;
			break;

		case STATE_START_BME_SENSOR:
			startBmeSensor();
			state = STATE_READ_BME_SENSOR;
			break;

		case STATE_READ_BME_SENSOR:
			if (readBmeSensor()) {
				state = (noWifiCycles) ? STATE_STORE_DATA : STATE_CONNECT_WIFI;
				break;
			}
			break;

		case STATE_CONNECT_WIFI:
			state = connectToWiFi() ? STATE_INIT_GSHEET : STATE_STORE_DATA;
			break;

		case STATE_INIT_GSHEET:
			initGSheet();
			state = STATE_APPEND_GSHEET;
			break;

		case STATE_APPEND_GSHEET:
			appendSensorValuesToGSheet();
			state = STATE_ENTER_DEEP_SLEEP;
			break;

		case STATE_STORE_DATA:
			storeData();
			state = STATE_ENTER_DEEP_SLEEP;
			break;

		case STATE_ENTER_DEEP_SLEEP:
			enterDeepSleepNow();
			break;

		case STATE_IDLE:
			// FFU/TBD
			break;

		case STATE_ERROR:
		default:
			DBGOUT_TS("Error state\n");
			state = STATE_ENTER_DEEP_SLEEP;
			break;
	}
}

static void	initDhtSensor()
{
	// initialize DHT sensor
	dht.begin();
}

static void	initBmeSensor()
{
	// initializes the sensor based on SPI library
	SPI.begin(BME_VSPI_SCK, BME_VSPI_MISO, BME_VSPI_MOSI, BME_VSPI_CS);
	bme680.begin(BME_VSPI_CS, SPI);

	const int8_t bme680Status = bme680.checkStatus();
	if(bme680Status)
	{
		if (bme680Status == BME68X_ERROR)
		{
			DBGOUT_TS("Sensor error: %s\n", bme680.statusString());
			return;
		}
		else if (bme680Status == BME68X_WARNING)
		{
			DBGOUT_TS("Sensor warning: %s\n", bme680.statusString());
		}
	}

	// set the default configuration for temperature, pressure and humidity
	bme680.setTPH();

	// set the heater configuration to 300 deg C for 100ms for Forced mode
	bme680.setHeaterProf(300, 100);
}

static void readBatteryVoltage()
{
	DBGOUT_TS("Battery voltage...\n");

	battMillivolts = 0;
	for (uint8_t i=0; i<MAX_RETRY_COUNT; i++)
	{
		// do not calculet here -> small values error propagation, due float operation!
		//battVolts += analogRead(ANALOG_BATT_GPIO) * ((RESISTOR_R1 + RESISTOR_R2)/RESISTOR_R2); // R1=(75K+5,7K), R2=100K
		battMillivolts += analogReadMilliVolts(ANALOG_BATT_GPIO);
	}
	/* #####################################
	 calculate averaged battery voltage
	     + <----
	            |
	           |R1|
	  VBATT     |-------> ANALOG_BATT_GPIO
	 (6V max)  |R2|         (3V3 max)
	            |
	     - <------------>     GND
	   ##################################### */
	battMillivolts = (battMillivolts/MAX_RETRY_COUNT);
	DBGOUT_TS("GPIO     %dmV, ", battMillivolts);
	battMillivolts = battMillivolts * ((RESISTOR_R1 + RESISTOR_R2)/RESISTOR_R2);
	DBGOUT("battery: %dmV\n", battMillivolts);
	DBGOUT_TS("------------------\n");
}

static bool wifiConnected()
{
	return (WiFi.status() == WL_CONNECTED);
}

static unsigned long getStartEpochTime()
{
	unsigned long epochTime = getTime();
	if (epochTime)
	{
		// get start timestamp
		epochTime -= (millis()/1000);
	}
	return epochTime;
}

static unsigned long getTime()
{
	time_t now;
	struct tm timeinfo;
	if (!getLocalTime(&timeinfo))
	{
		DBGOUT_TS("Failed to obtain time\n");
		return (0);
	}
	time(&now);
	return now;
}

static bool connectToWiFi()
{
	if (wifiConnected())
	{
		// get start timestamp
		epochTimeNow = getStartEpochTime();

		// just return if wifi connected
		// note: test purposes as on wakeup the wifi is always disconnected
		return true;
	}

	// connect to wifi
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true); // does not reconnect if modem/router turns wifi off for a couple of hours and then turns it on again
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

	DBGOUT_TS("Connecting to wifi...\n");
	int cnt = 0;
	do
	{
		delay(HALF_SECOND_DELAY);
		++cnt;
		if (cnt >= RECONNECT_WIFI_TIMES_MAX)
		{
			noWifiCycles = NO_WIFI_CYCLES_MAX;
			DBGOUT_TS("Failed to connect to wifi...\n");
			return false;
		}
	} while (!wifiConnected());

	// configure time
	configTime(0, 0, ntpServer);

	// get start timestamp
	epochTimeNow = getStartEpochTime();

	// esp32's IP address
	DBGOUT_TS("Connected with IP: %s\n", WiFi.localIP());
	return true;
}

static void initGSheet()
{
	DBGOUT_TS("Initialize 'ESP Google Sheet Client' v%s\n", ESP_GOOGLE_SHEET_CLIENT_VERSION);

	// set the callback for google api access token generation status (for debug only)
	GSheet.setTokenCallback(tokenStatusCallback);

	// set the seconds to refresh the auth token before expire (60 to 3540, default is 300 seconds)
	GSheet.setPrerefreshSeconds(TOKEN_REFRESH_SECONDS_TIMEOUT);

	// begin the access token generation for Google API authentication
	GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);
}

void tokenStatusCallback(TokenInfo info)
{
	if (info.status == token_status_error)
	{
		DBGOUT_TS("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
		DBGOUT_TS("Token error: %s\n", GSheet.getTokenError(info).c_str());
	}
	else
	{
		DBGOUT_TS("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
	}
}

static void readDhtSensor()
{
	float t;
	float hic;
	float h;
	float espT;
	unsigned char retryCnt = 0;

	DBGOUT_TS("Reading DHT sensor...\n");

	do {
		// temperature or humidity read takes about 250ms!
		h = dht.readHumidity();

		// read temperature as Celsius (default)
		t = dht.readTemperature();

		// compute heat index in Celsius (isFahreheit = false)
		hic = dht.computeHeatIndex(t, h, false);

		// read internal esp32 temperature sensor
		espT = temperatureRead();

		// check if any read failed and exit early without update
		if (isnan(h) || isnan(t) || isnan(espT) || isnan(hic))
		{
			++retryCnt;
			if (retryCnt > MAX_RETRY_COUNT)
			{
				DBGOUT_TS("DHT sensor read failed!\n");
			}
			delay(HALF_SECOND_DELAY);
		}
		else
		{
			// all sensors successfully read
			break;
		}
	} while (true);

	// update humidity and temperature values
	dhtTemperature = t;
	dhtHeatIndex = hic;
	dhtHumidity = h;
	esp32Temperature = espT;

	DBGOUT_TS("DHT22   H:%.2f%%, T:%.2fC, HI:%.2fC, Tesp32:%.2fC\n", dhtHumidity, dhtTemperature, dhtHeatIndex, esp32Temperature);
	DBGOUT_TS("------------------\n");
}

static void startBmeSensor()
{
	DBGOUT_TS("Starting BME sensor (0x%X)...\n", bme680.getOpMode());
	bme680.setOpMode(BME68X_FORCED_MODE);
	delayMicroseconds(bme680.getMeasDur());
}

static bool readBmeSensor()
{
	if (bme680.fetchData())
	{
		DBGOUT_TS("Reading BME sensor...\n");

		bme68xData data;
		bme680.getData(data);

		bmeTemperature = data.temperature;
		bmeHumidity = data.humidity;
		bmePressure = data.pressure / 10; // converted from pascal to hectopascal 0.1
		bmeGasResistance = data.gas_resistance;

		DBGOUT_TS("BME680  H:%.2f%%, T:%.2fC, P:%ihPa, G:%iohm, status:0x%X\n", bmeHumidity, bmeTemperature, (int) bmePressure, (int) bmeGasResistance, data.status);
		DBGOUT_TS("------------------\n");

		return true;
	}

	if ((millis() > WORK_MILLIS_TIMEOUT))
	{
		DBGOUT_TS("Millis timed out!\n");
		return true;
	}

	return false;
}

static bool appendSensorValuesToGSheet(void)
{
	FirebaseJson response;
	FirebaseJson valueRange;
	unsigned char gsheetErrCnt = 0;
	const unsigned char nrMeasurementSets = (wrBufferIdx - rdBufferIdx) % BUFFER_MAX_SIZE;

	DBGOUT_TS("Append spreadsheet values...\n");

	if (nrMeasurementSets)
	{
		DBGOUT_TS("There are %d set of measurements in buffer (%d,%d)\n", nrMeasurementSets, wrBufferIdx, rdBufferIdx);
	}

	do
	{
		if (!wifiConnected())
		{
			// no wifi, terminate here
			break;
		}

		if (millis() > WORK_MILLIS_TIMEOUT)
		{
			// timeout, terminate here
			DBGOUT_TS("Millis timed out!\n");
			break;
		}

		// call ready() repeatedly in loop for authentication checking and processing
		if (!GSheet.ready())
		{
			// until ready ...
			continue;
		}

		valueRange.add("majorDimension", "COLUMNS");
		if (rdBufferIdx != wrBufferIdx)
		{
			unsigned long pastEpochTime = epochTimeNow - ((wrBufferIdx - rdBufferIdx) % BUFFER_MAX_SIZE)*TIME_IN_DEEP_SLEEP;
			DBGOUT_TS("Get measurement (ts:%d) from buffer\n", pastEpochTime);

			// set values from buffer
			valueRange.set("values/[0]/[0]",  pastEpochTime);
			valueRange.set("values/[1]/[0]",  dhtTemperatureBuffer[rdBufferIdx]);
			valueRange.set("values/[2]/[0]",  dhtHumidityBuffer[rdBufferIdx]);
			valueRange.set("values/[3]/[0]",  battVoltsBuffer[rdBufferIdx]);
			valueRange.set("values/[4]/[0]",  esp32TemperatureBuffer[rdBufferIdx]);
			valueRange.set("values/[5]/[0]",  esp32WorkTimeBuffer[rdBufferIdx]);
			valueRange.set("values/[6]/[0]",  dhtHeatIndexBuffer[rdBufferIdx]);
			valueRange.set("values/[7]/[0]",  bmeTemperatureBuffer[rdBufferIdx]);
			valueRange.set("values/[8]/[0]",  bmeHumidityBuffer[rdBufferIdx]);
			valueRange.set("values/[9]/[0]",  bmePressureBuffer[rdBufferIdx]);
			valueRange.set("values/[10]/[0]", bmeGasResistanceBuffer[rdBufferIdx]);
		}
		else
		{
			// set last 'now' values
			valueRange.set("values/[0]/[0]", epochTimeNow);
			valueRange.set("values/[1]/[0]", (int) (dhtTemperature * CENTI_CONVERSION_FACTOR));
			valueRange.set("values/[2]/[0]", (int) (dhtHumidity * CENTI_CONVERSION_FACTOR));
			valueRange.set("values/[3]/[0]", (int) battMillivolts);
			valueRange.set("values/[4]/[0]", (int) (esp32Temperature * CENTI_CONVERSION_FACTOR));
			valueRange.set("values/[6]/[0]", (int) (dhtHeatIndex * CENTI_CONVERSION_FACTOR));
			valueRange.set("values/[7]/[0]", (int) (bmeTemperature * CENTI_CONVERSION_FACTOR));
			valueRange.set("values/[8]/[0]", (int) (bmeHumidity * CENTI_CONVERSION_FACTOR));
			valueRange.set("values/[9]/[0]", (int) bmePressure);
			valueRange.set("values/[10]/[0]", (int) bmeGasResistance);

			/* add 2ms, measured max. time from here until calling esp_deep_sleep_start() */
			valueRange.set("values/[5]/[0]", (unsigned int) millis() + 2);
		}

		// for google sheet API ref doc, go to https://developers.google.com/sheets/api/reference/rest/v4/spreadsheets.values/append
		// Append values to the spreadsheet
		const bool success = GSheet.values.append(&response /* returned response */, spreadsheetId /* spreadsheet Id to append */, "Sheet1!A2" /* range to append */, &valueRange /* data range to append */);

		if (success)
		{
			gsheetErrCnt = 0;
			response.toString(Serial, true);
			valueRange.clear();
			DBGOUT("\n");

			// buffer empty condition?
			if (rdBufferIdx == wrBufferIdx)
			{
				// then, we are done
				DBGOUT_TS("Append success!\n");
				break;
			}
			else
			{
				// increment read buffer index
				rdBufferIdx = (rdBufferIdx + 1) % BUFFER_MAX_SIZE;
			}
		}
		else
		{
			++gsheetErrCnt;
			DBGOUT_TS("Append error, reason: %s\n", GSheet.errorReason());
		}

	} while (gsheetErrCnt < MAX_GSHEET_RETRY_COUNT);

	DBGOUT_TS("Free heap %d\n", ESP.getFreeHeap());
	DBGOUT_TS("------------------\n");

	return (rdBufferIdx == wrBufferIdx);
}

static void storeData()
{
	DBGOUT_TS("Append measurement in buffer\n");
	dhtTemperatureBuffer[wrBufferIdx] = (int) (dhtTemperature * CENTI_CONVERSION_FACTOR);
	dhtHumidityBuffer[wrBufferIdx] = (int) (dhtHumidity * CENTI_CONVERSION_FACTOR);
	battVoltsBuffer[wrBufferIdx] = (int) battMillivolts;
	esp32TemperatureBuffer[wrBufferIdx] = (int) (esp32Temperature * CENTI_CONVERSION_FACTOR);
	dhtHeatIndexBuffer[wrBufferIdx] = (int) (dhtHeatIndex * CENTI_CONVERSION_FACTOR);
	bmeTemperatureBuffer[wrBufferIdx] = (int) (bmeTemperature * CENTI_CONVERSION_FACTOR);
	bmeHumidityBuffer[wrBufferIdx] = (int) (bmeHumidity * CENTI_CONVERSION_FACTOR);
	bmePressureBuffer[wrBufferIdx] = (int) bmePressure;
	bmeGasResistanceBuffer[wrBufferIdx] = (int) bmeGasResistance;

	esp32WorkTimeBuffer[wrBufferIdx] = (unsigned int) millis();
	wrBufferIdx = (wrBufferIdx + 1) % BUFFER_MAX_SIZE;
}

static void enterDeepSleepNow()
{
	#ifndef DEBUG
		DBGOUT_TS("ESP32 will wake up again in about %sms\n", String(TIME_IN_DEEP_SLEEP*1000UL - millis()));
		DBGFLUSH();

		// save working time, configure the wake up source and set time to sleep
		const unsigned long elapsedMillis = millis();
		const uint64_t elapsedMicroseconds = elapsedMillis * 1000ULL;

		esp_sleep_enable_timer_wakeup(MICROSECS_IN_DEEP_SLEEP - elapsedMicroseconds);
		esp_deep_sleep_start();
	#else
		// only debug purposes
		DBGOUT_TS("ESP32 will wake up again in about thirty seconds\n");
		DBGOUT_TS("------------------\n");
		delay(30000);
		epochTimeNow = getTime();
		state = STATE_CHECK_BATTVOLTAGE;
	#endif
}

static void printWakeupReason()
{
	esp_sleep_wakeup_cause_t wakeup_reason;

	wakeup_reason = esp_sleep_get_wakeup_cause();

	switch (wakeup_reason)
	{
	case ESP_SLEEP_WAKEUP_EXT0:
		DBGOUT_TS("Wakeup caused by external signal using RTC_IO\n");
		break;
	case ESP_SLEEP_WAKEUP_EXT1:
		DBGOUT_TS("Wakeup caused by external signal using RTC_CNTL\n");
		break;
	case ESP_SLEEP_WAKEUP_TIMER:
		DBGOUT_TS("Wakeup caused by timer\n");
		break;
	case ESP_SLEEP_WAKEUP_TOUCHPAD:
		DBGOUT_TS("Wakeup caused by touchpad\n");
		break;
	case ESP_SLEEP_WAKEUP_ULP:
		DBGOUT_TS("Wakeup caused by ULP program\n");
		break;
	default:
		DBGOUT_TS("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
		break;
	}
	DBGOUT_TS("------------------\n");
}

static int64_t getTimestamp()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}