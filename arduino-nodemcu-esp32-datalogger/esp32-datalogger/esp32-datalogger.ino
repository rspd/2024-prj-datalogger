/**
 * ESP32 Datalogger
 *
 * Used library            Version Pfad
 * WiFi                    2.0.0   C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17\libraries\WiFi
 * DHT sensor library      1.4.6   C:\Users\rpede\Documents\Arduino\libraries\DHT_sensor_library
 * ESP-Google-Sheet-Client 1.4.4   C:\Users\rpede\Documents\Arduino\libraries\ESP-Google-Sheet-Client
 * SPIFFS                  2.0.0   C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17\libraries\SPIFFS
 * SD                      2.0.0   C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17\libraries\SD
 * SPI                     2.0.0   C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17\libraries\SPI
 * Adafruit Unified Sensor 1.1.14  C:\Users\rpede\Documents\Arduino\libraries\Adafruit_Unified_Sensor
 * FS                      2.0.0   C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17\libraries\FS
 *
 * Used platform Version Pfad
 * esp32:esp32   2.0.17  C:\Users\rpede\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17
 *
 * Board Packages Used:
 * Arduino ESP32 Boards    "board": "esp32:esp32:esp32s3" (also "esp32:esp32:nodemcu-32s")
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
 * ESP32-S3-Nano
 * - https://www.waveshare.com/wiki/ESP32-S3-Nano
 */

#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include "time.h"
#include "defines.h" // see defines.h.template
#include "config.h"
#include <ESP_Google_Sheet_Client.h>

// for SD/SD_MMC mounting helper
#include <GS_SDHelper.h>

// initialize sensor before reading
static void	initSensor();

// read external battery voltage
static void readBatteryVoltage();

// check wifi status
static bool checkWiFi();

// connect to wifi
static bool connectToWiFi();

// initializes google sheet
static void initGSheet();

// read sensor values
static void readSensor();

// function that gets current epoch time
static unsigned long getTime();

// append sensor values to google sheet
static bool appendSensorValuesToGSheet();

// stores sensor values in buffer when no wifi connection
static void storeData();

// entering esp32 in deep sleep
static void enterDeepSleepNow();

// method to print the reason by which esp32 has been awaken from sleep
static void printWakeupReason();

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
	STATE_READ_SENSOR,
	STATE_CONNECT_WIFI,
	STATE_INIT_GSHEET,
	STATE_APPEND_GSHEET,
	STATE_STORE_DATA,
	STATE_ENTER_DEEP_SLEEP,
	STATE_IDLE,
	STATE_ERROR
} state_t;

// R1=(75K+5,7K), R2=100K
// measured values for R1 and R2
constexpr float	RESISTOR_R1 = 80.5f;
constexpr float	RESISTOR_R2 = 99.9f;

constexpr float CENTI_CONVERSION_FACTOR = 100.0f;

// timer variables
unsigned long epochTimeNow;
unsigned long utilTimeStamp;

// sensor variables
float temperature = 0.0;
float humidity = 0.0;
float esp32Temperature = 0.0;
int battMillivolts = 0;

// ntp server to request epoch time
const char *ntpServer = "pool.ntp.org";

// esp32-datalogger version
const char *VERSION = "1.0.4-beta";

// main machine state variable
state_t state;

// buffers used when no wifi connection (SRAM RTC SLOW MEMORY)
RTC_DATA_ATTR static int temperatureBuffer[BUFFER_MAX_SIZE];
RTC_DATA_ATTR static int humidityBuffer[BUFFER_MAX_SIZE];
RTC_DATA_ATTR static int esp32TemperatureBuffer[BUFFER_MAX_SIZE];
RTC_DATA_ATTR static int battVoltsBuffer[BUFFER_MAX_SIZE];
RTC_DATA_ATTR static unsigned int esp32WorkTimeBuffer[BUFFER_MAX_SIZE];
RTC_DATA_ATTR unsigned char rdBufferIdx = 0;
RTC_DATA_ATTR unsigned char wrBufferIdx = 0; // index to write next value

// initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

void setup()
{
	// turn all onboard LEDs off
	TURN_OFF_ALL_ESP32_LEDs();

	// Get timestamp
	// note: first time shall fail returning zero
	// and shall be set again after wifi connection
	epochTimeNow = getTime();

	// sensor data pin needs a pullup
	DHT_DATA_PULLUP();

	// starts serial interface
	DBGOUT_INIT();

	// print the wakeup reason for ESP32
	printWakeupReason();

	// check indexes consistency
	if (rdBufferIdx >= BUFFER_MAX_SIZE)
	{
		rdBufferIdx = rdBufferIdx % BUFFER_MAX_SIZE;
	}
	if (wrBufferIdx >= BUFFER_MAX_SIZE)
	{
		wrBufferIdx = wrBufferIdx % BUFFER_MAX_SIZE;
	}

	// next state
	state = STATE_POWER_ON;
}

void loop()
{
	// call ready() repeatedly in loop for authentication checking and processing
	const bool ready = GSheet.ready();

	switch (state)
	{
		case STATE_POWER_ON:
			state = STATE_INIT_SENSOR;
			break;

		case STATE_INIT_SENSOR:
			initSensor();
			state = STATE_CHECK_BATTVOLTAGE;
			break;

		case STATE_CHECK_BATTVOLTAGE:
			readBatteryVoltage();
			state = STATE_READ_SENSOR;
			break;

		case STATE_READ_SENSOR:
			readSensor();
			state = STATE_CONNECT_WIFI;
			break;

		case STATE_CONNECT_WIFI:
			state = connectToWiFi() ? STATE_INIT_GSHEET : STATE_STORE_DATA;
			break;

		case STATE_INIT_GSHEET:
			initGSheet();
			state = STATE_APPEND_GSHEET;
			break;

		case STATE_APPEND_GSHEET:
			if (!ready)
			{
				if (millis() > WORK_MILLIS_TIMEOUT)
				{
					state = STATE_STORE_DATA;
				}
				break;
			}
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
			DBGOUT(Serial.print, millis());
			DBGOUT(Serial.println, " error state");
			state = STATE_ENTER_DEEP_SLEEP;
			break;
	}
}

static void	initSensor()
{
	// initialize DHT sensor
	dht.begin();
	while (1)
	{
		delay(HALF_SECOND_DELAY);
		if (dht.readTemperature() != NAN)
		{
			break;
		}
		DBGOUT(Serial.print, millis());
		DBGOUT(Serial.println, " check DHT wiring?");
	};
}

static void readBatteryVoltage()
{
	battMillivolts = 0;
	for (uint8_t i=0; i<MAX_RETRY_COUNT; i++)
	{
		// do not calculet here -> small values error propagation, due float operation!
		//battVolts += analogRead(ANALOG_BATT_GPIO) * ((RESISTOR_R1 + RESISTOR_R2)/RESISTOR_R2); // R1=(75K+5,7K), R2=100K
		battMillivolts += analogReadMilliVolts(ANALOG_BATT_GPIO);
	}
	/*
	 calculate averaged battery voltage
	     + <----
	            |
	           |R1|
	  VBATT     |-------> ANALOG_BATT_GPIO
	 (6V max)  |R2|         (3V3 max)
	            |
	     - <------------>     GND
	*/
	battMillivolts = (battMillivolts/MAX_RETRY_COUNT) * ((RESISTOR_R1 + RESISTOR_R2)/RESISTOR_R2);
	DBGOUT(Serial.print, millis());
	DBGOUT(Serial.print, " battery ");
	DBGOUT(Serial.print, battMillivolts);
	DBGOUT(Serial.println, "V");
}

static bool checkWiFi()
{
	return (WiFi.status() == WL_CONNECTED);
}

static bool connectToWiFi()
{
	if (checkWiFi())
	{
		// just return if wifi connected
		// note: test purposes as on wakeup the wifi is always disconnected
		return true;
	}

	// connect to wifi
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true); // does not reconnect if modem/router turns wifi off for a couple of hours and then turns it on again
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

	DBGOUT(Serial.print, millis());
	DBGOUT(Serial.println, " connecting to wifi...");
	int cnt = 0;
	do
	{
		delay(HALF_SECOND_DELAY);
		++cnt;
		if (cnt >= RECONNECT_WIFI_TIMES_MAX)
		{
			DBGOUT(Serial.print, millis());
			DBGOUT(Serial.println, " failed to connect to wifi...");
			return false;
		}
	} while (!checkWiFi());

	// configure time
	configTime(0, 0, ntpServer);

	if (epochTimeNow == 0)
	{
		delay(SHORT_MILLIS_DELAY);
		epochTimeNow = getTime();
	}

	// esp32's IP address
	DBGOUT(Serial.print, millis());
	DBGOUT(Serial.print, " connected with IP: ");
	DBGOUT(Serial.println, WiFi.localIP());
	DBGOUT(Serial.println, "");
	return true;
}

static void initGSheet()
{
	DBGOUT(GSheet.printf, "ESP Google Sheet Client v%s\n\n", ESP_GOOGLE_SHEET_CLIENT_VERSION);

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
		DBGOUT(GSheet.printf, "Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
		DBGOUT(GSheet.printf, "Token error: %s\n", GSheet.getTokenError(info).c_str());
	}
	else
	{
		DBGOUT(GSheet.printf, "Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
	}
}

static void readSensor()
{
	float h;
	float t;
	float espT;
	unsigned char retryCnt = 0;

	DBGOUT(Serial.println, "\nReading sensor...");
	DBGOUT(Serial.println, "-----------------");

	do {
		bool ok = true;

		// temperature or humidity read takes about 250ms!
		h = dht.readHumidity();

		// read temperature as Celsius (default)
		t = dht.readTemperature();

		// read internal esp32 temperature sensor
		espT = temperatureRead();

		// check if any read failed and exit early without update
		if (isnan(h) || isnan(t) || isnan(espT))
		{
			++retryCnt;
			if (retryCnt > MAX_RETRY_COUNT)
			{
				DBGOUT(Serial.println, F("DHT sensor read failed!"));
			}
			delay(HALF_SECOND_DELAY);
		}
		else
		{
			break;
		}
	} while (true);

	// update humidity and temperature values
	humidity = h;
	temperature = t;
	esp32Temperature = espT;

	// compute heat index in Celsius (isFahreheit = false)
	float hic = dht.computeHeatIndex(temperature, humidity, false);

	DBGOUT(Serial.print, millis());
	DBGOUT(Serial.print, F(" H: "));
	DBGOUT(Serial.print, humidity);
	DBGOUT(Serial.print, F("%  T: "));
	DBGOUT(Serial.print, temperature);
	DBGOUT(Serial.print, F("C  (heat index: "));
	DBGOUT(Serial.print, hic);
	DBGOUT(Serial.println, F("C)"));
}

static unsigned long getTime()
{
	time_t now;
	struct tm timeinfo;
	if (!getLocalTime(&timeinfo))
	{
		DBGOUT(Serial.println, "Failed to obtain time");
		return (0);
	}
	time(&now);
	return now;
}

static bool appendSensorValuesToGSheet(void)
{
	FirebaseJson response;
	FirebaseJson valueRange;
	unsigned char gsheetErrCnt = 0;
	const unsigned char nrMeasurementSets = (wrBufferIdx - rdBufferIdx) % BUFFER_MAX_SIZE;

	DBGOUT(Serial.println, "\nAppend spreadsheet values...");
	DBGOUT(Serial.println, "----------------------------");

	if (nrMeasurementSets)
	{
		DBGOUT(Serial.print, "There are ");
		DBGOUT(Serial.print, nrMeasurementSets);
		DBGOUT(Serial.print, " set of measurements in buffer (");
		DBGOUT(Serial.print, wrBufferIdx);
		DBGOUT(Serial.print, ",");
		DBGOUT(Serial.print, rdBufferIdx);
		DBGOUT(Serial.println, ")");
	}

	do
	{
		valueRange.add("majorDimension", "COLUMNS");
		if (rdBufferIdx != wrBufferIdx)
		{
			unsigned long pastEpochTime = epochTimeNow - ((wrBufferIdx - rdBufferIdx) % BUFFER_MAX_SIZE)*TIME_IN_DEEP_SLEEP;
			DBGOUT(Serial.print, "Get measurement (ts:");
			DBGOUT(Serial.print, pastEpochTime);
			DBGOUT(Serial.println, ") from buffer");

			// set values from buffer
			valueRange.set("values/[0]/[0]", pastEpochTime);
			valueRange.set("values/[1]/[0]", temperatureBuffer[rdBufferIdx]);
			valueRange.set("values/[2]/[0]", humidityBuffer[rdBufferIdx]);
			valueRange.set("values/[3]/[0]", battVoltsBuffer[rdBufferIdx]);
			valueRange.set("values/[4]/[0]", esp32TemperatureBuffer[rdBufferIdx]);
			valueRange.set("values/[5]/[0]", esp32WorkTimeBuffer[rdBufferIdx]);
		}
		else
		{
			// set last 'now' values
			valueRange.set("values/[0]/[0]", epochTimeNow);
			valueRange.set("values/[1]/[0]", (int) (temperature * CENTI_CONVERSION_FACTOR));
			valueRange.set("values/[2]/[0]", (int) (humidity * CENTI_CONVERSION_FACTOR));
			valueRange.set("values/[3]/[0]", (int) battMillivolts);
			valueRange.set("values/[4]/[0]", (int) (esp32Temperature * CENTI_CONVERSION_FACTOR));
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

			// buffer empty condition?
			if (rdBufferIdx == wrBufferIdx)
			{
				// then, we are done
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
			DBGOUT(Serial.println, GSheet.errorReason());
		}
	} while (gsheetErrCnt < MAX_GSHEET_RETRY_COUNT);

	DBGOUT(Serial.println, "");
	DBGOUT(Serial.println, ESP.getFreeHeap());

	return (rdBufferIdx == wrBufferIdx);
}

static void storeData()
{
	DBGOUT(Serial.println, "Append measurement in buffer");
	temperatureBuffer[wrBufferIdx] = (int) (temperature * CENTI_CONVERSION_FACTOR);
	humidityBuffer[wrBufferIdx] = (int) (humidity * CENTI_CONVERSION_FACTOR);
	battVoltsBuffer[wrBufferIdx] = (int) battMillivolts;
	esp32TemperatureBuffer[wrBufferIdx] = (int) (esp32Temperature * CENTI_CONVERSION_FACTOR);
	esp32WorkTimeBuffer[wrBufferIdx] = (unsigned int) millis();
	wrBufferIdx = (wrBufferIdx + 1) % BUFFER_MAX_SIZE;
}

static void enterDeepSleepNow()
{
	DBGOUT(Serial.print, millis());
	DBGOUT(Serial.print, " ESP32 will wake up again in about ");
	DBGOUT(Serial.print, String(TIME_IN_DEEP_SLEEP*1000UL - millis()));
	DBGOUT(Serial.println, "ms");
	DBGFLUSH();

	// save working time, configure the wake up source and set time to sleep
	const unsigned long elapsedMillis = millis();
	const uint64_t elapsedMicroseconds = elapsedMillis * 1000ULL;
	#ifndef DEBUG
		esp_sleep_enable_timer_wakeup(MICROSECS_IN_DEEP_SLEEP - elapsedMicroseconds);
		esp_deep_sleep_start();
	#else
		// only debug purposes
		delay(10000);
		epochTimeNow = getTime();
		state = STATE_CHECK_BATTVOLTAGE;
	#endif
}

static void printWakeupReason()
{
	esp_sleep_wakeup_cause_t wakeup_reason;

	wakeup_reason = esp_sleep_get_wakeup_cause();

	DBGOUT(Serial.print, millis());

	switch (wakeup_reason)
	{
	case ESP_SLEEP_WAKEUP_EXT0:
		DBGOUT(Serial.println, " wakeup caused by external signal using RTC_IO");
		break;
	case ESP_SLEEP_WAKEUP_EXT1:
		DBGOUT(Serial.println, " wakeup caused by external signal using RTC_CNTL");
		break;
	case ESP_SLEEP_WAKEUP_TIMER:
		DBGOUT(Serial.println, " wakeup caused by timer");
		break;
	case ESP_SLEEP_WAKEUP_TOUCHPAD:
		DBGOUT(Serial.println, " wakeup caused by touchpad");
		break;
	case ESP_SLEEP_WAKEUP_ULP:
		DBGOUT(Serial.println, " wakeup caused by ULP program");
		break;
	default:
		DBGOUT(Serial.printf, " wakeup was not caused by deep sleep: %d\n", wakeup_reason);
		break;
	}
}