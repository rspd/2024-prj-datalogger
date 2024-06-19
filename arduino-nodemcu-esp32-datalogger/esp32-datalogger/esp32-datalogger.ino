/**
 * Some important links:
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

// connect to wifi
static bool connectToWiFi();

// check wifi connection
static bool wifiConnectionLost();

// reconnect to wifi
static void reconnectWiFi();

// initializes google sheet
static void initGSheet();

// read sensor values
static void readSensor();

// function that gets current epoch time
static unsigned long getTime();

// append sensor values to google sheet
static void appendSensorValuesToGSheet(float temperature, float humidity);

// method to print the reason by which ESP32 has been awaken from sleep
static void print_wakeup_reason();

enum Constants
{
	ONE_SECOND_DELAY = 1000,
	TOKEN_REFRESH_SECONDS_TIMEOUT = 10 * 60,
	IDLE_SECONDS_BEFORE_NEXT_MEAS = 5,
	IDLE_MILIS_BEFORE_NEXT_MEAS = IDLE_SECONDS_BEFORE_NEXT_MEAS * 60 * 1000,
	RECONNECT_WIFI_TIMES_MAX = 10,
};

typedef enum
{
	STATE_POWER_ON = 0,
	STATE_INIT_SENSOR = 1,
	STATE_CONNECT_WIFI = 2,
	STATE_INIT_GSHEET = 3,
	STATE_IDLE = 4,
	STATE_READ_SENSOR = 5,
	STATE_APPEND_GSHEET = 6,
	STATE_NO_WIFI = 7,
	STATE_RECONNECT_WIFI = 8,
	STATE_ENTER_DEEP_SLEEP = 9,
	STATE_ERROR
} state_t;

// timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = IDLE_MILIS_BEFORE_NEXT_MEAS;

// counters
unsigned long gsheetOkCnt = 0;
unsigned long gsheetErrCnt = 0;
unsigned long wifiDisconnectionCnt = 0;
unsigned int reconnectWifiCnt = 0;
unsigned int tokenStatusCallbackOkCnt = 0;
unsigned int tokenStatusCallbackErrCnt = 0;

// sensor variables
float humidity = 0.0;
float temperature = 0.0;

// wifi disconnected
bool wifiDisconnected = true;

// ntp server to request epoch time
const char *ntpServer = "pool.ntp.org";

state_t state;

// initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

void setup()
{
	// sensor data pin needs a pullup
	pinMode(DHTPIN, INPUT_PULLUP);

	// starts serial interface
	Serial.begin(115200);
	Serial.println();
	Serial.println();

	// print the wakeup reason for ESP32
	print_wakeup_reason();

	// configure the wake up source and set time to sleep
	esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
	Serial.print(millis());
	Serial.println(" setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " seconds");

	// configure time
	configTime(0, 0, ntpServer);

	// next state
	state = STATE_POWER_ON;
}

void loop()
{
	const bool readSensorNow = (millis() - lastTime) > timerDelay;

	// call ready() repeatedly in loop for authentication checking and processing
	const bool ready = GSheet.ready();

	switch (state)
	{
	case STATE_POWER_ON:
		state = STATE_INIT_SENSOR;
		break;

	case STATE_INIT_SENSOR:
		// initialize DHT sensor
		dht.begin();
		while (1)
		{
			delay(ONE_SECOND_DELAY);
			if (dht.readTemperature() != NAN)
			{
				state = STATE_CONNECT_WIFI;
				break;
			}
			Serial.print(millis());
			Serial.println(" check DHT wiring?");
		};
		break;

	case STATE_CONNECT_WIFI:
		reconnectWifiCnt = 0;
		state = (connectToWiFi() == true) ? STATE_INIT_GSHEET : STATE_ERROR;
		break;

	case STATE_INIT_GSHEET:
		initGSheet();
		// 1st sensor read after power up / connect
		lastTime = millis();
		state = STATE_READ_SENSOR;
		break;

	case STATE_IDLE:
		if (readSensorNow)
		{
			lastTime = millis();
			state = STATE_READ_SENSOR;
		}
		break;

	case STATE_READ_SENSOR:
		readSensor();
		printStatus();
		state = STATE_APPEND_GSHEET;
		if (wifiConnectionLost())
		{
			if (!wifiDisconnected)
			{
				wifiDisconnected = true;
				++wifiDisconnectionCnt;
			}
			state = STATE_RECONNECT_WIFI;
		}
		break;

	case STATE_APPEND_GSHEET:
		if (!ready)
		{
			break;
		}
		appendSensorValuesToGSheet(temperature, humidity);

		//GSheet.refreshToken();

		state = STATE_ENTER_DEEP_SLEEP;
		break;

	case STATE_RECONNECT_WIFI:
		do
		{
			reconnectWiFi();
			++reconnectWifiCnt;
		} while ((WiFi.status() != WL_CONNECTED) && (reconnectWifiCnt < RECONNECT_WIFI_TIMES_MAX));

		state = (reconnectWifiCnt < RECONNECT_WIFI_TIMES_MAX) ? STATE_READ_SENSOR : STATE_ERROR;
		break;

	case STATE_ENTER_DEEP_SLEEP:
		Serial.print(millis());
		Serial.println(" going to sleep now");
		Serial.flush();
		esp_deep_sleep_start();
		break;

	case STATE_ERROR:
	default:
		Serial.print(millis());
		Serial.println(" error state");
		state = STATE_ENTER_DEEP_SLEEP;
		break;
	}
}

static bool connectToWiFi()
{
	// connect to wifi
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true); // does not reconnect if modem/router turns wifi off for a couple of hours and then turns it on again
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

	int cnt = 0;
	do
	{
		Serial.print(millis());
		Serial.println(" connecting to wifi...");
		delay(ONE_SECOND_DELAY);
		++cnt;
		if (cnt == 10)
		{
			return false;
		}

		wifiDisconnected = WiFi.status() != WL_CONNECTED;

	} while (wifiDisconnected);

	// ESP32's IP address
	Serial.println();
	Serial.print(millis());
	Serial.print(" connected with IP: ");
	Serial.println(WiFi.localIP());
	Serial.println();
	return true;
}

static bool wifiConnectionLost()
{
	if (WiFi.status() != WL_CONNECTED)
	{
		Serial.print(millis());
		Serial.println(" no WiFi connection!");
		return true;
	}
	return false;
}

static void reconnectWiFi()
{
	Serial.print(millis());
	Serial.println(" reconnecting to WiFi...");
	WiFi.disconnect();
	delay(ONE_SECOND_DELAY);
	WiFi.reconnect();
	delay(ONE_SECOND_DELAY);
}

static void initGSheet()
{
	GSheet.printf("ESP Google Sheet Client v%s\n\n", ESP_GOOGLE_SHEET_CLIENT_VERSION);

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
		++tokenStatusCallbackErrCnt;
		GSheet.printf("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
		GSheet.printf("Token error: %s\n", GSheet.getTokenError(info).c_str());
	}
	else
	{
		++tokenStatusCallbackOkCnt;
		GSheet.printf("Token info: type = %s, status = %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
	}
}

static void readSensor()
{
	Serial.println("\nReading sensor...");
	Serial.println("-----------------");

	// temperature or humidity read takes about 250ms!
	float h = dht.readHumidity();

	// read temperature as Celsius (default)
	float t = dht.readTemperature();

	// check if any read failed and exit early without update
	if (isnan(h) || isnan(t))
	{
		Serial.println(F("DHT sensor read failed!"));
		return;
	}

	// update humidity and temperature values
	humidity = h;
	temperature = t;

	// compute heat index in Celsius (isFahreheit = false)
	float hic = dht.computeHeatIndex(temperature, humidity, false);

	Serial.print(millis());
	Serial.print(F(" H: "));
	Serial.print(humidity);
	Serial.print(F("%  T: "));
	Serial.print(temperature);
	Serial.print(F("C  (heat index: "));
	Serial.print(hic);
	Serial.println(F("C)"));
}

static void printStatus()
{
	Serial.println("\nStatus counters...");
	Serial.println("------------------");

	Serial.print("Wifi disconnections: ");
	Serial.println(wifiDisconnectionCnt);

	Serial.print("GSheet counters: ");
	Serial.print(gsheetOkCnt);
	Serial.print("/");
	Serial.print(gsheetErrCnt);
	Serial.println(" [ok/err]");

	Serial.print("Token counters: ");
	Serial.print(tokenStatusCallbackOkCnt);
	Serial.print("/");
	Serial.print(tokenStatusCallbackErrCnt);
	Serial.println(" [ok/err]");
}

static unsigned long getTime()
{
	time_t now;
	struct tm timeinfo;
	if (!getLocalTime(&timeinfo))
	{
		// Serial.println("Failed to obtain time");
		return (0);
	}
	time(&now);
	return now;
}

static void appendSensorValuesToGSheet(float temperature, float humidity)
{
	FirebaseJson response;

	Serial.println("\nAppend spreadsheet values...");
	Serial.println("----------------------------");

	FirebaseJson valueRange;

	// Get timestamp
	const unsigned long epochTime = getTime();

	valueRange.add("majorDimension", "COLUMNS");
	valueRange.set("values/[0]/[0]", epochTime);
	valueRange.set("values/[1]/[0]", temperature);
	valueRange.set("values/[2]/[0]", humidity);

	// for google sheet API ref doc, go to https://developers.google.com/sheets/api/reference/rest/v4/spreadsheets.values/append
	// Append values to the spreadsheet
	const bool success = GSheet.values.append(&response /* returned response */, spreadsheetId /* spreadsheet Id to append */, "Sheet1!A2" /* range to append */, &valueRange /* data range to append */);
	if (success)
	{
		++gsheetOkCnt;
		response.toString(Serial, true);
		valueRange.clear();
	}
	else
	{
		++gsheetErrCnt;
		Serial.println(GSheet.errorReason());
	}
	Serial.println();
	Serial.println(ESP.getFreeHeap());
}

static void print_wakeup_reason()
{
	esp_sleep_wakeup_cause_t wakeup_reason;

	wakeup_reason = esp_sleep_get_wakeup_cause();

	Serial.print(millis());

	switch (wakeup_reason)
	{
	case ESP_SLEEP_WAKEUP_EXT0:
		Serial.println(" wakeup caused by external signal using RTC_IO");
		break;
	case ESP_SLEEP_WAKEUP_EXT1:
		Serial.println(" wakeup caused by external signal using RTC_CNTL");
		break;
	case ESP_SLEEP_WAKEUP_TIMER:
		Serial.println(" wakeup caused by timer");
		break;
	case ESP_SLEEP_WAKEUP_TOUCHPAD:
		Serial.println(" wakeup caused by touchpad");
		break;
	case ESP_SLEEP_WAKEUP_ULP:
		Serial.println(" wakeup caused by ULP program");
		break;
	default:
		Serial.printf(" wakeup was not caused by deep sleep: %d\n", wakeup_reason);
		break;
	}
}