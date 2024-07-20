#ifndef CONFIG_H
#define CONFIG_H

///////////////////////////////////////////////////////////////////////////////
/// Debug purposes

#if 0
#define DBGOUT_INIT() do { \
	Serial.begin(115200);  \
	Serial.println();      \
	Serial.println();      \
} while (0)
#define DBGOUT(function, ...)  (function)(__VA_ARGS__)
#define DBGFLUSH() Serial.flush()
#else
#define DBGOUT_INIT()
#define DBGOUT(function, ...)
#define DBGFLUSH()
#endif

///////////////////////////////////////////////////////////////////////////////
/// ESP32 communication

// maximum of measurement sets that can be stored
// when no wifi connection for a time equal to
//   BUFFER_MAX_SIZE * TIME_IN_DEEP_SLEEP
// e.g 128 x 5 minutes = 640 minutes = 10 hours 40 minutes
// when wifi disconnection exceeds this time old measurement
// values are overwritten storing always the last measurement
// sets for the las 640 minutes
#define BUFFER_MAX_SIZE 128UL

#define	TOKEN_REFRESH_SECONDS_TIMEOUT (10 * 60UL)

#define RECONNECT_WIFI_TIMES_MAX 10UL

///////////////////////////////////////////////////////////////////////////////
/// ESP32 target configuration
//#define NODEMCU_ESP32_JOYIT
#define ESP32_NANO_S3_WAVESHARE

///////////////////////////////////////////////////////////////////////////////
/// ESP32 GPIOs configuration

#ifdef NODEMCU_ESP32_JOYIT
#define ANALOG_BATT_GPIO 33
#define DHTPIN            4
#define TURN_OFF_ALL_ESP32_LEDs() do { \
	pinMode(LED_BUILTIN, OUTPUT);      \
	digitalWrite(LED_BUILTIN, HIGH);   \
} while (0)
#endif /* NODEMCU_ESP32_JOYIT */

#ifdef ESP32_NANO_S3_WAVESHARE
#define ANALOG_BATT_GPIO 1
#define DHTPIN           5
#define TURN_OFF_ALL_ESP32_LEDs() do {   \
		pinMode(LED_BUILTIN, OUTPUT);    \
		pinMode(0, OUTPUT);              \
		pinMode(45, OUTPUT);             \
		pinMode(46, OUTPUT);             \
		digitalWrite(LED_BUILTIN, HIGH); \
		digitalWrite(0, HIGH);           \
		digitalWrite(45, HIGH);          \
		digitalWrite(46, HIGH);          \
	} while(0)
#endif /* ESP32_NANO_S3_WAVESHARE */

///////////////////////////////////////////////////////////////////////////////
/// DHT sensor configuration

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

#define DHT_DATA_PULLUP() pinMode(DHTPIN, INPUT_PULLUP);

///////////////////////////////////////////////////////////////////////////////
/// ESP32 wake up configuration

#define S_TO_uS_FACTOR 1000000ULL /* Conversion factor seconds to micro seconds */
#define TIME_IN_DEEP_SLEEP 300    /* Time ESP32 will go to sleep (in seconds)   */


///////////////////////////////////////////////////////////////////////////////

#endif /* CONFIG_H */