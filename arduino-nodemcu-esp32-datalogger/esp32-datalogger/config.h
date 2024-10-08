#ifndef CONFIG_H
#define CONFIG_H

///////////////////////////////////////////////////////////////////////////////
/// Debug purposes
// #define DEBUG

#ifdef DEBUG
#define DBGOUT_INIT() do { \
	Serial.begin(115200);  \
	while (!Serial)        \
	{                      \
		delay(10);         \
		_NOP();            \
	}                      \
	delay(1000);           \
	Serial.println("");    \
	Serial.println("");    \
} while (0)
#define DBGOUT_TS(fmt, ...) (Serial.printf("%09llu: " fmt, getTimestamp(), ##__VA_ARGS__))
#define DBGOUT(fmt, ...) (Serial.printf(fmt, ##__VA_ARGS__))
#define DBGFLUSH() Serial.flush()
#define DBGOUT_START() do {                  \
	printWakeupReason();                     \
	uint32_t Freq = getCpuFrequencyMhz();    \
	DBGOUT_TS("CPU Freq = %d MHz\n", Freq);  \
	Freq = getXtalFrequencyMhz();            \
	DBGOUT_TS("XTAL Freq = %d MHz\n", Freq); \
	Freq = getApbFrequency();                \
	DBGOUT_TS("APB Freq = %d Hz\n", Freq);   \
	DBGOUT_TS("------------------\n");       \
} while (0)

#else
#define DBGOUT_INIT()
#define DBGOUT_TS(...)
#define DBGOUT(...)
#define DBGFLUSH()
#define DBGOUT_START()
#endif

///////////////////////////////////////////////////////////////////////////////
/// ESP32 communication

// maximum of measurement sets that can be stored
// when no wifi connection for a time equal to
//   BUFFER_MAX_SIZE * TIME_IN_DEEP_SLEEP
// e.g 128 x 10 minutes = 1280 minutes = 21 hours 20 minutes
// when wifi disconnection exceeds this time old measurement
// values are overwritten storing always the last measurements
#define BUFFER_MAX_SIZE 128UL

#define	TOKEN_REFRESH_SECONDS_TIMEOUT (10 * 60UL)

#define RECONNECT_WIFI_TIMES_MAX 10UL

// do not connect wifi for at least NO_WIFI_CYCLES_MAX (after wifi disconnection)
// maximum time until next wifi connection is NO_WIFI_CYCLES_MAX * TIME_IN_DEEP_SLEEP
#define NO_WIFI_CYCLES_MAX 6UL

///////////////////////////////////////////////////////////////////////////////
/// ESP32 target configuration
//#define NODEMCU_ESP32_JOYIT
#define ESP32_NANO_S3_WAVESHARE

///////////////////////////////////////////////////////////////////////////////
/// ESP32 BME680 communication
// #define USE_SPI
#define USE_TWI

///////////////////////////////////////////////////////////////////////////////
/// ESP32 GPIOs configuration

#ifdef NODEMCU_ESP32_JOYIT
#define ANALOG_BATT_GPIO    33
#define DHTPIN               4

#ifdef USE_TWI
	#define BME_I2C_SDA     11 // <<-- to verify
	#define BME_I2C_SCL     12 // <<-- to verify
	#define BME_I2C_ADDR  0x77
#endif

#ifdef USE_SPI
	#define BME_VSPI_MOSI    23
	#define BME_VSPI_MISO    19
	#define BME_VSPI_SCK     18
	#define BME_VSPI_CS       5
#endif

#define TURN_OFF_ALL_ESP32_LEDs() do { \
	pinMode(LED_BUILTIN, OUTPUT);      \
	digitalWrite(LED_BUILTIN, HIGH);   \
} while (0)
#endif /* NODEMCU_ESP32_JOYIT */

#ifdef ESP32_NANO_S3_WAVESHARE
#define ANALOG_BATT_GPIO A0
#define DHTPIN           5

#ifdef USE_TWI
	#define BME_I2C_SDA     11
	#define BME_I2C_SCL     12
	#define BME_I2C_ADDR  0x77
#endif

#ifdef USE_SPI
	#define BME_VSPI_MOSI   38
	#define BME_VSPI_MISO   47
	#define BME_VSPI_SCK    48
	#define BME_VSPI_CS      7
#endif

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
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
#define DHTTYPE DHT22     // DHT 22  (AM2302), AM2321

#define DHT_DATA_PULLUP() pinMode(DHTPIN, INPUT_PULLUP);

///////////////////////////////////////////////////////////////////////////////
/// ESP32 wake up configuration

#ifdef DEBUG
#define TIME_IN_DEEP_SLEEP 30
#else
#define TIME_IN_DEEP_SLEEP 600   /* Time ESP32 will go to sleep (in seconds)   */
#endif
#define S_TO_uS_FACTOR 1000000ULL /* Conversion factor seconds to micro seconds */
#define MICROSECS_IN_DEEP_SLEEP (TIME_IN_DEEP_SLEEP * S_TO_uS_FACTOR)



///////////////////////////////////////////////////////////////////////////////

#endif /* CONFIG_H */