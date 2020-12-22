#ifndef _CONFIG_H
#define _CONFIG_H
#include <Arduino.h>

#define DEBUG_MODE true
#if DEBUG_MODE
#define debugSerial Serial
#define DEBUG_PRINTLN(x) debugSerial.println(x)
#define DEBUG_PRINT(x) debugSerial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

/* baudrate */
#define BAUDRATE_9600 9600
#define BAUDRATE_19200 19200
#define BAUDRATE_38400 38400
#define BAUDRATE_115200 115200

/* software Serial config */
#define MHZ19B_RX 18
#define MHZ19B_TX 19
#define MHZ19B_BUFF_SIZE 32

#define ZH03B_RX 5
#define ZH03B_TX 23
#define ZH03B_BUFF_SIZE 32

#define TINY_GSM_MODEM_SIM7020
/* Define the serial console for debug prints, if needed */
// #define TINY_GSM_DEBUG SerialMon
/* uncomment to dump all AT commands */
// #define DEBUG_DUMP_AT_COMMAND

#define UPLOAD_INTERVAL (4.5 * 60000)

#define SerialMon Serial
#define MONITOR_BAUDRATE 115200

/* ESP32 Boards */
#define SerialAT Serial2
#define AM7020_BAUDRATE 115200
#define AM7020_RESET 4

/* set GSM PIN */
#define GSM_PIN ""

// for taiwan mobile
#define APN "twm.nbiot"
#define BAND 28

/* ---mqtt config--- */
#define MQTT_BROKER "io.adafruit.com"
#define MQTT_PORT 1883

#define MQTT_USERNAME "<YOUR USERNAME>"
#define MQTT_PASSWORD "<YOUR AIO KEY>"

#define IAQ_CO2_TOPIC MQTT_USERNAME "/feeds/iaq.co2"
#define IAQ_PM25_TOPIC MQTT_USERNAME "/feeds/iaq.pm25"

#endif /* _CONFIG_H */