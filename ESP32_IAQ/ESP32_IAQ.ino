#include "config.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

#include <PubSubClient.h>
#include <TinyGsmClient.h>

SoftwareSerial mhz19bSerial, zh03bSerial;

const uint8_t mhz19b_read_concentration[] = {0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
const uint8_t zh03b_set_qa_mode[]         = {0xff, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
const uint8_t zh03b_read_data[]           = {0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger, AM7020_RESET);
#else
TinyGsm modem(SerialAT, AM7020_RESET);
#endif
TinyGsmClient tcpClient(modem);
PubSubClient  mqttClient(MQTT_BROKER, MQTT_PORT, tcpClient);

int  readMHZ19BCO2(void);
int  readZH03BPM25(void);
void setZH03BQAMode(void);

void mqttConnect(void);
void nbConnect(void);

void setup()
{
#if DEBUG_MODE
    debugSerial.begin(BAUDRATE_115200);
#endif
    SerialAT.begin(BAUDRATE_115200);

    randomSeed(analogRead(A0));
    nbConnect();

    mhz19bSerial.begin(BAUDRATE_9600, SWSERIAL_8N1, MHZ19B_RX, MHZ19B_TX, false, MHZ19B_BUFF_SIZE, 11);
    zh03bSerial.begin(BAUDRATE_9600, SWSERIAL_8N1, ZH03B_RX, ZH03B_TX, false, ZH03B_BUFF_SIZE, 11);
    setZH03BQAMode();
    mqttClient.setKeepAlive(600);
}

void loop()
{
    static unsigned long timer = 0;
    int                  co2, pm25;

    co2  = readMHZ19BCO2();
    pm25 = readZH03BPM25();

    if (!mqttClient.connected()) {
        if (!modem.isNetworkConnected()) {
            nbConnect();
        }
        SerialMon.println(F("=== MQTT NOT CONNECTED ==="));
        mqttConnect();
    }

    if (millis() >= timer) {
        SerialMon.print(F("co2 : "));
        SerialMon.println(co2);
        SerialMon.print(F("pm25: "));
        SerialMon.println(pm25);

        timer = millis() + UPLOAD_INTERVAL;
        if (co2 > 0) {
            SerialMon.print(F("Publish co2 : "));
            SerialMon.println(co2);
            mqttClient.publish(IAQ_CO2_TOPIC, String(co2).c_str());
        }
        if (pm25 > 0) {
            SerialMon.print(F("Publish pm25: "));
            SerialMon.println(pm25);
            mqttClient.publish(IAQ_PM25_TOPIC, String(pm25).c_str());
        }
    }
    mqttClient.loop();
}

void nbConnect(void)
{
    debugSerial.println(F("Initializing modem..."));
    while (!modem.init() || !modem.nbiotConnect(APN, BAND)) {
        debugSerial.print(F("."));
    };

    debugSerial.print(F("Waiting for network..."));
    while (!modem.waitForNetwork()) {
        debugSerial.print(F("."));
    }
    debugSerial.println(F(" success"));
}

void mqttConnect(void)
{
    SerialMon.print(F("Connecting to "));
    SerialMon.print(MQTT_BROKER);
    SerialMon.print(F("..."));

    // Connect to MQTT Broker
    String mqttid = ("MQTTID_" + String(random(65536)));
    while (!mqttClient.connect(mqttid.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
        SerialMon.println(F(" fail"));
    }
    SerialMon.println(F(" success"));
}

int readMHZ19BCO2(void)
{
    uint8_t       data[9], check_sum = 0x00;
    unsigned long timer;
    // write command to mhz19b
    mhz19bSerial.write(mhz19b_read_concentration, sizeof(mhz19b_read_concentration));
    mhz19bSerial.flush();

    timer = millis();
    while (mhz19bSerial.available() < 9 && millis() - timer < 1000)
        ;

    // read response
    if (mhz19bSerial.available() >= 9) {
        // read all data
        for (int ii = 0; ii < 9; ii++) {
            data[ii] = mhz19bSerial.read();
        }

        // calc check sum
        for (int ii = 1; ii < 8; ii++) {
            check_sum += data[ii];
        }
        // check sum
        if ((((check_sum & 0xFF) ^ 0xFF) + 1) != data[8]) {
            return -1;
        }
        return ((data[2] << 8) | data[3]);
    }
    return -2;
}

int readZH03BPM25(void)
{
    uint8_t       data[9], check_sum = 0x00;
    unsigned long timer;
    // write command to ZH03B
    zh03bSerial.write(zh03b_read_data, sizeof(zh03b_read_data));
    zh03bSerial.flush();

    timer = millis();
    while (zh03bSerial.available() < 9 && millis() - timer < 1000)
        ;

    // read response
    if (zh03bSerial.available() >= 9) {
        // read all data
        for (int ii = 0; ii < 9; ii++) {
            data[ii] = zh03bSerial.read();
        }

        // calc check sum
        for (int ii = 1; ii < 8; ii++) {
            check_sum += data[ii];
        }
        // check sum
        if ((((check_sum & 0xFF) ^ 0xFF) + 1) != data[8]) {
            return -1;
        }
        return ((data[2] << 8) | data[3]);
    }
    return -2;
}

void setZH03BQAMode(void)
{
    // write command to ZH03B
    zh03bSerial.write(zh03b_set_qa_mode, sizeof(zh03b_set_qa_mode));
    zh03bSerial.flush();
}
