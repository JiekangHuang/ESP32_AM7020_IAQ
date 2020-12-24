#include "config.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

#include <PubSubClient.h>
#include <TinyGsmClient.h>

SoftwareSerial mhz19bSerial, zh03bSerial;

// mhz219b uart read command
const uint8_t mhz19b_read_concentration[] = {0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
// zh03b uart set qa mode command
const uint8_t zh03b_set_qa_mode[] = {0xff, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
// zh03b uart read command
const uint8_t zh03b_read_data[] = {0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger, AM7020_RESET);
#else
// 建立 AM7020 modem（設定 Serial 及 EN Pin）
TinyGsm modem(SerialAT, AM7020_RESET);
#endif
// 在 modem 架構上建立 Tcp Client
TinyGsmClient tcpClient(modem);
// 在 Tcp Client 架構上建立 MQTT Client
PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, tcpClient);

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
    // AM7020 NBIOT 連線基地台
    nbConnect();
    // 建立  mhz19b Software Serial(rx_pin: 18, tx_pin: 19)
    mhz19bSerial.begin(BAUDRATE_9600, SWSERIAL_8N1, MHZ19B_RX, MHZ19B_TX, false, MHZ19B_BUFF_SIZE, 11);
    // 建立 zh03b Software Serial(rx_pin: 5, tx_pin: 23)
    zh03bSerial.begin(BAUDRATE_9600, SWSERIAL_8N1, ZH03B_RX, ZH03B_TX, false, ZH03B_BUFF_SIZE, 11);
    setZH03BQAMode();
    // 設定 MQTT KeepAlive time 為 600 秒
    mqttClient.setKeepAlive(600);
}

void loop()
{
    static unsigned long timer = 0;
    static String        pre_co2_color, pre_pm25_color;
    String               co2_color, pm25_color;
    int                  co2, pm25;

    // 讀取 co2 & pm2.5
    co2  = readMHZ19BCO2();
    pm25 = readZH03BPM25();

    // 檢查 MQTT Client 連線狀態
    if (!mqttClient.connected()) {
        // 檢查 NBIOT 連線狀態
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
            // 上傳 co2 到 MQTT Broker
            mqttClient.publish(IAQ_CO2_TOPIC, String(co2).c_str());
            // 判斷 CO2 對應顏色
            if (co2 > 1000) {
                // 紅色，#FF0000
                co2_color = "#FF0000";
            } else if (co2 > 700) {
                // 橘色，#F28500
                co2_color = "#F28500";
            } else {
                // 綠色，#008000
                co2_color = "#008000";
            }
            if (!pre_co2_color.equals(co2_color)) {
                pre_co2_color = co2_color;
                // 上傳 CO2 對應顏色
                mqttClient.publish(IAQ_CO2_COLOR_TOPIC, pre_co2_color.c_str());
                SerialMon.print(F("Publish co2 color : "));
                SerialMon.println(pre_co2_color);
            }
        }
        if (pm25 > 0) {
            SerialMon.print(F("Publish pm25: "));
            SerialMon.println(pm25);
            // 上傳 pm2.5 到 MQTT Broker
            mqttClient.publish(IAQ_PM25_TOPIC, String(pm25).c_str());
            // 判斷 PM2.5 對應顏色
            if (pm25 > 53) {
                // 紅色，#FF0000
                pm25_color = "#FF0000";
            } else if (pm25 > 35) {
                // 橘色，#F28500
                pm25_color = "#F28500";
            } else {
                // 綠色，#008000
                pm25_color = "#008000";
            }
            if (!pre_pm25_color.equals(pm25_color)) {
                pre_pm25_color = pm25_color;
                // 上傳 PM2.5 對應顏色
                mqttClient.publish(IAQ_PM25_COLOR_TOPIC, pre_pm25_color.c_str());
                SerialMon.print(F("Publish pm2.5 color : "));
                SerialMon.println(pre_pm25_color);
            }
        }
    }
    // MQTT Client polling
    mqttClient.loop();
}

/**
 * AM7020 NBIOT 連線基地台
 */
void nbConnect(void)
{
    debugSerial.println(F("Initializing modem..."));
    // 初始化 & 連線基地台
    while (!modem.init() || !modem.nbiotConnect(APN, BAND)) {
        debugSerial.print(F("."));
    };

    debugSerial.print(F("Waiting for network..."));
    // 等待網路連線
    while (!modem.waitForNetwork()) {
        debugSerial.print(F("."));
    }
    debugSerial.println(F(" success"));
}

/**
 * MQTT Client 連線
 */
void mqttConnect(void)
{
    SerialMon.print(F("Connecting to "));
    SerialMon.print(MQTT_BROKER);
    SerialMon.print(F("..."));

    /* Connect to MQTT Broker */
    // 亂數產生 MQTTID
    String mqttid = ("MQTTID_" + String(random(65536)));
    // MQTT Client 連線
    while (!mqttClient.connect(mqttid.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
        SerialMon.println(F(" fail"));
    }
    SerialMon.println(F(" success"));
}

/**
 * 讀取 MHZ219B 二氧化碳濃度
 * @return 二氧化碳濃度
 */
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

/**
 * 讀取 ZH03B PM2.5
 * @return PM2.5
 */
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

/**
 * 設定 ZH03B 資料接收為被動模式
 */
void setZH03BQAMode(void)
{
    // write command to ZH03B
    zh03bSerial.write(zh03b_set_qa_mode, sizeof(zh03b_set_qa_mode));
    zh03bSerial.flush();
}
