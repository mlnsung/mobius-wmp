#include <Arduino.h>
#include <SPI.h>

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#include <WiFi101.h>
#include <WiFiMDNSResponder.h>

#include "OneM2MClient.h"
#include "m0_ota.h"

#include <ArduinoUniqueID.h>

#include <Timer.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int ledPin = 13; // LED pin for connectivity status indicator

uint8_t USE_WIFI = 1;

#define WINC_CS   8
#define WINC_IRQ  7
#define WINC_RST  4
#define WINC_EN   2

#define WIFI_INIT 1
#define WIFI_CONNECT 2
#define WIFI_CONNECTED 3
#define WIFI_RECONNECT 4
uint8_t WIFI_State = WIFI_INIT;

unsigned long wifi_previousMillis = 0;
const long wifi_interval = 30; // count
const long wifi_led_interval = 100; // ms
uint16_t wifi_wait_count = 0;

unsigned long mqtt_previousMillis = 0;
unsigned long mqtt_interval = 8; // count
const unsigned long mqtt_base_led_interval = 250; // ms
unsigned long mqtt_led_interval = mqtt_base_led_interval; // ms
uint16_t mqtt_wait_count = 0;
unsigned long mqtt_watchdog_count = 0;

// for MQTT
#define _MQTT_INIT 1
#define _MQTT_CONNECT 2
#define _MQTT_CONNECTED 3
#define _MQTT_RECONNECT 4
#define _MQTT_READY 5
#define _MQTT_IDLE 6

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

char mqtt_id[16];
uint8_t MQTT_State = _MQTT_INIT;

char in_message[MQTT_MAX_PACKET_SIZE];
StaticJsonBuffer<MQTT_MAX_PACKET_SIZE> jsonBuffer;

unsigned long req_previousMillis = 0;
const long req_interval = 15000; // ms

unsigned long chk_previousMillis = 0;
const long chk_interval = 100; // ms
unsigned long chk_count = 0;

#define UPLOAD_UPLOADING 2
#define UPLOAD_UPLOADED 3
unsigned long uploading_previousMillis = 0;
const long uploading_interval = 10000; // ms
uint8_t UPLOAD_State = UPLOAD_UPLOADING;
uint8_t upload_retry_count = 0;

#define NCUBE_REQUESTED 1
#define NCUBE_REQUESTING 2

char req_id[10];
String state = "create_ae";
uint8_t nCube_State = NCUBE_REQUESTED;
uint8_t sequence = 0;

String strRef[8];
int strRef_length = 0;

#define QUEUE_SIZE 8
typedef struct _queue_t {
    uint8_t pop_idx;
    uint8_t push_idx;
    String ref[QUEUE_SIZE];
    String con[QUEUE_SIZE];
    String rqi[QUEUE_SIZE];
    unsigned long* previousMillis[QUEUE_SIZE];
} queue_t;

queue_t noti_q;
queue_t upload_q;

String body_str = "";

char resp_topic[48];
char noti_topic[48];

unsigned long system_watchdog = 0;

int int_test = 0;

// -----------------------------------------------------------------------------
// User Define

#define VBATPIN A7 // 배터리 전압 측정
#define VUSBPIN A8 // 배터리 충전 측정

char macAddress[] = "";

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

Timer angleT;
Timer batteryT;
Timer ssidT;
Timer rssiT;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3c ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* OLED Bitmap define */
/* bitmap image form flaticon.com (Flaticon license)
   Free for personal and commercial use with attribution. */
const unsigned char PROGMEM wifiBitmap [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x1F, 0xF8, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0xFF, 0xFF, 0xC0, 0x0F, 0xF8, 0x1F, 0xF0,
0x3F, 0x80, 0x01, 0xF8, 0x7E, 0x00, 0x00, 0x7E, 0xFC, 0x00, 0x00, 0x3F, 0xF0, 0x1F, 0xF8, 0x0F,
0xE0, 0xFF, 0xFE, 0x07, 0x01, 0xFF, 0xFF, 0x80, 0x03, 0xF0, 0x0F, 0xC0, 0x07, 0xC0, 0x03, 0xE0,
0x07, 0x80, 0x01, 0xE0, 0x06, 0x07, 0xE0, 0xE0, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x3F, 0xFC, 0x00,
0x00, 0x7F, 0xFE, 0x00, 0x00, 0x78, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char PROGMEM wifiErrorBitmap [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0x00, 0x1C, 0xE0,
0x00, 0x3F, 0x3C, 0xF0, 0x01, 0xFE, 0x3C, 0xF0, 0x07, 0xE0, 0x7F, 0xF8, 0x0F, 0x00, 0xFC, 0xFC,
0x1C, 0x00, 0xFC, 0xFC, 0x38, 0x1C, 0xFF, 0xFC, 0x20, 0xFC, 0xFF, 0xF8, 0x01, 0xE0, 0x3F, 0xF0,
0x03, 0x80, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xC0, 0x00, 0x00, 0x3F, 0xF0, 0x00,
0x00, 0x70, 0x78, 0x00, 0x00, 0x60, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00,
0x00, 0x0F, 0x80, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x07, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char PROGMEM wmpicon [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x30, 0x07, 0xE0, 0x0C,
0x30, 0x07, 0xE0, 0x0C, 0x31, 0x87, 0xE1, 0x8C, 0xFF, 0xF3, 0xCF, 0xFF, 0xFF, 0xF9, 0x9F, 0xFF,
0x33, 0xDC, 0x3B, 0xCC, 0x31, 0xDF, 0xFB, 0x8C, 0x31, 0xFF, 0xFF, 0x8C, 0x01, 0xFF, 0xFF, 0x80,
0x01, 0xFF, 0xFF, 0x80, 0x01, 0xEF, 0xF7, 0x80, 0x01, 0xCF, 0xF3, 0x80, 0x00, 0x0F, 0xF0, 0x00,
0x00, 0x0F, 0xF0, 0x00, 0x00, 0x0F, 0xF0, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0xFF, 0xFF, 0x00,
0x03, 0xFF, 0xFF, 0xC0, 0x07, 0xF8, 0x1F, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0x80, 0x01, 0xE0,
0x07, 0x80, 0x01, 0xE0, 0x07, 0x00, 0x00, 0xE0, 0x0F, 0x00, 0x00, 0xF0, 0x0F, 0x00, 0x00, 0xF0,
0x0F, 0x00, 0x00, 0xF0, 0x0F, 0x00, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00
};

const unsigned char PROGMEM cloudBitmap [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x07, 0xE0, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x7F, 0xFE, 0x00,
0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x07, 0xFC, 0x3F, 0x80,
0x0F, 0xF8, 0x0F, 0xC0, 0x1F, 0xFF, 0xC7, 0xE0, 0x1F, 0xCF, 0xE7, 0xF0, 0x3F, 0x87, 0xF3, 0xF8,
0x3F, 0x03, 0xC0, 0xFC, 0x3F, 0x03, 0xC0, 0xFC, 0x3F, 0xCF, 0xE1, 0xFC, 0x3F, 0xE7, 0xF3, 0xFC,
0x3F, 0xE3, 0xFF, 0xFC, 0x1F, 0xF0, 0x1F, 0xFC, 0x1F, 0xFC, 0x3F, 0xF8, 0x0F, 0xFF, 0xFF, 0xF0,
0x07, 0xFF, 0xFF, 0xE0, 0x01, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};



// Information of CSE as Mobius with MQTT
const String FIRMWARE_VERSION = "1.0.0.0";
String AE_NAME = "air1";
String AE_ID = "S" + AE_NAME;
const String CSE_ID = "/Mobius2";
const String CB_NAME = "Mobius";
const char* MOBIUS_MQTT_BROKER_IP = "13.124.129.36"; //"192.168.33.86";
const uint16_t MOBIUS_MQTT_BROKER_PORT = 1883;

OneM2MClient nCube;

// build tree of resource of oneM2M
void buildResource() {
    nCube.configResource(2, "/"+CB_NAME, AE_NAME);                       // AE resource

    nCube.configResource(3, "/"+CB_NAME+"/"+AE_NAME, "update");          // Container resource
    nCube.configResource(3, "/"+CB_NAME+"/"+AE_NAME, "angle");          // Container resource
    nCube.configResource(3, "/"+CB_NAME+"/"+AE_NAME, "ssid");          // Container resource
    nCube.configResource(3, "/"+CB_NAME+"/"+AE_NAME, "rssi");          // Container resource
    nCube.configResource(3, "/"+CB_NAME+"/"+AE_NAME, "battery");          // Container resource
    nCube.configResource(3, "/"+CB_NAME+"/"+AE_NAME, "workstatus");          // Container resource

    nCube.configResource(23, "/"+CB_NAME+"/"+AE_NAME+"/update", "sub");  // Subscription resource
    nCube.configResource(23, "/"+CB_NAME+"/"+AE_NAME+"/workstatus", "sub");     // Subscription resource
}

void showBatteryIcon(){
    float measuredvbat = analogRead(VBATPIN);
    float vbatMin = 3.7;
    float vbatMax = 4.2;
    float vbatPercent = 0.0;
    int batPercent = 0;
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    vbatPercent = (measuredvbat - vbatMin) / (vbatMax - vbatMin) * 100;
    batPercent = int(vbatPercent);

    // 프로그레스 바 바깥쪽 테두리를 그립니다.
    display.drawRect(113, 2, 124, 7, WHITE);
    display.drawLine(125, 3, 125, 6, WHITE);
    
    // 프로그레스 바 안쪽을 그립니다.
    int barWidth = batPercent * 10 / 100 + 114;  // 백분율 값을 너비로 변환합니다.
    display.fillRect(114, 3, barWidth, 6, WHITE);
}

void showBitmap(String TextValue, const unsigned char bitmap [], int width, int height, int textX, int textY, int textSize){
    display.clearDisplay();
    
    showBatteryIcon();
    display.drawBitmap(width, height, bitmap, 32, 32, 1);
    display.setTextSize(textSize);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(textX, textY);     // Start at top-left corner
    //display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.print(TextValue);

    display.display();

}

void bootBitmap(void){
    display.clearDisplay();

    display.drawBitmap(
        (display.width()  - 32 ) / 2,
        (display.height() - 32) / 2,
        wmpicon, 32, 32, 1);
    
    showBatteryIcon();

    display.display();
}

// SAMD serial Number
String uniqueId()
{
  String uniqueIdString;
  for (size_t i = 0; i < 8; i++)
  {
    if (UniqueID8[i] < 0x10)
      uniqueIdString += "0";
    uniqueIdString += String(UniqueID8[i], HEX);
  }
  
  return uniqueIdString;
}

// 센서 데이터 전송
void mpu6050value(){
    String value;
      /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
     if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
    Serial.print(accX); Serial.print("\t");
    Serial.print(accY); Serial.print("\t");
    Serial.print(accZ); Serial.print("\t");

    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");
    Serial.print(gyroZ); Serial.print("\t");

    Serial.print("\t");
#endif
    //Serial.print(kalAngleX); Serial.print("\t");
    //Serial.print(kalAngleY); Serial.print("\t");

    String value = String(kalAngleX) + "/" + String(kalAngleY);

    postProcess("angle", value);
#if 0 // Set to 1 to print the temperature
    Serial.print("\t");

    double temperature = (double)tempRaw / 340.0 + 36.53;
    Serial.print(temperature); Serial.print("\t");
#endif
    
}

// 배터리 퍼센트 전송
void batteryPercent(){
    float measuredvbat = analogRead(VBATPIN);
    float vbatMin = 3.7;
    float vbatMax = 4.2;
    float vbatPercent = 0.0;
    int batPercent = 0;
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    vbatPercent = (measuredvbat - vbatMin) / (vbatMax - vbatMin) * 100;
    batPercent = int(vbatPercent);

    postProcess("battery", String(batPercent));
    //Serial.print("VBat: " ); Serial.println(measuredvbat);
    //Serial.print(vbatPercent); Serial.println("%");
}

// SSID 전송
void ssidSend(){
    postProcess("ssid", WiFi.SSID());
}   


// RSSI 전송
void rssiSend(){
    // print the received signal strength:
    long rssi = WiFi.RSSI();
    String rssiString = String(rssi);
    postProcess("rssi", rssiString);
}   

// MQTT Pub
void postProcess(String cnt, String content) {
    if (state == "create_cin") {
        String con = "\"" + content + "\"";

        char rqi[10];
        rand_str(rqi, 8);
        upload_q.ref[upload_q.push_idx] = "/"+CB_NAME+"/"+AE_NAME+"/"+cnt;
        upload_q.con[upload_q.push_idx] = con;
        upload_q.rqi[upload_q.push_idx] = String(rqi);
        upload_q.push_idx++;
        if(upload_q.push_idx >= QUEUE_SIZE) {
            upload_q.push_idx = 0;
        }
        if(upload_q.push_idx == upload_q.pop_idx) {
            upload_q.pop_idx++;
            if(upload_q.pop_idx >= QUEUE_SIZE) {
                upload_q.pop_idx = 0;
            }
        }

        Serial.println(con + " pop : " + String(upload_q.pop_idx));
        Serial.println(con + " push : " + String(upload_q.push_idx));
        
        }
    
    
}

// Process notification of Mobius for control
void notiProcess() {
    if(noti_q.pop_idx != noti_q.push_idx) {
        Split(noti_q.ref[noti_q.pop_idx], '/');
        if(strRef[strRef_length-1] == "led") {
            //tasLed.setLED(noti_q.con[noti_q.pop_idx]);
            printf("led status : ")
            printf(noti_q.con[noti_q.pop_idx]);

            String resp_body = "";
            resp_body += "{\"rsc\":\"2000\",\"to\":\"\",\"fr\":\"" + nCube.getAeid() + "\",\"pc\":\"\",\"rqi\":\"" + noti_q.rqi[noti_q.pop_idx] + "\"}";
            nCube.response(mqtt, resp_body);

            Serial.println("2000 ---->");
        }
        else if(strRef[strRef_length-1] == "update") {
            if (noti_q.con[noti_q.pop_idx] == "active") {
                OTAClient.start();   // active OTAClient upgrad process

                String resp_body = "";
                resp_body += "{\"rsc\":\"2000\",\"to\":\"\",\"fr\":\"" + nCube.getAeid() + "\",\"pc\":\"\",\"rqi\":\"" + noti_q.rqi[noti_q.pop_idx] + "\"}";
                nCube.response(mqtt, resp_body);
            }
        }

        noti_q.pop_idx++;
        if(noti_q.pop_idx >= QUEUE_SIZE) {
            noti_q.pop_idx = 0;
        }
    }
}

//------------------------------------------------------------------------------

void setup() {
    WiFi.setPins(8,7,4,2);
    // configure the LED pin for output mode
    pinMode(ledPin, OUTPUT);

    //Initialize serial:
    Serial.begin(115200);
    //while(!Serial);
    
    noti_q.pop_idx = 0;
    noti_q.push_idx = 0;
    upload_q.pop_idx = 0;
    upload_q.push_idx = 0;

    WiFi_init();

    delay(1000);

    byte mac[6];
    WiFi.macAddress(mac);
    sprintf(mqtt_id, "nCube-%.2X%.2X", mac[1], mac[0]);
    unsigned long seed = mac[0] + mac[1];
    randomSeed(seed);

    delay(500);

    // User Defined setup -------------------------------------------------------
    Serial.begin(115200);
    display.clearDisplay();
    bootBitmap();
    Wire.begin();
    #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
    #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
    #endif

    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }

    delay(100); // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while (i2cRead(0x3B, i2cData, 6));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
    #ifdef RESTRICT_PITCH // Eq. 25 and 26
        double roll  = atan2(accY, accZ) * RAD_TO_DEG;
        double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
        double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
        double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    #endif

    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer = micros();

    //UniqueID8dump(Serial);
    //String AE_NAME = uniqueId();
    //String AE_ID = "S" + AE_NAME;
    //Serial.println(uniqueId());

    angleT.every(100,mpu6050value);
    batteryT.every(5000,batteryPercent);
    ssidT.every(10000,ssidSend);
    rssiT.every(10000,rssiSend);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
    //for(;;); // Don't proceed, loop forever
    }
    //bootBitmap();

    //--------------------------------------------------------------------------

    delay(500);

    String topic = "/oneM2M/resp/" + AE_ID + CSE_ID + "/json";
    topic.toCharArray(resp_topic, 64);

    topic = "/oneM2M/req" + CSE_ID + "/" + AE_ID + "/json";
    topic.toCharArray(noti_topic, 64);

    nCube.Init(CSE_ID, MOBIUS_MQTT_BROKER_IP, AE_ID);
    mqtt.setServer(MOBIUS_MQTT_BROKER_IP, MOBIUS_MQTT_BROKER_PORT);
    mqtt.setCallback(mqtt_message_handler);
    MQTT_State = _MQTT_INIT;

    buildResource();
    rand_str(req_id, 8);
    nCube_State = NCUBE_REQUESTED;

    //init OTA client
    OTAClient.begin(AE_NAME, FIRMWARE_VERSION);
}

void loop() {
    // nCube loop
    nCube_loop();

    // user defined loop
    notiProcess();
    

    //showBitmap("Data Sending", cloudBitmap, 56, 16, 10, 55, 1);

    angleT.update();
    batteryT.update();
    ssidT.update();
    rssiT.update();
}

//------------------------------------------------------------------------------
// nCube core functions
// nCube core 기능이니깐 건들지 마셈
//------------------------------------------------------------------------------

void nCube_loop() {
    WiFi_chkconnect();
    if (!mqtt.loop()) {
        MQTT_State = _MQTT_CONNECT;
        //digitalWrite(13, HIGH);
        mqtt_reconnect();
        //digitalWrite(13, LOW);
    }
    else {
        MQTT_State = _MQTT_CONNECTED;
    }

    chkState();
    publisher();
    otaProcess();
    uploadProcess();
}

void rand_str(char *dest, size_t length) {
    char charset[] = "0123456789"
    "abcdefghijklmnopqrstuvwxyz"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    while (length-- > 0) {
        //size_t index = (double) rand() / RAND_MAX * (sizeof charset - 1);
        //*dest++ = charset[index];

        size_t index = random(62);
        *dest++ = charset[index];
    }
    *dest = '\0';
}

void WiFi_init() {
    if(USE_WIFI) {
        // begin WiFi
        digitalWrite(ledPin, HIGH);
        WiFi.setPins(WINC_CS, WINC_IRQ, WINC_RST, WINC_EN);
        if (WiFi.status() == WL_NO_SHIELD) { // check for the presence of the shield:
            Serial.println("WiFi shield not present");
            //showBitmap("WIFI ERROR", wifiErrorBitmap, 56, 16, 10, 55, 1);
            // don't continue:
            while (true) {
                digitalWrite(ledPin, HIGH);
                delay(100);
                digitalWrite(ledPin, LOW);
                delay(100);
            }
        }
        digitalWrite(ledPin, LOW);
    }

    WIFI_State = WIFI_INIT;
}

void WiFi_chkconnect() {
    if(USE_WIFI) {
        if(WIFI_State == WIFI_INIT) {
            digitalWrite(ledPin, HIGH);

            Serial.println("beginProvision - WIFI_INIT");
            WiFi.beginProvision();

            showBitmap("connect wifi101-XXXX", wifiErrorBitmap, 56, 16, 10, 55, 1);
            //WiFi.begin("FILab", "badacafe00");

            WIFI_State = WIFI_CONNECT;
            wifi_previousMillis = 0;
            wifi_wait_count = 0;
            noti_q.pop_idx = 0;
            noti_q.push_idx = 0;
            upload_q.pop_idx = 0;
            upload_q.push_idx = 0;
        }
        else if(WIFI_State == WIFI_CONNECTED) {
            if (WiFi.status() == WL_CONNECTED) {
                return;
            }

            wifi_wait_count = 0;
            if(WIFI_State == WIFI_CONNECTED) {
                WIFI_State = WIFI_RECONNECT;
                wifi_previousMillis = 0;
                wifi_wait_count = 0;
                noti_q.pop_idx = 0;
                noti_q.push_idx = 0;
                upload_q.pop_idx = 0;
                upload_q.push_idx = 0;
            }
            else {
                WIFI_State = WIFI_CONNECT;
                wifi_previousMillis = 0;
                wifi_wait_count = 0;
                noti_q.pop_idx = 0;
                noti_q.push_idx = 0;
                upload_q.pop_idx = 0;
                upload_q.push_idx = 0;
            }
            //nCube.MQTT_init(AE_ID);
        }
        else if(WIFI_State == WIFI_CONNECT) {
            unsigned long currentMillis = millis();
            if (currentMillis - wifi_previousMillis >= wifi_led_interval) {
                wifi_previousMillis = currentMillis;
                if(wifi_wait_count++ >= wifi_interval) {
                    wifi_wait_count = 0;
                    if (WiFi.status() != WL_CONNECTED) {
                        Serial.println("Provisioning......");
                    }
                }
                else {
                    if(wifi_wait_count % 2) {
                        digitalWrite(ledPin, HIGH);
                    }
                    else {
                        digitalWrite(ledPin, LOW);
                    }
                }
            }
            else {
                if (WiFi.status() == WL_CONNECTED) {
                    // you're connected now, so print out the status:
                    printWiFiStatus();

                    digitalWrite(ledPin, LOW);

                    WIFI_State = WIFI_CONNECTED;
                    wifi_previousMillis = 0;
                    wifi_wait_count = 0;
                    noti_q.pop_idx = 0;
                    noti_q.push_idx = 0;
                    upload_q.pop_idx = 0;
                    upload_q.push_idx = 0;
                }
            }
        }
        else if(WIFI_State == WIFI_RECONNECT) {
            digitalWrite(ledPin, HIGH);

            unsigned long currentMillis = millis();
            if (currentMillis - wifi_previousMillis >= wifi_led_interval) {
                wifi_previousMillis = currentMillis;
                if(wifi_wait_count++ >= wifi_interval) {
                    wifi_wait_count = 0;
                    if (WiFi.status() != WL_CONNECTED) {
                        Serial.print("Attempting to connect to SSID: ");
                        Serial.println("previous SSID");

                        WiFi.begin();
                    }
                }
                else {
                    if(wifi_wait_count % 2) {
                        digitalWrite(ledPin, HIGH);
                    }
                    else {
                        digitalWrite(ledPin, LOW);
                    }
                }
            }
            else {
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.println("Connected to wifi");
                    printWiFiStatus();

                    //showBitmap("Connected!", wifiBitmap, 56, 16, 10, 55, 1);

                    digitalWrite(ledPin, LOW);

                    WIFI_State = WIFI_CONNECTED;
                    wifi_previousMillis = 0;
                    wifi_wait_count = 0;
                }
            }
        }
    }
}

void printWiFiStatus() {
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

void otaProcess() {
    if(WIFI_State == WIFI_CONNECTED && MQTT_State == _MQTT_CONNECTED) {
        if (OTAClient.finished()) {
        }
        else {
            OTAClient.poll();
        }
    }
}

void publisher() {
    unsigned long currentMillis = millis();
    if (currentMillis - req_previousMillis >= req_interval) {
        req_previousMillis = currentMillis;

        rand_str(req_id, 8);
        nCube_State = NCUBE_REQUESTED;
    }

    if(nCube_State == NCUBE_REQUESTED && WIFI_State == WIFI_CONNECTED && MQTT_State == _MQTT_CONNECTED) {
        if (state == "create_ae") {
            Serial.print(state + " - ");
            Serial.print(String(sequence));
            Serial.print(" - ");
            Serial.println(String(req_id));
            nCube_State = NCUBE_REQUESTING;
            body_str = nCube.createAE(mqtt, req_id, 0, "3.14");

            if (body_str == "0") {
            Serial.println(F("REQUEST Failed"));
          }
          else {
            Serial.print("Request [");
            Serial.print(nCube.getReqTopic());
            Serial.print("] ----> ");
            Serial.println(body_str.length()+1);
            Serial.println(body_str);
          }
            //digitalWrite(ledPin, HIGH);
        }
        else if (state == "create_cnt") {
            Serial.print(state + " - ");
            Serial.print(String(sequence));
            Serial.print(" - ");
            Serial.println(String(req_id));
            nCube_State = NCUBE_REQUESTING;
            body_str = nCube.createCnt(mqtt, req_id, sequence);
            if (body_str == "0") {
            Serial.println(F("REQUEST Failed"));
          }
          else {
            Serial.print("Request [");
            Serial.print(nCube.getReqTopic());
            Serial.print("] ----> ");
            Serial.println(body_str.length()+1);
            Serial.println(body_str);
          }
            //digitalWrite(ledPin, HIGH);
        }
        else if (state == "delete_sub") {
            Serial.print(state + " - ");
            Serial.print(String(sequence));
            Serial.print(" - ");
            Serial.println(String(req_id));
            nCube_State = NCUBE_REQUESTING;
            body_str = nCube.deleteSub(mqtt, req_id, sequence);
            if (body_str == "0") {
            Serial.println(F("REQUEST Failed"));
          }
          else {
            Serial.print("Request [");
            Serial.print(nCube.getReqTopic());
            Serial.print("] ----> ");
            Serial.println(body_str.length()+1);
            Serial.println(body_str);
          }
            //digitalWrite(ledPin, HIGH);
        }
        else if (state == "create_sub") {
            Serial.print(state + " - ");
            Serial.print(String(sequence));
            Serial.print(" - ");
            Serial.println(String(req_id));
            nCube_State = NCUBE_REQUESTING;
            body_str = nCube.createSub(mqtt, req_id, sequence);
            if (body_str == "0") {
            Serial.println(F("REQUEST Failed"));
          }
          else {
            Serial.print("Request [");
            Serial.print(nCube.getReqTopic());
            Serial.print("] ----> ");
            Serial.println(body_str.length()+1);
            Serial.println(body_str);
          }
            //digitalWrite(ledPin, HIGH);
        }
        else if (state == "create_cin") {
        }
    }
}

unsigned long mqtt_sequence = 0;
void chkState() {
    unsigned long currentMillis = millis();
    if (currentMillis - chk_previousMillis >= chk_interval) {
        chk_previousMillis = currentMillis;

        system_watchdog++;
        if(system_watchdog > 9000) {
            if(system_watchdog % 2) {
                digitalWrite(ledPin, HIGH);
            }
            else {
                digitalWrite(ledPin, LOW);
            }
        }
        else if(system_watchdog > 18000) {
            NVIC_SystemReset();
        }

        if(WIFI_State == WIFI_CONNECT) {
            Serial.println("WIFI_CONNECT");
            MQTT_State = _MQTT_INIT;
        }
        else if(WIFI_State == WIFI_RECONNECT) {
            Serial.println("WIFI_RECONNECT");
            MQTT_State = _MQTT_INIT;
        }
        else if(WIFI_State == WIFI_CONNECTED && MQTT_State == _MQTT_INIT) {
            MQTT_State = _MQTT_CONNECT;
        }

        if(MQTT_State == _MQTT_CONNECT) {
            Serial.println("_MQTT_CONNECT");
        }

        /*if(WIFI_State == WIFI_CONNECTED && MQTT_State == _MQTT_CONNECTED) {
            chk_count++;
            if(chk_count >= 100) {
                chk_count = 0;

                //noInterrupts();
                body_str = nCube.heartbeat(mqtt);
                // char seq[10];
                // sprintf(seq, "%ld", ++mqtt_sequence);
                // mqtt.publish("/nCube/count/test", seq, strlen(seq));
                // Serial.println(String(mqtt_sequence));
                //interrupts();

                if (body_str == "Failed") {
                    Serial.println(F("Heartbeat Failed"));
                }
                else {
                    Serial.print("Send heartbeat [");
                    Serial.print(nCube.getHeartbeatTopic());
                    Serial.print("] ----> ");
                    Serial.println(body_str.length()+1);
                    Serial.println(body_str);
                    system_watchdog = 0;
                }
            }
        }*/
    }
}

void Split(String sData, char cSeparator)
{
    int nCount = 0;
    int nGetIndex = 0 ;
    strRef_length = 0;

    String sTemp = "";
    String sCopy = sData;

    while(true) {
        nGetIndex = sCopy.indexOf(cSeparator);

        if(-1 != nGetIndex) {
            sTemp = sCopy.substring(0, nGetIndex);
            strRef[strRef_length++] = sTemp;
            sCopy = sCopy.substring(nGetIndex + 1);
        }
        else {
            strRef[strRef_length++] = sCopy;
            break;
        }
        ++nCount;
    }
}

void mqtt_reconnect() {
    if(WIFI_State == WIFI_CONNECTED && MQTT_State == _MQTT_CONNECT) {
        unsigned long currentMillis = millis();
        if (currentMillis - mqtt_previousMillis >= mqtt_led_interval) {
            mqtt_led_interval = mqtt_base_led_interval + random(mqtt_base_led_interval);
            mqtt_previousMillis = currentMillis;
            if(mqtt_wait_count++ >= (mqtt_interval)) {
                mqtt_wait_count = 0;

                Serial.print("Attempting MQTT connection...");
                // Attempt to connect
                //rand_str(mqtt_id, 8);
                if (mqtt.connect(mqtt_id)) {
                    mqtt_watchdog_count = 0;
                    Serial.println("connected");

                    if (mqtt.subscribe(resp_topic)) {
                        Serial.println(String(resp_topic) + " Successfully subscribed");
                    }

                    if (mqtt.subscribe(noti_topic)) {
                        Serial.println(String(noti_topic) + " Successfully subscribed");
                    }

                    MQTT_State = _MQTT_CONNECTED;
                    nCube.reset_heartbeat();
                }
                else {
                    Serial.print("failed, rc=");
                    Serial.print(mqtt.state());
                    Serial.println(" try again in 2 seconds");
                    mqtt_watchdog_count++;
                    if(mqtt_watchdog_count > 10) {
                        NVIC_SystemReset();
                    }
                }
            }
            else {
                if(mqtt_wait_count % 2) {
                    digitalWrite(ledPin, HIGH);
                }
                else {
                    digitalWrite(ledPin, LOW);
                }
            }
        }
    }
}

void mqtt_message_handler(char* topic_in, byte* payload, unsigned int length) {
    String topic = String(topic_in);

    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] <---- ");
    Serial.println(length);

    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    //noInterrupts();

    if (topic.substring(8,12) == "resp") {
        memset((char*)in_message, '\0', length+1);
        memcpy((char*)in_message, payload, length);
        JsonObject& resp_root = jsonBuffer.parseObject(in_message);

        if (!resp_root.success()) {
            Serial.println(F("parseObject() failed"));
            jsonBuffer.clear();
            return;
        }

        wifiClient.flush();

        resp_handler(resp_root["rsc"], resp_root["rqi"]);

        jsonBuffer.clear();
    }
    else if(topic.substring(8,11) == "req") {
        memset((char*)in_message, '\0', length+1);
        memcpy((char*)in_message, payload, length);
        JsonObject& req_root = jsonBuffer.parseObject(in_message);

        if (!req_root.success()) {
            Serial.println(F("parseObject() failed"));
            return;
        }

        wifiClient.flush();

        noti_handler(req_root["pc"]["m2m:sgn"]["sur"], req_root["rqi"], req_root["pc"]["m2m:sgn"]["nev"]["rep"]["m2m:cin"]["con"]);

        jsonBuffer.clear();
    }
    //interrupts();
    system_watchdog = 0;
}

void noti_handler(String sur, String rqi, String con) {
    if (state == "create_cin") {
        Serial.print("<---- ");
        if(sur.charAt(0) != '/') {
            sur = '/' + sur;
            Serial.println(sur);
        }
        else {
            Serial.println(sur);
        }

        String valid_sur = nCube.validSur(sur);
        if (valid_sur != "empty") {
            noti_q.ref[noti_q.push_idx] = valid_sur;
            noti_q.con[noti_q.push_idx] = con;
            noti_q.rqi[noti_q.push_idx] = rqi;
            noti_q.push_idx++;
            if(noti_q.push_idx >= QUEUE_SIZE) {
                noti_q.push_idx = 0;
            }
            if(noti_q.push_idx == noti_q.pop_idx) {
                noti_q.pop_idx++;
                if(noti_q.pop_idx >= QUEUE_SIZE) {
                    noti_q.pop_idx = 0;
                }
            }
        }
    }
}

void resp_handler(int response_code, String response_id) {
    String request_id = String(req_id);

    if (request_id == response_id) {
        if (response_code == 2000 || response_code == 2001 || response_code == 2002 || response_code == 4105 || response_code == 4004) {
            Serial.print("<---- ");
            Serial.println(response_code);
            if (state == "create_ae") {
                sequence++;
                if(sequence >= nCube.getAeCount()) {
                    state = "create_cnt";
                    sequence = 0;
                }
                rand_str(req_id, 8);
                nCube_State = NCUBE_REQUESTED;
            }
            else if (state == "create_cnt") {
                sequence++;
                if(sequence >= nCube.getCntCount()) {
                    state = "delete_sub";
                    sequence = 0;
                }
                rand_str(req_id, 8);
                nCube_State = NCUBE_REQUESTED;
            }
            else if(state == "delete_sub") {
                sequence++;
                if(sequence >= nCube.getSubCount()) {
                    state = "create_sub";
                    sequence = 0;
                }
                rand_str(req_id, 8);
                nCube_State = NCUBE_REQUESTED;
            }
            else if (state == "create_sub") {
                sequence++;
                if(sequence >= nCube.getSubCount()) {
                    state = "create_cin";
                    sequence = 0;
                }
                rand_str(req_id, 8);
                nCube_State = NCUBE_REQUESTED;
            }
            else if (state == "create_cin") {
                upload_retry_count = 0;
                if(upload_q.pop_idx == upload_q.push_idx) {

                }
                else {
                    *upload_q.previousMillis[upload_q.pop_idx] = millis();

                    upload_q.pop_idx++;
                    if(upload_q.pop_idx >= QUEUE_SIZE) {
                        upload_q.pop_idx = 0;
                    }
                }
            }
            //digitalWrite(ledPin, LOW);
            UPLOAD_State = UPLOAD_UPLOADED;
        }
    }
}

void uploadProcess() {
    if(WIFI_State == WIFI_CONNECTED && MQTT_State == _MQTT_CONNECTED) {
        unsigned long currentMillis = millis();
        if (currentMillis - uploading_previousMillis >= uploading_interval) {
            uploading_previousMillis = currentMillis;

            if (state == "create_cin") {
                if(UPLOAD_State == UPLOAD_UPLOADING) {
                    Serial.println("upload timeout");
                }

                UPLOAD_State = UPLOAD_UPLOADED;
                upload_retry_count++;
                if(upload_retry_count > 2) {
                    upload_retry_count = 0;
                    if(upload_q.pop_idx == upload_q.push_idx) {

                    }
                    else {
                        upload_q.pop_idx++;
                        if(upload_q.pop_idx >= QUEUE_SIZE) {
                            upload_q.pop_idx = 0;
                        }
                    }
                }
            }
        }

        if((UPLOAD_State == UPLOAD_UPLOADED) && (upload_q.pop_idx != upload_q.push_idx)) {
            if (wifiClient.available()) {
                return;
            }

            uploading_previousMillis = millis();
            UPLOAD_State = UPLOAD_UPLOADING;

            upload_q.rqi[upload_q.pop_idx].toCharArray(req_id, 10);

            Serial.println("pop : " + String(upload_q.pop_idx));
            Serial.println("push : " + String(upload_q.push_idx));
            //noInterrupts();
            body_str = nCube.createCin(mqtt, upload_q.rqi[upload_q.pop_idx], upload_q.ref[upload_q.pop_idx], upload_q.con[upload_q.pop_idx]);
            wifiClient.flush();
            //interrupts();
            if (body_str == "0") {
            Serial.println(F("REQUEST Failed"));
          }
          else {
                system_watchdog = 0;
            Serial.print("Request [");
            Serial.print(nCube.getReqTopic());
            Serial.print("] ----> ");
            Serial.println(body_str.length()+1);
            Serial.println(body_str);
          }
            //digitalWrite(ledPin, HIGH);
        }
    }
}