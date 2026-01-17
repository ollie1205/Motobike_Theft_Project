#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <WiFiS3.h>
#include <RadioLib.h>
#include <ThingSpeak.h>
#include <moto_theft_detection_inferencing.h>
#include "config.h"

// ---------- WiFi Credentials ----------
const char* ssid = "-------------";
const char* password = "-------------";

// ---------- ThingSpeak Settings ----------
unsigned long channelID = -------;
const char* writeAPIKey = "----------------";

Adafruit_MPU6050 mpu;
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static size_t feature_ix = 0;

TinyGPSPlus gps;

// ---------- WiFi & Client ----------
WiFiClient client;
int status = WL_IDLE_STATUS;
const int MAX_WIFI_ATTEMPTS = 3;

//making global variables for the data that is being collected
float idle = 0;
float theft = 0;
float lat = 0.0;
float lng = 0.0;

int theftcounter = 0;

//booleans used to determine whether to send using wifi or lorawan
bool useWifi = true;
bool useLora = false;

// variables used to check whether another alert can be sent
unsigned long lastAlertTime = 0;
unsigned long lastLoRa = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  delay(5000);

  Serial.print("Initializing LoRa Hardware... ");
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Success!");
  } else {
    Serial.print("Failed, code ");
    Serial.println(state);
  }

  mpu.begin();

  connectToWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    ThingSpeak.begin(client);
    useWifi = true;
  } else {
    Serial.println("WiFi Unavailable. Switching to LoRaWAN!");
    useWifi = false;
    connectToLoRaWAN();
  }
}

void loop() {

  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  features[feature_ix++] = a.acceleration.x;
  features[feature_ix++] = a.acceleration.y;
  features[feature_ix++] = a.acceleration.z;
  features[feature_ix++] = g.gyro.x;
  features[feature_ix++] = g.gyro.y;
  features[feature_ix++] = g.gyro.z;

  if (feature_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    run_inference();
    feature_ix = 0;
  }

  delay(25);  // 40 Hz
}

void run_inference() {
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = [](size_t offset, size_t length, float* out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
  };

  ei_impulse_result_t result;
  run_classifier(&signal, &result, false);
  idle = result.classification[0].value;
  theft = result.classification[1].value;

  Serial.print("Idle: ");
  Serial.println(idle);
  Serial.print("Theft: ");
  Serial.println(theft);

  if (theft >= 0.75) {
    theftcounter++;
  } else {
    theftcounter = 0;
  }

  if (theftcounter >= 3) {
    if (millis() - lastAlertTime > 15000) {
      if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6);
        lat = gps.location.lat();
        Serial.print(gps.location.lng(), 6);
        lng = gps.location.lng();
      } else {
        Serial.println("Warning: GPS has no lock");
      }
      sendAlert();
      lastAlertTime = millis();
      theftcounter = 0;
    } else {
      Serial.print("Upload time limit reached will attempt again");
    }
  }
}



void sendAlert() {
  if (WiFi.status() == WL_CONNECTED) {
    ThingSpeak.setField(1, 1);
    ThingSpeak.setField(2, idle);
    ThingSpeak.setField(3, theft);
    ThingSpeak.setField(4, lat);
    ThingSpeak.setField(5, lng);

    int httpCode = ThingSpeak.writeFields(channelID, writeAPIKey);

    if (httpCode == 200) {
      Serial.print("Data sent to ThingSpeak successfully!");
    } else {
      Serial.print(" Error pushing data. HTTP code: ");
      Serial.println(httpCode);
    }
  } else {
    if (!useLora) {
      Serial.println("Both LoRaWAN and WiFi unnconnected, reattempting LoRaWAN join.");
      connectToLoRaWAN();
    }

    if (useLora) {
      //checks the time since last message on lorawan, makes sure to keep to both duty cycle limit and TTN fair use.
      if (millis() - lastLoRa > (uplinkIntervalSeconds * 1000UL)) {
        Serial.println("Sending via LoRaWAN...");

        //Changing the data into different integer sizes
        int32_t latInt = lat * 10000;
        int32_t lngInt = lng * 10000;
        uint8_t idleByte = (uint8_t)(idle * 100);    // e.g. 0.12 becomes 12
        uint8_t theftByte = (uint8_t)(theft * 100);  // e.g. 0.88 becomes 88

        //Preparing the payload for TTN
        uint8_t payload[11];
        payload[0] = 0x01;

        payload[1] = (latInt >> 24) & 0xFF;
        payload[2] = (latInt >> 16) & 0xFF;
        payload[3] = (latInt >> 8) & 0xFF;
        payload[4] = latInt & 0xFF;

        payload[5] = (lngInt >> 24) & 0xFF;
        payload[6] = (lngInt >> 16) & 0xFF;
        payload[7] = (lngInt >> 8) & 0xFF;
        payload[8] = lngInt & 0xFF;

        payload[9] = idleByte;
        payload[10] = theftByte;


        //Sending the payload
        int state = node.sendReceive(payload, sizeof(payload));

        if (state == RADIOLIB_ERR_NONE) {
          Serial.println("Data sent to TTN succesfully.");
          lastLoRa = millis();
        } else {
          Serial.print("LoRaWAN error:");
          Serial.println(state);
        }
      } else {
        Serial.print("LoRa send halted due to duty cycle limit.");
      }
    }
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < MAX_WIFI_ATTEMPTS) {
    status = WiFi.begin(ssid, password);
    delay(3000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n WiFi connected!");
    Serial.print(" SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n Failed to connect after multiple attempts.");
  }
}

void connectToLoRaWAN() {
  Serial.print("Connecting to LoRaWAN:");

  int state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);

  if (state == RADIOLIB_ERR_NONE) {
    state = node.activateOTAA();

    if (state == RADIOLIB_LORAWAN_NEW_SESSION) {
      Serial.println("Joined Successfully!");
      useLora = true;
    } else {
      Serial.print("Join Failed. Error Code: ");
      Serial.println(state);
    }
  } else {
    Serial.print("Session Setup Failed. Code: ");
    Serial.println(state);
  }
}