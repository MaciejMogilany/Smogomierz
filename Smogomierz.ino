#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#elif defined ARDUINO_ARCH_ESP32
#include <WiFi.h>
#endif

/*

  Szkic używa 495928 bajtów (47%) pamięci programu. Maksimum to 1044464 bajtów.
  Zmienne globalne używają 54744 bajtów (66%) pamięci dynamicznej, pozostawiając 27176 bajtów dla zmiennych lokalnych. Maksimum to 81920 bajtów.

  Szkic używa 495764 bajtów (47%) pamięci programu. Maksimum to 1044464 bajtów.
  Zmienne globalne używają 54684 bajtów (66%) pamięci dynamicznej, pozostawiając 27236 bajtów dla zmiennych lokalnych. Maksimum to 81920 bajtów.

*/

#include <Wire.h>

#include "FS.h"
#include <ArduinoJson.h> // 6.9.0 or later
#include "src/WiFiManager.h" // https://github.com/jakerabid/WiFiManager // 12.03.2019
#include "src/bme280.h" // https://github.com/zen/BME280_light // CUSTOMIZED! 8.04.2019
#include "src/HTU21D.h" // https://github.com/enjoyneering/HTU21D // 12.03.2019
#include "src/Adafruit_BMP280.h" // https://github.com/adafruit/Adafruit_BMP280_Library // 12.03.2019
#include "src/SHT1x.h" // https://github.com/practicalarduino/SHT1x // 12.03.2019
#include <DHT.h>

#include "src/SdsDustSensor.h" // SDS011/SDS021 - https://github.com/lewapek/sds-dust-sensors-arduino-library // 29.03.2019

#include "src/spiffs.h"
#include "src/config.h"
#include "defaultConfig.h"
#include "src/autoupdate.h"
#include "src/smoglist.h"

#include "src/luftdaten.h"
#include "src/airmonitor.h"
#include "src/thing_speak.h"
#include "src/ESPinfluxdb.h" // https://github.com/hwwong/ESP_influxdb // 12.03.2019

/*
  Podłączenie czujnikow dla ESP8266 NodeMCU:
  BME280/BMP280: VIN - 3V; GND - G; SCL - D4; SDA - D3
  SHT1x: VIN - 3V; GND - G; SCL - D5; DATA/SDA - D6 wymaga rezystora 10k podłaczonego do VCC
  SHT21/HTU21D: VIN - 3V; GND - G; SCL - D5; SDA - D6
  DHT22: VIN - 3V; GND - G; D7
  SDS011/21: VIN - 5V; GND - G; TX - D1; RX - D2


  Connection of sensors on ESP8266 NodeMCU:
  BME280/BMP280: VIN - 3V; GND - G; SCL - D4; SDA - D3
  SHT1x: VIN - 3V; GND - G; SCL - D5; DATA/SDA - D6 required pull-up resistor 10k to VCC
  SHT21/HTU21D: VIN - 3V; GND - G; SCL - D5; SDA - D6
  DHT22: VIN - 3V; GND - G; D7
  SDS011/21: VIN - 5V; GND - G; TX - D1; RX - D2
*/

// TEMP/HUMI/PRESS Sensor config - START
// BME280 config
#define ASCII_ESC 27
char bufout[10];
BME280<> BMESensor;

// BMP280 config
Adafruit_BMP280 bmp; //I2C

// Serial for SHT21/HTU21D config
HTU21D  myHTU21D(HTU21D_RES_RH12_TEMP14);

// DHT22 config
#define DHTPIN 13 // D7 on NodeMCU/WeMos board
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// SHT1x – Config
#define dataPin 14 //D5
#define clockPin 12 //D6
SHT1x sht1x(dataPin, clockPin);
// TEMP/HUMI/PRESS Sensor config - END

// DUST Sensor config - START
// SDS011/21 config
SdsDustSensor sds(5, 4);
float SDSpm25, SDSpm10;
// DUST Sensor config - END

char device_name[20];

unsigned long DUST_interval = 60 * 1000; // 1 minute
unsigned long previous_DUST_Millis = 0;

unsigned long SENDING_FREQUENCY_interval = 60 * 1000; // 1 minute
unsigned long previous_SENDING_FREQUENCY_Millis = 0;

unsigned long SENDING_DB_FREQUENCY_interval = 60 * 1000; // 1 minute
unsigned long previous_SENDING_DB_FREQUENCY_Millis = 0;

unsigned long previous_2sec_Millis = 0;
unsigned long TwoSec_interval = 2 * 1000; // 2 second

unsigned long REBOOT_interval = 24 * 60 * 60 * 1000; // 24 hours
unsigned long previous_REBOOT_Millis = 0;

int pmMeasurements[10][3];
int iPM, averagePM1, averagePM25, averagePM4, averagePM10 = 0;
float currentTemperature, currentHumidity, currentPressure = 0;
float calib = 1;

bool need_update = false;
char SERVERSOFTWAREVERSION[255] = "";
char CURRENTSOFTWAREVERSION[255] = "";

ESP8266WebServer WebServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiClient espClient;
PubSubClient mqttclient(espClient);

// check TEMP/HUMI/PRESS Sensor - START
bool checkHTU21DStatus() {
  int temperature_HTU21D_Int = int(myHTU21D.readTemperature());
  int humidity_HTU21D_Int = int(myHTU21D.readHumidity());
  if ((temperature_HTU21D_Int == 0 && humidity_HTU21D_Int == 0) || (temperature_HTU21D_Int == 255 && humidity_HTU21D_Int == 255)) {
    if (DEBUG) {
      Serial.println("No data from HTU21D sensor!\n");
    }
    return false;
  } else {
    return true;
  }
}

bool checkBmeStatus() {
  int temperature_BME280_Int = BMESensor.temperature;
  int pressure_BME280_Int = (BMESensor.seaLevelForAltitude(MYALTITUDE));
  int humidity_BME280_Int = BMESensor.humidity;
  if (temperature_BME280_Int == 0 && pressure_BME280_Int == 0 && humidity_BME280_Int == 0) {
    if (DEBUG) {
      Serial.println("No data from BME280 sensor!\n");
    }
    return false;
  } else {
    return true;
  }
}

bool checkBmpStatus() {
  int temperature_BMP_Int = bmp.readTemperature();
  int pressure_BMP_Int = bmp.readPressure();
  if (temperature_BMP_Int == 0 && pressure_BMP_Int == 0) {
    if (DEBUG) {
      Serial.println("No data from BMP280 sensor!\n");
    }
    return false;
  } else {
    return true;
  }
}

bool checkDHT22Status() {
  int humidity_DHT_Int = dht.readHumidity();
  int temperature_DHT_Int = dht.readTemperature();
  if (humidity_DHT_Int == 0 && temperature_DHT_Int == 0) {
    if (DEBUG) {
      Serial.println("No data from DHT22 sensor!\n");
    }
    return false;
  } else {
    return true;
  }
}

bool checkSHT1xStatus() {
  int humidity_SHT1x_Int = sht1x.readHumidity();
  int temperature_SHT1x_Int = sht1x.readTemperatureC();
  if (humidity_SHT1x_Int == 0 && temperature_SHT1x_Int == 0) {
    if (DEBUG) {
      Serial.println("No data from SHT1x sensor!\n");
    }
    return false;
  } else {
    return true;
  }
}
// check TEMP/HUMI/PRESS Sensor - END

void minutesToSeconds() {
  DUST_interval = 1000; // 1 second
  SENDING_FREQUENCY_interval = 1000;
  SENDING_DB_FREQUENCY_interval = 1000;
}

void MQTTreconnect() {
  // Loop until we're reconnected
  if (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttclient.connect("ESP8266Client", MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println("\n");
    }
  }
}

// default translation - english
#include "src/default_intl.h"

#include "src/translator.h"

// all HTML content
#include "src/html-content.h"

// library doesnt support arguments :/
#include "src/webserver.h"

void setup() {
  Serial.begin(115200);
  delay(10);

  fs_setup();
  delay(10);

  loadtranslation(SELECTED_LANGUAGE);
  delay(10);

  // DUST SENSOR setup - START
  if (!strcmp(DUST_MODEL, "SDS011/21")) {
    sds.begin();  //SDS011/21 sensor begin
    if (FREQUENTMEASUREMENT == true) {
      sds.wakeup();
      sds.setQueryReportingMode().toString(); // ensures sensor is in 'query' reporting mode
      sds.setContinuousWorkingPeriod().toString(); // ensures sensor has continuous working period - default but not recommended
    } else {
      sds.setCustomWorkingPeriod(1);
      WorkingStateResult state = sds.sleep();
    }
  }
  delay(10);
  // DUST SENSOR setup - END

  if (SENDING_FREQUENCY < DUST_TIME) {
    SENDING_FREQUENCY = DUST_TIME;
  }
  if (SENDING_DB_FREQUENCY == 0) {
    SENDING_DB_FREQUENCY = SENDING_FREQUENCY;
  }
  delay(10);

  if (FREQUENTMEASUREMENT == true) {
    minutesToSeconds();
  }

  if (strcmp(DUST_MODEL, "Non")) {
    DUST_interval = DUST_interval * DUST_TIME;
  }
  if (DEEPSLEEP_ON == true) {
    if (LUFTDATEN_ON or AIRMONITOR_ON or SMOGLIST_ON or THINGSPEAK_ON or INFLUXDB_ON or MQTT_ON) {
      SENDING_FREQUENCY_interval = SENDING_FREQUENCY_interval * SENDING_FREQUENCY;
    }
  } else {
    if (LUFTDATEN_ON or AIRMONITOR_ON or SMOGLIST_ON) {
      SENDING_FREQUENCY_interval = SENDING_FREQUENCY_interval * SENDING_FREQUENCY;
    }
    if (THINGSPEAK_ON or INFLUXDB_ON or MQTT_ON) {
      SENDING_DB_FREQUENCY_interval = SENDING_DB_FREQUENCY_interval * SENDING_DB_FREQUENCY;
    }
  }
  delay(10);

  // TEMP/HUMI/PRESS Sensor seturp - START
  if (!strcmp(THP_MODEL, "BME280")) {
    Wire.begin(0, 2);
    BMESensor.begin();
  } else if (!strcmp(THP_MODEL, "BMP280")) {
    Wire.begin(0, 2);
    bmp.begin();
  } else if (!strcmp(THP_MODEL, "HTU21")) {
    myHTU21D.begin();
  } else if (!strcmp(THP_MODEL, "DHT22")) {
    dht.begin();
  } else if (!strcmp(THP_MODEL, "SHT1x")) {
  }
  delay(10);
  // TEMP/HUMI/PRESS Sensor setup - END

  // get ESP id
  if (DEVICENAME_AUTO) {
    sprintf(device_name, "Smogomierz-%06X", ESP.getChipId());
  } else {
    strncpy(device_name, DEVICENAME, 20);
  }

  Serial.print("Device name: ");
  Serial.println(device_name);

  WiFiManager wifiManager;
  wifiManager.autoConnect(device_name);

  delay(250);

  if (!wifiManager.autoConnect(device_name)) {
    Serial.println("Failed to connect...");
    delay(1000);
    ESP.reset(); //reset and try again
    delay(5000);
  }

  // check update
  if (checkUpdate(0) == true) {
    need_update = true;
  } else {
    need_update = false;
  }

  if (MQTT_ON) {
    mqttclient.setServer(MQTT_HOST, MQTT_PORT);
  }

  if (INFLUXDB_ON) {
    Influxdb influxdb(INFLUXDB_HOST, INFLUXDB_PORT);
    if (influxdb.opendb(INFLUXDB_DATABASE, DB_USER, DB_PASSWORD) != DB_SUCCESS) {
      Serial.println("Opening InfluxDB failed");
    } else {
      Serial.println("Opening InfluxDB succeed");
    }
  }

  //  WebServer config - Start
  WebServer.on("/", HTTP_GET,  handle_root);
  WebServer.on("/config", HTTP_POST, handle_config_post);
  WebServer.on("/config", HTTP_GET, handle_config);
  WebServer.on("/update", HTTP_GET, handle_update);
  WebServer.on("/api", HTTP_GET, handle_api);
  WebServer.on("/erase_wifi", HTTP_GET, erase_wifi);
  WebServer.on("/restore_config", HTTP_GET, restore_config);
  WebServer.on("/fwupdate", HTTP_GET, fwupdate);
  WebServer.on("/autoupdateon", HTTP_GET, autoupdateon);
  WebServer.onNotFound(handle_root);

  httpUpdater.setup(&WebServer, "/update");
  //  WebServer Config - End

  // Check if config.h exist in ESP data folder
  WebServer.begin();

  MDNS.begin(device_name);

  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPServer ready! http://%s.local/\n", device_name);
  delay(300);
}

void loop() {
  if (need_update == true && AUTOUPDATE_ON) {
    for (int i = 0; i < 5 ; i++) {
      doUpdate(0);
      delay(1000);
    }
  }
  delay(10);

  pm_calibration();

  // DUST SENSOR refresh data - START
  PmResult SDSdata = sds.queryPm();
  // DUST SENSOR refresh data - END
  delay(10);

  //webserverShowSite(WebServer, BMESensor, data);
  WebServer.handleClient();
  delay(10);

  MDNS.update();

  yield();

  if (strcmp(DUST_MODEL, "Non")) {
    unsigned long current_DUST_Millis = millis();
    if (FREQUENTMEASUREMENT == true ) {
      if (current_DUST_Millis - previous_DUST_Millis >= DUST_interval) {
        takeNormalnPMMeasurements();
        previous_DUST_Millis = millis();
      }
    }
    if (DEEPSLEEP_ON == true) {
      Serial.println("\nDeepSleep Mode!\n");

      takeSleepPMMeasurements();
      delay(10);

      if (LUFTDATEN_ON or AIRMONITOR_ON or SMOGLIST_ON) {
        takeTHPMeasurements();
        sendDataToExternalServices();
      }
      if (THINGSPEAK_ON or INFLUXDB_ON or MQTT_ON) {
        takeTHPMeasurements();
        sendDataToExternalDBs();
      }

      Serial.println("Going into deep sleep for " + String(SENDING_FREQUENCY) + " minutes!");
      ESP.deepSleep(SENDING_FREQUENCY * 60 * 1000000); // *1000000 - secunds
      delay(10);

    } else {
      if (current_DUST_Millis - previous_DUST_Millis >= DUST_interval) {
        takeSleepPMMeasurements();
        previous_DUST_Millis = millis();
      }
    }
  } else {
    if (DEEPSLEEP_ON == true) {
      Serial.println("\nDeepSleep Mode!\n");
      unsigned long current_2sec_Millis = millis();
      previous_2sec_Millis = millis();
      while (previous_2sec_Millis - current_2sec_Millis <= TwoSec_interval * 10) {
        WebServer.handleClient();
        delay(10);
        yield();
        previous_2sec_Millis = millis();
      }
      if (LUFTDATEN_ON or AIRMONITOR_ON or SMOGLIST_ON) {
        takeTHPMeasurements();
        sendDataToExternalServices();
      }
      if (THINGSPEAK_ON or INFLUXDB_ON or MQTT_ON) {
        takeTHPMeasurements();
        sendDataToExternalDBs();
      }
      delay(10);
      Serial.println("Going into deep sleep for " + String(SENDING_FREQUENCY) + " minutes!");
      ESP.deepSleep(SENDING_FREQUENCY * 60 * 1000000); // *1000000 - secunds
      delay(10);
    }
  }

  if (LUFTDATEN_ON or AIRMONITOR_ON or SMOGLIST_ON) {
    unsigned long current_SENDING_FREQUENCY_Millis = millis();
    if (current_SENDING_FREQUENCY_Millis - previous_SENDING_FREQUENCY_Millis >= SENDING_FREQUENCY_interval) {
      takeTHPMeasurements();
      sendDataToExternalServices();
      previous_SENDING_FREQUENCY_Millis = millis();
    }
  }

  if (THINGSPEAK_ON or INFLUXDB_ON or MQTT_ON) {
    unsigned long current_SENDING_DB_FREQUENCY_Millis = millis();
    if (current_SENDING_DB_FREQUENCY_Millis - previous_SENDING_DB_FREQUENCY_Millis >= SENDING_DB_FREQUENCY_interval) {
      takeTHPMeasurements();
      sendDataToExternalDBs();
      previous_SENDING_DB_FREQUENCY_Millis = millis();
    }
  }

  unsigned long current_REBOOT_Millis = millis();
  if (current_REBOOT_Millis - previous_REBOOT_Millis >= REBOOT_interval) {
    Serial.println("autoreboot...");
    delay(1000);
    previous_REBOOT_Millis = millis();
    ESP.reset();
    delay(5000);
  }

} // loop() - END

void sendDataToExternalServices() {

  if (LUFTDATEN_ON) {
    sendDataToLuftdaten(currentTemperature, currentPressure, currentHumidity, averagePM1, averagePM25, averagePM4, averagePM10);
    if (DEBUG) {
      Serial.println("Sending measurement data to the LuftDaten service!\n");
    }
  }

  if (AIRMONITOR_ON) {
    sendDataToAirMonitor(currentTemperature, currentPressure, currentHumidity, averagePM1, averagePM25, averagePM4, averagePM10);
    if (DEBUG) {
      Serial.println("Sending measurement data to the AirMonitor service!\n");
    }
  }

  if (SMOGLIST_ON) {
    sendDataToSmoglist(currentTemperature, currentPressure, currentHumidity, averagePM1, averagePM25, averagePM4, averagePM10);
    if (DEBUG) {
      Serial.println("Sending measurement data to the Smoglist service!\n");
    }
  }

}

void sendDataToExternalDBs() {

  if (MQTT_ON) {
    if (!mqttclient.connected()) {
      MQTTreconnect();
    }
    mqttclient.loop();
    delay(10);
  }

  if (THINGSPEAK_ON) {
    sendDataToThingSpeak(currentTemperature, currentPressure, currentHumidity, averagePM1, averagePM25, averagePM4, averagePM10);
    if (DEBUG) {
      Serial.println("Sending measurement data to the Thingspeak service!\n");
    }
  }

  if (INFLUXDB_ON) {
    Influxdb influxdb(INFLUXDB_HOST, INFLUXDB_PORT);
    if (influxdb.opendb(INFLUXDB_DATABASE, DB_USER, DB_PASSWORD) != DB_SUCCESS) {
      Serial.println("Opening database failed");
    } else {
      dbMeasurement row(device_name);
      if (!strcmp(DUST_MODEL, "SDS011/21")) {
        row.addField("pm1", averagePM1);
        row.addField("pm25", averagePM25);
        row.addField("pm10", averagePM10);
      } else {
        if (DEBUG) {
          Serial.println("\nNo measurements from SDS011/21!\n");
        }
      }
      if (!strcmp(THP_MODEL, "BME280")) {
        if (checkBmeStatus() == true) {
          row.addField("temperature", (currentTemperature));
          row.addField("pressure", (currentPressure));
          row.addField("humidity", (currentHumidity));
        } else {
          if (DEBUG) {
            Serial.println("No measurements from BME280!\n");
          }
        }
      } else if (!strcmp(THP_MODEL, "HTU21")) {
        if (checkHTU21DStatus() == true) {
          row.addField("temperature", (currentTemperature));
          row.addField("humidity", (currentHumidity));
        } else {
          if (DEBUG) {
            Serial.println("No measurements from HTU21D!\n");
          }
        }
      } else if (!strcmp(THP_MODEL, "BMP280")) {
        if (checkBmpStatus() == true) {
          row.addField("temperature", (currentTemperature));
          row.addField("pressure", (currentPressure));
        } else {
          if (DEBUG) {
            Serial.println("No measurements from BMP280!\n");
          }
        }
      } else if (!strcmp(THP_MODEL, "DHT22")) {
        if (checkDHT22Status() == true) {
          row.addField("temperature", (currentTemperature));
          row.addField("humidity", (currentHumidity));
        } else {
          if (DEBUG) {
            Serial.println("No measurements from DHT22!\n");
          }
        }
      } else if (!strcmp(THP_MODEL, "SHT1x")) {
        if (checkSHT1xStatus() == true) {
          row.addField("temperature", (currentTemperature));
          row.addField("humidity", (currentHumidity));
        } else {
          if (DEBUG) {
            Serial.println("No measurements from SHT1x!\n");
          }
        }
      }

      if (influxdb.write(row) == DB_SUCCESS) {
        if (DEBUG) {
          Serial.println("Data sent to InfluxDB\n");
        }
      } else {
        if (DEBUG) {
          Serial.println("Error sending data to InfluxDB\n");
        }
      }
      row.empty();
    }
  }

  if (MQTT_ON) {
    if (strcmp(DUST_MODEL, "Non")) {
      if (DEBUG) {
        Serial.println("Measurements from PM Sensor!\n");
      }
      mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/PM1").c_str(), String(averagePM1).c_str(), true);
      mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/PM2.5").c_str(), String(averagePM25).c_str(), true);
      mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/PM10").c_str(), String(averagePM10).c_str(), true);
      if (averagePM25 <= 10) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/airquality").c_str(), "EXCELLENT", true);
      } else if (averagePM25 > 10 && averagePM25 <= 20) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/airquality").c_str(), "GOOD", true);
      } else if (averagePM25 > 20 && averagePM25 <= 25) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/airquality").c_str(), "FAIR", true);
      } else if (averagePM25 > 25 && averagePM25 <= 50) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/airquality").c_str(), "INFERIOR", true);
      } else if (averagePM25 > 50) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/airquality").c_str(), "POOR", true);
      } else {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/airquality").c_str(), "UNKNOWN", true);
      }
    }
    if (!strcmp(THP_MODEL, "BME280")) {
      if (checkBmeStatus() == true) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/temperature").c_str(), String(currentTemperature).c_str(), true);
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/pressure").c_str(), String(currentPressure).c_str(), true);
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/humidity").c_str(), String(currentHumidity).c_str(), true);
      } else {
        if (DEBUG) {
          Serial.println("No measurements from BME280!\n");
        }
      }
    }

    if (!strcmp(THP_MODEL, "BMP280")) {
      if (checkBmpStatus() == true) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/temperature").c_str(), String(currentTemperature).c_str(), true);
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/pressure").c_str(), String(currentPressure).c_str(), true);
      } else {
        if (DEBUG) {
          Serial.println("No measurements from BMP280!\n");
        }
      }
    }

    if (!strcmp(THP_MODEL, "HTU21")) {
      if (checkHTU21DStatus() == true) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/temperature").c_str(), String(currentTemperature).c_str(), true);
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/humidity").c_str(), String(currentHumidity).c_str(), true);
      } else {
        if (DEBUG) {
          Serial.println("No measurements from HTU21!\n");
        }
      }
    }

    if (!strcmp(THP_MODEL, "DHT22")) {
      if (checkDHT22Status() == true) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/temperature").c_str(), String(currentTemperature).c_str(), true);
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/humidity").c_str(), String(currentHumidity).c_str(), true);
      } else {
        if (DEBUG) {
          Serial.println("No measurements from DHT22!\n");
        }
      }
    }

    if (!strcmp(THP_MODEL, "SHT1x")) {
      if (checkDHT22Status() == true) {
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/temperature").c_str(), String(currentTemperature).c_str(), true);
        mqttclient.publish(String("Smogomierz-" + String(ESP.getChipId()) + "/sensor/humidity").c_str(), String(currentHumidity).c_str(), true);
      } else {
        if (DEBUG) {
          Serial.println("No measurements from SHT1x!\n");
        }
      }
    }

    if (DEEPSLEEP_ON == true) {
      mqttclient.disconnect();
    }
  }

}

void takeTHPMeasurements() {
  if (!strcmp(THP_MODEL, "BME280")) {
    BMESensor.refresh();
    delay(10);
    if (checkBmeStatus() == true) {
      if (DEBUG) {
        Serial.println("Measurements from BME280!\n");
      }
      currentTemperature = BMESensor.temperature;
      currentPressure = BMESensor.seaLevelForAltitude(MYALTITUDE);
      currentHumidity = BMESensor.humidity;
    } else {
      if (DEBUG) {
        Serial.println("No measurements from BME280!\n");
      }
    }
  } else if (!strcmp(THP_MODEL, "HTU21")) {
    if (checkHTU21DStatus() == true) {
      if (DEBUG) {
        Serial.println("Measurements from HTU21!\n");
      }
      currentTemperature = myHTU21D.readTemperature();
      currentHumidity = myHTU21D.readHumidity();
    } else {
      if (DEBUG) {
        Serial.println("No measurements from HTU21D!\n");
      }
    }
  } else if (!strcmp(THP_MODEL, "BMP280")) {
    if (checkBmpStatus() == true) {
      if (DEBUG) {
        Serial.println("Measurements from BMP280!\n");
      }
      currentTemperature = bmp.readTemperature();
      currentPressure = (bmp.readPressure()) / 100;
    } else {
      if (DEBUG) {
        Serial.println("No measurements from BMP280!\n");
      }
    }
  } else if (!strcmp(THP_MODEL, "DHT22")) {
    if (checkDHT22Status() == true) {
      if (DEBUG) {
        Serial.println("Measurements from DHT22!\n");
      }
      currentTemperature = dht.readTemperature();
      currentHumidity = dht.readHumidity();
    } else {
      if (DEBUG) {
        Serial.println("No measurements from DHT22!\n");
      }
    }
  } else if (!strcmp(THP_MODEL, "SHT1x")) {
    if (checkSHT1xStatus() == true) {
      if (DEBUG) {
        Serial.println("Measurements from SHT1x!\n");
      }
      currentTemperature = sht1x.readTemperatureC();
      currentHumidity = sht1x.readHumidity();
    } else {
      if (DEBUG) {
        Serial.println("No measurements from SHT1x!\n");
      }
    }
  }

}

void takeNormalnPMMeasurements() {
  PmResult SDSdata = sds.queryPm();
  delay(1000);
  if (SDSdata.isOk()) {
    pmMeasurements[iPM][0] = int(calib * 0);
    pmMeasurements[iPM][1] = int(calib * SDSdata.pm25);
    pmMeasurements[iPM][2] = int(calib * SDSdata.pm10);
  } else {
    Serial.println("\nCould not read values from SDS sensor :( ");
  }
  if (DEBUG) {
    Serial.print("\n\nPM measurement number: ");
    Serial.print(iPM);
    Serial.print("\nValue of PM1: ");
    Serial.print(pmMeasurements[iPM][0]);
    Serial.print("\nValue of PM2.5: ");
    Serial.print(pmMeasurements[iPM][1]);
    Serial.print("\nValue of PM10: ");
    Serial.print(pmMeasurements[iPM][2]);
  }
  if (++iPM == NUMBEROFMEASUREMENTS) {
    averagePM();
    iPM = 0;
  }
}

void takeSleepPMMeasurements() {
  if (DEBUG) {
    Serial.print("\nTurning ON PM sensor...");
  }

  if (!strcmp(DUST_MODEL, "SDS011/21")) {
    sds.wakeup();
    sds.setQueryReportingMode().toString(); // ensures sensor is in 'query' reporting mode
    sds.setContinuousWorkingPeriod().toString(); // ensures sensor has continuous working period - default but not recommended
    unsigned long current_2sec_Millis = millis();
    previous_2sec_Millis = millis();
    while (previous_2sec_Millis - current_2sec_Millis <= TwoSec_interval * 10) {
      WebServer.handleClient();
      yield();
      previous_2sec_Millis = millis();
    }
    previous_2sec_Millis = 0;
  }

  int counterNM1 = 0;
  while (counterNM1 < NUMBEROFMEASUREMENTS) {
    unsigned long current_2sec_Millis = millis();
    if (current_2sec_Millis - previous_2sec_Millis >= TwoSec_interval) {
      PmResult SDSdata = sds.queryPm();
      delay(1000);
      takeNormalnPMMeasurements();
      counterNM1++;
      previous_2sec_Millis = millis();
    }
    WebServer.handleClient();
    yield();
    delay(10);
  }
  if (DEBUG) {
    Serial.print("\nTurning OFF PM sensor...\n");
  }

  if (!strcmp(DUST_MODEL, "SDS011/21")) {
    sds.setCustomWorkingPeriod(1);
    WorkingStateResult state = sds.sleep();
  }
}

void pm_calibration() {
  // Automatic calibration - START
  if (!strcmp(MODEL, "white")) {
    if (!strcmp(THP_MODEL, "BME280")) {
      BMESensor.refresh();
      delay(10);
      if (int(BMESensor.temperature) < 5 or int(BMESensor.humidity) > 60) {
        calib1 = float((200 - (BMESensor.humidity)) / 150);
        calib2 = calib1 / 2;
        calib = calib2;
      } else {
        calib = calib1;
      }
    } else if (!strcmp(THP_MODEL, "HTU21")) {
      if (int(myHTU21D.readTemperature()) < 5 or int(myHTU21D.readCompensatedHumidity()) > 60) {
        calib1 = float((200 - (myHTU21D.readCompensatedHumidity())) / 150);
        calib2 = calib1 / 2;
        calib = calib2;
      } else {
        calib = calib1;
      }
    } else if (!strcmp(THP_MODEL, "DHT22")) {
      if (int(dht.readTemperature()) < 5 or int(dht.readHumidity()) > 60) {
        calib1 = float((200 - (dht.readHumidity())) / 150);
        calib2 = calib1 / 2;
        calib = calib2;
      } else {
        calib = calib1;
      }

    } else if (!strcmp(THP_MODEL, "SHT1x")) {
      if (int(sht1x.readTemperatureC()) < 5 or int(sht1x.readHumidity()) > 60) {
        calib1 = float((200 - (sht1x.readHumidity())) / 150);
        calib2 = calib1 / 2;
        calib = calib2;
      } else {
        calib = calib1;
      }
    }
  }
  // Automatic calibration - END

  if (!strcmp(THP_MODEL, "BME280")) {
    calib = calib1;
  } else if (!strcmp(THP_MODEL, "HTU21")) {
    calib = calib1;
  } else if (!strcmp(THP_MODEL, "DHT22")) {
    calib = calib1;
  } else if (!strcmp(THP_MODEL, "SHT1x")) {
    calib = calib1;
  } else if (!strcmp(THP_MODEL, "BMP280")) {
    calib = calib1;
  }

}

void averagePM() {
  averagePM1 = 0;
  averagePM25 = 0;
  averagePM10 = 0;
  for (int i = 0; i < NUMBEROFMEASUREMENTS; i++) {
    averagePM1 += pmMeasurements[i][0];
    averagePM25 += pmMeasurements[i][1];
    averagePM10  += pmMeasurements[i][2];
  }
  averagePM1 = averagePM1 / NUMBEROFMEASUREMENTS;
  averagePM25 = averagePM25 / NUMBEROFMEASUREMENTS;
  averagePM10 = averagePM10 / NUMBEROFMEASUREMENTS;
  if (DEBUG) {
    Serial.print("\n\nAverage PM1: ");
    Serial.print(averagePM1);
    Serial.print("\nAverage PM2.5: ");
    Serial.print(averagePM25);
    Serial.print("\nAverage PM10: ");
    Serial.print(averagePM10);
  }
}
