/*
 *  
 *  translation file - english translation
 *  
 */

#pragma once

char EN_INTL_LANG[] PROGMEM = "en";

char EN_INTL_INDEX_PAGE[] PROGMEM = "Measurements";
char EN_INTL_CONFIG_PAGE[] PROGMEM = "Configuration";
char EN_INTL_UPDATE_PAGE[] PROGMEM = "Update";

char EN_INTL_WEATHER[] PROGMEM = "Weather";
char EN_INTL_TEMPERATURE[] PROGMEM = "Temperature";
char EN_INTL_HUMIDITY[] PROGMEM = "Humidity";
char EN_INTL_PRESSURE[] PROGMEM = "Pressure";
char EN_INTL_DEWPOINT[] PROGMEM = "Dewpoint";

char EN_INTL_AIRPOLLUTION[] PROGMEM = "Air pollution";
char EN_INTL_SAVED[] PROGMEM = "SAVED!";

char EN_INTL_POSTCONFIG_INFO[] PROGMEM = "everything looks OK, in a moment the Smogomierz will restart";
char EN_INTL_INSTRUCIONSLINK[] PROGMEM = "All instructions and descriptions[in polish] are available {GITHUB_LINK}.";
char EN_INTL_DEVICENAME[] PROGMEM = "Device Name";
char EN_INTL_DEVICENAMEAUTO[] PROGMEM = "Automatic name generation";
char EN_INTL_SELECTEDLANGUAGE[] PROGMEM = "Language";
char EN_INTL_TEMPHUMIPRESSSENSOR[] PROGMEM = "Temp/Humi/Press Sensor";
char EN_INTL_PMSENSOR[] PROGMEM = "PM2.5/PM10 Sensor";

char EN_INTL_FREQUENTMEASUREMENTONOFF[] PROGMEM = "Frequent measurement";
char EN_INTL_FREQUENTMEASUREMENTINFO[] PROGMEM = "frequent measurements - every few seconds, shorten the life span of the PM sensor.";
char EN_INTL_MEASUREMENTFREQUENCY[] PROGMEM = "Make PM measurements every";
char EN_INTL_AVERAGELASTRESULT[] PROGMEM = "Average result from last";
char EN_INTL_PMMEASUREMENTS[] PROGMEM = "PM measurements";
char EN_INTL_SENDINGINTERVAL[] PROGMEM = "Sending measurements to external services every";
char EN_INTL_SECONDS[] PROGMEM = "seconds";
char EN_INTL_MINUTES[] PROGMEM = "minutes";

char EN_INTL_DEEPSLEEPINFO[] PROGMEM = "DeepSleep put the device into deep sleep between a series of measurements - significantly lower energy consumption (the possibility of working on the battery for several weeks), but lack of continuous access to the web interface. The web interface will only be available for about {INTERFACEWWWONTIME} seconds every {SENDING_FREQUENCY} minutes (and right after the device reboots). Sleep time will be the same as the period for sending measurements to external services. DeepSleep requires connection of D0 and RST pins on ESP8266!";

char EN_INTL_DISPLAYPM1[] PROGMEM = "Display of PM1 measurements";
char EN_INTL_ALTITUDEINFO[] PROGMEM = "Altitude above sea level (required for correct pressure measurements. You can check it {WSPOLRZEDNE_GPS_LINK})";

char EN_INTL_SECURECONFIGUPDATEPAGE[] PROGMEM = "Secure the Configuration and Update pages(default: admin/password)";
char EN_INTL_SECURELOGIN[] PROGMEM = "Login";
char EN_INTL_SECUREPASSWD[] PROGMEM = "Password";
char EN_INTL_SECURELOGOUTINFO[] PROGMEM = "Restart your web browser to log out!";

char EN_INTL_LUFTDATENSENDING[] PROGMEM = "Sending data to the {LUFTDATEN_LINK} service(requires filling out {LUFTDATENFORM_LINK})"; 

char EN_INTL_AIRMONITORSENDING[] PROGMEM = "Sending data to the {AIRMONITOR_LINK} service(requires filling out {AIRMONITORFORM_LINK}; Sensor: e.g. HPMA115S0)";
char EN_INTL_AIRMONITORCHARTS[] PROGMEM = "Displaying charts from the AirMonitor site";
char EN_INTL_AIRMONITORCOORDINATESINFO[] PROGMEM = "Geographical coordinates(you can check it {LATLONG_LINK}";
char EN_INTL_AIRMONITORLATITUDE[] PROGMEM = "Latitude";
char EN_INTL_AIRMONITORLONGITUDE[] PROGMEM = "Longitude";

char EN_INTL_THINGSPEAKSENDING[] PROGMEM = "Sending data to the {THINGSPEAK_LINK} service:";
char EN_INTL_THINGSPEAKCHARTS[] PROGMEM = "Displaying charts from the ThingSpeak site";
char EN_INTL_THINGSPEAKAPIKEY[] PROGMEM = "ThingSpeak API_KEY";
char EN_INTL_THINGSPEAKCHANNELID[] PROGMEM = "ThingSpeak Channel ID";

char EN_INTL_INFLUXDBSENDING[] PROGMEM = "Sending data to the InfluxDB";
char EN_INTL_INFLUXDBSERVER[] PROGMEM = "InfluxDB database address";
char EN_INTL_INFLUXDBPORT[] PROGMEM = "InfluxDB port";
char EN_INTL_INFLUXDBNAME[] PROGMEM = "Name of the database";
char EN_INTL_INFLUXDBUSER[] PROGMEM = "Database user";
char EN_INTL_INFLUXDBPASSWD[] PROGMEM = "Database password";

char EN_INTL_MQTTSENDING[] PROGMEM = "Sending data to the MQTT server";
char EN_INTL_MQTTSERVER[] PROGMEM = "MQTT server address";
char EN_INTL_MQTTPORT[] PROGMEM = "MQTT port";
char EN_INTL_MQTTUSER[] PROGMEM = "MQTT user";
char EN_INTL_MQTTPASSWD[] PROGMEM = "MQTT password";

char EN_INTL_CALIBMETHOD[] PROGMEM = "Calibration method";
char EN_INTL_CALIB1[] PROGMEM = "calib1";
char EN_INTL_CALIB2[] PROGMEM = "calib2";
char EN_INTL_SOFTWATEVERSION[] PROGMEM = "Software version";

char EN_INTL_ERASEWIFICONFIG[] PROGMEM = "Erase WiFi Config";
char EN_INTL_RESTORESETTINGS[] PROGMEM = "Restore default settings";
char EN_INTL_SAVE[] PROGMEM = "Save";
char EN_INTL_YES[] PROGMEM = "Yes";
char EN_INTL_NO[] PROGMEM = "No";
char EN_INTL_WITHOUTSENSOR[] PROGMEM = "Without sensor";
char EN_INTL_WITHOUTCALIBRATION[] PROGMEM = "Without calibration";
char EN_INTL_AUTOMATICCALIBRATION[] PROGMEM = "Automatic calibration";

char EN_INTL_INTL_EN[] PROGMEM = "english";
char EN_INTL_INTL_PL[] PROGMEM = "polish";

char EN_INTL_FWUPDATEAVALIBLE[] PROGMEM = "Firmware update available!";
char EN_INTL_AUTOUPDATEON[] PROGMEM = "Autoupdate";
char EN_INTL_MANUALUPDATEBUTTON[] PROGMEM = "Manual Upgrade";
char EN_INTL_FWUPDATEBUTTON[] PROGMEM = "Firmware Upgrade";
char EN_INTL_AUTOUPDATEONBUTTON[] PROGMEM = "Autoupdate On";
char EN_INTL_AUTOUPDATEWARNING[] PROGMEM = "Automatic updates are performed via an unencrypted connection (HTTP, not HTTPS). It can be dangerous and allow interception of packets and hacking the device. You can use the Autoupdate once by clicking the \"{TEXT_FWUPDATEBUTTON}\" button or turn on Autoupdate permanently. You can change your mind at any time and disable Autoupdate in the Configuration. After selecting \"{TEXT_FWUPDATEBUTTON}\" or turning on Autoupdate, the device will save the current configuration and restart. It may take several seconds.";
char EN_INTL_UPDATEPAGEAUTOUPDATEWARNING[] PROGMEM = "Automatic updates are performed via an unencrypted connection (HTTP, not HTTPS). It can be dangerous and allow interception of packets and hacking the device.";

char EN_INTL_SELECTUPDATEFILE[] PROGMEM = "Select .bin file";
char EN_INTL_SUBMITUPDATE[] PROGMEM = "Update!";
char EN_INTL_CURRENTSOFTVERSION[] PROGMEM = "The currently used version of the firmware";
char EN_INTL_SERVERSOFTWAREVERSION[] PROGMEM = "The latest available firmware version";
char EN_INTL_LATESTAVAILABLESOFT[] PROGMEM = "Latest version of the software available {SMOGOMIERZRELEASES_LINK}.";

char EN_INTL_HERE[] PROGMEM = "here";
char EN_INTL_THEFORM[] PROGMEM = "the form";