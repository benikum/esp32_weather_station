// Das ist mein bisheriger Stand meines Wetterstaions projekt. Kannst du mir helfen die Fehler zu beheben?

// Arduino
#include <Arduino.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <vector>
#include <Ticker.h>
#include <sys/time.h>
#include <time.h>

// Internet
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

// Protokolle
#include <HardwareSerial.h>
#include <Wire.h>

// Sensoren
#include <BH1750.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

// Netzwerk Konfiguration
#define WIFI_SSID "ESP32-config"
#define WIFI_PASSWORD "benikumwetter" // min. 8 charachter

// Rain Sensor
#define BUCKET_SIZE 5
#define HOPPER_AREA 160

// Standard Config
#define DATABASE_SERVER "http://uploadserver.com/post-data.php"
#define UPLOAD_START "06:00"
#define UPLOAD_END "22:00"
#define INTERVAL 900
#define MAX_LOG_SIZE 1024
#define MAX_BUFFER_SIZE 96
#define TEMP_CORRECT 0
#define PRES_CORRECT 0
#define SOLAR_MULTIPLIER 4
#define BATTERY_MULTIPLIER 3

// Pin Definitionen
#define PERIPHERALS 23
#define DEBUG_JUMPER 13
#define LED_DEBUG 2
#define LED_ACTIVE 32
#define HALL_SENSOR 34
#define BATTERY_VOLTAGE 39
#define SOLAR_VOLTAGE 36
#define MODEM_RESET 5

// Strukturen
struct Measure {
  uint32_t timestamp;
  uint32_t uptime;
  float temperature;
  float pressure;
  float humidity;
  float rain;
  uint32_t light;
  uint16_t solar;
  uint16_t battery;
};
struct HttpAnswer {
  String resonse;
  int8_t code;
  unsigned long timestamp;
};

// Globale Variablen
Preferences preferences;
AsyncWebServer server(80);
DNSServer dnsServer;

BH1750 lightSensor(0x23);
Adafruit_BME280 thermoSensor;

HardwareSerial sim800l(2);
Ticker ledActiveTicker;

bool ledActiveState = true;
bool lightSensorStatus = false;
bool thermoSensorStatus = false;

RTC_DATA_ATTR uint16_t bucketTippedCounter = 0;
RTC_DATA_ATTR uint16_t measureIteration = 0;
RTC_DATA_ATTR uint32_t nextMeasureTime = 0;
RTC_DATA_ATTR uint32_t firstStartupTime = 0;
RTC_DATA_ATTR std::vector<Measure> measureBuffer;
RTC_DATA_ATTR std::vector<const char*> eventLog;

// RTC time
uint32_t getRTCtime() {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return tv_now.tv_sec;
}
uint32_t secondsSincePowerUp() {
  return getRTCtime() - firstStartupTime;
}
// bool setRTCTime() {
//   HttpAnswer answer = sendHttpGET(preferences.getString("time-url", TIME_SERVER));
// 
//   if (answer.code != 200) {
//     writeLog("setRTCTime(): Serveranfrage nicht erfolgreich: " + answer.code);
//     return false;
//   }
// 
//   String htmlTime = answer.resonse;
// 
//   int startIndex = htmlTime.indexOf("\"unixtime\":");
//   if (startIndex == -1) {
//     writeLog("setRTCTime(): 'unixtime' nicht gefunden");
//     return false;
//   }
// 
//   startIndex += strlen("\"unixtime\":");
//   int endIndex = htmlTime.indexOf(',', startIndex);
//   if (endIndex == -1) {
//     endIndex = htmlTime.indexOf('}', startIndex);
//   }
// 
//   if (endIndex == -1) {
//     writeLog("setRTCTime(): Ende des 'unixtime'-Wertes nicht gefunden");
//     return false;
//   }
// 
//   unsigned int timeDifference = (millis() - answer.timestamp) / 1000;
// 
//   htmlTime.substring(startIndex, endIndex).trim();
//   long unixTime = htmlTime.toInt() + timeDifference;
// 
//   struct timeval tv = { .tv_sec = unixTime, .tv_usec = 0 };
//   settimeofday(&tv, NULL);
//   return true;
// }
unsigned int timeStringToSeconds(String time) {
  int hour = time.substring(0, 2).toInt();
  int minute = time.substring(3, 5).toInt();
  return hour * 3600 + minute * 60;
}
bool isDayTime() {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);

  // Umrechnen der aktuellen Zeit in Sekunden seit Tagesbeginn
  struct tm *localTime = localtime(&tv_now.tv_sec);
  unsigned int secondsSinceMidnight = localTime->tm_hour * 3600 + localTime->tm_min * 60 + localTime->tm_sec;

  // Nachtzeit ist von 22:00 bis 08:00
  unsigned int dayStart = timeStringToSeconds(preferences.getString("upload-start", UPLOAD_START));    // 06:00 Uhr in Sekunden
  unsigned int dayEnd = timeStringToSeconds(preferences.getString("upload-end", UPLOAD_END));  // 22:00 Uhr in Sekunden

  // Überprüfen, ob die aktuelle Zeit in den Nachtbereich fällt
  return (secondsSinceMidnight >= dayStart && secondsSinceMidnight < dayEnd);
}
int secondsToNextMeasure() {
  return preferences.getInt("interval", INTERVAL) * measureIteration - secondsSincePowerUp();
}

// Log
size_t logSize() {
  size_t totalLength = 0;
  for (const char* str : eventLog) {
    totalLength += strlen(str);
  }
  return totalLength;
}
void writeLog(String logText, bool timeStamp = true, bool newLine = true) {
  // add timeStamp before logText
  if (timeStamp) logText = String(getRTCtime()) + ": " + logText;
  // print logText to Serial Monitor 
  newLine ? Serial.println(logText) : Serial.print(logText);
  // add & char if next text should be in same line
  if (!newLine) logText += '&';

  // check if log has space
  size_t newTextSize = strlen(logText.c_str());
  int maxLogSize = preferences.getInt("max-log-size", MAX_LOG_SIZE);
  while (logSize() + newTextSize > maxLogSize) {
    eventLog.erase(eventLog.begin());
  }
  // save new logText
  eventLog.push_back(logText.c_str());
}
String getLogs() {
  String logString = "LOG EXPORT TIME - " + String(getRTCtime()) + '\n';
  for (const auto& entry : eventLog) {
    String str = String(entry);
    if (str.charAt(str.length() - 1) == '&') {
      str.remove(str.length() - 1);
    } else {
      str += '\n';
    }
    logString += str;
  }
  return logString;
}

// Rain Sensor
void handleBucketTipped() {
  ++bucketTippedCounter;
  writeLog("Bucket tipped: " + String(bucketTippedCounter));
}
float getRainSensorValue() {
  float rainValue = bucketTippedCounter * 10 * BUCKET_SIZE / HOPPER_AREA;
  bucketTippedCounter = 0;
  return rainValue;
}

// Sensor Measure
void startSensors() {
  writeLog("Starting Sensors");

  // Stromversorgung an
  pinMode(PERIPHERALS, OUTPUT);
  digitalWrite(PERIPHERALS, HIGH);

  // ADC definieren
  pinMode(SOLAR_VOLTAGE, INPUT);
  pinMode(BATTERY_VOLTAGE, INPUT);

  delay(50);

  // BH1750 starten
  Wire.begin(21, 22);
  lightSensorStatus = lightSensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  if (!lightSensorStatus) {
    writeLog("Could not detect BH1750");
  }

  // BME280 starten
  thermoSensorStatus = thermoSensor.begin(0x76);
  if (!thermoSensorStatus) {
    writeLog("Could not detect BME280");
  }
}
Measure getRawSensorData() {
  // maybe implement lightSensor.measurementReady
  return (Measure) {
    getRTCtime(),
    secondsSincePowerUp(),
    (thermoSensorStatus ? thermoSensor.readTemperature() : 0),
    (thermoSensorStatus ? thermoSensor.readPressure() : 0),
    (thermoSensorStatus ? thermoSensor.readHumidity() : 0),
    getRainSensorValue(),
    (lightSensorStatus ? (uint32_t) lightSensor.readLightLevel() : 0),
    analogRead(SOLAR_VOLTAGE),
    analogRead(BATTERY_VOLTAGE)
  };
}
Measure applySensorCorrections(const Measure& measure) {
  return (Measure) {
    measure.timestamp,
    measure.uptime,
    measure.temperature + preferences.getInt("temperature-correct", TEMP_CORRECT),
    measure.pressure + preferences.getInt("pressure-correct", PRES_CORRECT),
    measure.humidity,
    measure.rain,
    measure.light,
    (uint16_t) map(measure.solar, 0, 4095, 0, 3280 * preferences.getInt("solar-multiplier", SOLAR_MULTIPLIER)),
    (uint16_t) map(measure.battery, 0, 4095, 0, 3280 * preferences.getInt("battery-multiplier", BATTERY_MULTIPLIER))
  };
}
String convertToHttpString(const Measure& measure) {
  // default structure
  String httpData = 
    "api_key=" + preferences.getString("api-key", "null") +
    "&device_id=" + preferences.getString("device-id", "null") +
    "&time=" + String(measure.timestamp) +
    "&uptime=" + String(measure.uptime);
    
  // optional values
  if (thermoSensorStatus) {
    httpData += "&temperature=" + String(measure.temperature, 2);
    httpData += "&pressure=" + String((measure.pressure * 100), 2);
    httpData += "&humidity=" + String(measure.humidity, 2);
  }
  httpData += "&rain=" + String(measure.rain, 2);
  if (lightSensorStatus) httpData += "&light=" + String(measure.light);
  httpData += "&solar=" + String(measure.solar);
  httpData += "&battery=" + String(measure.battery);
  if (eventLog.size() > 0) httpData += "&log=" + getLogs();
  return httpData;
}
void saveMeasureToBuffer(Measure values) {
  if (measureBuffer.size() >= MAX_BUFFER_SIZE) {
    measureBuffer.erase(measureBuffer.begin());
  }
  measureBuffer.push_back(values);
}

// SIM800L Modem
String sendSIMCommand(const char* command, unsigned int timeout = 5000, unsigned int waittime = 500, bool debugPrint = true) {
  if (debugPrint) {
    writeLog("Send   : " + String(command));
    writeLog("Receive: ", true, false);
  }
  sim800l.println(command);
  sim800l.flush();

  String response;
  unsigned long startTime = millis();
  unsigned long lastResponse = startTime + timeout;
  
  while ((millis() < startTime + timeout) && (millis() < lastResponse + waittime))  {
    if (sim800l.available()) {
      char c = sim800l.read();
      lastResponse = millis();
      // if (c == '\n') continue;
      response += c;
    }
  }
  if (debugPrint) {
    writeLog(response, false, true);
  }
  return response;
}
bool isBatteryConnected() {
  return (analogRead(BATTERY_VOLTAGE) > 600);
}
bool isSIMModuleResponding() {
  return (sendSIMCommand("AT").indexOf("OK") != -1);
}
bool isSIMModuleRegistered() {
  String response = sendSIMCommand("AT+CREG?", 5000, 100, false);
  return (response.indexOf("0,1") != -1 || response.indexOf("0,5") != -1);
}
void startSIMModule() {
  writeLog("Starting Serial connection to SIM800L");

  if (!isBatteryConnected()) {
    writeLog("startSIMModule(): battery n/c - internet not possible");    
    return;
  }

  // Maybe still or already connected
  if (isSIMModuleRegistered()) {
    writeLog("(already registered)", false, true);
    return;
  }

  pinMode(MODEM_RESET, OUTPUT);
  sim800l.begin(9600, SERIAL_8N1, 16, 17);
  while (!sim800l);

  // Try to wake up
  sim800l.println("AT");
  delay(50);
  sim800l.println("AT+CFUN=1,1");
  delay(200);

  if (!isSIMModuleResponding()) {
    writeLog("startSIMModule(): no response");
    return;
  }

  writeLog("waiting for registration", true, false);
  delay(20000);

  int breakpoint = 0;
  while (breakpoint < 12) {
    ++breakpoint;
    if (isSIMModuleRegistered()) {
      writeLog("(registered)", false, true);
      return;
    } else {
      writeLog(".", false, false);
      delay(5000);
    }
  }
  writeLog("(failed)", false, true);
}
int8_t waitForNextResponse(unsigned int timeout) {
  String response;
  unsigned long startTime = millis();
  unsigned long lastResponse = startTime + timeout;
  
  while ((millis() < startTime + timeout) && (millis() < lastResponse + 500)) {
    if (sim800l.available()) {
      char c = sim800l.read();
      response += c;
    }
    if (response.indexOf("+HTTPACTION:") != -1) {
      lastResponse = millis();
    }
  }

  int commaIndex = response.indexOf(",");
  if (commaIndex == -1) {
    writeLog("waitForNextResponse(): comma not found");
    return -1;
  }
  return response.substring(commaIndex + 1, commaIndex + 3).toInt();
}
void enableGPRS() {
  sendSIMCommand("AT+SAPBR=3,1,CONTYPE,GPRS");                   // Content Type setzen
  sendSIMCommand("AT+SAPBR=3,1,APN,\"internet\"");               // Access Point Name setzen
  // sendSIMCommand("AT+SAPBR=3,1,USER,\"Username\"");              // Username
  // sendSIMCommand("AT+SAPBR=3,1,PWD,\"Password\"");               // Password
  sendSIMCommand("AT+SAPBR=1,1");                                // GPRS aktivieren (schnell blinken)
}
void disableGPRS() {
  sendSIMCommand("AT+SAPBR=0,1");                                // GPRS ausschalten
}
HttpAnswer sendHttpRequest(String url, String data = "") {
  if (!isSIMModuleRegistered) {
    writeLog("sendHttpRequest(): modem not registered");
    return (HttpAnswer) {"", -1, millis()};
  }
  sendSIMCommand("AT+HTTPINIT");                                 // HTTP Sitzung aktivieren
  sendSIMCommand("AT+HTTPPARA=CID,1");                           // Trägerprofil definieren
  sendSIMCommand(("AT+HTTPPARA=URL,\"" + url + "\"").c_str());                    // URL
  sendSIMCommand("AT+HTTPPARA=CONTENT,application/x-www-form-urlencoded");        // Datenformat
  if (data != "") {
    sendSIMCommand(("AT+HTTPDATA=" + String(data.length()) + ",10000").c_str());  // Datenmenge 
    sendSIMCommand(data.c_str());                                                 // Daten setzen
  }
  sendSIMCommand(data == "" ? "AT+HTTPACTION=1" : "AT+HTTPACTION=0");             // HTTP Anfrage senden

  unsigned long httpPostTime = millis();                         // Zeit speichern
  int8_t code = waitForNextResponse(60000);                   // Auf Antwort warten
  String response = sendSIMCommand("AT+HTTPREAD");               // Empfangene Daten auslesen

  sendSIMCommand("AT+HTTPTERM");                                 // HTTP Sitzung beenden
  return (HttpAnswer) {response, code, httpPostTime};
}
bool uploadMeasureToDatabase(Measure values) {
  String httpData = convertToHttpString(values);
  HttpAnswer uploadAnwser;
  
  for (int i = 0; i < 5; i++) {
    uploadAnwser = sendHttpRequest(preferences.getString("server-url", DATABASE_SERVER), httpData);
    writeLog(String(uploadAnwser.code) + uploadAnwser.resonse);

    if (uploadAnwser.code != -1) {
      if (uploadAnwser.code == 200) {
        writeLog("(success)");
        eventLog.clear();
        return true;
      }
      break;
    }
    delay(2000);
  }
  writeLog("(failed)");
  return false;
}

// Konfiguration
void startWebServer() {
  const char* htmlPage = R"=====(<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Setup Konfiguration</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 0;
        display: flex; flex-direction: column; align-items: center;
        justify-content: flex-start; height: 100vh; width: 100vw;
        overflow-x: hidden; background-color: #eee; }

        .container { width: 80%; padding: 20px; max-width: 600px;
        box-shadow: 0 0 10px #333; background-color: #fff; }

        h1 { text-align: center; font-size: 34px; margin: 20px 0; }

        h2 { font-size: 28px; margin: 20px 0 10px; }
        
        label, a { display: block; font-size: 24px; margin: 5px 0; }
        
        a { text-decoration: none; }
        
        input, textarea { width: calc(100% - 22px); padding: 10px;
        margin-bottom: 20px; font-size: 22px; border: 1px solid #ccc;
        border-radius: 5px; }
        
        button { width: 100%; margin: 20px 0 40px; padding: 10px; font-size: 28px;
        background-color: #02f; color: #fff; border: none; border-radius: 5px;
        cursor: pointer; }
        
        button:hover { background-color: #02c; }

        #restart-send { background-color: #f20; }
        
        #restart-send:hover { background-color: #c20; }
        
    </style>
</head>
<body>
    <div class="container">
        <h1>Setup Konfiguration</h1>
        <h2>Gerätedaten</h2>
        <label for="device-id">Geräte ID</label>
        <input type="text" id="device-id" maxlength="16" placeholder="esp32-wetter" required>
        <label for="buffer-size">Maximale Buffer-Größe (Einträge)</label>
        <input type="number" id="buffer-size" min="1" max="256" value="%BUFFER_SIZE%" required>
        <label for="log-size">Maximale Log-Größe (Zeichen)</label>
        <input type="number" id="log-size" min="64" max="4096" value="%LOG_SIZE%" required>
        
        <h2>Datenkorrekturen</h2>
        <label for="pressure">Luftdruck (hPa)</label>
        <input type="number" id="pressure" min="0" max="2000" placeholder="%CURRENT_PRESSURE%" step="0.01">
        <label for="temperature">Temperatur (°C)</label>
        <input type="number" id="temperature" min="-100" max="100" placeholder="%CURRENT_TEMPERATURE%" step="0.01">
        <label for="humidity">Luftfeuchte (%)</label>
        <input type="number" id="humidity" min="-100" max="100" placeholder="%CURRENT_HUMIDITY%" step="0.01">
        <label for="light">Lichtintensität (lx)</label>
        <input type="number" id="light" min="0" max="65536" placeholder="%CURRENT_LIGHT%" step="0.01">
        <label for="solar">Solar Spannung (mV)</label>
        <input type="number" id="solar" min="0" max="10000" placeholder="%CURRENT_SOLAR%">
        <label for="battery">Batterie Spannung (mV)</label>
        <input type="number" id="battery" min="0" max="10000" placeholder="%CURRENT_BATTERY%">
        
        <h2>Sensorparameter</h2>
        <label for="hopper-area">Trichter Fläche (cm²)</label>
        <input type="number" id="hopper-area" min="0" max="10000" value="%HOPPER_AREA%" step="0.01" required>
        <label for="bucket-size">Wippe Größe (ml)</label>
        <input type="number" id="bucket-size" min="0" max="10000" value="%BUCKET_SIZE%" step="0.01" required>
        
        <h2>Netzwerk</h2>
        <label for="server-url">Server URL</label>
        <input type="url" id="server-url" maxlength="200" value="%SERVER_URL%" required>
        <label for="api-key">API Schlüssel</label>
        <input type="text" id="api-key" maxlength="100" value="%API_KEY%" required>
        
        <h2>Upload</h2>
        <label for="interval">Intervall (s)</label>
        <input type="number" id="interval" min="60" max="86400" value="%INTERVAL%" required>
        <label for="upload-start">Upload Zeit von</label>
        <input type="time" id="upload-start" value="%UPLOAD_START%" required>
        <label for="upload-end">Upload Zeit bis</label>
        <input type="time" id="upload-end" value="%UPLOAD_END%" required>
        
        <button type="button" id="config-send" onclick="sendConfig()">Konfiguration senden</button>
        <textarea id="config-response" placeholder="Antwort von ESP32..." style="display: none;" readonly></textarea>
        
        <h1>Systemzeit Einstellen</h1>
        <label>Aktualisiert die RTC Zeit auf die Systemzeit deines Geräts</label>
        <button type="button" id="rtc-send" onclick="sendTime()">Zeit aktualisieren</button>
        <textarea id="rtc-response" placeholder="Antwort von ESP32..." style="display: none;" readonly></textarea>
        
        <h1>Modem Befehle</h1>
        <label>Sendet Befehle an das eingebaute Modem</label>
        <input type="text" id="modem-command" maxlength="100" value="AT">
        <button type="button" id="modem-send" onclick="sendModemCommand()">Befehl senden</button>
        <textarea id="modem-response" placeholder="Antwort des Modems..." style="display: none;" readonly></textarea>
        
        <h1>Webserver ausschalten</h1>
        <button type="button" id="restart-send" onclick="fetch('/restart')">Webserver beenden und ESP32 neu starten</button>
        
        <h1>About</h1>
        <label id="creator">Creator: benikum</label>
        <label id="sw-version">Software Version: 1</label>
    </div>
    <script>
        function sendConfig() {
            const params = new URLSearchParams({
                "device-id": document.getElementById("device-id").value,
                "buffer-size": document.getElementById("buffer-size").value,
                "log-size": document.getElementById("log-size").value,
                "pressure": document.getElementById("pressure").value,
                "temperature": document.getElementById("temperature").value,
                "humidity": document.getElementById("humidity").value,
                "light": document.getElementById("light").value,
                "solar": document.getElementById("solar").value,
                "battery": document.getElementById("battery").value,
                "hopper-area": document.getElementById("hopper-area").value,
                "bucket-size": document.getElementById("bucket-size").value,
                "server-url": document.getElementById("server-url").value,
                "api-key": document.getElementById("api-key").value,
                "interval": document.getElementById("interval").value,
                "upload-start": document.getElementById("upload-start").value,
                "upload-end": document.getElementById("upload-end").value,
            });
            
            var sendButton = document.getElementById("config-send");
            var responseText = document.getElementById("config-response");

            sendButton.disabled = true;
            responseText.style.display = "block";
            
            fetch("/submit?" + params.toString())
                .then(response => response.text())
                .then(text => {
                    responseText.rows = Math.min(text.split("\n").length, 10);
                    responseText.value = text;
                })
                .catch(error => {
                    console.error("Error:", error);
                    alert("Fehler beim Übermitteln der Konfiguration.");
                })
                .finally(() => {
                    sendButton.disabled = false;
                });
        }

        function sendTime() {
            const timestamp = Math.floor(Date.now() / 1000); // Unix Timestamp in Sekunden

            var sendButton = document.getElementById("rtc-send");
            var responseText = document.getElementById("rtc-response");

            sendButton.disabled = true;
            responseText.style.display = "block";

            fetch("/set-time?timestamp=" + timestamp.toString())
                .then(response => response.text())
                .then(text => {
                    responseText.rows = Math.min(text.split("\n").length, 10);
                    responseText.value = text;
                })
                .catch(error => {
                    console.error("Error:", error);
                    alert("Fehler beim Senden der Zeit.");
                })
                .finally(() => {
                    sendButton.disabled = false;
                });
        }

        function sendModemCommand() {
            var command = document.getElementById("modem-command").value.trim();
            if (command === "") return;
            
            var sendButton = document.getElementById("modem-send");
            var responseText = document.getElementById("modem-response");
            
            sendButton.disabled = true;
            responseText.style.display = "block";
            
            fetch("/modem?command=" + encodeURIComponent(command))
                .then(response => response.text())
                .then(text => {
                    responseText.rows = Math.min(text.split("\n").length, 10);
                    responseText.value = text;
                })
                .catch(error => {
                    console.error("Error:", error);
                    alert("Fehler beim Senden des Modem-Befehls.");
                })
                .finally(() => {
                    sendButton.disabled = false;
                });
        }
    </script>
</body>
</html>)=====";

  ledActiveTicker.attach_ms(300, []() {
    ledActiveState = !ledActiveState;
    digitalWrite(LED_ACTIVE, ledActiveState ? HIGH : LOW);
  });

  WiFi.softAPConfig(IPAddress(4, 3, 2, 1), IPAddress(4, 3, 2, 1), IPAddress(255, 255, 255, 0));

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  writeLog("WLAN AP gestartet");
  writeLog("IP Adresse: " + WiFi.softAPIP().toString());
  
  dnsServer.start(53, "*", WiFi.softAPIP());

  Measure currentMeasure;

  // https://github.com/CDFER/Captive-Portal-ESP32

	// server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });	// windows 11 captive portal workaround
	// server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });								// Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the esp32 :)

	// server.on("/generate_204", [](AsyncWebServerRequest *request) { request->redirect("http://" + WiFi.softAPIP().toString()); });		   // android captive portal redirect
	// server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request) { request->redirect("http://" + WiFi.softAPIP().toString()); });  // apple call home
	// server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect("http://" + WiFi.softAPIP().toString()); });			   // windows call home

	// server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); });	// webpage icon

  server.on("/", HTTP_ANY, [htmlPage, &currentMeasure](AsyncWebServerRequest *request) {
    currentMeasure = getRawSensorData();

    String page = htmlPage;

    page.replace("%LOG_SIZE%", String(preferences.getInt("max-log-size", MAX_BUFFER_SIZE)));
    page.replace("%LOG_SIZE%", String(preferences.getInt("max-log-size", MAX_LOG_SIZE)));

    page.replace("%CURRENT_PRESSURE%", String(currentMeasure.pressure / 100));
    page.replace("%CURRENT_TEMPERATURE%", String(currentMeasure.temperature));
    page.replace("%CURRENT_HUMIDITY%", String(currentMeasure.humidity));
    page.replace("%CURRENT_LIGHT%", String(currentMeasure.light));
    page.replace("%CURRENT_SOLAR%", String(currentMeasure.solar));
    page.replace("%CURRENT_BATTERY%", String(currentMeasure.battery));

    page.replace("%HOPPER_AREA%", String(preferences.getInt("hopper-area", HOPPER_AREA)));
    page.replace("%BUCKET_SIZE%", String(preferences.getInt("bucket-size", BUCKET_SIZE)));

    page.replace("%SERVER_URL%", preferences.getString("server-url", DATABASE_SERVER));
    page.replace("%API_KEY%", preferences.getString("api-key", ""));

    page.replace("%INTERVAL%", String(preferences.getInt("interval", INTERVAL)));
    page.replace("%UPLOAD_START%", preferences.getString("upload-start", UPLOAD_START));
    page.replace("%UPLOAD_END%", preferences.getString("upload-end", UPLOAD_END));

    request->send(200, "text/html", page);
  });

  server.on("/submit", HTTP_GET, [&currentMeasure](AsyncWebServerRequest *request) {
    // optional configs
    if (request->hasArg("pressure")) {
      String pressure = request->arg("pressure");
      preferences.putFloat("pressure-correct", currentMeasure.pressure - pressure.toFloat());
    } if (request->hasArg("temperature")) {
      String temperature = request->arg("temperature");
      preferences.putFloat("temperature-correct", currentMeasure.temperature - temperature.toFloat());
    } if (request->hasArg("humidity")) {
      String humidity = request->arg("humidity");
      preferences.putFloat("humidity-correct", currentMeasure.humidity - humidity.toFloat());
    } if (request->hasArg("light")) {
      String light = request->arg("light");
      preferences.putFloat("light-correct", currentMeasure.light - light.toFloat());
    } if (request->hasArg("solar")) {
      String solar = request->arg("solar");
      preferences.putFloat("solar-multiplier", solar.toFloat() / currentMeasure.solar);
    } if (request->hasArg("battery")) {
      String battery = request->arg("battery");
      preferences.putFloat("battery-multiplier", battery.toFloat() / currentMeasure.battery);
    }
    // required configs
    if (!(request->hasArg("device-id") &&
        request->hasArg("buffer-size") &&
        request->hasArg("log-size") &&
        request->hasArg("hopper-area") &&
        request->hasArg("bucket-size") &&
        request->hasArg("server-url") &&
        request->hasArg("api-key") &&
        request->hasArg("interval") &&
        request->hasArg("upload-start") &&
        request->hasArg("upload-end"))) {
      request->send(400, "text/plain", "ERROR: Required data not provided");
      return;
    }
    
    String deviceId = request->arg("device-id");
    String maxBufferSize = request->arg("buffer-size");
    String maxLogSize = request->arg("log-size");

    String hopperArea = request->arg("hopper-area");
    String bucketSize = request->arg("bucket-size");

    String serverUrl = request->arg("server-url");
    String apiKey = request->arg("api-key");

    String interval = request->arg("interval");
    String uploadStart = request->arg("upload-start");
    String uploadEnd = request->arg("upload-end");

    preferences.putString("device-id", deviceId);
    preferences.putInt("max-buffer-size", maxBufferSize.toInt());
    preferences.putInt("max-log-size", maxLogSize.toInt());

    preferences.putInt("hopper-area", hopperArea.toInt());
    preferences.putInt("bucket-size", bucketSize.toInt());

    preferences.putString("server-url", serverUrl);
    preferences.putString("api-key", apiKey);

    preferences.putInt("interval", interval.toInt());
    preferences.putString("upload-start", uploadStart);
    preferences.putString("upload-end", uploadEnd);

    writeLog("--> Konfiguration gespeichert! <--");
    request->send(200, "text/Plain", "SUCCESS: Saved config");
  });

  server.on("/modem", HTTP_GET, [](AsyncWebServerRequest *request) {
    const char* command = request->arg("command").c_str();
    writeLog("(webserver) " + request->client()->remoteIP().toString() + " issued modem command: " + String(command));
    String response = sendSIMCommand(command);
    request->send(200, "text/plain", response);
  });
	
  server.on("/set-time", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("timestamp")) {
      const long unixTime = request->arg("timestamp").toInt();

      if (unixTime > 0) {
        writeLog("(webserver) " + request->client()->remoteIP().toString() + " updated system time to: " + String(unixTime) + "s, before: " + String(getRTCtime()));

        struct timeval tv = { .tv_sec = unixTime, .tv_usec = 0 };
        settimeofday(&tv, NULL);

        request->send(200, "text/plain", "Timestamp received");
      } else {
        request->send(400, "text/plain", "Invalid timestamp");
      }
    } else {
      request->send(400, "text/plain", "Timestamp not provided");
    }
  });

  server.on("restart", HTTP_ANY, [](AsyncWebServerRequest *request) {
      request->send(200);
      delay(2000);
      writeLog("RESTART");
      Serial.flush();
      ESP.restart();
  });

  server.onNotFound([](AsyncWebServerRequest *request) { request->redirect("http://" + WiFi.softAPIP().toString()); });

  server.begin();
  writeLog("Server gestartet");
}
void enableDeepSleep() {
  disableGPRS();
  int nextMeasure = secondsToNextMeasure();
  long sleepTime = (nextMeasure - 60);

  if (sleepTime <= 60) {
    if (sleepTime <= 10) sleepTime = 10;
    writeLog("Short deep sleep enabled: " + String(sleepTime) + "s");
  } else {
    sendSIMCommand("AT+CFUN=0"); // Minimalen Modus anschalten
    digitalWrite(PERIPHERALS, LOW); // Sensoren ausschalten

    writeLog("Deep sleep enabled: " + String(sleepTime) + "s");
  }

  Serial.flush();
  
  esp_sleep_enable_timer_wakeup(sleepTime * 1000000);
  esp_sleep_enable_ext0_wakeup((gpio_num_t) HALL_SENSOR, digitalRead(HALL_SENSOR) ? LOW : HIGH);
  esp_deep_sleep_start();
}

// SETUP
void setup() {
  preferences.begin("setup", false);

  // woke up because rain sensor tipped
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    handleBucketTipped();
    enableDeepSleep();
    return;
  }

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), handleBucketTipped, CHANGE);

  // Starte Serielle Verbindung
  Serial.begin(115200);
  while (!Serial);

  // Definiere LEDs
  pinMode(LED_ACTIVE, OUTPUT);
  pinMode(LED_DEBUG, OUTPUT);

  // turn on red exterior LED
  digitalWrite(LED_ACTIVE, HIGH);
  digitalWrite(LED_DEBUG, HIGH);
  
  // Starte Module
  startSensors();
  startSIMModule();

  // Debug
  pinMode(DEBUG_JUMPER, INPUT);
  if (!digitalRead(DEBUG_JUMPER)) {
    startWebServer();
    unsigned long startTime = millis();
    while (millis() < startTime + 600000) {
      dnsServer.processNextRequest();
      delay(10); // Slow down for efficiency
    }
    writeLog("Abandoned config web server because it took too long!");
    writeLog("RESTART");
    Serial.flush();
    ESP.restart();
  }
  digitalWrite(LED_DEBUG, LOW);
  
  // Warte auf naechste Messzeit falls das noch dauert
  int waitTime = secondsToNextMeasure();
  if (waitTime > 0) {
    writeLog("Wait for next measure time: " + String(waitTime));
    delay(waitTime * 1000);
  }

  writeLog("Measure Iteration: " + String(measureIteration));
  ++measureIteration;

  // Read Sensors
  Measure rawValues = getRawSensorData();
  Measure currentValues = applySensorCorrections(rawValues);

  if (isDayTime()) {
    writeLog("Try to upload " + String(measureBuffer.size()) + " measures (max 5 tries per m.)");

    digitalWrite(PERIPHERALS, LOW); // deactivate sensors earlier
    enableGPRS();
    if (uploadMeasureToDatabase(currentValues)) {
      // upload buffered measures
      for (Measure bufferMeasure : measureBuffer) {
        uploadMeasureToDatabase(bufferMeasure);
      }
    } else {
      saveMeasureToBuffer(currentValues);
    }
  } else {
    saveMeasureToBuffer(currentValues);
  }
  enableDeepSleep();
}

void loop() {
  enableDeepSleep();
}