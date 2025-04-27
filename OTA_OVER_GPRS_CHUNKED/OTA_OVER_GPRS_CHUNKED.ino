/*
  Project Name: Performing OTA Over GPRS Using Chunked Transfer Encoding
  Author: Barkın Sarıkartal
  Date: 26 April 2025
  Description:
    This project is written for ESP32-S3-DevKitC-1 and Waveshare's SIM7600X 4G Module
    to perform OTA (Over the Air) updates by fetching a bin file from a server using
    GPRS without Wi-Fi.

    Use this code if your server responds with HTTP header "Transer-Encoding: Chunked".
    But if your server responds with HTTP header "Content-Length", then use the other
    fetching bin file code in this repository (OTA_OVER_GPRS.ino).

    Sadly, chunked transfer encodings can't have Content-Length headers.
    So in my case, to verify integrity of the bin file, I have a link
    that looks like "yourserver.com/yourCRC32path" that returns a JSON
    like {"Hash": "d7c1c20a"} that has the CRC32 value of my bin file
    in the server and the code is built on this system.

    You need to customize your partition scheme in order to be able to fetch and save
    your bin file to your SPIFFS. I prefer "8M with spiffs 3MB APP/1.5 MB SPIFFS"
    partition scheme.

  Arduino IDE Version: 2.3.4
  ESP32 Board Package Version: 3.0.1
  ESP32 Variant: ESP32-S3-DevKitC-1 N16R8 (16MB Flash, 8MB PSRAM)
  GitHub: https://github.com/barkinsarikartal/ESP32-and-Waveshare-SIM7600X-4G-Module
  License: MIT License
*/

#define TINY_GSM_MODEM_SIM7600
#define UART_BAUD 115200
#define SerialAT Serial1
#define MODEM_TX 16 // Connect SIM Module's TX pin to ESP32's GPIO16
#define MODEM_RX 15 // Connect SIM Module's RX pin to ESP32's GPIO15

#include <ArduinoHttpClient.h>  // Version 0.6.0, by Arduino
#include <SoftwareSerial.h>     // Comes with Arduino IDE
#include <TinyGsmClient.h>      // Version 0.12.0, by Volodymyr Shymanskyy
#include <ArduinoJson.h>        // Version 7.3.0, by Benoit Blanchon
#include <SSLClient.h>          // Version 1.3.2, by V Govorovski, used for SSL connections
#include <Arduino.h>            // Comes with ESP32 Board Package
#include <SPIFFS.h>             // Comes with ESP32 Board Package
#include <CRC32.h>              // Version 2.0.0, by Christopher Baker, used for calculating CRC32 value of bin file
#include "esp_ota_ops.h"        // Comes with ESP32 Board Package

const char apn[]  = "internet"; // Enter your own sim card's apn.
const char user[] = "";
const char pass[] = "";
const char* updateFileName = "/firmware.bin"; // File name on SPIFFS
const char* server = "yourserver.com"; // The domain name of the server hosting the bin file
const char* fileAddressPath = "/yourbinfile"; // The specific path on the server where the bin file can be accessed
const char* crc32AddressPath = "/yourCRC32path"; // The specific path on the server where the CRC32 value of the bin file can be accessed
// const char* bearerToken = "yourbearertoken"; // Uncomment if you are using a bearer token

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
SSLClient secure_layer(&client);

void setup() {
  Serial.begin(115200);
  delay(10);

  // Starting SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to initialize SPIFFS.");
    delay(50);
    ESP.restart();
  }

  // Checking if this file already exists
  if (SPIFFS.exists(updateFileName)) {
    if (SPIFFS.remove(updateFileName)) {
      Serial.println("Deleted " + String(updateFileName) + " from SPIFFS.");
    }
  }
  
  // Printing SPIFFS information
  size_t totalSPIFFSBytes = SPIFFS.totalBytes();
  size_t usedSPIFFSBytes = SPIFFS.usedBytes();
  Serial.println("SPIFFS Information:");
  Serial.printf("%-6s: %10u bytes\n", "Total", totalSPIFFSBytes);
  Serial.printf("%-6s: %10u bytes\n", "Used", usedSPIFFSBytes);
  Serial.printf("%-6s: %10u bytes\n", "Empty", totalSPIFFSBytes - usedSPIFFSBytes);
  delay(3000);
  
  if (SIMStartFunction()) { // Connecting to GPRS
    Serial.println("--------------------");

    String expectedCRC = GetOTACRC32();
    if (expectedCRC.isEmpty()) return;

    if (FetchAndSaveBin()) { // Fetching the bin file
      if (VerifyCRC(expectedCRC)) { // Verifying the integrity of the fetched file
        if (PerformOTA()) { // Performing OTA if new bin file is fetched
          SPIFFS.end(); // Closing SPIFFS for safe restart
          delay(2000);
          ESP.restart(); // Restarting ESP32 to boot with new bin file
        }
      }
      else {
        Serial.println("CRC32 Values Are Not Matching.");
      }
    }
  }
}

void loop() {
 // Empty loop
}

bool SIMStartFunction() { // Function to connect SIM Module to GPRS
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(100);
  if(modem.restart()){
    if(modem.waitForNetwork()){
      Serial.println("SIM Module Network Pass.");
      if(modem.gprsConnect(apn, user, pass)) {
        Serial.println("SIM Module GPRS Pass.");
        String modemIP = modem.getLocalIP();
        Serial.println("Modem IP: " + modemIP);
        int signalQuality = modem.getSignalQuality();
        Serial.println("Signal Quality: " + String(signalQuality));
        return true;
      }
      else{
        Serial.println("Couldn't connect to GPRS.");
        return false;
      }
    }
    else{
      Serial.println("Couldn't connect to Network.");
      return false;
    }
  }
  else{
    Serial.println("Couldn't Restart SIM Module.");
    return false;
  }
}

String GetOTACRC32() {
  HttpClient crc32http(secure_layer, server, 443);
  crc32http.beginRequest();
  crc32http.get(crc32AddressPath);
  // crc32http.sendHeader("Authorization", String("Bearer ") + bearerToken); // Uncomment if you are using a bearer token
  crc32http.endRequest();
  int statusCode = crc32http.responseStatusCode();
  int contentLength = crc32http.contentLength();
  if (statusCode == 200) {
    String getHash;
    String payload = crc32http.responseBody();
    if (payload.length() > 0) {
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);
      if (!error) {
        const char* hashValue = doc["Hash"];
        getHash = String(hashValue);
      }
      else {
        Serial.print("JSON Parsing Error: "); Serial.println(error.c_str());
        crc32http.stop();
        return "";
      }
    }
    else{
      Serial.println("Payload <= 0 Error.");
      return "";
    }
    crc32http.stop();
    Serial.println("Server's CRC32 Value: " + getHash);
    return getHash;
  }
  else {
    Serial.printf("HTTP GET Unsuccessful, Error: %d\n", statusCode);
    crc32http.stop();
    return "";
  }
}

bool FetchAndSaveBin() { // Function to fetch and save bin file from the server
  // Starting connection
  HttpClient http(secure_layer, server, 443);
  http.beginRequest();
  http.get(fileAddressPath);
  // http.sendHeader("Authorization", String("Bearer ") + bearerToken);  // Uncomment if you are using a bearer token
  http.endRequest();

  int httpCode = http.responseStatusCode();
  if (httpCode != 200) {
    Serial.printf("HTTP GET Unsuccessful, Error: %d\n", httpCode);
    http.stop();
    return false;
  }

  // Chunked control
  String chunkedVal = GetHeaderValue(http, HTTP_HEADER_TRANSFER_ENCODING);
  if (chunkedVal != HTTP_HEADER_VALUE_CHUNKED) {
    Serial.println("Data is not chunked.");
  }
  else {
    Serial.println("Data is chunked.");
  }
  http.skipResponseHeaders();

  File file = SPIFFS.open(updateFileName, FILE_WRITE);
  if (!file) {
    Serial.println("Couldn't Open SPIFFS File to Write..");
    http.stop();
    return false;
  }

  // Fetching the file
  unsigned long timeout = millis();
  int totalBytes = 0;
  while (!http.endOfBodyReached() && http.connected()) {
    if (http.available()) {
      char buffer[128];
      int bytesRead = http.readBytes(buffer, sizeof(buffer));
      if (bytesRead > 0) {
        timeout = millis();
        file.write((uint8_t*)buffer, bytesRead);
        totalBytes += bytesRead;
        Serial.printf("====== bytes fetched so far: %7d ======\n", totalBytes);
      }
    }
    if (millis() - timeout > 30000) break;
    delay(10);
  }
  file.close();
  http.stop();

  Serial.print("Fetched Bytes:  "); Serial.println(totalBytes);
  if (totalBytes == 0) {
    Serial.println("Couldn't fetch the whole file.");
    SPIFFS.remove(updateFileName);
    return false;
  }
  return true;
}

String GetHeaderValue(HttpClient& client, const char* headerName) {
  while (client.headerAvailable()) {
    String name = client.readHeaderName();
    if (name.equalsIgnoreCase(headerName)) {
      return client.readHeaderValue();
    }
    client.readHeaderValue();
  }
  return "";
}

bool VerifyCRC(String expected) {
  File file = SPIFFS.open(updateFileName, "rb"); // Open the file in binary reading mode
  CRC32 crc;
  
  uint8_t buffer[512];
  size_t bytesRead;
  while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
    crc.update(buffer, bytesRead);
  }
  uint32_t crcValue = crc.finalize();
  char crcHexString[9]; // 8 characters + null terminator
  sprintf(crcHexString, "%08x", crcValue);
  String crcString = String(crcHexString);
  Serial.println("CRC32 Value of Bin File: " + crcString);

  file.close();

  return crcString == expected;
}

bool PerformOTA() {
  Serial.println("OTA function started.");
  File file = SPIFFS.open(updateFileName, "r");
  if (!file) return false;

  esp_ota_handle_t otaHandle;
  const esp_partition_t *otaPartition = esp_ota_get_next_update_partition(NULL);
  
  if (esp_ota_begin(otaPartition, OTA_SIZE_UNKNOWN, &otaHandle) != ESP_OK) {
    Serial.println("Couldn't begin OTA.");
    return false;
  }

  uint8_t buffer[1024];
  size_t len;
  while ((len = file.read(buffer, sizeof(buffer))) > 0) {
    if (esp_ota_write(otaHandle, buffer, len) != ESP_OK) {
      Serial.println("OTA writing error.");
      return false;
    }
  }

  if (esp_ota_end(otaHandle) == ESP_OK) {
    esp_ota_set_boot_partition(otaPartition);
    Serial.println("OTA update successful.");
    return true;
  }
  Serial.println("OTA update failed.");
  return false;
}