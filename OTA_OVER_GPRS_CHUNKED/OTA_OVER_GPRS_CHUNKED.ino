/**
 * ============================================================================
 * ESP32 OTA Updates over GPRS with Chunked Transfer Encoding
 * ============================================================================
 * 
 * @project    Performing OTA Over GPRS Using Chunked Transfer Encoding
 * @author     Barkın Sarıkartal
 * @date       April 26, 2025
 * @license    MIT License
 * @repository https://github.com/barkinsarikartal/ESP32-and-Waveshare-SIM7600X-4G-Module
 * 
 * @description
 *   This project enables Over-The-Air (OTA) firmware updates for ESP32-S3-DevKitC-1
 *   using cellular GPRS connectivity via the Waveshare SIM7600X 4G Module,
 *   eliminating the need for WiFi connectivity.
 * 
 *   The code downloads bin files from a remote server using GPRS and stores them
 *   in SPIFFS before applying the firmware update.  
 *
 *   The implementation specifically handles chunked transfer encoding from HTTP
 *   servers that don't provide Content-Length headers.
 * 
 * @hardware
 *   - ESP32-S3-DevKitC-1 N16R8 (16MB Flash, 8MB PSRAM)
 *   - Waveshare SIM7600X 4G Module
 *   - SIM card with active data plan
 * 
 * @software
 *   - Arduino IDE v2.3.4
 *   - ESP32 Board Package v3.0.1
 * 
 * @notes
 *   IMPORTANT: This implementation is specific for servers that respond with the
 *   HTTP header "Transfer-Encoding: Chunked". For servers that use standard
 *   "Content-Length" headers, please use the alternative implementations
 *   (OTA_OVER_GPRS.ino or OTA_OVER_GPRS_CRC32) in this repository.
 *   
 *   A custom partition scheme is required: "8M with spiffs 3MB APP/1.5 MB SPIFFS"
 *   for optimal storage allocation between application and update files.
 * 
 * ============================================================================
 */

#define TINY_GSM_MODEM_SIM7600
#define UART_BAUD 115200
#define SerialAT Serial1
#define MODEM_TX 16 // Connect SIM Module's RX pin to ESP32's GPIO16
#define MODEM_RX 15 // Connect SIM Module's TX pin to ESP32's GPIO15

// External Libraries
#include <ArduinoHttpClient.h>  // Version 0.6.0, by Arduino
#include <TinyGsmClient.h>      // Version 0.12.0, by Volodymyr Shymanskyy
#include <SSLClient.h>          // Version 1.3.2, by V Govorovski, used for SSL connections

// Built-in Libraries
#include <SoftwareSerial.h>     // Comes with Arduino IDE
#include <Arduino.h>            // Comes with ESP32 Board Package
#include <SPIFFS.h>             // Comes with ESP32 Board Package
#include "esp_ota_ops.h"        // Comes with ESP32 Board Package

// Network Configuration
const char apn[]  = "internet"; // Enter your own sim card's APN
const char user[] = "";
const char pass[] = "";

// Update Configuration
const char* updateFileName = "/firmware.bin";    // File name on SPIFFS
const char* server = "yourserver.com";           // Server hosting the bin file
const char* fileAddressPath = "/yourbinfile";    // The specific path on the server where the bin file can be accessed
// const char* bearerToken = "yourbearertoken";  // Uncomment if using a bearer token

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
SSLClient secure_layer(&client);

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("ESP32 OTA Updates over GPRS using SIM7600 with Chunked Transfer...");

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to initialize SPIFFS.");
    delay(50);
    ESP.restart();
  }

  // Check if this file already exists
  if (SPIFFS.exists(updateFileName)) {
    if (SPIFFS.remove(updateFileName)) {
      Serial.println("Deleted " + String(updateFileName) + " from SPIFFS.");
    }
  }
  
  // Display SPIFFS information
  size_t totalSPIFFSBytes = SPIFFS.totalBytes();
  size_t usedSPIFFSBytes = SPIFFS.usedBytes();
  Serial.println("SPIFFS Information:");
  Serial.printf("%-6s: %10u bytes\n", "Total", totalSPIFFSBytes);
  Serial.printf("%-6s: %10u bytes\n", "Used", usedSPIFFSBytes);
  Serial.printf("%-6s: %10u bytes\n", "Empty", totalSPIFFSBytes - usedSPIFFSBytes);
  delay(3000);
  
  // Execute OTA update sequence
  if (SIMStartFunction()) { // Connecting to GPRS
    Serial.println("--------------------");

    if (FetchAndSaveBin()) {
      Serial.println("Firmware binary downloaded.");

      if (PerformOTA()) {
        Serial.println("OTA update completed. Restarting...");
        SPIFFS.end(); // Closing SPIFFS for safe restart
        delay(2000);
        ESP.restart(); // Restarting ESP32 to boot with new bin file
      }
    }
  }
}

void loop() {
  // Empty loop - all work is done in setup()
  delay(1000);
}

/**
 * Connect SIM Module to cellular network and GPRS
 * 
 * @return true if connection successful, false otherwise
 */
bool SIMStartFunction() {
  Serial.println("\nInitializing SIM7600 module...");
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(100);
  
  // Restart modem
  Serial.println("Restarting modem...");
  if (!modem.restart()) {
    Serial.println("Could not restart SIM module.");
    return false;
  }
  Serial.println("Restarted SIM module.");
  
  // Connect to cellular network
  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println("Network connection failed.");
    return false;
  }
  Serial.println("Network connected.");
  
  // Connect to GPRS
  Serial.print("Connecting to GPRS...");
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println("GPRS connection failed.");
    return false;
  }
  
  // Connection successful, display network information
  Serial.println("GPRS connected.");
  String modemIP = modem.getLocalIP();
  Serial.println("Modem IP: " + modemIP);
  int signalQuality = modem.getSignalQuality();
  Serial.println("Signal Quality: " + String(signalQuality));
  
  return true;
}

/**
 * Download firmware binary from server using chunked transfer encoding
 * 
 * @return true if download successful, false otherwise
 */
bool FetchAndSaveBin() {
  Serial.println("\nDownloading firmware binary...");
  Serial.print("Endpoint: ");
  Serial.println(String(server) + fileAddressPath);

  // Initialize HTTP connection
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
    Serial.println("Warning: Data is not using chunked transfer encoding.");
  }
  else {
    Serial.println("Using chunked transfer encoding.");
  }

  // Skip remaining headers
  http.skipResponseHeaders();

  // Open file for writing
  File file = SPIFFS.open(updateFileName, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to Open SPIFFS File to Write..");
    http.stop();
    return false;
  }

  // Download file
  Serial.println("Downloading firmware...");
  unsigned long timeout = millis();
  int totalBytes = 0, progressCounter = 0;;

  // Loop until download complete or error
  while (!http.endOfBodyReached() && http.connected()) {
    if (http.available()) {
      char buffer[128];
      int bytesRead = http.readBytes(buffer, sizeof(buffer));

      if (bytesRead > 0) {
        file.write((uint8_t*)buffer, bytesRead);
        totalBytes += bytesRead;
        timeout = millis();

        // Show progress periodically
        if (++progressCounter % 40 == 0) {
          Serial.printf("Download progress: %d bytes\n", totalBytes);
        }
      }
    }
    
    if (millis() - timeout > 30000) {
      Serial.println("Timeout error!");
      file.close();
      http.stop();
      SPIFFS.remove(updateFileName);
      return false;
    }

    delay(10);
  }

  // Close file and connection
  file.close();
  http.stop();

  // Check download success
  Serial.println("\nDownload completed");
  Serial.printf("Total downloaded: %d bytes\n", totalBytes);

  if (totalBytes == 0) {
    Serial.println("No data downloaded.");
    SPIFFS.remove(updateFileName);
    return false;
  }

  return true;
}

/**
 * Helper function to extract specific HTTP header value
 * 
 * @param client      HTTP client object
 * @param headerName  Name of header to find
 * @return            Value of header or empty string if not found
 */
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

/**
 * Perform OTA update with downloaded firmware
 * 
 * @return true if update successful, false otherwise
 */
bool PerformOTA() {
  Serial.println("\nStarting OTA update process...");

  // Open the firmware file
  File file = SPIFFS.open(updateFileName, "r");
  if (!file) {
    Serial.println("Could not open firmware file.");
    return false;
  }

  size_t fileSize = file.size();
  Serial.printf("Firmware size: %d bytes\n", fileSize);

  // Get the next OTA partition
  const esp_partition_t *otaPartition = esp_ota_get_next_update_partition(NULL);
  if (otaPartition == NULL) {
    Serial.println("Failed to get OTA partition.");
    file.close();
    return false;
  }

  Serial.printf("Writing to partition: %s\n", otaPartition->label);

  // Begin OTA process
  esp_ota_handle_t otaHandle;
  if (esp_ota_begin(otaPartition, OTA_SIZE_UNKNOWN, &otaHandle) != ESP_OK) {
    Serial.println("Failed to begin OTA.");
    file.close();
    return false;
  }

  // Write firmware data to partition
  uint8_t buffer[1024];
  size_t bytesRead = 0;
  size_t totalBytesWritten = 0;
  int progressCounter = 0;

  Serial.println("Flashing firmware...");
  while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
    if (esp_ota_write(otaHandle, buffer, bytesRead) != ESP_OK) {
      Serial.println("OTA write failed.");
      return false;
    }

    totalBytesWritten += bytesRead;

    // Show progress periodically
    if (++progressCounter % 32 == 0 || totalBytesWritten == fileSize) {
      float progress = (float)totalBytesWritten / fileSize * 100;
      Serial.printf("Flashing progress: %d bytes (%.1f%%)\n", 
                    totalBytesWritten, progress);
    }
  }

  file.close();

  // Finalize OTA update
  if (esp_ota_end(otaHandle) != ESP_OK) {
    Serial.println("Failed to end OTA process.");
    return false;
  }

  // Set boot partition
  if (esp_ota_set_boot_partition(otaPartition) != ESP_OK) {
    Serial.println("Failed to set boot partition.");
    return false;
  }
  return true;
}