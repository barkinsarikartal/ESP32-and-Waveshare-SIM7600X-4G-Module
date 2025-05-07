/**
 * ============================================================================
 * ESP32 GPS Tracker with SIM7600X 4G Module
 * ============================================================================
 * 
 * @project    Printing GPS Data
 * @author     Barkın Sarıkartal
 * @date       May 5, 2025
 * @license    MIT License
 * @repository https://github.com/barkinsarikartal/ESP32-and-Waveshare-SIM7600X-4G-Module
 * 
 * @description
 *   This project reads GPS data from Waveshare's SIM7600X 4G Module connected
 *   to an ESP32-S3-DevKitC-1 and outputs the location information to the 
 *   Serial Monitor.
 * 
 * @hardware
 *   - ESP32-S3-DevKitC-1 N16R8 (16MB Flash, 8MB PSRAM)
 *   - Waveshare SIM7600X 4G Module
 *   - External GPS antenna
 * 
 * @software
 *   - Arduino IDE v2.3.4
 *   - ESP32 Board Package v3.0.1
 * 
 * @notes
 *   IMPORTANT: The GPS antenna must have clear view of the sky to receive 
 *   satellite signals properly.
 *
 *   First couple of attempts to receive GPS signals may be unsuccessful.
 *   However, with patience, the system will receive GPS signals if the
 *   antenna has a clear view of the sky.
 * 
 *   For posting GPS data to a server, please refer to the alternative version
 *   in this repository: GPS_TRACKER_HTTP_POST.ino
 * 
 * ============================================================================
 */

#define TINY_GSM_MODEM_SIM7600
#define UART_BAUD 115200
#define SerialAT Serial1
#define MODEM_TX 16              // Connect SIM Module's RX pin to ESP32's GPIO16
#define MODEM_RX 15              // Connect SIM Module's TX pin to ESP32's GPIO15
#define GPS_UPDATE_INTERVAL 3000 // GPS update interval in milliseconds

// External libraries
#include <TinyGsmClient.h>      // Version 0.12.0, by Volodymyr Shymanskyy

// Built-in Libraries
#include <SoftwareSerial.h>     // Comes with Arduino IDE
#include <Arduino.h>            // Comes with ESP32 Board Package

unsigned long lastGpsUpdate = 0;   // Timestamp for non-blocking operation
float lat = 0, lon = 0, speed = 0; // Initialize GPS coordinates (latitude, longitude) and speed variables by zero

bool isGpsAvailable = false; // Global indicator of whether SIM module is connected to GPS

// Create modem and client instances
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 GPS Tracker with SIM7600 Starting...");
  delay(3000);
  if (InitializeSimModule()) { // Connecting to GPRS
    Serial.println("SIM module successfully initialized");
  }
  else {
    Serial.println("SIM module initialization failed");
  }
}

void loop() {
  // Use non-blocking approach for GPS updates
  if (isGpsAvailable && (millis() - lastGpsUpdate >= GPS_UPDATE_INTERVAL)) {
    UpdateAndPrintGpsInfo();
    lastGpsUpdate = millis();
  }
}

/**
 * Initialize the SIM module and connect to GPS
 * 
 * @return boolean indicating success or failure
 */
bool InitializeSimModule() {
  // Initialize serial communication with the modem
  Serial.println("Initializing modem...");
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(300);
  
  // Restart modem
  Serial.println("Restarting modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem");
    return false;
  }
  
  // Display modem info
  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);
  
  // Connect to network
  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork(60000L)) {
    Serial.println("Network connection failed");
    return false;
  }
  Serial.println("Network connected");
  
  // Display signal quality
  int signalQuality = modem.getSignalQuality();
  Serial.print("Signal quality: ");
  Serial.println(signalQuality);
  
  // Enable GPS
  Serial.println("Enabling GPS...");
  if (!modem.enableGPS()) {
    Serial.println("GPS enabling failed");
    return false;
  }
  
  Serial.println("GPS enabled successfully");
  isGpsAvailable = true;
  return true;
}

/**
 * Update and print GPS information
 */
void UpdateAndPrintGpsInfo() {
  if (modem.getGPS(&lat, &lon, &speed)) {
    speed = speed * 1.852; // Convert speed from knots to kilometers per hour (km/h)
    
    // Format GPS data for display
    char buffer[100];
    snprintf(buffer, sizeof(buffer), 
             "Latitude: %.6f, Longitude: %.6f, Speed: %.2f km/h",
             lat, lon, speed);
    Serial.println(buffer);
  }
  else {
    Serial.println("Failed to get GPS data.");
  }
}