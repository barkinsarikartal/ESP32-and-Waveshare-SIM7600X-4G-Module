# ESP32 and Waveshare SIM7600X 4G Module GPRS and GNSS Examples
This repository contains GPRS and GNSS examples for ESP32 and Waveshare's SIM7600X 4G Module.

## Modules Used
- ESP32-S3-DevKitC-1 (N16R8)
- Waveshare SIM7600X 4G Communication Module

## Connections

All codes in this repository are based on the following connection diagram:

<p align="left">
  <img src="https://github.com/user-attachments/assets/59254c7d-436a-45bf-b58b-4623220ae620" width="45%" />
</p>

## Additional Notes

Personally, I prefer to use both modules by powering them via USB. This ensures that both modules work stably. <strong>DO NOT POWER THE SIM MODULE DIRECTLY FROM ESP32!!!</strong>
Doing this may fry your ESP32 because SIM modules may draw high currents up to 2 Amps when connecting to network.
