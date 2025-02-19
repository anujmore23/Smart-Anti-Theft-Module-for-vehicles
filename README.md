Smart Anti-Theft Vehicle System 🚗🔒

A vehicle security system that provides real-time GPS tracking and remote immobilization using SMS-based control. This system is designed to prevent vehicle theft and offer a simple, efficient way for users to monitor their vehicles remotely.

📌 Features

✅ GPS Tracking – Real-time location updates using the NEO-7M module✅ Remote Immobilization – Disable the vehicle via SMS commands✅ 4G LTE Communication – Uses the SIM7600A-H module for secure messaging✅ SMS-based User Interface – No need for an app; control via text messages✅ Low Power Consumption – Optimized for automotive applications

🛠️ Hardware Components

STM32 NUCLEO-L476RG (ARM Cortex-M4 MCU)

GPS NEO-7M (Real-time tracking)

SIM7600A-H (4G LTE for SMS Communication)

Relay Module (Vehicle Immobilization Control)

Power Supply (Stable voltage regulation for automotive use)

📜 SMS Commands

Command

Function

Get Location

Retrieves the real-time GPS location

Lock

Saves the current parked location

Immobilize

Disables the vehicle ignition (anti-theft)

Mobilize

Restores the ignition control

📁 Project Structure

Smart-Anti-Theft-Vehicle-System/
├── Firmware/           # STM32 firmware (main.c, HAL libraries)
├── Schematics/         # Circuit diagrams, pinout details
├── Docs/               # Project report, research notes
├── Images/             # Screenshots and demonstration pictures
├── README.md           # Project Overview (this file)
└── .gitignore          # Ignore build files, binaries, and temp files

🚀 Installation & Setup

1️⃣ Flash the Firmware

Open STM32CubeIDE

Compile and flash the provided firmware to STM32 NUCLEO-L476RG

2️⃣ Hardware Connections

Connect the GPS NEO-7M, SIM7600A-H, and Relay Module according to the Schematics/Hardware_Wiring.png

Insert a valid SIM card into the SIM7600A-H module

3️⃣ Send SMS Commands

Send Get Location to receive GPS coordinates via SMS

Send Lock to set the parked location

Send Immobilize to disable vehicle ignition

Send Mobilize to enable vehicle ignition

📊 Performance Metrics

GPS Fix Time: ~61.25 ms (Avg.)

SMS Response Time: ~50.55 ms (Avg.)

Relay Actuation Delay: < 100 ms

🎯 Applications

Personal Vehicle Security – Protect against unauthorized access

Fleet Management – Monitor commercial vehicles remotely

Car Rental & Leasing – Prevent unauthorized use and theft

📽️ Demonstration

📌 Watch a video demo here: [YouTube Link] (Add link if available)

👨‍💻 Author

Developed by Anuj More📍 Master’s Student in Electrical and Computer Engineering📧 LinkedIn📂 GitHub

📜 License

This project is licensed under the MIT License – feel free to use and modify!

