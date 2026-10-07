# 🪖 Helmet Detectors
## AI-Based Smart Helmet for Rider Safety, Monitoring & Accident Detection

<p align="center">

  <img src="Images/project_front.jpg" alt="Helmet Detectors" width="700">

</p>

<p align="center">
  <b>An intelligent IoT-based smart helmet designed to improve motorcycle rider safety using AI, sensors, GPS, RFID and real-time emergency communication.</b>
</p>

<p align="center">

![ESP32](https://img.shields.io/badge/ESP32-DevKit-red?style=for-the-badge&logo=espressif)
![ESP32-CAM](https://img.shields.io/badge/ESP32--CAM-Camera-blue?style=for-the-badge)
![Arduino](https://img.shields.io/badge/Arduino-C%2B%2B-00979D?style=for-the-badge&logo=arduino)
![Python](https://img.shields.io/badge/Python-3.x-3776AB?style=for-the-badge&logo=python)
![FastAPI](https://img.shields.io/badge/FastAPI-API-009688?style=for-the-badge&logo=fastapi)
![RFID](https://img.shields.io/badge/RFID-RC522-orange?style=for-the-badge)
![GPS](https://img.shields.io/badge/GPS-NEO--6M-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Project-Completed-success?style=for-the-badge)

</p>

---

## 👨‍💻 Team — Group 2

### `helmet_detectors`

| No. | Name | Student ID |
|:---:|---|:---:|
| 1 | **MD. Siam Afroz Tanmoy** | **0112230123** |
| 2 | Alfi Shahriar | 0112320293 |
| 3 | Irfan Jafri | 0112410371 |
| 4 | Naima Islam Nabila | 0112320270 |
| 5 | Asia Kabir Shafa | 0112320295 |

**Department of Computer Science & Engineering**  
**United International University (UIU)**  
**Dhaka, Bangladesh**

---

# 📌 Table of Contents

- [About the Project](#-about-the-project)
- [Project Objectives](#-project-objectives)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Hardware Components](#-hardware-components)
- [Software & Technologies](#-software--technologies)
- [How the System Works](#-how-the-system-works)
- [AI-Based Helmet Detection](#-ai-based-helmet-detection)
- [Drowsiness Detection](#-drowsiness-detection)
- [RFID Rider Authentication](#-rfid-rider-authentication)
- [Accident Detection](#-accident-detection)
- [Emergency Response](#-emergency-response)
- [Project Structure](#-project-structure)
- [Installation & Setup](#-installation--setup)
- [Configuration](#-configuration)
- [Running the Project](#-running-the-project)
- [Project Images](#-project-images)
- [Demo Video](#-demo-video)
- [Documentation](#-documentation)
- [Future Improvements](#-future-improvements)
- [Limitations](#-limitations)
- [Team Contribution](#-team-contribution)
- [Acknowledgement](#-acknowledgement)
- [License](#-license)

---

# 🚀 About the Project

**Helmet Detectors** is an AI-based smart motorcycle helmet developed to improve rider safety through continuous monitoring, intelligent detection and emergency response.

The system integrates multiple sensors and modules into a **3D-printed helmet body**. It combines an **ESP32 DevKit**, **ESP32-CAM**, **RFID**, **MPU6050**, **MQ-3 alcohol sensor**, **GPS**, **LDR**, **OLED display**, **buzzer**, and other supporting components.

The project provides four major safety functions:

1. **AI-Based Helmet and Rider Detection**
2. **Intelligent Rider Safety and Monitoring**
3. **Accident Detection and Emergency Response**
4. **Driver/Rider Drowsiness Detection and Alert**

The completed prototype was physically assembled on a 3D-printed helmet body, with the electronic components mounted directly on the helmet.

---

# 🎯 Project Objectives

The main objectives of this project are:

- To develop an intelligent helmet-based rider safety system.
- To detect whether the rider is wearing a helmet.
- To identify visible signs of rider drowsiness.
- To authenticate the rider using RFID.
- To monitor alcohol levels.
- To detect possible motorcycle accidents.
- To provide a short cancellation period after accident detection.
- To obtain the accident location using GPS.
- To send emergency notifications remotely.
- To provide local warnings through a buzzer.
- To display important system information through an OLED display.
- To monitor environmental light conditions.
- To integrate all major safety features into a single wearable prototype.

---

# ⭐ Key Features

## 🤖 1. AI-Based Helmet & Rider Detection

The ESP32-CAM captures the rider's image and sends it to the AI/VLM-based processing system.

The system checks:

- Whether a rider is visible.
- Whether a helmet is being worn.
- Whether the rider appears drowsy.
- Eye condition/state.
- Confidence of the AI result.
- Additional detection notes.

The result is then returned to the ESP32-CAM and used by the main system.

---

## 🛡️ 2. Intelligent Rider Safety & Monitoring

The helmet continuously monitors several rider and environmental conditions.

### Monitoring includes:

- 🍺 Alcohol level
- 💡 Ambient light
- 🪪 Rider authentication
- 📍 GPS information
- 📺 OLED status
- 🔊 Local buzzer alerts
- 📊 Real-time dashboard information

---

## 🪪 3. RFID-Based Rider Authentication

The original project proposal specified a fingerprint sensor.

However, in the **final implementation**, the fingerprint sensor was replaced with an:

> **RC522 RFID Module**

An authorized RFID tag is used to authenticate the rider and control the safety system.

### RFID workflow

```text
RFID Tag Detected
       │
       ▼
Check Authorized UID
       │
   ┌───┴────┐
   │        │
Valid     Invalid
   │        │
   ▼        ▼
ARM      Reject
System   Access
