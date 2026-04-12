

🤖 Robo-Limb: AI-Powered Supernumerary Robotic Arm

An AI-powered wearable robotic limb designed to assist individuals with physical and visual disabilities in performing daily object manipulation tasks independently.


---

📌 Overview

Robo-Limb is a non-invasive, portable robotic arm that integrates:

🎤 Voice Commands

👁️ Computer Vision

🦾 Robotic Manipulation


The system can detect, locate, and grasp objects autonomously, reducing dependency on caregivers.


---

🚀 Key Features

🔊 Voice-Controlled Operation
Users can control the system using natural voice commands.

📷 Computer Vision-Based Object Detection
Detects and identifies objects using a camera module.

🤖 Autonomous Grasping Mechanism
Calculates position and performs object pickup using servo motors.

🎯 Motion Planning & Kinematics
Ensures accurate and efficient arm movement.

🗣️ Spoken Feedback System
Provides audio feedback for visually impaired users.

⚡ Lightweight & Portable Design



---

🧠 System Workflow

1. Start System


2. Voice Command Input


3. Voice Recognition


4. Camera Scans Environment


5. Object Detection (Vision Module)


6. Object Localization


7. Motion Planning


8. Servo Motor Control


9. Object Grasping


10. Task Completion Feedback




---

🏗️ Tech Stack

Hardware

Arduino / Microcontroller

Servo Motors (MG995, MG996R, DS3225, HV2060MG)

Ultrasonic Sensor / Camera Module

Power Supply (Battery + Buck Converter)


Software

Python

OpenCV

Machine Learning / Object Detection Models

Arduino IDE

Serial Communication



---

🔧 Project Structure

robo-limb/
│── computer_vision/
│   ├── robolimb.py
│   └── detection modules
│
│── hardware/
│   ├── arduino_code.ino
│   └── circuit diagrams
│
│── models/
│   └── trained ML models
│
│── docs/
│   └── project report / presentation
│
│── README.md


---

⚙️ Installation & Setup

1️⃣ Clone the Repository

git clone https://github.com/Kartikey9936/robo-limb.git
cd robo-limb

2️⃣ Create Virtual Environment

python -m venv .venv
source .venv/bin/activate   # Linux/Mac
.venv\Scripts\activate      # Windows

3️⃣ Install Dependencies

pip install -r requirements.txt

4️⃣ Run the System

python computer_vision/robolimb.py


---

🔌 Hardware Setup

Connect servo motors to Arduino PWM pins

Connect camera to system (USB / ESP32-CAM)

Use buck converter for stable power supply

Ensure proper grounding between all components



---

📊 Applications

♿ Assistive technology for disabled individuals

👴 Elderly care support

🏥 Rehabilitation systems

🏭 Industrial object handling

🤖 Human augmentation systems



---

📈 Future Improvements

🔥 Real-time embedded AI (Edge AI)

🧠 Advanced object recognition models

📡 IoT integration

🦾 More degrees of freedom (DoF)

🎮 Mobile app control



---

💰 Budget (Approx.)

Component	Quantity	Cost (INR)

Servo Motors	Multiple	~7000+
Webcam	1	~1000
Sensors	Multiple	~900
Microcontroller	1	~500
Others	—	~2000



---

👨‍💻 Contributors

Kartikey Kesharwani

Team VIBHAV, NIT Hamirpur



---

📜 License

This project is open-source and available under the MIT License.


---

