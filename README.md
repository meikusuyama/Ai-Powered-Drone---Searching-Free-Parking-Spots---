# Ai-Powered-Drone---Searching-Free-Parking-Spots---

_By Mei Kusuyama, Nancy Ta, Aayush Sugali, Isaac Zhi Kang, and Enzo Emami_

This repository documents our project at De Anza College in Spring 2025 in collaboration with Infineon Technologies. Our autonomous drone is capable of detecting and flying to empty parking spaces using a custom-trained computer vision model. This version of the system runs entirely on a single Raspberry Pi 4B, handling both real-time video inference and flight control.

## 1. Project Overview
The drone is designed to assist drivers in locating free parking spaces in crowded lots by detecting open spots from a bird's-eye view and navigating directly above them. It uses:

- A YOLOv8n object detection model trained on labeled parking lot images

- A Raspberry Pi Camera Module 3 Wide for video input

- A Pixhawk 2.4.8 flight controller for drone navigation (via ArduPilot and DroneKit)

- Logic written in Python to make navigation decisions based on model predictions

We used the Infineon PSoC 6 AI Evaluation Kit and their siren detection model to support emergency response applications with:

- An LED strip connected to the PSoC board is triggered when a siren is detected
- Wi-Fi-based TCP communication, where the board would send alerts to a mobile device such as an iPhone when a siren was heard

## 2. Model Training with YOLOv8n
We used Ultralytics YOLOv8n, the lightest and fastest version of the YOLOv8 object detection framework.

### Dataset
- The training dataset was taken from [Arpitpatel1706's repository](https://github.com/Arpitpatel1706/car-parking-slot-occupancy-detection-using-YOLOv8---openCV), which contains labeled parking images categorized into free and occupied slots.

- The model was trained using <ins>train_model.ipynb</ins>, with the associated configuration in data.yaml.

- After training, the best-performing model weights were saved as **best.pt**.    

## 3. Model Testing
To validate the trained model:

- We used <ins>slot_detection.ipynb</ins> to run inference on sample footage and confirmed that it accurately detected free spots.

- Test and demonstration videos are included in this repository to showcase the model's effectiveness.

## 4. Drone Control & Integration
We used:

- ArduPilot firmware on the Pixhawk flight controller

- MAVProxy to allow command-line interfacing with the flight stack

- DroneKit Python to issue commands to the drone from within our operational script

Our main operational script is <ins>test_takeoff.py</ins>, which performs the following:

1. Arms the drone and commands takeoff to a set altitude.

2. Continuously captures frames from the Pi Camera and runs inference using the YOLOv8n model on each frame.

3. Calibrates a perimeter around the parking lot by travelling in each cardinal direction until no more parking boxes, occupied or unoccupied, are detected.

4. Once the searching begins and if a free parking spot is detected, it calculates the offset from the center and sends velocity commands to center the drone above it. When successfully centered, the drone hovers, sends its GPS location to a remote device, then returns and lands.
   
5. If a free parking spot is not detected, the drone generates a random coordinate within the previously defined perimeter. It then ensures that this new location is not too close to any of the last three visited random coordinates, thereby improving coverage and reducing redundancy. As the drone flies toward the new location, it actively scans for free spots en route. Once a free spot is detected, it follows the procedure defined in step 4.

## 5. Infineon PSoC 6 AI Board Integration
To explore emergency response applications, we also integrated Infineon’s PSoC 6 AI evaluation board. Using their ready-to-deploy siren detection model, we:

- Connected an LED strip to the board to visually indicate when a siren is detected.

- Used TCP communication to relay this detection to an iPhone for testing mobile notification concepts.

- Implemented this system to enhance public safety communication in crowded parking environments by alerting bystanders to nearby law enforcement presence.


## 6. Drone Demonstration
We also included a short video demonstrating the drone autonomously taking off to a height of 10 meters and returning to land. This showcases the control logic in action under real-world conditions.

[Drone Flight Demonstration Video](https://drive.google.com/file/d/1KxCxmjvK91ACncRWegSC54-HyLtKUthf/view?usp=drive_link)

## 7. Future Development
While the drone’s Raspberry Pi controlled the movement system and the parking slot detection algorithm functioned well independently, we encountered a significant challenge when both the camera and GPS modules were active simultaneously. Operating the camera at high frame rates appeared to interfere with GPS performance, likely due to the limited processing capacity of the Raspberry Pi. We explored several workarounds, including lowering the camera’s frame rate, rerouting MAVProxy serial connections, and adjusting GPS failsafe parameters. However, despite these efforts, the interference persisted during high-load operations.

### Next Steps

Redesign the system to incorporate two Raspberry Pis communicating via TCP, enabling more robust parallel processing and improved task distribution. With the dual-board system, we aim to achieve smoother performance and enhanced reliability throughout the parking spot detection and the drone’s autonomous operations.

This next-phase version will be documented in a future repository.
