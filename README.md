# SailSwarm Project Repository

Welcome to the SailSwarm project repository! This repository contains the code developed for a swarm of autonomous sailboats, part of the SailSwarm project at the University of Konstanz. The project was conducted under the guidance of PhD candidate Pranav Kedia, within the German Excellence Cluster "Center for the Advanced Study of Collective Behavior".

# Project Overview

The SailSwarm project aims to explore the collective behavior and autonomous navigation of a swarm of sailboats. By leveraging a fleet of small, interconnected boats, we aim to develop efficient, scalable algorithms for autonomous sailing and collaborative task execution on water.

# Hardware Components

The autonomous sailboats are equipped with the following hardware:

+ **ESP32 Microcontroller:** Serving as the main processing unit, responsible for executing the control algorithms and communication between boats.

+ **Servo Motors:** Used for adjusting the sail and rudder, enabling precise control of the boat's direction and speed.

+ **MPU9250 IMU:** An Inertial Measurement Unit providing real-time orientation and motion data to aid in navigation and stability control.

# Software Components

The codebase is primarily written in Arduino (using C++) and is structured to manage the various aspects of autonomous sailing, including:

+ **Navigation and Path Planning:** Algorithms to determine the optimal path based on wind conditions and waypoints.

+ **Control Systems:** PID controllers for sail and rudder adjustments to maintain desired headings and speeds.

+ **Communication Protocols:** Mechanisms for inter-boat communication and coordination, ensuring cohesive swarm behavior.

# Getting Started

To get started with the SailSwarm project, you will need the following tools and libraries:

+ Arduino IDE: For compiling and uploading code to the ESP32.
  
+ Required Libraries: Install the necessary Arduino libraries for ESP32, servo control, and IMU integration.

# Acknowledgments

This project was made possible through the support and resources provided by the University of Konstanz and the German Excellence Cluster "Center for the Advanced Study of Collective Behavior". Special thanks to PhD candidate Pranav Kedia for his guidance and supervision throughout the project.

# License

This project is licensed under the MIT License. See the LICENSE file for details.

Feel free to explore the repository, raise issues for any bugs or enhancements.

# Contact Information:
+ Primary Author: Charlie Apolinsky
+ Email: capolinsky@bowdoin.edu
+ Project Lead: Pranav Kedia
+ Institution: University of Konstanz, Center for the Advanced Study of Collective Behavior. Internship and funding through DAAD Rise.

Happy sailing!
