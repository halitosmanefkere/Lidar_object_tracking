# LIDAR Real-time Visualization and Object Tracking (Version 4.7)

## Overview

This project involves real-time LIDAR data visualization and object tracking using Python. 
The system detects both static and dynamic objects in a 2D space and tracks the nearest and farthest objects, 
displaying relevant data such as their distance and speed. 
The application is intended to be used in areas such as robotics, autonomous driving, and environmental mapping.

- **Author:** Halit Osman Efkere
- **Date:** 06.09.2024

## Features

- **Real-time LIDAR Data:** Visualize 360-degree LIDAR sensor data in real-time.
- **Object Tracking:** Detects and tracks the nearest and farthest objects from the sensor.
- **Speed Estimation:** Estimates the speed of approaching objects.
- **Static vs. Moving Objects:** Differentiates between walls, obstacles, and dynamic objects such as people or vehicles.

## Hardware

- **LIDAR Sensor:** Neato Robotics LIDAR sensor.
- **Baud Rate:** 115200
- **Port:** COM7
- **Communication Protocol:** Serial communication using Python `serial` library.

## Installation

### Demostration Video
[![Watch the video](https://img.youtube.com/vi/ujWjjEKjeMM/0.jpg)](https://youtu.be/ujWjjEKjeMM)


### Dependencies

Ensure you have Python 3.x installed and the following libraries:

```bash
pip install matplotlib
pip install pyserial
```

## Usage

Once the script is running, it will display a plot where:

- The nearest object is highlighted in green.
- The farthest object is highlighted in red.
- Approaching dynamic objects are tracked in real-time, and their speed is estimated.
- Static objects such as walls and obstacles are differentiated from moving objects.

The plot will update in real-time as the LIDAR sensor captures data, making it suitable for tracking dynamic environments.

### Communication Setup

The LIDAR sensor communicates using the Serial protocol at a baud rate of 115200. Ensure that the LIDAR sensor is connected to the correct serial port on your system (default is COM7). You may modify the port or baud rate directly in the script if necessary.

## Future Improvements

- **Object Classification:** Implementing algorithms to better classify the detected objects based on shape and movement.
- **Speed Accuracy:** Improving the speed estimation algorithm for more accurate tracking of moving objects.
- **Data Optimization:** Further optimizing data processing to handle more complex environments.

## License

This project is licensed under the MIT License.

## Contact

If you have any questions or want to collaborate, feel free to reach out to me:

- **Email:** efkere.h.o@gmail.com
- **LinkedIn:** linkedin.com/in/halit-osman-e-771044252
```
