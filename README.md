# Elderly Fall Detection System

## Tech Used:
1. STM32 Microcontrollers
1. Adafruit 9-DOF Accel/Mag/Gyro+Temp Breakout Board - LSM9DS1
1. UART Communication
1. KiCad (For Electrical Schematics)
1. C
   
## Description:
This project is an elderly fall detection system that makes use of two STM32 Microcontrollers. One STM32 is worn by the user and constantly monitors sensor data, to detect a fall. Upon successful detection of a fall, the STM32 sends a signal via UART communication
to the receiving STM32, which is usually with the caregiver. The receiving STM32 sends a PWM signal to activate a Piezo buzzer and alert the caregiver. 

## Project Images:
![IMG_2345](https://github.com/user-attachments/assets/67522fd7-e0b5-4828-a28c-818e73f870ef)
![IMG_2352](https://github.com/user-attachments/assets/d71592e1-1b0f-493c-934b-dadba0b422b6)


