# Handheld Game Console

##  Project Description

This project is a **Handheld Game Console** developed as a **fully working academic embedded system**. It is designed to be simple, intuitive, and accessible, with a special focus on usability for **mentally challenged individuals**, while remaining engaging and usable for everyone.

The system uses a joystick-based input mechanism and physical buttons to navigate through on-screen interactions displayed via an I2C LCD. A working demonstration video is included with the repository to showcase real-time functionality.

---

## 🎯 Objective

* To design and implement an interactive handheld console using an STM32 microcontroller
* To demonstrate real-time embedded system concepts such as ADC, interrupts, timers, and I2C communication
* To build an accessible and easy-to-use interface suitable for users with cognitive challenges

---

##  Features

* Fully functional handheld embedded system
* Joystick-based navigation using ADC
* Button controls using external interrupts (EXTI)
* Menu-driven interface displayed on an I2C LCD
* Real-time response using SysTick timer
* Simple and user-friendly interaction model

---

## 🛠️ Hardware Requirements

* **Microcontroller**: STM32F401CCUx
* Joystick module
* I2C LCD display (16x2 with PCF8574)
* Push buttons
* Power supply

---

##  Software & Tools

* **IDE**: Keil µVision
* **Programming Language**: Embedded C
* **Communication Protocols**: ADC, I2C, EXTI

---

##  System Overview

* The joystick provides analog input to the MCU via ADC channels
* Button presses are handled using EXTI interrupts
* The LCD communicates through the I2C protocol to display menus and system responses
* SysTick timer ensures consistent timing and responsiveness
* The entire system is optimized for simplicity and reliability

---

## 🎮 Games Included

The handheld console includes **two simple and accessible games** designed to encourage engagement, memory, and motor coordination. These games are intentionally minimalistic to ensure ease of use for mentally challenged individuals, while remaining enjoyable for everyone.

###  Memory Game

* Simple pattern or sequence recognition
* Joystick used for navigation and selection
* Button used for confirmation
* Calm gameplay with clear visual feedback on the LCD

###  Platform Runner

* A basic side-scrolling style game
* Joystick used for movement/navigation
* Button used for actions such as jump or select
* Simple visuals and mechanics to avoid cognitive overload

---

##  Working Demonstration

Videos demonstrating the working of the handheld game console is attached in the repository.

---

##  License

This project is licensed under the **MIT License**.

---

## 📌 Note

This project is developed purely for **academic and learning purposes**, showcasing practical embedded system design and implementation.
