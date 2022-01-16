# Staircase-Lighting-System

A microcontroller device for controlling stair lighting based on the principle of full event programming with the implementation of a finite control machine has been developed.
The control unit is implemented on the basis of the microcontroller family STM32 (STM32F103C8T6). 
The software implementation of the control algorithm is carried out in the C programming language in the Keil development environment and with the help of CubeMx CAD.
Based on the protocol of data transmission according to official documentation and the use of modern solutions for interaction with RGB-LEDs, the connection between the MCU and the LED strip was implemented using DMA technology to minimize the hardware costs of the microcontroller.
The prototype of the stair lighting system in a private residential building is implemented on the basis of a simplified circuit implementation using LED strip based on WS2812B LEDs using a debug board STM32.
____________________________________________________________________________________________________________________________________________
# Area of improvement

The STM32F103C8T6 costs 10.31 USD and has the 12 DMA-channels (12 stairs).
The STM32F030C8T6 costs  2.34 USD and has the  5 DMA-channels (5 stairs).
It is possible to base the solution on the cheapest microcontroller in STM32 series STM32F030C8T6.
Also it is possible to implement the solution outdoors using IP63 protection standard or higher.
____________________________________________________________________________________________________________________________________________
# Repository content

The repository consist of:
Player design (VHDL-project)
Player waveform (files that are needed to be placed with the Player design folder)
Player specifications in Ukranian and English
____________________________________________________________________________________________________________________________________________
