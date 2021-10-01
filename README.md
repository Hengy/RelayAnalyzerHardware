# RelayAnalyzerHardware
PCB and Firmware repository for the Relay Analyzer

This contains the final report for the Relay Analyzer 2021 Capstone project by Matthew Hengeveld and Kevin MacIntosh.

Also included are PCB schematics and designs for PCBs designed by Matthew Hengeveld, as well as firmware for STM32F767ZI Nucleo development board used in the project.

![Programmable Power Supply PCB](https://github.com/Hengy/RelayAnalyzerHardware/blob/main/PCB.png)

# Summary of project
The Relay Analyzer is a test and measurement project that automatically tests and determines several characteristics about a electromechanical relay, including switching times, coil current and voltage, coil resistance and power, and also samples the contacts to produce a waveform of any contact bounce.

The project is based on an STM32F767ZI Nucleo development board. Five circuits were designed and simulated. Five corresponding PCBs were designed, manufactured, populated, and tested as part of the project. These circuits were the programmable power supply, trigger front end, bounce ADC front end, coil resistance front end, and relay interface.

The Relay Analyzer was designed to use a high-speed custom USB class interface to a PC running a custom design and programmed GUI application. The application was designed in Qt, and written in C++.
