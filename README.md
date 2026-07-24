# SDR-TRX-on-ESP32-A1S
This repository describes an SDR transceiver based on the ESP32 A1S processor, using an ILI9341 color touchscreen display, and a Si5351 oscillator. The program was written on the Arduino platform and is still in development, but the most important features have already been implemented and tested.

# General Idea

![Basic idea](/schem_gen.jpg)

The transceiver was built using a phase-based SSB signal generation method. Due to the breadth of the topic and the abundance of descriptions available online, this method will not be described here. However, I encourage you to study this topic theoretically, which will help you understand the presented solution.

Crucial to the solution is the use of **Tayloe mixer on FST3253, A/D conversion, Hilbert transforms, digital filters, and D/A conversion** in the audio paths of the receiver and transmitter.

## Local oscilator 

![Generator Si5351](/si5351.jpg)

The Si5351 acts as a local oscillator, clocking the transmitter and receiver mixers at a frequency that ensures proper detection or modulation. In this case, the clock signals have a frequency twice that of the useful signal, with an additional 90-degree shift between them, achieved using an additional shaping circuit based on integrated circuits 7474 and 7486. It is possible to generate the mixer signals directly within the Si5351, but for operation above 3.5 MHz, the controller needs to be modified, but the biggest inconvenience is the need to reset the PLL synchronization loop. This synchronization generates delays that are perceived as strong clicks during receive mode.

## Tayloe Mixer

![Mixer FST3253](/fst3253.jpg)

The FST3253 is a dual, quadruple, high-speed digital keyer that acts as a keyed mixer for both the transmitter and receiver. During reception, the mixer extracts the I and Q components from the received signal for further digital processing. During transmission, it modulates the RF signal with I and Q components, creating a lower or upper sideband signal.

## Processor ESP32 A1S

![Procesor ESP32A1S](/esp32a1s.jpg)

The core of the system is the ESP32 A1S processor, which is suitable for implementing the project because it contains built-in internal A/D and D/A converters and operates at a speed that allows for sufficiently fast sampling and the use of digital filters of desired complexity. Although the A/D and D/A converters have a resolution of 16 bits, this seems sufficient for a design that doesn't aspire to perfection, but rather addresses basic digital processing issues. Additionally, the processor has two cores, allowing for the separation of tasks that don't require immediate processing from those related to audio streaming. An additional advantage of the processor is the ability to use two I2c channels for communication with peripherals and a channel for communication with the display. All of these features are discussed in more detail in the following descriptions.

