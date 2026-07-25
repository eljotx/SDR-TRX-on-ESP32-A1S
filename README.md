# SDR-TRX-on-ESP32-A1S -> JACK
This repository describes an SDR transceiver based on the ESP32 A1S processor, using an ILI9341 color touchscreen display, and a Si5351 oscillator. The program was written on the Arduino platform and is still in development, but the most important features have already been implemented and tested.

![JACK](/jack.jpg)

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

The core of the system is the ESP32 A1S processor, which is suitable for implementing the project because it contains built-in internal A/D and D/A converters and operates at a speed that allows for sufficiently fast sampling and the use of digital filters of desired complexity. Although the A/D and D/A converters have a resolution of 16 bits, this seems sufficient for a design that doesn't aspire to perfection, but rather addresses basic digital processing issues. Additionally, the processor has two cores, allowing for the separation of tasks that don't require immediate processing from those related to audio streaming. An additional advantage of the processor is the ability to use two I2c channels for communication with peripherals and a channel SPI for communication with the display. All of these features are discussed in more detail in the following descriptions.

# How receiver works
The antenna signal reaches a Tayloe mixer, where I and Q signals are generated. Both signals contain sideband information, which is used for digital data processing. To achieve this, the analog I and Q signals are digitized using 16-bit A/D converters. A Hilbert transform is then applied to one of the channels (shifting the signal's phase by 90 degrees). The signals are then digitally filtered using a BiQuad digital filter. Three filters are available: two low-pass filters at 3.6 kHz and 2.4 kHz, and a bandpass filter at 800 Hz for CW reception. Finally, the I and Q signals are converted back to analog using 16-bit D/A converters and summed, reducing the sideband of the received signal. The received sideband is selected in the Tayloe mixer by changing the phase of the heterodyne signals, reducing the load on the digital software.

![Receiver](/rx_block.jpg)

The figure shows the signal waveform during reception in more detail. The antenna signal passes through an adjustable PIN diode attenuator and an ERA6 amplifier, and is then detected in the Taylor mixer. The I and Q signals extracted from the mixer, after summing, detection, and amplification, are used to adjust the gain using the PIN attenuator, and also, through AGC signal measurement, visualize the received signal strength. Simultaneously, the extracted I and Q signals undergo digital processing, as described earlier, and after conversion to analog and amplification, they can be received via headphones or a loudspeaker.

# How transceiver works

## SSB mode
In SSB mode, the transmitter initially digitizes the microphone signal using A/D converters. Subsequently, a Hilbert transform is performed on one of the signals, followed by a common LPF digital filtering with a 2.4 kHz bandwidth. This creates two digital I and Q signals, which are then converted to analog using D/A converters. The SSB signal is formed in a Tayloe mixer using two heterodyne signals that are 90 degrees out of phase. Similar to receiving, the sideband is selected by determining which heterodyne signal has a +90 degree offset relative to the other. This signal is then amplified and fed to the antenna. 

## CW mode
In CW mode, the Tayloe mixer is disabled (heterodyne stop), and an additional keyed heterodyne is activated at the device's operating frequency, with a sideband offset of approximately +800 Hz, depending on the sideband used.

![Transceiver](/tx_block.jpg)

In addition to the previously described CW and SSB signal generation process, the figure shows how common circuit elements are used. Specifically, the microphone signal is pre-amplified in the same amplifier used for reception, and after shaping the I and Q channels, it feeds the Tayloe mixer. The final output signal is generated at the Tayloe mixer output in SSB mode or directly from the Si5351 generator in CW mode.
