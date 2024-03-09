# Quadmod
![Image](https://raw.githubusercontent.com/dchwebb/Quadmod/master/Graphics/panel.jpg "icon")

Quadmod
--------

Quadmod is a Eurorack modular four channel effect module. A modulation section allows either phasing or flanging followed by a delay. As part of the Mountjoy Modular polyphonic range the module uses a series of intermodulation paths to convert a four channel input to a stereo output.

### Phaser
The phaser uses 4 (configurable up to 16) all pass filters to create the phasing effect. Each channel's all pass filter is 45° out of phase with the last. The regeneration (feedback) control progressively mixes one channel to the next to widen the stereo spectrum.

### Flanger
The flanger uses multiple short modulated delays to create a flanger/chorus effect. The delay times are modulated with a 45° phase shift from channel to channel. A wide option (on by default) mixes a further 180° phase shifted version of each channel's signal to the next channel.


Technical
---------

Quadmod uses an STM32563 microcontroller which controls a four channel Audio Codec (AsahiKasei AK4619) using SPI for configuration and SAI (Serial Ausio Interface) peripheral to manage the 4 in and 4 out I2S audio streams. The microcontroller reads the front panel controls with internal 12 bit ADCs.

![Image](https://raw.githubusercontent.com/dchwebb/Quadmod/master/Graphics/components.jpg "icon")

A USB socket is provided for serial configuration and a web editor using WebSerial provides a UI for editing settings. Three LEDs are controlled with PWM from the MCU to indicate modulation LFO rates and delay times.

Construction is a sandwich of three PCBs with a component board, a controls board and a panel. PCBs designed in Kicad v7.

[Components schematic](https://raw.githubusercontent.com/dchwebb/Quadmod/master/Hardware/Quadmod_Components.pdf)

[Controls schematic](https://raw.githubusercontent.com/dchwebb/Quadmod/master/Hardware/Quadmod_Controls.pdf)

In addition the the Eurorack +/-12V rails (which have polarity protection and filtering) a digital 3.3v rail is supplied using a TI TPS561201 and analog 3.3v is generated with an LM1117-3.3.

Errata
------

Version 1 of the component PCB had microcontroller pins PC0 (SAI2_SD_A) and PC1 (DLY_TIME_POT) flipped.