Firmware for Wemos D1 Mini I2C motor shield which allows for vibration commands to be sent.

See also:

* https://github.com/smartin015/i2c_toothbrush for application (vibrating toothbrushes to MIDI to make music)


## Flashing

See https://hackaday.io/project/18439-motor-shield-reprogramming for original flashing instructions

Flashing the motor shield requires shorting RTS to 3V (to enable programming mode), then using D1/D2 as TX/RX with GND and 3V3 connected on your FTDI Friend or other serial programmer device.

I use STM32Cube Programmer to program the device over USB/UART. The code must be opened with STM32Cube IDE.
