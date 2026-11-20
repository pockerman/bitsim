\page bitsim_example_arduino_usb An Arduino over USB

The first step to hardware in the loop: the laptop tells a board what to do. Here it switches the
LED of an Arduino on and off over the USB cable, with
\ref bitsim::boards::arduino::ArduinoConnectorUSBBase "ArduinoConnectorUSBBase".

## What you need

- an Arduino UNO, or any board with an LED on `LED_BUILTIN`, on `/dev/ttyACM0`;
- the sketch in the comment at the top of `arduino_usb.cpp`, uploaded with the Arduino IDE. It
  reads a line from the serial port at 9600 baud and switches the LED on for `ON` and off for `OFF`.

## Commands

A command is a \ref bitsim::boards::arduino::ArduinoCMDBase "ArduinoCMDBase" whose `get_cmd()` gives
the text to send. The example has two, `ArduinoONCMD` and `ArduinoOFFCMD`; the connector writes
their text, with a new line, to the serial port.

## Running the example

@code{.sh}
./arduino_usb
@endcode

Type `ON` or `OFF` and Enter to switch the LED, and `e` to switch it off and stop. If the board is
on another port, change `/dev/ttyACM0` in the code; on Linux you may need to be in the `dialout`
group to open it.

\ref bitsim_example_arduino_wifi does the same over WiFi, and \ref bitsim_example_real_dc_motors
drives the motors of a robot from a board.

## The code

\include arduino_usb.cpp
