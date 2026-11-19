\page bitrl_example_arduino_wifi An Arduino over WiFi

\ref bitrl_example_arduino_usb over WiFi: the board runs a small web server, and the laptop sends
it `ON` and `OFF` in HTTP requests with
\ref bitrl::boards::arduino::ArduinoConnectorWIFIBase "ArduinoConnectorWIFIBase". The board
answers, and the example prints the answer.

## What you need

- an Arduino UNO R4 WiFi, or another board that the `WiFiS3` library supports;
- the sketch in the comment at the top of `arduino_wifi.cpp`, with your network's name and password
  in `ssid` and `password`. It joins the network, prints its address on the serial monitor and
  listens on port 8005;
- the laptop on the same network.

## Running the example

Put the board's address in the code, `http://192.168.0.70:8005` in the example, and run

@code{.sh}
./arduino_wifi
@endcode

Type `ON` or `OFF` and Enter; the board switches its LED and answers `LED is ON` or `LED is OFF`.
`e` switches it off and stops.

## The code

\include arduino_wifi.cpp
