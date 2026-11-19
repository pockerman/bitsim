\page bitrl_example_mqtt_camera Camera frames over MQTT

A camera on a robot sends its pictures to the laptop. Here each frame is a JPEG, base64 encoded, in
an MQTT message, and the example decodes it with
\ref bitrl::sensors::CameraMessage "CameraMessage" and shows it with OpenCV.
\ref bitrl_example_pi_camera_line_following streams a Raspberry Pi camera over HTTP instead, which
suits a continuous stream better.

## What you need

- bitrl built with MQTT and OpenCV; see \ref bitrl_install;
- an MQTT broker on `localhost:1883`, such as Mosquitto;
- `camera_feed.py` in this example's directory, which needs Python, `opencv-python` and
  `paho-mqtt`, and a camera that OpenCV can open as device 0.

## Running the example

@code{.sh}
mosquitto              # the broker, if none is running
./mqtt_camera
python camera_feed.py  # in another terminal
@endcode

`camera_feed.py` publishes frames on the topic `camera`. The example shows each frame it receives
in a window and waits for a key before the next one.

## The code

\include mqtt_camera.cpp
