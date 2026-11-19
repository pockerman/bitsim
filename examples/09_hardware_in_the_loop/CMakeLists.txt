# These need a board, a broker or a camera server; see their pages
BITRL_ADD_EXAMPLE(arduino_usb)
BITRL_ADD_EXAMPLE(arduino_wifi)
BITRL_ADD_EXAMPLE(mqtt_ultrasonic_sensor REQUIRES MQTT)
BITRL_ADD_EXAMPLE(mqtt_camera REQUIRES MQTT OPENCV)
# With --simulated-motors it runs without a board
BITRL_ADD_EXAMPLE(real_dc_motors REQUIRES CHRONO MQTT SMOKE_TEST SMOKE_ARGS --simulated-motors)
BITRL_ADD_EXAMPLE(pi_camera_line_following REQUIRES CHRONO OPENCV)
