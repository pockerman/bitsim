# These need a board, a broker or a camera server; see their pages
BITSIM_ADD_EXAMPLE(arduino_usb)
BITSIM_ADD_EXAMPLE(arduino_wifi)
BITSIM_ADD_EXAMPLE(mqtt_ultrasonic_sensor REQUIRES MQTT)
BITSIM_ADD_EXAMPLE(mqtt_camera REQUIRES MQTT OPENCV)
# With --simulated-motors it runs without a board
BITSIM_ADD_EXAMPLE(real_dc_motors REQUIRES CHRONO MQTT SMOKE_TEST SMOKE_ARGS --simulated-motors)
BITSIM_ADD_EXAMPLE(pi_camera_line_following REQUIRES CHRONO OPENCV)
