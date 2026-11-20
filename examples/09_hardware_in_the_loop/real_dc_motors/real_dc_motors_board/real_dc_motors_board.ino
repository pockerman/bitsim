// real_dc_motors_board: the motor side of the real_dc_motors example
//
// Drives the two wheel motors of a differential drive robot and talks to the
// simulation over MQTT:
//
//   subscribes to bitsim/dc_board/command  {"seq": 12, "left_speed": 2.5, "right_speed": 2.5}
//   publishes  to bitsim/dc_board/state    {"ack": 12, "left_distance": 0.53, "right_distance": 0.52,
//                                          "left_speed": 2.49, "right_speed": 2.51, "t_ms": 81234}
//
// Speeds are wheel speeds in rad/s and distances are in m, counted from when
// the board started. The simulation turns the distances back into wheel angles
// with the same WHEEL_RADIUS, so keep it equal to the wheel radius of the
// simulated robot.
//
// Hardware: an Arduino UNO R4 WiFi or Nano 33 IoT, a TB6612FNG dual motor
// driver, and two DC gear motors with quadrature encoders, such as the
// JGA25-370. Libraries: ArduinoMqttClient, from the library manager.
//
// Lift the robot so that its wheels turn freely before you run this.
//
// Copy arduino_secrets.example.h to arduino_secrets.h and fill it in first.

#include <ArduinoMqttClient.h>

#if defined(ARDUINO_UNOR4_WIFI)
#include <WiFiS3.h>
#else
#include <WiFiNINA.h>
#endif

#include "arduino_secrets.h"

// ---------------------------------------------------------------------------
// Configuration. Change these to match your hardware. The motor constants and
// the controller gains are those of the "motors" section of
// robots/bitsim_diff_drive_robot.json, so the simulated motors behave the same
// ---------------------------------------------------------------------------
const float SUPPLY_VOLTAGE = 12.0;   // the driver supply [V]
const float GEAR_RATIO = 30.0;       // rotor turns per wheel turn
const float EMF_CONSTANT = 0.0127;   // at the rotor [V s/rad]
const long COUNTS_PER_TURN = 660;    // encoder counts per wheel turn: 2 edges x 11 pulses x 30
const float WHEEL_RADIUS = 0.1;      // the wheel of the simulated robot [m]

const float SPEED_KP = 0.03;         // duty cycle per rad/s of error
const float SPEED_KI = 0.1;          // duty cycle per rad of error
const float SPEED_FILTER = 0.02;     // time constant of the speed filter [s]
const unsigned long CONTROL_PERIOD_MS = 10;

// The TB6612FNG: a PWM pin and two direction pins per motor, and standby
const int STANDBY_PIN = 12;

struct MotorPins
{
    int pwm;
    int in1;
    int in2;
    int encoder_a; // must be able to interrupt
    int encoder_b;
    bool reversed; // the motors face opposite ways on the robot
};

const MotorPins PINS[2] = {
    {5, 8, 9, 2, 4, false}, // left
    {6, 10, 11, 3, 7, true}, // right
};

const char *COMMAND_TOPIC = "bitsim/dc_board/command";
const char *STATE_TOPIC = "bitsim/dc_board/state";
const char *NAMES[2] = {"left", "right"};

// The network is slow compared with the control loop, so we only talk to it
// every NETWORK_PERIOD_MS
const unsigned long NETWORK_PERIOD_MS = 20;

// Stop the motors if the simulation goes quiet for this long
const unsigned long COMMAND_TIMEOUT_MS = 500;

// ---------------------------------------------------------------------------
// The motors
// ---------------------------------------------------------------------------
volatile long encoder_counts[2] = {0, 0};

// One count per edge of channel A. Channel B tells the direction
void on_encoder(int m)
{
    const bool a = digitalRead(PINS[m].encoder_a);
    const bool b = digitalRead(PINS[m].encoder_b);
    const int step = (a == b) ? 1 : -1;
    encoder_counts[m] += PINS[m].reversed ? -step : step;
}
void on_left_encoder() { on_encoder(0); }
void on_right_encoder() { on_encoder(1); }

struct SpeedController
{
    float target = 0.0;
    float measured = 0.0;
    float integral = 0.0;
    long last_counts = 0;
};

SpeedController controllers[2];

long read_counts(int m)
{
    noInterrupts();
    const long counts = encoder_counts[m];
    interrupts();
    return counts;
}

void set_duty(int m, float duty)
{
    duty = constrain(duty, -1.0, 1.0);
    if (PINS[m].reversed)
    {
        duty = -duty;
    }
    digitalWrite(PINS[m].in1, duty >= 0.0 ? HIGH : LOW);
    digitalWrite(PINS[m].in2, duty >= 0.0 ? LOW : HIGH);
    analogWrite(PINS[m].pwm, (int)(fabs(duty) * 255.0));
}

// The same controller as the SIMULATION backend of bitsim: the speed from the
// encoder counts, filtered, then a back-EMF feedforward and a PI controller
void control(int m, float period)
{
    SpeedController &c = controllers[m];

    const long counts = read_counts(m);
    const float raw_speed = (counts - c.last_counts) * 2.0 * PI / (COUNTS_PER_TURN * period);
    c.last_counts = counts;
    c.measured += (raw_speed - c.measured) * period / (SPEED_FILTER + period);

    const float feedforward = c.target * GEAR_RATIO * EMF_CONSTANT / SUPPLY_VOLTAGE;
    const float error = c.target - c.measured;
    const float unclamped = feedforward + SPEED_KP * error + c.integral;
    const float duty = constrain(unclamped, -1.0, 1.0);

    // Anti-windup: stop integrating while saturated
    if (duty == unclamped || error * unclamped < 0.0)
    {
        c.integral += SPEED_KI * error * period;
    }
    set_duty(m, duty);
}

void stop_motors()
{
    for (int m = 0; m < 2; ++m)
    {
        controllers[m].target = 0.0;
        controllers[m].integral = 0.0;
        set_duty(m, 0.0);
    }
}

// ---------------------------------------------------------------------------
// The network
// ---------------------------------------------------------------------------
WiFiClient wifi_client;
MqttClient mqtt_client(wifi_client);

long command_seq = -1;
unsigned long last_command_ms = 0;

// Finds "key": in a flat JSON object and reads the number after it. This is
// enough for our messages and saves a JSON library
bool json_number(const char *text, const char *key, double &value)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *start = strstr(text, pattern);
    if (start == nullptr)
    {
        return false;
    }
    char *end = nullptr;
    value = strtod(start + strlen(pattern), &end);
    return end != start + strlen(pattern);
}

void on_message(int size)
{
    char text[160];
    int n = 0;
    while (mqtt_client.available() && n < (int)sizeof(text) - 1)
    {
        text[n++] = (char)mqtt_client.read();
    }
    text[n] = '\0';
    (void)size;

    double seq = 0.0;
    double left = 0.0;
    double right = 0.0;
    if (json_number(text, "seq", seq) && json_number(text, "left_speed", left) &&
        json_number(text, "right_speed", right))
    {
        command_seq = (long)seq;
        controllers[0].target = (float)left;
        controllers[1].target = (float)right;
        last_command_ms = millis();
    }
}

void connect_to_network()
{
    // The control loop stops while we wait, so stop the motors
    stop_motors();
    digitalWrite(STANDBY_PIN, LOW);

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("Connecting to WiFi ");
        Serial.println(SECRET_SSID);
        WiFi.begin(SECRET_SSID, SECRET_PASS);
        delay(2000);
    }

    while (!mqtt_client.connected())
    {
        Serial.print("Connecting to the MQTT broker ");
        Serial.println(SECRET_BROKER);
        if (!mqtt_client.connect(SECRET_BROKER, SECRET_BROKER_PORT))
        {
            Serial.print("failed, error ");
            Serial.println(mqtt_client.connectError());
            delay(2000);
        }
    }

    mqtt_client.onMessage(on_message);
    mqtt_client.subscribe(COMMAND_TOPIC);
    Serial.println("Connected");

    digitalWrite(STANDBY_PIN, HIGH);
}

void publish_state()
{
    mqtt_client.beginMessage(STATE_TOPIC);
    mqtt_client.print("{\"ack\": ");
    mqtt_client.print(command_seq);
    for (int m = 0; m < 2; ++m)
    {
        const float distance = read_counts(m) * 2.0 * PI * WHEEL_RADIUS / COUNTS_PER_TURN;
        mqtt_client.print(", \"");
        mqtt_client.print(NAMES[m]);
        mqtt_client.print("_distance\": ");
        mqtt_client.print(distance, 5);
        mqtt_client.print(", \"");
        mqtt_client.print(NAMES[m]);
        mqtt_client.print("_speed\": ");
        mqtt_client.print(controllers[m].measured, 4);
    }
    mqtt_client.print(", \"t_ms\": ");
    mqtt_client.print(millis());
    mqtt_client.print("}");
    mqtt_client.endMessage();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    pinMode(STANDBY_PIN, OUTPUT);
    for (int m = 0; m < 2; ++m)
    {
        pinMode(PINS[m].pwm, OUTPUT);
        pinMode(PINS[m].in1, OUTPUT);
        pinMode(PINS[m].in2, OUTPUT);
        pinMode(PINS[m].encoder_a, INPUT_PULLUP);
        pinMode(PINS[m].encoder_b, INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(PINS[0].encoder_a), on_left_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PINS[1].encoder_a), on_right_encoder, CHANGE);

    connect_to_network();
}

void loop()
{
    const unsigned long now = millis();

    static unsigned long last_control = 0;
    if (now - last_control >= CONTROL_PERIOD_MS)
    {
        const float period = (now - last_control) / 1000.0;
        last_control = now;
        for (int m = 0; m < 2; ++m)
        {
            control(m, period);
        }
    }

    static unsigned long last_network = 0;
    if (now - last_network >= NETWORK_PERIOD_MS)
    {
        last_network = now;

        if (WiFi.status() != WL_CONNECTED || !mqtt_client.connected())
        {
            connect_to_network();
        }

        mqtt_client.poll();

        if (now - last_command_ms > COMMAND_TIMEOUT_MS)
        {
            controllers[0].target = 0.0;
            controllers[1].target = 0.0;
        }

        publish_state();
    }
}
