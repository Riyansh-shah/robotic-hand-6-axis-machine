/**
 * 6-Axis Robotic Arm + Extruder Firmware 
 * Board: Arduino Mega 2560 (with RAMPS 1.4 or similar shield)
 *
 * Implements the binary serial protocol matching control/protocol.py.
 */

#include <Arduino.h>
#include <AccelStepper.h>

// --- System Configuration & Constants ---
#define NUM_AXES 7 // 6 Joints + 1 Extruder
#define BAUD_RATE 115200

// Conversion Scales (Match Python Protocol)
#define ANGLE_SCALE 100.0f
#define EXTRUDER_SCALE 1000.0f

// --- Protocol Bytes ---
#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

// Commands
#define CMD_MOVE           0x01
#define CMD_HOME           0x02
#define CMD_SET_SPEED      0x03
#define CMD_ENABLE         0x04
#define CMD_DISABLE        0x05
#define CMD_QUERY          0x06
#define CMD_SET_TEMP       0x07
#define CMD_SET_FAN        0x08
#define CMD_EMERGENCY_STOP 0xFF

// Responses 
#define RESP_ACK  0xAA
#define RESP_NACK 0x55
#define RESP_DATA 0x66

// --- Hardware Pins (Mocks assuming typical CNC shield) ---
const uint8_t STEP_PINS[NUM_AXES] = {54, 60, 46, 26, 36, 42, 48}; // Example layout
const uint8_t DIR_PINS[NUM_AXES]  = {55, 61, 48, 28, 34, 40, 49};
const uint8_t ENABLE_PIN = 38; // Shared enable pin

const uint8_t ENDSTOP_PINS[NUM_AXES] = {3, 2, 14, 15, 18, 19, 255}; // Extruder (index 6) has no endstop
const uint8_t HEATER_PIN = 10;
const uint8_t THERMISTOR_PIN = A0;

// --- Global States ---
AccelStepper* steppers[NUM_AXES];
bool motors_enabled = false;
float current_temperature = 25.0f; 
float target_temperature = 0.0f;
uint8_t fan_speed = 0;
uint16_t error_flags = 0;

// Internal Serial Buffer
const uint8_t MAX_PAYLOAD_SIZE = 64;

// Forward Declarations
void handle_command(uint8_t cmd, uint8_t len, uint8_t* payload);
void send_ack();
void send_nack(uint8_t error_code);
void send_data();
uint8_t calculate_checksum(uint8_t* data, uint8_t len);
float read_temperature();
void update_heater_pid();

void setup() {
    Serial.begin(BAUD_RATE);
    
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH); // Disable steppers initially (Logic High = Disabled on A4988/TMC)

    // Initialize Stepper Motors
    for (int i = 0; i < NUM_AXES; i++) {
        steppers[i] = new AccelStepper(AccelStepper::DRIVER, STEP_PINS[i], DIR_PINS[i]);
        steppers[i]->setMaxSpeed(1000.0);
        steppers[i]->setAcceleration(500.0);
    }

    // Initialize Limit Switches
    for (int i = 0; i < NUM_AXES - 1; i++) {
        pinMode(ENDSTOP_PINS[i], INPUT_PULLUP);
    }
    
    pinMode(HEATER_PIN, OUTPUT);
    digitalWrite(HEATER_PIN, LOW);
}

void loop() {
    // Process Serial Input State Machine
    static uint8_t state = 0;
    static uint8_t cmd = 0;
    static uint8_t payload_len = 0;
    static uint8_t payload[MAX_PAYLOAD_SIZE];
    static uint8_t payload_idx = 0;

    while (Serial.available() > 0) {
        uint8_t b = Serial.read();

        switch (state) {
            case 0: // Wait for SYNC 1
                if (b == SYNC_BYTE_1) state = 1;
                break;
            case 1: // Wait for SYNC 2
                if (b == SYNC_BYTE_2) state = 2;
                else state = 0; // Reset
                break;
            case 2: // Read Command Code
                cmd = b;
                state = 3;
                break;
            case 3: // Read Payload Length
                payload_len = b;
                if (payload_len > MAX_PAYLOAD_SIZE) {
                    state = 0; // Packet too large, drop it
                    // Could send_nack here, but sync is likely lost
                } else if (payload_len == 0) {
                    state = 5; // Go straight to checksum
                } else {
                    payload_idx = 0;
                    state = 4;
                }
                break;
            case 4: // Read Payload
                payload[payload_idx++] = b;
                if (payload_idx >= payload_len) {
                    state = 5;
                }
                break;
            case 5: // Read and Verify Checksum
                uint8_t calc_cs = calculate_checksum(payload, payload_len);
                if (b == calc_cs) {
                    handle_command(cmd, payload_len, payload);
                } else {
                    send_nack(0xEE); // Checksum Error
                }
                state = 0; // Reset for next frame
                break;
        }
    }

    // Move Motors if enabled
    if (motors_enabled) {
        for (int i = 0; i < NUM_AXES; i++) {
            steppers[i]->run();
        }
    }
    
    // Update Heater PID loop (avoid blocking)
    static unsigned long last_pid_time = 0;
    if (millis() - last_pid_time > 100) {
        current_temperature = read_temperature();
        update_heater_pid();
        last_pid_time = millis();
    }
}

// --- Protocol Implementation ---

void handle_command(uint8_t cmd, uint8_t len, uint8_t* payload) {
    switch (cmd) {
        case CMD_MOVE: {
            if (len < 20) { // 6 * int16 (12) + int32 (4) + float (4) = 20
                send_nack(0x01);
                return;
            }
            // Parse joint angles (12 bytes)
            for (int i = 0; i < 6; i++) {
                int16_t angle_raw = payload[i*2] | (payload[i*2 + 1] << 8);
                float angle_deg = (float)angle_raw / ANGLE_SCALE;
                
                // TODO: Map Degrees to Stepper Steps depending on gear ratio
                long target_steps = (long)(angle_deg * 200.0); // Arbitrary mapping (replace with real kinematic ratio)
                steppers[i]->moveTo(target_steps);
            }

            // Parse extruder pos (4 bytes)
            int32_t e_raw = payload[12] | ((int32_t)payload[13] << 8) | ((int32_t)payload[14] << 16) | ((int32_t)payload[15] << 24);
            float e_pos = (float)e_raw / EXTRUDER_SCALE;
            long e_steps = (long)(e_pos * 400.0); // Arbitrary steps/mm
            steppers[6]->moveTo(e_steps);

            // Ignore feedrate payload[16..19] for now in this mock hardware script
            
            send_ack();
            break;
        }
        case CMD_HOME: {
            // Homing Sequence for each axis matching RAMPS style
            for (int i = 0; i < NUM_AXES - 1; i++) {
                // Determine direction (assuming negative moves toward 0)
                steppers[i]->setSpeed(-500.0);
                // Step until switch triggers (LOW due to INPUT_PULLUP)
                while (digitalRead(ENDSTOP_PINS[i]) == HIGH) {
                    steppers[i]->runSpeed();
                }
                steppers[i]->setCurrentPosition(0);
            }
            send_ack();
            break;
        }
        case CMD_ENABLE:
            motors_enabled = true;
            digitalWrite(ENABLE_PIN, LOW); // Logic Low enables A4988 
            send_ack();
            break;
        case CMD_DISABLE:
        case CMD_EMERGENCY_STOP:
            motors_enabled = false;
            digitalWrite(ENABLE_PIN, HIGH);
            error_flags |= (cmd == CMD_EMERGENCY_STOP) ? 0x01 : 0x00;
            send_ack();
            break;
        case CMD_SET_TEMP:
            if (len >= 4) memcpy(&target_temperature, payload, 4);
            send_ack();
            break;
        case CMD_SET_FAN:
            if (len >= 1) fan_speed = payload[0];
            send_ack();
            break;
        case CMD_QUERY:
            send_data();
            break;
        default:
            send_nack(0xFF); // Unknown Command
    }
}

uint8_t calculate_checksum(uint8_t* data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

void send_ack() {
    uint8_t frame[] = {SYNC_BYTE_1, SYNC_BYTE_2, RESP_ACK, 0, 0}; // Checksum of 0 is 0
    Serial.write(frame, sizeof(frame));
}

void send_nack(uint8_t error_code) {
    uint8_t frame[] = {SYNC_BYTE_1, SYNC_BYTE_2, RESP_NACK, 1, error_code, error_code};
    Serial.write(frame, sizeof(frame));
}

void send_data() {
    uint8_t payload[22];
    
    // Load Angles
    for(int i = 0; i < 6; i++) {
        // Mock current pos reversed lookup
        float pos_deg = steppers[i]->currentPosition() / 200.0f;
        int16_t angle_raw = (int16_t)(pos_deg * ANGLE_SCALE);
        payload[i*2] = angle_raw & 0xFF;
        payload[i*2 + 1] = (angle_raw >> 8) & 0xFF;
    }

    // Load E Pos
    float e_mm = steppers[6]->currentPosition() / 400.0f;
    int32_t e_raw = (int32_t)(e_mm * EXTRUDER_SCALE);
    payload[12] = e_raw & 0xFF;
    payload[13] = (e_raw >> 8) & 0xFF;
    payload[14] = (e_raw >> 16) & 0xFF;
    payload[15] = (e_raw >> 24) & 0xFF;

    // Load Temp
    memcpy(&payload[16], &current_temperature, 4);

    // Load Errors
    payload[20] = error_flags & 0xFF;
    payload[21] = (error_flags >> 8) & 0xFF;

    uint8_t cs = calculate_checksum(payload, 22);

    Serial.write(SYNC_BYTE_1);
    Serial.write(SYNC_BYTE_2);
    Serial.write(RESP_DATA);
    Serial.write(22);
    Serial.write(payload, 22);
    Serial.write(cs);
}

// --- Sensor & PID Implementation ---

float read_temperature() {
    int raw_adc = analogRead(THERMISTOR_PIN);
    if (raw_adc == 0) return 0.0f; // Prevent div by 0

    // Typical 100k NTC Thermistor Steinhart-Hart
    float r_series = 4700.0f;
    float r_thermistor = r_series * (1023.0f / (float)raw_adc - 1.0f);
    
    // Beta parameters
    float t0 = 298.15f; // 25C
    float r0 = 100000.0f;
    float beta = 3950.0f;
    
    float steinhart = r_thermistor / r0;
    steinhart = log(steinhart);
    steinhart /= beta;
    steinhart += 1.0f / t0;
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f; // Convert K to C
    
    return steinhart;
}

void update_heater_pid() {
    static float integral = 0.0f;
    static float last_error = 0.0f;
    
    // PID Tuning Constants (would be manually tuned)
    const float Kp = 15.0f;
    const float Ki = 0.8f;
    const float Kd = 40.0f;
    
    if (target_temperature <= 0.0f || !motors_enabled) {
        analogWrite(HEATER_PIN, 0);
        integral = 0.0f;
        return;
    }
    
    float error = target_temperature - current_temperature;
    integral += error;
    
    // Anti-windup
    if (integral > 100.0f) integral = 100.0f;
    if (integral < -100.0f) integral = -100.0f;
    
    float derivative = error - last_error;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    int pwm_val = (int)output;
    if (pwm_val > 255) pwm_val = 255;
    if (pwm_val < 0) pwm_val = 0;
    
    analogWrite(HEATER_PIN, pwm_val);
    last_error = error;
}
