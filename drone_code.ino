#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <esp_task_wdt.h> 

// ==========================================
// ===          USER CONFIGURATION        ===
// ==========================================

const char* WIFI_SSID       = "SSID";       
const char* WIFI_PASSWORD   = "Password";   
const char* MQTT_BROKER_IP  = "172.20.**.*";        
const int   MQTT_PORT       = 1883;
const char* MQTT_CLIENT_ID  = "esp32-drone-nimsara-final";

#define MAG_DECLINATION  -1.95 

float FORCE_ACC_X = 0;
float FORCE_ACC_Y = 0;
float FORCE_ACC_Z = 0;
float FORCE_GYRO_X = 0;
float FORCE_GYRO_Y = 0;
float FORCE_GYRO_Z = 0;

// ==========================================
// ===          SYSTEM CONSTANTS          ===
// ==========================================

#define WDT_TIMEOUT 3 
#define D_TERM_LPF_HZ 20.0f  
#define Q_ANGLE 0.005f 
#define Q_BIAS  0.0001f
#define R_ACCEL 0.1f    
#define R_MAG   0.8f    

#define RAD_TO_DEG 57.2957795131
#define DEG_TO_RAD 0.01745329251

#define ESC1_PIN 26 
#define ESC2_PIN 32 
#define ESC3_PIN 25 
#define ESC4_PIN 27 

Servo esc1, esc2, esc3, esc4;

const int MPU_ADDR = 0x68;
int MAG_ADDR = 0x1E; 
bool is_qmc_mag = false;

#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define MAX_SAFE_THROTTLE 1900
#define IDLE_THROTTLE 1050  
#define HOVER_THROTTLE 1450 

#define LOW_BATTERY_LIMIT 10.5 
#define CRASH_ANGLE_LIMIT 45.0f 

#define THROTTLE_SLEW_RATE_PER_CYCLE 20 
#define CORE1_LOOP_INTERVAL_MS 10  
#define CORE1_MUTEX_TIMEOUT_MS 2   
#define TELEMETRY_INTERVAL_MS 150  
#define CONNECTION_TIMEOUT_MS 4000 
#define SEA_LEVEL_PRESSURE_HPA 1013.25 

#define NOMINAL_BATTERY_VOLTAGE 12.0 
float voltage_compensation_factor = 1.0;

TaskHandle_t autoTaskHandle = NULL;
SemaphoreHandle_t pidMutex;

Adafruit_BMP280 bmp; 
bool bmp_detected = false; 
bool mag_detected = false;
float current_altitude_m = 0.0;
float current_vspeed_mps = 0.0; 
float ground_reference_altitude = 0.0f;

struct Core1to0_Data {
    float Kp_p = 1.0, Ki_p = 0.00, Kd_p = .0; 
    float Kp_r = 2.5, Ki_r = 0.00, Kd_r = 8.0;
    float Kp_y = 5.0, Ki_y = 0.00, Kd_y = 0.0; 
    bool escsArmed = false;
    bool autoModeActive = false;
    int targetThrottle = MIN_THROTTLE;
    float man_setpoint_pitch = 0;
    float man_setpoint_roll = 0;
    float man_setpoint_yaw = 0; 
};

struct Core0to1_Data {
    float pidPitchOut = 0;
    float pidRollOut = 0;
    float pidYawOut = 0;
    float telemetry_pitch = 0;
    float telemetry_roll = 0;
    float telemetry_yaw = 0;
    float compass_heading = 0;
    int status_code = 0; 
    float telemetry_altitude_m = 0.0;
    float vertical_speed_mps = 0.0;
};

Core1to0_Data core1to0_data;
Core1to0_Data core1_local_C1to0_data; 
Core0to1_Data core0to1_data;
Core0to1_Data core1_local_C0to1_data;

float prev_input_pitch = 0, integral_pitch = 0;
float prev_input_roll = 0, integral_roll = 0;
float prev_input_yaw = 0, integral_yaw = 0;
float pitch = 0, roll = 0, yaw = 0;
float yaw_offset = 0; 

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0; 
float accX_offset = 0, accY_offset = 0, accZ_offset = 0;
int mag_offset_x = 91, mag_offset_y = -192, mag_offset_z = 47;

float dt;
unsigned long prevMicros;

const int BATTERY_PIN = 34;
const float ADC_MAX_VOLTAGE = 3.3;
const int ADC_SAMPLES = 20;
const float VOLTAGE_MULTIPLIER = 20.18; 
float batteryVoltage = 12.0; 

#define LED_PIN 2

int currentThrottle = MIN_THROTTLE; 
int esc1_speed = MIN_THROTTLE, esc2_speed = MIN_THROTTLE;
int esc3_speed = MIN_THROTTLE, esc4_speed = MIN_THROTTLE;

unsigned long lastCommandTime = 0;
unsigned long lastTelemetrySend = 0;
unsigned long lastLoopTime = 0;
unsigned long lastReconnectAttempt = 0;
bool failsafe_active = false;

const char* COMMAND_TOPIC   = "esp32/simple_drone/command";
const char* TELEMETRY_TOPIC = "esp32/simple_drone/telemetry";
const char* STATUS_TOPIC    = "drone/control/status";
const char* MODE_TOPIC      = "drone/control/mode";

WiFiClient espClient;
PubSubClient client(espClient);

enum LED_Status {
    LED_WIFI_SEARCHING, // Fast blink
    LED_MQTT_CONNECTING, // Slow blink
    LED_READY_TO_ARM,   // Short blips
    LED_ARMED,          // Solid ON
    LED_FAILSAFE        // Rapid strobe
};

class LowPassFilter {
    float alpha;
    float last_out;
  public:
    LowPassFilter() { alpha = 1.0; last_out = 0.0; }
    void init(float cutoff_freq, float dt) {
        float rc = 1.0f / (2.0f * 3.14159f * cutoff_freq);
        alpha = dt / (rc + dt);
    }
    float update(float input) {
        last_out = last_out + alpha * (input - last_out);
        return last_out;
    }
};

LowPassFilter lpf_pitch_d, lpf_roll_d, lpf_yaw_d;

class NotchFilter {
  private:
    float x1, x2, y1, y2;
    float a0, a1, a2, b1, b2;
  public:
    void init(float center_freq_hz, float bandwidth_hz, float sample_rate_hz) {
        float w0 = 2.0f * PI * center_freq_hz / sample_rate_hz;
        float bw = 2.0f * PI * bandwidth_hz / sample_rate_hz;
        float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bw * w0 / sinf(w0));
        
        a0 = 1.0f;
        a1 = -2.0f * cosf(w0);
        a2 = 1.0f;
        float b0 = 1.0f + alpha;
        b1 = -2.0f * cosf(w0);
        b2 = 1.0f - alpha;

        // Normalize
        a0 /= b0; a1 /= b0; a2 /= b0;
        b1 /= b0; b2 /= b0;
        x1 = x2 = y1 = y2 = 0;
    }

    float update(float x) {
        float y = a0*x + a1*x1 + a2*x2 - b1*y1 - b2*y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
};

// Create filters for each axis
NotchFilter notchPitch, notchRoll;

void updateLED_Pattern() {
    static unsigned long lastToggle = 0;
    static bool ledState = false;
    unsigned long now = millis();
    int interval = 500; 

    LED_Status currentPattern;

    if (failsafe_active) currentPattern = LED_FAILSAFE;
    else if (core1_local_C1to0_data.escsArmed) currentPattern = LED_ARMED;
    else if (WiFi.status() != WL_CONNECTED) currentPattern = LED_WIFI_SEARCHING;
    else if (!client.connected()) currentPattern = LED_MQTT_CONNECTING;
    else currentPattern = LED_READY_TO_ARM;

    switch (currentPattern) {
        case LED_WIFI_SEARCHING: interval = 100; break;  
        case LED_MQTT_CONNECTING: interval = 500; break; 
        case LED_READY_TO_ARM:   interval = 1000; break; 
        case LED_ARMED:          digitalWrite(LED_PIN, HIGH); ledState = true; return; 
        case LED_FAILSAFE:       interval = 50; break;   
    }

    if (now - lastToggle >= interval) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastToggle = now;
    }
}

class DroneEKF {
public:
    float q0, q1, q2, q3; 
    float bx, by, bz;     
    float P[7][7];        

    DroneEKF() {
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
        bx = 0.0f; by = 0.0f; bz = 0.0f;
        for(int i=0; i<7; i++) {
            for(int j=0; j<7; j++) P[i][j] = 0.0f;
            P[i][i] = 0.01f; 
        }
    }

    void predict(float gx, float gy, float gz, float dt) {
        float gxc = gx - bx;
        float gyc = gy - by;
        float gzc = gz - bz;
        float half_dt = 0.5f * dt;
        
        float q0_new = q0 + half_dt * (-q1*gxc - q2*gyc - q3*gzc);
        float q1_new = q1 + half_dt * ( q0*gxc - q3*gyc + q2*gzc);
        float q2_new = q2 + half_dt * ( q3*gxc + q0*gyc - q1*gzc);
        float q3_new = q3 + half_dt * (-q2*gxc + q1*gyc + q0*gzc);
        
        q0 = q0_new; q1 = q1_new; q2 = q2_new; q3 = q3_new;
        normalizeQuaternion();
        
        for(int i=0; i<4; i++) P[i][i] += Q_ANGLE * dt;
        for(int i=4; i<7; i++) P[i][i] += Q_BIAS * dt;
    }

    void updateAccel(float ax, float ay, float az) {
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        if(norm < 0.1f) return; 
        ax /= norm; ay /= norm; az /= norm;
        float vx = 2.0f * (q1*q3 - q0*q2);
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        float ex = ax - vx;
        float ey = ay - vy;
        float ez = az - vz;
        float err_norm = sqrtf(ex*ex + ey*ey + ez*ez);
        if (err_norm > 0.0f) {
            float K = (P[0][0] / (P[0][0] + R_ACCEL)); 
            q0 += K * (q1*ey - q2*ex);
            q1 += K * (q0*ey - q3*ex);
            q2 += K * (-q0*ex + q3*ey);
            q3 += K * (q1*ex + q2*ey);
            normalizeQuaternion();
            for(int i=0; i<7; i++) P[i][i] *= (1.0f - K); 
        }
    }
    
    void normalizeQuaternion() {
        float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm == 0.0f) { q0 = 1.0f; return; }
        norm = 1.0f / norm; 
        q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
    }

    void getEuler(float &roll, float &pitch, float &yaw) {
        roll  = atan2f(2.0f * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * RAD_TO_DEG;
        pitch = -asinf(constrain(2.0f * (q1*q3 - q0*q2), -1.0f, 1.0f)) * RAD_TO_DEG; 
        yaw   = atan2f(2.0f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * RAD_TO_DEG;
    }
};

DroneEKF ekf;

void setup_wifi();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void attempt_mqtt_reconnect();
void autoModeTask(void* pvParameters);
void logToDashboard(String message);
void update_escs_Core1();
void publishTelemetry_Core1();
float readBatteryVoltage(); 
void calibrateSensors();
void enableI2CBypass(); 
void initMagnetometer(); 
void emergencyDisarm_Core1();
void activateFailsafe_Core1();

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float computePID(float input, float target, float &prev_input, float &integral, LowPassFilter &lpf, float Kp, float Ki, float Kd, float dt, bool armed, int throttle_val) {
    float error = target - input;
    if(armed && throttle_val > 1100) {
        if(abs(error) < 30) { 
            integral += (error * dt);
            integral = constrain(integral, -80, 80); 
        }
    } else { integral = 0; }
    float P = Kp * error;
    float I = Ki * integral;
    float D = 0;
    if(dt > 0) {
        float raw_derivative = -(input - prev_input) / dt;
        D = Kd * lpf.update(raw_derivative); 
    }
    prev_input = input;
    return P + I + D;
}

float computePID_Yaw(float input, float target, float &prev_input, float &integral, LowPassFilter &lpf, float Kp, float Ki, float Kd, float dt, bool armed, int throttle_val) {
    float error = target - input;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    if(armed && throttle_val > 1100) {
        integral += (error * dt);
        integral = constrain(integral, -80, 80);
    } else { integral = 0; }
    float P = Kp * error;
    float I = Ki * integral;
    float D = 0;
    if(dt > 0) {
        float dInput = input - prev_input;
        if (dInput > 180) dInput -= 360;
        if (dInput < -180) dInput += 360;
        float raw_derivative = -dInput / dt;
        D = Kd * lpf.update(raw_derivative);
    }
    prev_input = input;
    return P + I + D;
}

void enableI2CBypass() {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6A); Wire.write(0x00); Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x37); Wire.write(0x02); Wire.endTransmission();
}

void initMagnetometer() {
    Wire.beginTransmission(0x1E);
    if (Wire.endTransmission() == 0) {
        MAG_ADDR = 0x1E; is_qmc_mag = false;
        Wire.beginTransmission(MAG_ADDR); Wire.write(0x00); Wire.write(0x70); Wire.endTransmission(); 
        Wire.beginTransmission(MAG_ADDR); Wire.write(0x01); Wire.write(0x20); Wire.endTransmission(); 
        Wire.beginTransmission(MAG_ADDR); Wire.write(0x02); Wire.write(0x00); Wire.endTransmission(); 
        mag_detected = true;
        return;
    }
    Wire.beginTransmission(0x0D);
    if (Wire.endTransmission() == 0) {
        MAG_ADDR = 0x0D; is_qmc_mag = true;
        Wire.beginTransmission(MAG_ADDR); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();
        Wire.beginTransmission(MAG_ADDR); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission();
        mag_detected = true;
        return;
    }
    mag_detected = false;
}

void calibrateSensors() {
    if (FORCE_ACC_Z != 0) {
        accX_offset = FORCE_ACC_X; accY_offset = FORCE_ACC_Y; accZ_offset = FORCE_ACC_Z;
        gyroX_offset = FORCE_GYRO_X; gyroY_offset = FORCE_GYRO_Y; gyroZ_offset = FORCE_GYRO_Z;
        return;
    }
    long ax_sum=0, ay_sum=0, az_sum=0, gx_sum=0, gy_sum=0, gz_sum=0;
    int samples = 1000;
    digitalWrite(LED_PIN, HIGH); delay(1000);
    for(int i=0; i<samples; i++) {
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);
        ax_sum += int16_t(Wire.read()<<8 | Wire.read());
        ay_sum += int16_t(Wire.read()<<8 | Wire.read());
        az_sum += int16_t(Wire.read()<<8 | Wire.read());
        Wire.read(); Wire.read(); 
        gx_sum += int16_t(Wire.read()<<8 | Wire.read());
        gy_sum += int16_t(Wire.read()<<8 | Wire.read());
        gz_sum += int16_t(Wire.read()<<8 | Wire.read());
        delay(2);
    }
    accX_offset = ax_sum / samples; accY_offset = ay_sum / samples;
    accZ_offset = (az_sum / samples) - 4096.0f; 
    gyroX_offset = gx_sum / samples; gyroY_offset = gy_sum / samples; gyroZ_offset = gz_sum / samples;
    digitalWrite(LED_PIN, LOW); 
}

void readBarometer_Core0() {
    if(!bmp_detected) return; 
    unsigned long now = millis();
    static unsigned long last_baro_read = 0;
    if (now - last_baro_read >= 50) { 
        float raw_altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
        if (!core1to0_data.escsArmed) {
            ground_reference_altitude = raw_altitude;
            current_altitude_m = 0.0f; current_vspeed_mps = 0.0f;
        } else {
            float relative_altitude = raw_altitude - ground_reference_altitude;
            float baro_dt = (now - last_baro_read) / 1000.0f; 
            if (baro_dt > 0.0f) current_vspeed_mps = (relative_altitude - current_altitude_m) / baro_dt;
            current_altitude_m = (current_altitude_m * 0.70f) + (relative_altitude * 0.30f);
        }
        last_baro_read = now;
    }
}

void autoModeTask(void* pvParameters) {
    esp_task_wdt_add(NULL);
    Core1to0_Data local_c1_data;
    Core0to1_Data local_c0_output;
    static float last_Ki_p = 0, last_Ki_r = 0, last_Ki_y = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 4; 
    float f_mx = 0, f_my = 0, f_mz = 0; 
    int mag_read_counter = 0;
    prevMicros = micros(); 

    for (;;) {
        esp_task_wdt_reset();
        unsigned long now = micros();
        dt = (now - prevMicros) / 1000000.0f;
        prevMicros = now;
        if(dt > 0.04f || dt <= 0.0f) dt = 0.004f; 
        
        lpf_pitch_d.init(D_TERM_LPF_HZ, dt); 
        lpf_roll_d.init(D_TERM_LPF_HZ, dt); 
        lpf_yaw_d.init(D_TERM_LPF_HZ, dt);

        if (xSemaphoreTake(pidMutex, (TickType_t) 1) == pdTRUE) {
            memcpy(&local_c1_data, (void*)&core1to0_data, sizeof(Core1to0_Data));
            xSemaphoreGive(pidMutex);
        }

        readBarometer_Core0(); 
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);
        if(Wire.available() < 14) { vTaskDelayUntil(&xLastWakeTime, xFrequency); continue; } 
        
        int16_t ax_raw = Wire.read()<<8 | Wire.read();
        int16_t ay_raw = Wire.read()<<8 | Wire.read();
        int16_t az_raw = Wire.read()<<8 | Wire.read();
        Wire.read(); Wire.read(); 
        int16_t gx_raw = Wire.read()<<8 | Wire.read();
        int16_t gy_raw = Wire.read()<<8 | Wire.read();
        int16_t gz_raw = Wire.read()<<8 | Wire.read();

        bool new_mag = false;
        if(mag_detected) {
            mag_read_counter++;
            if (mag_read_counter >= 5) { 
                mag_read_counter = 0;
                if (is_qmc_mag) { Wire.beginTransmission(MAG_ADDR); Wire.write(0x00); }
                else { Wire.beginTransmission(MAG_ADDR); Wire.write(0x03); }
                Wire.endTransmission(false);
                Wire.requestFrom(MAG_ADDR, 6, true);
                if(Wire.available() >= 6) {
                   int16_t mx_raw, my_raw, mz_raw;
                   if (is_qmc_mag) { mx_raw = (Wire.read() | Wire.read() << 8); my_raw = (Wire.read() | Wire.read() << 8); mz_raw = (Wire.read() | Wire.read() << 8); }
                   else { mx_raw = (Wire.read() << 8) | Wire.read(); mz_raw = (Wire.read() << 8) | Wire.read(); my_raw = (Wire.read() << 8) | Wire.read(); }
                   f_mx = (float)mx_raw - mag_offset_x; f_my = (float)my_raw - mag_offset_y; f_mz = (float)mz_raw - mag_offset_z;
                   new_mag = true;
                }
            }
        }

        float accX = ((float)ax_raw - accX_offset) / 4096.0f; 
        float accY = ((float)-ay_raw - accY_offset) / 4096.0f;
        float accZ = ((float)az_raw - accZ_offset) / 4096.0f;
        
        // --- Raw Gyro Conversion ---
        float rawGyroX = ((float)gx_raw - gyroX_offset) / 65.5f * DEG_TO_RAD; 
        float rawGyroY = ((float)-gy_raw - gyroY_offset) / 65.5f * DEG_TO_RAD;
        float gyroZ    = ((float)gz_raw - gyroZ_offset) / 65.5f * DEG_TO_RAD;

        // --- Apply Dynamic Notch Filtering to remove motor noise ---
        float gyroX = notchRoll.update(rawGyroX);
        float gyroY = notchPitch.update(rawGyroY);

        // --- Update EKF with filtered data ---
        ekf.predict(gyroX, gyroY, gyroZ, dt);
        float accMag = sqrtf(accX*accX + accY*accY + accZ*accZ);
        if (fabsf(accMag - 1.0f) < 0.25f) ekf.updateAccel(accX, accY, accZ);

        float r, p, y_ekf;
        ekf.getEuler(r, p, y_ekf);
        roll = r; pitch = p; 

        if (new_mag) {
            float rollRad = roll * DEG_TO_RAD; float pitchRad = pitch * DEG_TO_RAD;
            float X_h = f_mx * cosf(pitchRad) + f_my * sinf(rollRad) * sinf(pitchRad) + f_mz * sinf(pitchRad) * cosf(rollRad); 
            float Y_h = f_my * cosf(rollRad) - f_mz * sinf(rollRad);
            float magYaw = atan2f(-Y_h, X_h) * RAD_TO_DEG + MAG_DECLINATION;
            if(magYaw < 0) magYaw += 360; if(magYaw >= 360) magYaw -= 360;
            local_c0_output.compass_heading = magYaw; 
        }

        if(y_ekf > 180) y_ekf -= 360; if(y_ekf < -180) y_ekf += 360;
        yaw = y_ekf; 

        local_c0_output.status_code = 0;
        if (local_c1_data.escsArmed) {
            if (fabsf(roll) > CRASH_ANGLE_LIMIT || fabsf(pitch) > CRASH_ANGLE_LIMIT) local_c0_output.status_code = 99; 
        }

        if(local_c1_data.Ki_p != last_Ki_p) { integral_pitch = 0; last_Ki_p = local_c1_data.Ki_p; }
        if(local_c1_data.Ki_r != last_Ki_r) { integral_roll = 0; last_Ki_r = local_c1_data.Ki_r; }
        if(local_c1_data.Ki_y != last_Ki_y) { integral_yaw = 0; last_Ki_y = local_c1_data.Ki_y; }

        int activeThrottle = local_c1_data.targetThrottle;
        if (activeThrottle < 1050 || !local_c1_data.escsArmed) {
            integral_pitch = 0; integral_roll = 0; integral_yaw = 0;
            prev_input_pitch = pitch; prev_input_roll = roll; prev_input_yaw = yaw;
            local_c0_output.pidPitchOut = 0; local_c0_output.pidRollOut = 0; local_c0_output.pidYawOut = 0;
        } else {
            local_c0_output.pidPitchOut = computePID(pitch, local_c1_data.man_setpoint_pitch, prev_input_pitch, integral_pitch, lpf_pitch_d, local_c1_data.Kp_p, local_c1_data.Ki_p, local_c1_data.Kd_p, dt, local_c1_data.escsArmed, activeThrottle);
            local_c0_output.pidRollOut = computePID(roll, local_c1_data.man_setpoint_roll, prev_input_roll, integral_roll, lpf_roll_d, local_c1_data.Kp_r, local_c1_data.Ki_r, local_c1_data.Kd_r, dt, local_c1_data.escsArmed, activeThrottle);
            float pid_y = 0;
            if (activeThrottle > 1150) { 
                 pid_y = computePID_Yaw(yaw, local_c1_data.man_setpoint_yaw, prev_input_yaw, integral_yaw, lpf_yaw_d, local_c1_data.Kp_y, local_c1_data.Ki_y, local_c1_data.Kd_y, dt, local_c1_data.escsArmed, activeThrottle);
                 pid_y = constrain(pid_y, -150.0f, 150.0f); 
            }
            local_c0_output.pidYawOut = pid_y;
        }

        local_c0_output.telemetry_pitch = pitch; local_c0_output.telemetry_roll = roll; local_c0_output.telemetry_yaw = yaw;
        local_c0_output.telemetry_altitude_m = current_altitude_m; local_c0_output.vertical_speed_mps = current_vspeed_mps;

        if (xSemaphoreTake(pidMutex, (TickType_t) 1) == pdTRUE) {
            memcpy((void*)&core0to1_data, &local_c0_output, sizeof(Core0to1_Data));
            xSemaphoreGive(pidMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

void setup() {
    Serial.begin(115200);
    
    // Configure Watchdog
    esp_task_wdt_config_t wdt_config = { .timeout_ms = WDT_TIMEOUT * 1000, .idle_core_mask = (1 << 1), .trigger_panic = true };
    esp_task_wdt_init(&wdt_config); 
    esp_task_wdt_add(NULL); 

    // Initialize ESCs
    esc1.setPeriodHertz(400); esc2.setPeriodHertz(400); esc3.setPeriodHertz(400); esc4.setPeriodHertz(400); 
    esc1.attach(ESC1_PIN, 1000, 2000); esc2.attach(ESC2_PIN, 1000, 2000); 
    esc3.attach(ESC3_PIN, 1000, 2000); esc4.attach(ESC4_PIN, 1000, 2000);

    // Initialize I2C
    Wire.begin(21, 22); 
    Wire.setClock(100000); 
    Wire.setTimeOut(10); 

    // Configure MPU6050
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); 
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); 
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(); 
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); 

    // Magnetometer & Sensor Calibration
    enableI2CBypass(); delay(100); 
    initMagnetometer(); delay(100);
    calibrateSensors(); 
    esp_task_wdt_reset(); 

    // --- Initialize Notch Filters ---
    // Targets 300Hz (common motor noise) at a 250Hz sample rate (4ms loop)
    notchPitch.init(200, 40, 250); 
    notchRoll.init(200, 40, 250);
    // -------------------------------------

    // Barometer Setup
    yaw_offset = 0;
    if (!bmp.begin(0x76) && !bmp.begin(0x77)) bmp_detected = false; else bmp_detected = true;
    if (bmp_detected) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X8, 
                        Adafruit_BMP280::SAMPLING_X8, Adafruit_BMP280::FILTER_X16, 
                        Adafruit_BMP280::STANDBY_MS_1000);
    }
    
    // Safety delay & ESC initialization pulse
    esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000); 
    esc3.writeMicroseconds(1000); esc4.writeMicroseconds(1000);
    for(int i=0; i<30; i++) { delay(100); esp_task_wdt_reset(); }

    // Networking & Tasks
    setup_wifi(); 
    client.setServer(MQTT_BROKER_IP, MQTT_PORT); 
    client.setCallback(mqtt_callback); 
    client.setBufferSize(512);
    
    pidMutex = xSemaphoreCreateMutex();
    
    // Launch Flight Task on Core 0
    xTaskCreatePinnedToCore(autoModeTask, "EKF_Task", 10000, NULL, 2, &autoTaskHandle, 0);
}

void logToDashboard(String message) { if (client.connected()) client.publish(STATUS_TOPIC, message.c_str()); }

void setup_wifi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) { delay(500); esp_task_wdt_reset(); }
}

float readBatteryVoltage() {
    long total = 0;
    for(int i=0; i<ADC_SAMPLES; i++){ total += analogRead(BATTERY_PIN); delay(2); }
    return (total/(float)ADC_SAMPLES/4095.0f)*ADC_MAX_VOLTAGE * VOLTAGE_MULTIPLIER;
}

void readSharedData_Core1() {
    if (xSemaphoreTake(pidMutex, (TickType_t)CORE1_MUTEX_TIMEOUT_MS/portTICK_PERIOD_MS) == pdTRUE) {
        memcpy(&core1_local_C1to0_data, (void*)&core1to0_data, sizeof(Core1to0_Data));
        memcpy(&core1_local_C0to1_data, (void*)&core0to1_data, sizeof(Core0to1_Data));
        xSemaphoreGive(pidMutex);
    }
}

void writeC1toC0Data_Core1() {
    if (xSemaphoreTake(pidMutex, (TickType_t)CORE1_MUTEX_TIMEOUT_MS/portTICK_PERIOD_MS) == pdTRUE) {
        memcpy((void*)&core1to0_data, &core1_local_C1to0_data, sizeof(Core1to0_Data));
        xSemaphoreGive(pidMutex);
    }
}

void emergencyDisarm_Core1(){
    core1_local_C1to0_data.escsArmed = false; core1_local_C1to0_data.targetThrottle = MIN_THROTTLE;
    currentThrottle = MIN_THROTTLE; writeC1toC0Data_Core1(); logToDashboard("DISARMED"); failsafe_active = false;
}

void activateFailsafe_Core1() {
    if(!failsafe_active) {
        logToDashboard("FAILSAFE"); failsafe_active = true;
        if(currentThrottle > HOVER_THROTTLE) core1_local_C1to0_data.targetThrottle = HOVER_THROTTLE - 150; 
        else core1_local_C1to0_data.targetThrottle = max(IDLE_THROTTLE + 50, currentThrottle - 100);
    }
    static unsigned long failsafeStart = 0;
    if(failsafeStart == 0) failsafeStart = millis();
    if(millis() - failsafeStart > 8000) emergencyDisarm_Core1();
    writeC1toC0Data_Core1();
}

void update_escs_Core1(){
    
    // Auto-Disarm on Crash Detection
    if (core1_local_C0to1_data.status_code == 99 && core1_local_C1to0_data.escsArmed) emergencyDisarm_Core1();

    if(!core1_local_C1to0_data.escsArmed){
        esc1.writeMicroseconds(MIN_THROTTLE); esc2.writeMicroseconds(MIN_THROTTLE);
        esc3.writeMicroseconds(MIN_THROTTLE); esc4.writeMicroseconds(MIN_THROTTLE);
        esc1_speed = MIN_THROTTLE; esc2_speed = MIN_THROTTLE;
        esc3_speed = MIN_THROTTLE; esc4_speed = MIN_THROTTLE;
    } else {
        int thr = currentThrottle < IDLE_THROTTLE ? IDLE_THROTTLE : currentThrottle;
        float safeBatt = (batteryVoltage < 5.0f) ? NOMINAL_BATTERY_VOLTAGE : batteryVoltage;
        voltage_compensation_factor = constrain(NOMINAL_BATTERY_VOLTAGE / safeBatt, 0.8f, 1.3f);
        float p = core1_local_C0to1_data.pidPitchOut * voltage_compensation_factor;
        float r = core1_local_C0to1_data.pidRollOut * voltage_compensation_factor;
        float y = core1_local_C0to1_data.pidYawOut * voltage_compensation_factor;

        esc1_speed = constrain(thr - p - r + y, MIN_THROTTLE, MAX_SAFE_THROTTLE); 
        esc2_speed = constrain(thr - p + r - y, MIN_THROTTLE, MAX_SAFE_THROTTLE); 
        esc3_speed = constrain(thr + p - r - y, MIN_THROTTLE, MAX_SAFE_THROTTLE); 
        esc4_speed = constrain(thr + p + r + y, MIN_THROTTLE, MAX_SAFE_THROTTLE); 

        esc1.writeMicroseconds(esc1_speed); esc2.writeMicroseconds(esc2_speed);
        esc3.writeMicroseconds(esc3_speed); esc4.writeMicroseconds(esc4_speed); 
    }
}

void gradualThrottleUpdate_Core1(){
    if (!core1_local_C1to0_data.escsArmed) { currentThrottle = MIN_THROTTLE; return; }
    int target = core1_local_C1to0_data.targetThrottle;
    if(currentThrottle < target) currentThrottle = min(currentThrottle + THROTTLE_SLEW_RATE_PER_CYCLE, target);
    else if(currentThrottle > target) currentThrottle = max(currentThrottle - THROTTLE_SLEW_RATE_PER_CYCLE, target);
}

void publishTelemetry_Core1() {
    StaticJsonDocument<384> doc;
    if (core1_local_C1to0_data.escsArmed) { doc["r1"] = esc1_speed; doc["r2"] = esc2_speed; doc["r3"] = esc3_speed; doc["r4"] = esc4_speed; } 
    else { doc["r1"] = 0; doc["r2"] = 0; doc["r3"] = 0; doc["r4"] = 0; }
    doc["p"] = (int)(core1_local_C0to1_data.telemetry_pitch * 100.0f) / 100.0f;
    doc["r"] = (int)(core1_local_C0to1_data.telemetry_roll * 100.0f) / 100.0f;
    doc["y"] = (int)(core1_local_C0to1_data.telemetry_yaw * 100.0f) / 100.0f;
    doc["head"] = (int)(core1_local_C0to1_data.compass_heading);
    doc["a"] = (int)(core1_local_C0to1_data.telemetry_altitude_m * 100.0f) / 100.0f;
    doc["b"] = (int)(batteryVoltage * 100.0f) / 100.0f;
    doc["arm"] = core1_local_C1to0_data.escsArmed ? 1 : 0;
    char output[384]; serializeJson(doc, output); client.publish(TELEMETRY_TOPIC, output);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length){
    char message[length + 1]; memcpy(message, payload, length); message[length] = '\0';
    lastCommandTime = millis(); failsafe_active = false; readSharedData_Core1();
    
    if (strcmp(topic, MODE_TOPIC) == 0) {
        if (strcmp(message, "ARM") == 0) { 
            if(client.connected()) { 
                core1_local_C1to0_data.escsArmed = true; 
                // YAW LOCK: Prevent snap to North on Arming
                core1_local_C1to0_data.man_setpoint_yaw = core1_local_C0to1_data.telemetry_yaw; 
                logToDashboard("ARMED"); 
            } 
        }
        else if (strcmp(message, "DISARM") == 0) emergencyDisarm_Core1();
    } else if (strcmp(topic, COMMAND_TOPIC) == 0) {
        char* token = strtok(message, ",");
        if (token != NULL) {
            core1_local_C1to0_data.targetThrottle = constrain(map(atoi(token),0,100,1000,1900),1000,1900);
            token = strtok(NULL, ","); if (token != NULL) {
                core1_local_C1to0_data.man_setpoint_yaw = mapFloat(atof(token),-100.0f,100.0f,-179.0f,179.0f);
                token = strtok(NULL, ","); if (token != NULL) {
                    core1_local_C1to0_data.man_setpoint_pitch = mapFloat(atof(token),-100.0f,100.0f,-30.0f,30.0f);
                    token = strtok(NULL, ","); if (token != NULL) {
                        core1_local_C1to0_data.man_setpoint_roll = mapFloat(atof(token),-100.0f,100.0f,-30.0f,30.0f);
                    }
                }
            }
        }
    }
    writeC1toC0Data_Core1();
}

void attempt_mqtt_reconnect() {
    if (client.connect(MQTT_CLIENT_ID)) { client.subscribe(COMMAND_TOPIC); client.subscribe(MODE_TOPIC); }
}

void loop(){
    esp_task_wdt_reset(); readSharedData_Core1(); 
    updateLED_Pattern();
    if (core1_local_C1to0_data.escsArmed && (millis() - lastCommandTime > CONNECTION_TIMEOUT_MS)) activateFailsafe_Core1(); 
    if(WiFi.status() != WL_CONNECTED && !core1_local_C1to0_data.escsArmed) { WiFi.disconnect(); WiFi.reconnect(); }
    if(WiFi.status() == WL_CONNECTED && !client.connected() && millis() - lastReconnectAttempt > 5000) { lastReconnectAttempt = millis(); attempt_mqtt_reconnect(); }
    if(WiFi.status() == WL_CONNECTED) client.loop();
    if(millis() - lastLoopTime >= CORE1_LOOP_INTERVAL_MS){
        lastLoopTime = millis(); writeC1toC0Data_Core1(); gradualThrottleUpdate_Core1(); update_escs_Core1(); 
        if(millis() - lastTelemetrySend > TELEMETRY_INTERVAL_MS){ lastTelemetrySend = millis(); batteryVoltage = readBatteryVoltage(); if(client.connected()) publishTelemetry_Core1(); }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
}
