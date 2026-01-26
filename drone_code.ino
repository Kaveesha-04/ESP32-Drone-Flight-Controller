/*
 * ESP32 Drone Code - V13.8 (NO COMPASS FLIP)
 * AUTHOR: Nimsara Karunarathna
 * STATUS: FLIGHT READY
 * * --- CONFIGURATION ---
 * 1. [NAV] Heading Lock ENABLED (Finds real North on boot).
 * 2. [NAV] Compass 180 Flip REMOVED (Standard orientation).
 * 3. [TUNE] Safe Mode ENABLED (100Hz Loop, Smooth Idle).
 */

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

// --- WIFI & MQTT CREDENTIALS ---
const char* WIFI_SSID       = "22K_ROUTER";       
const char* WIFI_PASSWORD   = "#Mal@dialog";   
const char* MQTT_BROKER_IP  = "192.168.8.197";        
const int   MQTT_PORT       = 1883;
const char* MQTT_CLIENT_ID  = "esp32-drone-nimsara-final";

// --- LOCATION ---
#define MAG_DECLINATION  -1.95 // Moratuwa, Sri Lanka

// --- HARDWARE CALIBRATION (PASTE YOUR CALIBRATED VALUES HERE) ---
// Currently set to 0. Please run the calibration sketch if you haven't!
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

// --- PID & FILTERING ---
#define D_TERM_LPF_HZ 20.0f  
#define Q_ANGLE 0.005f 
#define Q_BIAS  0.0001f
#define R_ACCEL 0.1f    
#define R_MAG   0.8f    

#define RAD_TO_DEG 57.2957795131
#define DEG_TO_RAD 0.01745329251

// --- ESC PINS ---
#define ESC1_PIN 26 // Front Left (CW)
#define ESC2_PIN 32 // Front Right (CCW)
#define ESC3_PIN 25 // Rear Left (CCW)
#define ESC4_PIN 27 // Rear Right (CW)

Servo esc1, esc2, esc3, esc4;

// --- SENSOR ADDRESSES ---
const int MPU_ADDR = 0x68;
int MAG_ADDR = 0x1E; 
bool is_qmc_mag = false;

#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define MAX_SAFE_THROTTLE 1900
#define IDLE_THROTTLE 1050  // Safe Idle (Smooth Start)
#define HOVER_THROTTLE 1450 

// --- FLIGHT CONSTANTS ---
#define LOW_BATTERY_LIMIT 10.5 
#define CRASH_ANGLE_LIMIT 65.0f 

#define THROTTLE_SLEW_RATE_PER_CYCLE 20 
#define CORE1_LOOP_INTERVAL_MS 10  // 100Hz (Smooth & Stable)
#define CORE1_MUTEX_TIMEOUT_MS 5   
#define TELEMETRY_INTERVAL_MS 150  
#define CONNECTION_TIMEOUT_MS 4000 
#define SEA_LEVEL_PRESSURE_HPA 1013.25 

// --- VOLTAGE COMPENSATION ---
#define NOMINAL_BATTERY_VOLTAGE 12.0 // 3S LiPo Nominal
float voltage_compensation_factor = 1.0;

TaskHandle_t autoTaskHandle = NULL;
SemaphoreHandle_t pidMutex;

// --- BMP280 ---
Adafruit_BMP280 bmp; 
bool bmp_detected = false; 
bool mag_detected = false;
float current_altitude_m = 0.0;
float current_vspeed_mps = 0.0; 
float ground_reference_altitude = 0.0f;

// --- SHARED DATA STRUCTURES ---
struct Core1to0_Data {
    // Safe Mode PIDs
    float Kp_p = 2.5, Ki_p = 0.00, Kd_p = 8.0; 
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
    int status_code = 0; // 0=OK, 99=CRASH
    float telemetry_altitude_m = 0.0;
    float vertical_speed_mps = 0.0;
};

Core1to0_Data core1to0_data;
Core1to0_Data core1_local_C1to0_data; 
Core0to1_Data core0to1_data;
Core0to1_Data core1_local_C0to1_data;

// --- GLOBAL VARS FOR CORE 0 ---
float prev_input_pitch = 0, integral_pitch = 0;
float prev_input_roll = 0, integral_roll = 0;
float prev_input_yaw = 0, integral_yaw = 0;
float pitch = 0, roll = 0, yaw = 0;
float yaw_offset = 0; // Stores Heading Lock Offset

// --- CALIBRATION VARS ---
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

// ================================================================
// ===            HELPER CLASS: LOW PASS FILTER                 ===
// ================================================================
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

// ================================================================
// ===            EXTENDED KALMAN FILTER (EKF) CLASS            ===
// ================================================================

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
        float norm = sqrt(ax*ax + ay*ay + az*az);
        if(norm < 0.1f) return; 
        ax /= norm; ay /= norm; az /= norm;
        
        float vx = 2.0f * (q1*q3 - q0*q2);
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        
        float ex = ax - vx;
        float ey = ay - vy;
        float ez = az - vz;
        
        float err_norm = sqrt(ex*ex + ey*ey + ez*ez);
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
        float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm == 0.0f) { q0 = 1.0f; return; }
        norm = 1.0f / norm; 
        q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
    }

    void getEuler(float &roll, float &pitch, float &yaw) {
        roll  = atan2(2.0f * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * RAD_TO_DEG;
        pitch = -asin(constrain(2.0f * (q1*q3 - q0*q2), -1.0f, 1.0f)) * RAD_TO_DEG; 
        yaw   = atan2(2.0f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * RAD_TO_DEG;
    }
};

DroneEKF ekf;

// ================================================================
// ===              FUNCTION PROTOTYPES                         ===
// ================================================================

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

// ================================================================
// ===                  PID FUNCTIONS                           ===
// ================================================================

float computePID(float input, float target, float &prev_input, float &integral, LowPassFilter &lpf, float Kp, float Ki, float Kd, float dt, bool armed, int throttle_val) {
    float error = target - input;
    
    if(armed && throttle_val > 1100) {
        if(abs(error) < 30) { 
            integral += (error * dt);
            integral = constrain(integral, -80, 80); 
        }
    } else { 
        integral = 0; 
    }
    
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
    } else { 
        integral = 0; 
    }

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

// ================================================================
// ===               SENSOR & HARDWARE FUNCTIONS                ===
// ================================================================

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
        Serial.println("MAG: HMC5883L (0x1E)");
        return;
    }
    Wire.beginTransmission(0x0D);
    if (Wire.endTransmission() == 0) {
        MAG_ADDR = 0x0D; is_qmc_mag = true;
        Wire.beginTransmission(MAG_ADDR); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();
        Wire.beginTransmission(MAG_ADDR); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission();
        mag_detected = true;
        Serial.println("MAG: QMC5883L (0x0D)");
        return;
    }
    Serial.println("MAG: NOT FOUND");
    mag_detected = false;
}

void calibrateSensors() {
    if (FORCE_ACC_Z != 0) {
        accX_offset = FORCE_ACC_X; accY_offset = FORCE_ACC_Y; accZ_offset = FORCE_ACC_Z;
        gyroX_offset = FORCE_GYRO_X; gyroY_offset = FORCE_GYRO_Y; gyroZ_offset = FORCE_GYRO_Z;
        Serial.println(">> USING HARDCODED OFFSETS");
        return;
    }
    
    long ax_sum=0, ay_sum=0, az_sum=0, gx_sum=0, gy_sum=0, gz_sum=0;
    int samples = 1000;
    digitalWrite(LED_PIN, HIGH); delay(500); digitalWrite(LED_PIN, LOW); delay(500); digitalWrite(LED_PIN, HIGH);
    for(int i=0; i<samples; i++) {
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);
        if(Wire.available() < 14) { i--; continue; } 
        ax_sum += int16_t(Wire.read()<<8 | Wire.read());
        ay_sum += int16_t(Wire.read()<<8 | Wire.read());
        az_sum += int16_t(Wire.read()<<8 | Wire.read());
        Wire.read(); Wire.read(); 
        gx_sum += int16_t(Wire.read()<<8 | Wire.read());
        gy_sum += int16_t(Wire.read()<<8 | Wire.read());
        gz_sum += int16_t(Wire.read()<<8 | Wire.read());
        delay(2);
    }
    accX_offset = ax_sum / samples;
    accY_offset = ay_sum / samples;
    accZ_offset = (az_sum / samples) - 4096.0; 
    gyroX_offset = gx_sum / samples;
    gyroY_offset = gy_sum / samples;
    gyroZ_offset = gz_sum / samples;
    digitalWrite(LED_PIN, LOW); 
}

void readBarometer_Core0() {
    if(!bmp_detected) return; 
    unsigned long now = millis();
    static unsigned long last_baro_read = 0;
    if (now - last_baro_read >= 50) { 
        float raw_altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
        if (!core1_local_C1to0_data.escsArmed) {
            ground_reference_altitude = raw_altitude;
            current_altitude_m = 0.0f; 
            current_vspeed_mps = 0.0f;
        } else {
            float relative_altitude = raw_altitude - ground_reference_altitude;
            float baro_dt = (now - last_baro_read) / 1000.0; 
            if (baro_dt > 0.0) current_vspeed_mps = (relative_altitude - current_altitude_m) / baro_dt;
            current_altitude_m = (current_altitude_m * 0.70) + (relative_altitude * 0.30);
        }
        last_baro_read = now;
    }
}

// ================================================================
// ===                CORE 0 TASK (FLIGHT LOOP)                 ===
// ================================================================

void autoModeTask(void* pvParameters) {
    Serial.println("EKF Flight Task running on Core 0");
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
       esp_task_wdt_add(NULL); 
    #else
       esp_task_wdt_add(NULL);
    #endif

    Core1to0_Data local_c1_data;
    Core0to1_Data local_c0_output;
    static float last_Ki_p = 0, last_Ki_r = 0, last_Ki_y = 0;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 4; // 250Hz Loop
    
    prevMicros = micros(); 
    int mag_read_counter = 0;

    for (;;) {
        esp_task_wdt_reset();

        unsigned long now = micros();
        dt = (now - prevMicros) / 1000000.0;
        prevMicros = now;
        if(dt > 0.04 || dt <= 0.0) dt = 0.004; 
        
        lpf_pitch_d.init(D_TERM_LPF_HZ, dt);
        lpf_roll_d.init(D_TERM_LPF_HZ, dt);
        lpf_yaw_d.init(D_TERM_LPF_HZ, dt);

        if (xSemaphoreTake(pidMutex, (TickType_t) 0) == pdTRUE) {
            memcpy(&local_c1_data, (void*)&core1to0_data, sizeof(Core1to0_Data));
            xSemaphoreGive(pidMutex);
        }

        readBarometer_Core0(); 

        // 3. Fast Sensor Read (MPU6050)
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);
        
        if(Wire.available() < 14) { 
             vTaskDelayUntil(&xLastWakeTime, xFrequency); 
             continue; 
        } 
        
        int16_t ax_raw = Wire.read()<<8 | Wire.read();
        int16_t ay_raw = Wire.read()<<8 | Wire.read();
        int16_t az_raw = Wire.read()<<8 | Wire.read();
        Wire.read(); Wire.read(); 
        int16_t gx_raw = Wire.read()<<8 | Wire.read();
        int16_t gy_raw = Wire.read()<<8 | Wire.read();
        int16_t gz_raw = Wire.read()<<8 | Wire.read();

        // 4. Throttled Magnetometer Read
        float f_mx = 0, f_my = 0, f_mz = 0;
        bool new_mag = false;
        
        if(mag_detected) {
            mag_read_counter++;
            if (mag_read_counter >= 5) { 
                mag_read_counter = 0;
                if (is_qmc_mag) {
                    Wire.beginTransmission(MAG_ADDR); Wire.write(0x00); Wire.endTransmission(false);
                    Wire.requestFrom(MAG_ADDR, 6, true);
                } else {
                    Wire.beginTransmission(MAG_ADDR); Wire.write(0x03); Wire.endTransmission(false);
                    Wire.requestFrom(MAG_ADDR, 6, true);
                }
                
                if(Wire.available() >= 6) {
                   int16_t mx_raw, my_raw, mz_raw;
                   if (is_qmc_mag) {
                       mx_raw = (Wire.read() | Wire.read() << 8);
                       my_raw = (Wire.read() | Wire.read() << 8);
                       mz_raw = (Wire.read() | Wire.read() << 8);
                   } else {
                       mx_raw = (Wire.read() << 8) | Wire.read();
                       mz_raw = (Wire.read() << 8) | Wire.read(); 
                       my_raw = (Wire.read() << 8) | Wire.read(); 
                   }
                   f_mx = mx_raw - mag_offset_x;
                   f_my = my_raw - mag_offset_y;
                   f_mz = mz_raw - mag_offset_z;
                   new_mag = true;
                }
            }
        }

        // 5. Apply Offsets & Convert
        float accX = ((float)ax_raw - accX_offset) / 4096.0; 
        float accY = ((float)-ay_raw - accY_offset) / 4096.0;
        float accZ = ((float)az_raw - accZ_offset) / 4096.0;
        float gyroX = ((float)gx_raw - gyroX_offset) / 65.5 * DEG_TO_RAD; 
        float gyroY = ((float)-gy_raw - gyroY_offset) / 65.5 * DEG_TO_RAD;
        float gyroZ = ((float)gz_raw - gyroZ_offset) / 65.5 * DEG_TO_RAD;

        // 6. EKF Prediction
        ekf.predict(gyroX, gyroY, gyroZ, dt);
        
        float accMag = sqrt(accX*accX + accY*accY + accZ*accZ);
        if (abs(accMag - 1.0f) < 0.25f) { 
            ekf.updateAccel(accX, accY, accZ);
        }

        float r, p, y;
        ekf.getEuler(r, p, y);
        roll = r; pitch = p; 

        // 7. Mag Fusion (NO FLIP)
        if (new_mag) {
            float rollRad = roll * DEG_TO_RAD;
            float pitchRad = pitch * DEG_TO_RAD;
            
            float X_h = f_mx * cos(pitchRad) + (-f_my) * sin(rollRad) * sin(pitchRad) + f_mz * sin(pitchRad) * cos(rollRad); 
            float Y_h = (-f_my) * cos(rollRad) - f_mz * sin(rollRad);
            
            float magYaw = atan2(Y_h, X_h) * RAD_TO_DEG;
            magYaw += MAG_DECLINATION; 
            
            // --- HEADING LOCK LOGIC ---
            float current_fused_yaw = y + yaw_offset;
            if(current_fused_yaw > 360) current_fused_yaw -= 360;
            if(current_fused_yaw < 0) current_fused_yaw += 360;

            float error = magYaw - current_fused_yaw;
            if(error > 180) error -= 360;
            if(error < -180) error += 360;

            yaw_offset += error * 0.01; 
        }

        y += yaw_offset;
        if(y > 360) y -= 360;
        if(y < 0) y += 360;
        
        yaw = y;

        // 8. Crash Detection
        local_c0_output.status_code = 0;
        if (local_c1_data.escsArmed) {
            //if (abs(roll) > CRASH_ANGLE_LIMIT || abs(pitch) > CRASH_ANGLE_LIMIT) {
            //    local_c0_output.status_code = 99; 
            //    local_c1_data.escsArmed = false;
            //}
        }

        // 9. PID Control
        if(local_c1_data.Ki_p != last_Ki_p) { integral_pitch = 0; last_Ki_p = local_c1_data.Ki_p; }
        if(local_c1_data.Ki_r != last_Ki_r) { integral_roll = 0; last_Ki_r = local_c1_data.Ki_r; }
        if(local_c1_data.Ki_y != last_Ki_y) { integral_yaw = 0; last_Ki_y = local_c1_data.Ki_y; }

        int activeThrottle = local_c1_data.targetThrottle;

        if (activeThrottle < 1050 || !local_c1_data.escsArmed) {
            integral_pitch = 0; integral_roll = 0; integral_yaw = 0;
            prev_input_pitch = pitch; prev_input_roll = roll; prev_input_yaw = yaw;
            local_c0_output.pidPitchOut = 0;
            local_c0_output.pidRollOut = 0;
            local_c0_output.pidYawOut = 0;
        } else {
            float pid_pitch = computePID(pitch, local_c1_data.man_setpoint_pitch, prev_input_pitch, integral_pitch, lpf_pitch_d, local_c1_data.Kp_p, local_c1_data.Ki_p, local_c1_data.Kd_p, dt, local_c1_data.escsArmed, activeThrottle);
            float pid_roll = computePID(roll, local_c1_data.man_setpoint_roll, prev_input_roll, integral_roll, lpf_roll_d, local_c1_data.Kp_r, local_c1_data.Ki_r, local_c1_data.Kd_r, dt, local_c1_data.escsArmed, activeThrottle);
            float pid_yaw = 0;
            if (activeThrottle > 1150) { 
                 pid_yaw = computePID_Yaw(yaw, local_c1_data.man_setpoint_yaw, prev_input_yaw, integral_yaw, lpf_yaw_d, local_c1_data.Kp_y, local_c1_data.Ki_y, local_c1_data.Kd_y, dt, local_c1_data.escsArmed, activeThrottle);
                 pid_yaw = constrain(pid_yaw, -150.0, 150.0); 
            }
            local_c0_output.pidPitchOut = pid_pitch;
            local_c0_output.pidRollOut = pid_roll;
            local_c0_output.pidYawOut  = pid_yaw;
        }

        local_c0_output.telemetry_pitch = pitch;
        local_c0_output.telemetry_roll = roll;
        local_c0_output.telemetry_yaw = yaw;
        local_c0_output.telemetry_altitude_m = current_altitude_m; 
        local_c0_output.vertical_speed_mps = current_vspeed_mps;

        if (xSemaphoreTake(pidMutex, (TickType_t) 0) == pdTRUE) {
            memcpy((void*)&core0to1_data, &local_c0_output, sizeof(Core0to1_Data));
            xSemaphoreGive(pidMutex);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// ================================================================
// ===                      SETUP                               ===
// ================================================================

void setup() {
    Serial.begin(115200);
    
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << 1),     
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL); 

    // --- ESC SETUP ---
    esc1.setPeriodHertz(400); 
    esc2.setPeriodHertz(400); 
    esc3.setPeriodHertz(400); 
    esc4.setPeriodHertz(400); 

    esc1.attach(ESC1_PIN, 1000, 2000);
    esc2.attach(ESC2_PIN, 1000, 2000);
    esc3.attach(ESC3_PIN, 1000, 2000);
    esc4.attach(ESC4_PIN, 1000, 2000);

    // --- I2C SETUP ---
    Wire.begin(21, 22); 
    Wire.setClock(400000); 
    Wire.setTimeOut(1000);
    delay(10);
    
    Serial.println("Starting ESP32 Drone V13.8 (Heading Lock + No Flip)");
    pinMode(LED_PIN, OUTPUT);

    // --- MPU6050 INIT ---
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); 
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); 
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(); 
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); 

    enableI2CBypass();
    delay(100);
    initMagnetometer();
    delay(100);

    // --- CALIBRATION ---
    Serial.println(">> CALIBRATING...");
    calibrateSensors(); 
    esp_task_wdt_reset(); 
    Serial.println(">> CALIBRATION DONE.");

    // --- NEW: FIND NORTH ON BOOT ---
    Serial.println(">> FINDING NORTH...");
    delay(500);
    float avg_mag_heading = 0;
    int valid_readings = 0;
    
    for(int i=0; i<50; i++) {
        if (is_qmc_mag) {
             Wire.beginTransmission(MAG_ADDR); Wire.write(0x00); Wire.endTransmission(false);
             Wire.requestFrom(MAG_ADDR, 6, true);
        } else {
             Wire.beginTransmission(MAG_ADDR); Wire.write(0x03); Wire.endTransmission(false);
             Wire.requestFrom(MAG_ADDR, 6, true);
        }

        if(Wire.available() >= 6) {
            int16_t mx, my, mz;
            if(is_qmc_mag) {
                mx = Wire.read() | Wire.read() << 8;
                my = Wire.read() | Wire.read() << 8;
                mz = Wire.read() | Wire.read() << 8;
            } else {
                mx = (Wire.read() << 8) | Wire.read();
                mz = (Wire.read() << 8) | Wire.read();
                my = (Wire.read() << 8) | Wire.read();
            }
            float fx = mx - mag_offset_x;
            float fy = my - mag_offset_y;
            // Basic Heading calculation for offset (NO 180 FLIP)
            float heading = atan2(-fy, fx) * 57.2957795 + MAG_DECLINATION; 
            if(heading < 0) heading += 360;
            if(heading > 360) heading -= 360;
            
            avg_mag_heading += heading;
            valid_readings++;
        }
        delay(10);
    }
    
    if(valid_readings > 0) {
        yaw_offset = avg_mag_heading / valid_readings;
        Serial.print(">> NORTH FOUND AT: "); Serial.println(yaw_offset);
    }

    // --- BAROMETER ---
    if (!bmp.begin(0x76)) {  
        if (!bmp.begin(0x77)) { bmp_detected = false; Serial.println("BMP280 NOT Found"); } 
        else { bmp_detected = true; Serial.println("BMP280 Found at 0x77"); }
    } else { bmp_detected = true; Serial.println("BMP280 Found at 0x76"); }

    if (bmp_detected) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X8, Adafruit_BMP280::SAMPLING_X8, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_1000);
    }
    
    // --- ESC INIT ---
    Serial.println(">> ARMING ESCs...");
    esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000); esc4.writeMicroseconds(1000);
    
    for(int i=0; i<30; i++) { 
        delay(100);
        esp_task_wdt_reset(); 
    }

    // --- WIFI SETUP ---
    Serial.println(">> CONNECTING WIFI...");
    setup_wifi(); 
    
    client.setServer(MQTT_BROKER_IP, MQTT_PORT);
    client.setCallback(mqtt_callback);
    client.setBufferSize(512);

    pidMutex = xSemaphoreCreateMutex();
    
    xTaskCreatePinnedToCore(autoModeTask, "EKF_Task", 10000, NULL, 2, &autoTaskHandle, 0);
    Serial.println(">> SYSTEM READY.");
}

void logToDashboard(String message) {
    if (client.connected()) { client.publish(STATUS_TOPIC, message.c_str()); }
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to SSID: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        esp_task_wdt_reset(); 
        
        if (millis() - start > 10000) {
            Serial.println("\n\n!! WIFI FAILURE !!");
            return; 
        }
    }

    Serial.println("");
    Serial.println(">> WiFi Connected!");
    Serial.print(">> IP Address: ");
    Serial.println(WiFi.localIP()); 
}

float readBatteryVoltage() {
    long total = 0;
    for(int i=0;i<ADC_SAMPLES;i++){ total += analogRead(BATTERY_PIN); delay(2); }
    float raw = total/(float)ADC_SAMPLES;
    return (raw/4095.0)*ADC_MAX_VOLTAGE * VOLTAGE_MULTIPLIER;
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
    core1_local_C1to0_data.escsArmed = false;
    core1_local_C1to0_data.targetThrottle = MIN_THROTTLE;
    currentThrottle = MIN_THROTTLE; 
    writeC1toC0Data_Core1(); 
    logToDashboard("DISARMED"); 
    failsafe_active = false;
}

void activateFailsafe_Core1() {
    if(!failsafe_active) {
        logToDashboard("FAILSAFE: DESCENDING");
        failsafe_active = true;
        if(currentThrottle > HOVER_THROTTLE) {
            core1_local_C1to0_data.targetThrottle = HOVER_THROTTLE - 150; 
        } else {
            core1_local_C1to0_data.targetThrottle = max(IDLE_THROTTLE + 50, currentThrottle - 100);
        }
    }
    
    static unsigned long failsafeStart = 0;
    if(failsafeStart == 0) failsafeStart = millis();
    if(millis() - failsafeStart > 8000) {
        emergencyDisarm_Core1();
    }
    writeC1toC0Data_Core1();
}

void update_escs_Core1(){
    if (core1_local_C1to0_data.escsArmed) {
        digitalWrite(LED_PIN, HIGH); 
    } else {
        digitalWrite(LED_PIN, LOW); 
    }

    if (core1_local_C0to1_data.status_code == 99 && core1_local_C1to0_data.escsArmed) {
        logToDashboard("CRASH DETECTED - DISARMING");
        emergencyDisarm_Core1();
    }

    if(!core1_local_C1to0_data.escsArmed){
        esc1.writeMicroseconds(MIN_THROTTLE); esc2.writeMicroseconds(MIN_THROTTLE);
        esc3.writeMicroseconds(MIN_THROTTLE); esc4.writeMicroseconds(MIN_THROTTLE);
        esc1_speed = MIN_THROTTLE; esc2_speed = MIN_THROTTLE;
        esc3_speed = MIN_THROTTLE; esc4_speed = MIN_THROTTLE;
    } else {
        int thr = currentThrottle < IDLE_THROTTLE ? IDLE_THROTTLE : currentThrottle;
        voltage_compensation_factor = constrain(NOMINAL_BATTERY_VOLTAGE / batteryVoltage, 0.8, 1.3);
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
    if (!core1_local_C1to0_data.escsArmed) {
        currentThrottle = MIN_THROTTLE;
        return; 
    }
    int target = core1_local_C1to0_data.targetThrottle;
    if(currentThrottle < target) currentThrottle = min(currentThrottle + THROTTLE_SLEW_RATE_PER_CYCLE, target);
    else if(currentThrottle > target) currentThrottle = max(currentThrottle - THROTTLE_SLEW_RATE_PER_CYCLE, target);
}

void publishTelemetry_Core1(){
    StaticJsonDocument<384> doc;
    if (core1_local_C1to0_data.escsArmed) {
        doc["r1"] = esc1_speed; doc["r2"] = esc2_speed;
        doc["r3"] = esc3_speed; doc["r4"] = esc4_speed; 
    } else {
        doc["r1"] = 0; doc["r2"] = 0; doc["r3"] = 0; doc["r4"] = 0;
    }
    doc["p"] = (int)(core1_local_C0to1_data.telemetry_pitch * 100) / 100.0;
    doc["r"] = (int)(core1_local_C0to1_data.telemetry_roll * 100) / 100.0;
    doc["y"] = (int)(core1_local_C0to1_data.telemetry_yaw * 100) / 100.0;
    doc["a"] = (int)(core1_local_C0to1_data.telemetry_altitude_m * 100) / 100.0;
    doc["b"] = (int)(batteryVoltage * 100) / 100.0;
    doc["arm"] = core1_local_C1to0_data.escsArmed ? 1 : 0;
    char output[384]; 
    serializeJson(doc, output);
    client.publish(TELEMETRY_TOPIC, output);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length){
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    lastCommandTime = millis();
    failsafe_active = false; 
    readSharedData_Core1();
    
    if (strcmp(topic, MODE_TOPIC) == 0) {
        if (strcmp(message, "ARM") == 0) { 
            if(client.connected()) { 
                core1_local_C1to0_data.escsArmed = true; 
                logToDashboard("ARMED"); 
            } 
        }
        else if (strcmp(message, "DISARM") == 0) { emergencyDisarm_Core1(); }
    }
    else if (strcmp(topic, COMMAND_TOPIC) == 0) {
        char* token = strtok(message, ",");
        if (token != NULL) {
             int raw_thr = atoi(token);
             core1_local_C1to0_data.targetThrottle = constrain(map(raw_thr,0,100,1000,1900),1000,1900);
             token = strtok(NULL, ",");
             if (token != NULL) {
                 float raw_yaw = atof(token);
                 core1_local_C1to0_data.man_setpoint_yaw = mapFloat(raw_yaw,-100.0,100.0,-179.0,179.0);
                 token = strtok(NULL, ",");
                 if (token != NULL) {
                     float raw_pitch = atof(token);
                     core1_local_C1to0_data.man_setpoint_pitch = mapFloat(raw_pitch,-100.0,100.0,-30.0,30.0);
                     token = strtok(NULL, ",");
                     if (token != NULL) {
                         float raw_roll = atof(token);
                         core1_local_C1to0_data.man_setpoint_roll = mapFloat(raw_roll,-100.0,100.0,-30.0,30.0);
                     }
                 }
             }
        }
    }
    writeC1toC0Data_Core1();
}

void attempt_mqtt_reconnect() {
    if (client.connect(MQTT_CLIENT_ID)) { 
        client.subscribe(COMMAND_TOPIC); 
        client.subscribe(MODE_TOPIC); 
        logToDashboard("MQTT Reconnected"); 
    }
}

void loop(){
    esp_task_wdt_reset();
    readSharedData_Core1(); 
    
    if (core1_local_C1to0_data.escsArmed && (millis() - lastCommandTime > CONNECTION_TIMEOUT_MS)) { 
        activateFailsafe_Core1(); 
    }
    
    bool is_connected = (WiFi.status() == WL_CONNECTED);
    if(!is_connected) { 
        if (!core1_local_C1to0_data.escsArmed) { WiFi.disconnect(); WiFi.reconnect(); }
    }
    
    if(is_connected && !client.connected()) { 
        if (millis() - lastReconnectAttempt > 5000) { 
            lastReconnectAttempt = millis(); 
            attempt_mqtt_reconnect(); 
        } 
    }
    
    if(is_connected) { client.loop(); }
    
    if(millis() - lastLoopTime >= CORE1_LOOP_INTERVAL_MS){
        lastLoopTime = millis();
        writeC1toC0Data_Core1();
        gradualThrottleUpdate_Core1(); 
        update_escs_Core1(); 
        if(millis() - lastTelemetrySend > TELEMETRY_INTERVAL_MS){ 
            lastTelemetrySend = millis(); 
            batteryVoltage = readBatteryVoltage(); 
            if(client.connected()) publishTelemetry_Core1(); 
        }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
}