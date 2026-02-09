/*
 * IMU Angle Calculator (Complementary Filter)
 * Hardware: MPU6050
 * Output: Pitch, Roll, Yaw (deg)
 */

#include <Wire.h>

const int MPU_ADDR = 0x68;

// Variables for raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Calibration offsets
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Variables for calculated angles
float accel_pitch, accel_roll;
float gyro_pitch, gyro_roll, gyro_yaw;
float pitch, roll, yaw;

// Time tracking for integration
long last_time;
float dt;

// Complementary Filter Weight
// 0.90 means we trust Gyro 90% and Accel 10%
const float ALPHA = 0.90; 

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // --- CALIBRATION ROUTINE ---
  Serial.println("Calibrating... KEEP SENSOR STILL!");

  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int buffersize = 2000;
  
  for (int i = 0; i < buffersize; i++)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Start reading at Gyro Registers (0x43)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    
    gx_sum += (int16_t)(Wire.read()<<8 | Wire.read());
    gy_sum += (int16_t)(Wire.read()<<8 | Wire.read());
    gz_sum += (int16_t)(Wire.read()<<8 | Wire.read());
    delay(2); // Small delay to prevent bus saturation
  }
  
  gx_offset = gx_sum / (float)buffersize;
  gy_offset = gy_sum / (float)buffersize;
  gz_offset = gz_sum / (float)buffersize;

  Serial.print("Offsets -> X:"); Serial.print(gx_offset);
  Serial.print(" Y:"); Serial.print(gy_offset);
  Serial.print(" Z:"); Serial.println(gz_offset);
  Serial.println("Calibration Done.");
  delay(2000);

  last_time = micros();
}

void loop()
{
  // --- 1. Calculate Delta Time (dt) ---
  // Used for integrating gyro rates
  unsigned long current_time = micros();
  dt = (current_time - last_time) / 1000000.0; // Convert micros to seconds
  last_time = current_time;

  // --- 2. Read Raw Data ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = Wire.read()<<8 | Wire.read();
  int16_t ay = Wire.read()<<8 | Wire.read();
  int16_t az = Wire.read()<<8 | Wire.read();
  int16_t temp = Wire.read()<<8 | Wire.read(); // Discard or use for calib
  int16_t gx = Wire.read()<<8 | Wire.read();
  int16_t gy = Wire.read()<<8 | Wire.read();
  int16_t gz = Wire.read()<<8 | Wire.read();

  // --- 3. Calibration (OPTIONAL BUT RECOMMENDED) ---
  // You should subtract offsets here (measured when sensor is flat/still)
  float gx_corrected = gx - gx_offset;
  float gy_corrected = gy - gy_offset;
  float gz_corrected = gz - gz_offset;

  // --- 4. Calculate Accelerometer Angles ---
  // 57.296 = 180 / PI
  float accel_roll  = atan2(ay, az) * 57.296;
  float accel_pitch = atan2(-ax, sqrt((float)ay*ay + (float)az*az)) * 57.296;

  // --- 5. Gyro Integration ---
  // Default sensitivity is 131.0 for +/- 250deg/s
  // If you want +/- 500deg/s, you must set register 0x1B to 0x08 in setup()
  float gyro_rate_x = gx_corrected / 131.0; 
  float gyro_rate_y = gy_corrected / 131.0;
  float gyro_rate_z = gz_corrected / 131.0;

  // --- 6. The Complementary Filter ---
  // Filter: Angle = 0.90 * (Gyro_Integration) + 0.10 * (Accel_Snapshot)
  // This fuses the stability of the Gyro with the non-drifting absolute reference of Gravity
 
  // Roll (Rotation around X-axis)
  roll  = ALPHA * (roll  + gyro_rate_x * dt) + (1.0 - ALPHA) * accel_roll;

  // Pitch (Rotation around Y-axis)
  pitch = ALPHA * (pitch + gyro_rate_y * dt) + (1.0 - ALPHA) * accel_pitch;

  // Yaw (Rotation around Z-axis)
  yaw = yaw + gyro_rate_z * dt;

  // --- 7. Print Data ---
  Serial.print("Roll (X):"); Serial.print(roll);
  Serial.print(" Pitch (Y):"); Serial.print(pitch);
  Serial.print(" Yaw (Z):"); Serial.println(yaw);
  
  // Run loop at ~100Hz max to avoid clogging Serial buffer
  delay(10); 
}