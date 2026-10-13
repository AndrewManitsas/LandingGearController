#include "FastIMU.h"

#include <Wire.h>
#include <math.h>
#include <VL53L0X.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6500 IMU;                //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

// Read at nominal 100 Hz; readable reports at 2 Hz, viewer frames at 20 Hz.
const uint32_t SAMPLE_INTERVAL_US = 10000UL;
const uint32_t VIEWER_OUTPUT_INTERVAL_US = 50000UL;
const uint32_t MONITOR_OUTPUT_INTERVAL_US = 500000UL;
// Start in human-readable mode. V enables the original viewer frames; M returns.
bool viewerOutputEnabled = false;
const float FILTER_TIME_CONSTANT_S = 0.50f;
const float MAX_SAMPLE_GAP_S = 0.20f;
const float RAD_TO_DEGREES = 57.2957795f;
uint32_t lastSampleUs = 0;
uint32_t lastOutputUs = 0;
bool attitudeInitialized = false;
float pitchDeg = 0.0f;
float rollDeg = 0.0f;

// Keep one sensor selected until the array's power and wiring are confirmed.
// Change to 3 only after connecting individual XSHUT lines and suitable power.
#ifndef TOF_SENSOR_COUNT
#define TOF_SENSOR_COUNT 1
#endif
static_assert(TOF_SENSOR_COUNT == 1 || TOF_SENSOR_COUNT == 3, "Select one sensor or the three-sensor array.");
VL53L0X tof[TOF_SENSOR_COUNT];
const uint8_t TOF_XSHUT_PINS[3] = {2, 3, 4};
const uint8_t TOF_ADDRESSES[3] = {0x30, 0x31, 0x32};
const uint16_t TOF_PERIOD_MS = 50;
const uint32_t TOF_STALE_US = 250000UL;
bool tofAvailable[TOF_SENSOR_COUNT] = {};
bool tofHasReading[TOF_SENSOR_COUNT] = {};
bool tofReadingValid[TOF_SENSOR_COUNT] = {};
bool tofCommunicationError[TOF_SENSOR_COUNT] = {};
uint16_t tofDistanceMm[TOF_SENSOR_COUNT] = {};
uint8_t tofRangeStatus[TOF_SENSOR_COUNT] = {};
uint32_t tofLastReadingUs[TOF_SENSOR_COUNT] = {};

// One beam checks clearance and repeatability, not terrain slope or material.
// These conservative bench-demo thresholds must be tuned to the demonstration.
const uint16_t LANDING_MIN_MM = 100;
const uint16_t LANDING_MAX_MM = 500;
const uint16_t LANDING_MAX_SPREAD_MM = 20;
const float LANDING_MAX_TILT_DEG = 10.0f;
const uint8_t LANDING_WINDOW_SIZE = 10;
uint16_t landingDistances[LANDING_WINDOW_SIZE] = {};
uint8_t landingSampleCount = 0;
uint8_t landingNextSample = 0;
uint16_t landingSpreadMm = 0;
const uint32_t LANDING_READY_HOLD_US = 1000000UL;
bool landingConditionsHeld = false;
uint32_t landingConditionsSinceUs = 0;
bool landingReady = false;
const __FlashStringHelper *landingState = nullptr;
const __FlashStringHelper *landingReason = nullptr;

// Virtual actuator settings: these are demonstration angles, not calibrated travel.
// No servo library is used and no output pins are driven in this milestone.
const float TARGET_PITCH_DEG = 0.0f;
const float STABILIZER_CENTER_DEG = 90.0f;
const float STABILIZER_MIN_DEG = 65.0f;
const float STABILIZER_MAX_DEG = 115.0f;
const float STABILIZER_GAIN = 1.0f; // servo degrees per degree of pitch error
const float PITCH_DEADBAND_DEG = 1.0f;
const float GEAR_STOWED_DEG = 0.0f;
const float GEAR_DEPLOYED_DEG = 90.0f;
// Latched virtual deployment: only S or board reset clears this command.
bool gearDeployRequested = false;
bool actuatorCommandsValid = false;
float stabilizerCommandDeg = STABILIZER_CENTER_DEG;
float gearCommandDeg = GEAR_STOWED_DEG;

// Choose the short angular difference across the -180/+180 degree boundary.
float wrapDegrees(float angle)
{
	while (angle > 180.0f) angle -= 360.0f;
	while (angle < -180.0f) angle += 360.0f;
	return angle;
}

bool updateAttitude(float dt)
{
	// FastIMU returns acceleration in g and gyro rates in degrees/second.
	// Acceleration ratios give angles without converting to m/s^2.
	const float ax = accelData.accelX;
	const float ay = accelData.accelY;
	const float az = accelData.accelZ;
	const float gx = gyroData.gyroX;
	const float gy = gyroData.gyroY;
	const float gz = gyroData.gyroZ;

	if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) || !isfinite(gx) || !isfinite(gy) || !isfinite(gz) || ax * ax + ay * ay + az * az < 0.000001f)
	{
		// Prevent invalid values from contaminating all subsequent estimates.
		attitudeInitialized = false;
		return false;
	}

	// Assumes +Z points upward when level: roll about X, pitch about Y.
	const float accelRoll = atan2f(ay, az) * RAD_TO_DEGREES;
	const float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEGREES;

	if (!attitudeInitialized || dt > MAX_SAMPLE_GAP_S)
	{
		// Initialize at actual tilt. After a long gap, restart because motion
		// during the missing interval was not observed.
		rollDeg = accelRoll;
		pitchDeg = accelPitch;
		attitudeInitialized = true;
		return true;
	}

	// Gyros track short-term motion; the gravity reference corrects drift.
	// The weight uses actual dt so timing variation does not change the time constant.
	const float alpha = FILTER_TIME_CONSTANT_S / (FILTER_TIME_CONSTANT_S + dt);
	const float predictedRoll = wrapDegrees(rollDeg + gx * dt);
	const float predictedPitch = pitchDeg + gy * dt;
	rollDeg = wrapDegrees(predictedRoll + (1.0f - alpha) * wrapDegrees(accelRoll - predictedRoll));
	pitchDeg = alpha * predictedPitch + (1.0f - alpha) * accelPitch;
	return true;
}

// Checked register reads supplement the compact driver: reject a short I2C
// response rather than accidentally displaying a missing byte as a distance.
bool readTofRegisters(uint8_t index, uint8_t reg, uint8_t *data, uint8_t count)
{
	Wire.beginTransmission(tof[index].getAddress());
	Wire.write(reg);

	if (Wire.endTransmission() != 0)
	{
		return false;
	}

	if (Wire.requestFrom(tof[index].getAddress(), count) != count)
	{
		return false;
	}

	for (uint8_t i = 0; i < count; ++i)
	{
		data[i] = Wire.read();
	}
	
	return true;
}

void initializeTof()
{
	#if TOF_SENSOR_COUNT == 3
		// All sensors start at 0x29. Hold all off before enabling them one at a time.
		for (uint8_t i = 0; i < TOF_SENSOR_COUNT; ++i)
		{
			digitalWrite(TOF_XSHUT_PINS[i], LOW);
			pinMode(TOF_XSHUT_PINS[i], OUTPUT);
		}
		delay(10);
	#endif

	for (uint8_t index = 0; index < TOF_SENSOR_COUNT; ++index)
	{
		#if TOF_SENSOR_COUNT == 3
			// XSHUT is NOT 5 V tolerant. Release it to its on-board pull-up;
			// never drive HIGH or use INPUT_PULLUP on the Uno.
			pinMode(TOF_XSHUT_PINS[index], INPUT);
			delay(10);
		#endif
			tof[index].setTimeout(100);
			tofAvailable[index] = tof[index].init();

		#if TOF_SENSOR_COUNT == 3
			if (tofAvailable[index])
			{
				tof[index].setAddress(TOF_ADDRESSES[index]);
				tofAvailable[index] = tof[index].last_status == 0;
			}
		#endif
			if (tofAvailable[index])
			{
				tofAvailable[index] = tof[index].setMeasurementTimingBudget(33000);
				if (tofAvailable[index])
				{
					tof[index].startContinuous(TOF_PERIOD_MS);
					tofAvailable[index] = tof[index].last_status == 0;
				}
			}

		#if TOF_SENSOR_COUNT == 3
			if (!tofAvailable[index])
			{
				// A failed sensor might still be at 0x29; isolate it before the next.
				digitalWrite(TOF_XSHUT_PINS[index], LOW);
				pinMode(TOF_XSHUT_PINS[index], OUTPUT);
			}
		#endif
			Serial.print(F("ToF ")); Serial.print(index + 1);
			if (tofAvailable[index])
			{
				Serial.print(F(" online at 0x"));
				Serial.println(tof[index].getAddress(), HEX);
			}
			else Serial.println(F(" unavailable: check wiring, then reset."));
	}
}

void updateOneTof(uint8_t index)
{
	if (!tofAvailable[index]) return;
	uint8_t ready = 0;
	if (!readTofRegisters(index, VL53L0X::RESULT_INTERRUPT_STATUS, &ready, 1))
	{
		tofCommunicationError[index] = true;
		tofReadingValid[index] = false;
		return;
	}
	// Check readiness without blocking for the sensor's measurement cycle.
	if ((ready & 0x07) == 0) return;

	// Read the result block atomically: status is byte 0; range is bytes 10/11.
	uint8_t result[12] = {};
	if (!readTofRegisters(index, VL53L0X::RESULT_RANGE_STATUS, result, sizeof(result)))
	{
		tofCommunicationError[index] = true;
		tofReadingValid[index] = false;
		return;
	}
	tof[index].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
	tofCommunicationError[index] = tof[index].last_status != 0;
	// A new result after a long gap must not reuse pre-gap stability samples.
	if (index == 0 && tofHasReading[index] && micros() - tofLastReadingUs[index] > TOF_STALE_US)
	{
		landingSampleCount = 0;
		landingNextSample = 0;
	}
	tofHasReading[index] = true;
	tofLastReadingUs[index] = micros();
	// Raw device status 11 means ranging complete successfully. This is NOT
	// the ST API's remapped status 0; display the raw code for troubleshooting.
	tofRangeStatus[index] = (result[0] & 0x78) >> 3;
	const uint16_t distance = (uint16_t(result[10]) << 8) | result[11];
	tofReadingValid[index] = !tofCommunicationError[index] && tofRangeStatus[index] == 11 && distance < 8190;
	if (tofReadingValid[index])
	{
		tofDistanceMm[index] = distance;
		// Add only completed measurements, never repeated polling of an old value.
		if (index == 0)
		{
			landingDistances[landingNextSample] = distance;
			landingNextSample = (landingNextSample + 1) % LANDING_WINDOW_SIZE;
			if (landingSampleCount < LANDING_WINDOW_SIZE)
			{
				++landingSampleCount;
			}
		}
	}
}

void publishOneTofTelemetry(uint8_t index)
{
	Serial.print(F(" | ToF")); Serial.print(index + 1); Serial.print(F(": "));

	if (!tofAvailable[index])
	{
		Serial.print(F("UNAVAILABLE"));
	}
	else if (tofCommunicationError[index])
	{
		Serial.print(F("I2C ERROR"));
	}
	else if (!tofHasReading[index])
	{
		Serial.print(F("WAITING"));
	}
	else if (micros() - tofLastReadingUs[index] > TOF_STALE_US)
	{
		Serial.print(F("STALE"));
	}
	else if (!tofReadingValid[index])
	{
		Serial.print(F("INVALID (status "));
		Serial.print(tofRangeStatus[index]);
		Serial.print(')');
	}
	else
	{
		Serial.print(tofDistanceMm[index]);
		Serial.print(F(" mm VALID"));
	}
}

void updateTof()
{
	for (uint8_t index = 0; index < TOF_SENSOR_COUNT; ++index)
	{
		updateOneTof(index);
	}
}

void publishTofTelemetry()
{
	for (uint8_t index = 0; index < TOF_SENSOR_COUNT; ++index)
	{
		publishOneTofTelemetry(index);
	}
}

void evaluateLandingConditions(bool attitudeValid)
{
	landingReady = false;
	landingState = F("UNKNOWN");
	landingReason = F("invalid sensor data");
	const bool freshRange = tofAvailable[0] && tofHasReading[0] && !tofCommunicationError[0] && tofReadingValid[0] && micros() - tofLastReadingUs[0] <= TOF_STALE_US;
	
	if (!attitudeValid || !freshRange)
	{
		// Rebuild the entire window after a fault or stale result.
		landingSampleCount = 0;
		landingNextSample = 0;
		return;
	}
	landingState = F("UNSAFE");
	
	if (fabsf(pitchDeg) > LANDING_MAX_TILT_DEG || fabsf(rollDeg) > LANDING_MAX_TILT_DEG)
	{
		landingReason = F("platform tilted");
		return;
	}
	
	if (tofDistanceMm[0] < LANDING_MIN_MM || tofDistanceMm[0] > LANDING_MAX_MM)
	{
		landingReason = F("distance outside demo window");
		return;
	}
	
	if (landingSampleCount < LANDING_WINDOW_SIZE)
	{
		landingState = F("UNKNOWN");
		landingReason = F("collecting 10 readings");
		return;
	}
	
	uint16_t minimum = landingDistances[0];
	uint16_t maximum = minimum;

	for (uint8_t i = 1; i < LANDING_WINDOW_SIZE; ++i)
	{
		if (landingDistances[i] < minimum) minimum = landingDistances[i];
		if (landingDistances[i] > maximum) maximum = landingDistances[i];
	}
	landingSpreadMm = maximum - minimum;

	if (landingSpreadMm > LANDING_MAX_SPREAD_MM || minimum < LANDING_MIN_MM || maximum > LANDING_MAX_MM)
	{
		landingReason = F("distance window unstable or out of range");
		return;
	}
	landingReady = true;
	landingState = F("READY");
	landingReason = F("bench checks passed");
}

void updateLandingReadiness(bool attitudeValid)
{
	// First evaluate the existing range window, freshness and tilt checks.
	evaluateLandingConditions(attitudeValid);
	const uint32_t nowUs = micros();
	if (!landingReady)
	{
		// Any failed check breaks the continuous one-second qualification.
		landingConditionsHeld = false;
		return;
	}
	if (!landingConditionsHeld)
	{
		landingConditionsHeld = true;
		landingConditionsSinceUs = nowUs;
	}
	if (nowUs - landingConditionsSinceUs < LANDING_READY_HOLD_US)
	{
		landingReady = false;
		landingState = F("UNKNOWN");
		landingReason = F("qualifying for 1 second");
	}
}

void readDemoCommands()
{
	// Serial Monitor: D requests deployment while READY; S stows. Newlines are ignored.
	// Limit work per loop so a stream of incoming text cannot delay sampling.
	for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count)
	{
		const char command = Serial.read();
		if (command == 'D' || command == 'd')
		{
			// Repeated D cannot undo an already latched deployment.
			if (gearDeployRequested || landingReady)
			{
				gearDeployRequested = true;
				Serial.println(F("[COMMAND] Gear DEPLOYED (virtual, latched)."));
			}
			else
			{
				Serial.print(F("[BLOCKED] Deployment: "));
				Serial.println(landingReason);
			}
		}
		else if (command == 'S' || command == 's')
		{
			gearDeployRequested = false;
			Serial.println(F("[COMMAND] Gear STOWED (virtual)."));
		}
		else if (command == 'V' || command == 'v')
		{
			viewerOutputEnabled = true;
			Serial.println(F("[DISPLAY] Viewer frames enabled at 20 Hz. M returns to readable reports."));
		}
		else if (command == 'M' || command == 'm')
		{
			viewerOutputEnabled = false;
			Serial.println(F("[DISPLAY] Readable reports enabled at 2 Hz."));
		}
	}
}

void updateActuatorCommands(bool attitudeValid)
{
	// Gear state is independent of permission to begin a new deployment.
	gearCommandDeg = gearDeployRequested ? GEAR_DEPLOYED_DEG : GEAR_STOWED_DEG;
	actuatorCommandsValid = attitudeValid;
	if (!attitudeValid)
	{
		// Neutralize the stabilizer, but retain the latched gear command.
		stabilizerCommandDeg = STABILIZER_CENTER_DEG;
		return;
	}

	// Proportional response only: this demo shows a correction to measured tilt.
	// There is no aerodynamic plant, so it does not prove physical stabilization.
	float pitchError = TARGET_PITCH_DEG - pitchDeg;

	if (fabsf(pitchError) <= PITCH_DEADBAND_DEG) 
	{
		pitchError = 0.0f;
	}

	stabilizerCommandDeg = constrain(STABILIZER_CENTER_DEG + STABILIZER_GAIN * pitchError, STABILIZER_MIN_DEG, STABILIZER_MAX_DEG);
}

void outputActuatorCommands()
{
	// Hardware output placeholder. Deliberately empty: both servos stay disconnected.
	// Later, calibrated Servo.write() calls belong here, after power and linkage
	// checks. Verify STABILIZER_GAIN's sign with the actual mechanism first.
}

void publishActuatorTelemetry()
{
	Serial.print(F("  Actuators  : Gear "));
	Serial.print(gearDeployRequested ? F("DEPLOYED") : F("STOWED"));
	Serial.print(F(" (")); Serial.print(gearCommandDeg, 0); Serial.print(F(" deg)"));
	Serial.print(F(" | Stabilizer ")); Serial.print(stabilizerCommandDeg, 1);
	Serial.println(F(" deg | VIRTUAL"));
	if (!actuatorCommandsValid)
	{
		Serial.println(F("  IMU warning: Invalid sample; stabilizer held neutral."));
	}
	if (gearDeployRequested && !landingReady)
	{
		Serial.println(F("  Gear warning: Readiness lost. Gear remains deployed until S."));
	}
}

void publishViewerFrame()
{
	// Preserve the six-value protocol expected by tools/IMU.py.
	Serial.print('S');
	Serial.print(accelData.accelX, 2); Serial.print('/');
	Serial.print(accelData.accelY, 2); Serial.print('/');
	Serial.print(accelData.accelZ, 2); Serial.print('/');
	Serial.print(gyroData.gyroX, 2); Serial.print('/');
	Serial.print(gyroData.gyroY, 2); Serial.print('/');
	Serial.print(gyroData.gyroZ, 2); Serial.println('E');
}

void publishTelemetry()
{
	if (viewerOutputEnabled)
	{
		// Skip invalid frames so the viewer never treats them as real measurements.
		if (actuatorCommandsValid) publishViewerFrame();
		else Serial.println(F("[IMU] Invalid sample; frame omitted."));
		return;
	}
	// Plain text works in Arduino Serial Monitor without terminal escape codes.
	// Flash-resident labels and individual prints avoid dynamic RAM allocations.
	Serial.println();
	Serial.print(F("--- Landing Gear Demo | ")); Serial.print(millis() / 1000UL);
	Serial.println(F(" s ---"));
	Serial.print(F("  Sequence   : "));
	if (gearDeployRequested) Serial.println(F("DEPLOYED"));
	else if (landingReady) Serial.println(F("READY TO DEPLOY"));
	else Serial.println(F("STOWED"));
	Serial.print(F("  Readiness  : ")); Serial.print(landingState);
	Serial.print(F(" - ")); Serial.println(landingReason);
	Serial.print(F("  Attitude   : "));
	if (actuatorCommandsValid)
	{
		Serial.print(F("Pitch ")); Serial.print(pitchDeg, 1);
		Serial.print(F(" deg | Roll ")); Serial.print(rollDeg, 1); Serial.println(F(" deg"));
	}
	else Serial.println(F("INVALID (angles unavailable)"));
	Serial.print(F("  Range      :")); publishTofTelemetry(); Serial.println();
	Serial.print(F("  Stability  : ")); Serial.print(landingSampleCount);
	Serial.print('/'); Serial.print(LANDING_WINDOW_SIZE); Serial.print(F(" readings"));
	if (landingConditionsHeld)
	{
		Serial.print(F(" | Spread ")); Serial.print(landingSpreadMm); Serial.print(F(" mm"));
		const uint32_t heldUs = micros() - landingConditionsSinceUs;
		Serial.print(F(" | Qualification "));
		Serial.print(heldUs >= LANDING_READY_HOLD_US ? 100 : heldUs / 10000UL);
		Serial.print('%');
	}
	Serial.println();
	publishActuatorTelemetry();
}

void setup()
{
	Wire.begin();
	Wire.setClock(400000); //400khz clock

	#ifdef WIRE_HAS_TIMEOUT
		// Bound a stuck I2C transaction on supported AVR Wire versions.
		Wire.setWireTimeout(25000, true);
	#endif

	Serial.begin(115200);
	while (!Serial)
	{
		;
	}

	int err = IMU.init(calib, IMU_ADDRESS);
	if (err != 0)
	{
		Serial.print("Error initializing IMU: ");
		Serial.println(err);
		while (true)
		{
			;
		}
	}
	
	#ifdef PERFORM_CALIBRATION
		Serial.println("FastIMU calibration & data example");
		if (IMU.hasMagnetometer())
		{
			delay(1000);
			Serial.println("Move IMU in figure 8 pattern until done.");
			delay(3000);
			IMU.calibrateMag(&calib);
			Serial.println("Magnetic calibration done!");
		}
		else
		{
			delay(5000);
		}

		delay(5000);
		Serial.println("Keep IMU level.");
		delay(5000);
		IMU.calibrateAccelGyro(&calib);
		Serial.println("Calibration done!");
		Serial.println("Accel biases X/Y/Z: ");
		Serial.print(calib.accelBias[0]);
		Serial.print(", ");
		Serial.print(calib.accelBias[1]);
		Serial.print(", ");
		Serial.println(calib.accelBias[2]);
		Serial.println("Gyro biases X/Y/Z: ");
		Serial.print(calib.gyroBias[0]);
		Serial.print(", ");
		Serial.print(calib.gyroBias[1]);
		Serial.print(", ");
		Serial.println(calib.gyroBias[2]);
		if (IMU.hasMagnetometer())
		{
			Serial.println("Mag biases X/Y/Z: ");
			Serial.print(calib.magBias[0]);
			Serial.print(", ");
			Serial.print(calib.magBias[1]);
			Serial.print(", ");
			Serial.println(calib.magBias[2]);
			Serial.println("Mag Scale X/Y/Z: ");
			Serial.print(calib.magScale[0]);
			Serial.print(", ");
			Serial.print(calib.magScale[1]);
			Serial.print(", ");
			Serial.println(calib.magScale[2]);
		}
		delay(5000);
		// Apply calibration and check this second initialization too.
		err = IMU.init(calib, IMU_ADDRESS);
		if (err != 0)
		{
			Serial.print(F("Error reinitializing calibrated IMU: "));
			Serial.println(err);
			while (true) { }
		}
	#endif

	initializeTof();

	// Exclude calibration and sensor initialization delays from integration timing.
	lastSampleUs = micros();
	lastOutputUs = lastSampleUs;
	Serial.println(F("Attitude estimation active: +Z upward when level."));
	Serial.println(F("=== LANDING GEAR DEMO | VIRTUAL ACTUATORS ==="));
	Serial.println(F("D: deploy when READY | S: stow | V: viewer frames | M: readable reports"));
	Serial.println(F("Demo limits: 100-500 mm, tilt <=10 deg, spread <=20 mm, then 1 s qualification."));
	Serial.println(F("Deployment stays latched until S or reset. Reports update twice per second."));
}

void loop()
{
	const uint32_t nowUs = micros();
	// Unsigned subtraction remains correct across micros() rollover.
	const uint32_t elapsedUs = nowUs - lastSampleUs;
	if (elapsedUs < SAMPLE_INTERVAL_US) return;
	lastSampleUs = nowUs;

	IMU.update();
	IMU.getAccel(&accelData);
	IMU.getGyro(&gyroData);

	// Integrate over measured elapsed time rather than assuming exactly 10 ms.
	const bool valid = updateAttitude(elapsedUs * 0.000001f);
	updateTof();
	updateLandingReadiness(valid);
	// Process commands against this cycle's readings, not old readiness.
	readDemoCommands();
	updateActuatorCommands(valid);
	outputActuatorCommands();
	if (nowUs - lastOutputUs >= (viewerOutputEnabled ? VIEWER_OUTPUT_INTERVAL_US : MONITOR_OUTPUT_INTERVAL_US))
	{
		lastOutputUs = nowUs;
		publishTelemetry();
	}
}