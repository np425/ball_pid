#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <HCSR04.h>
#include <Servo.h>
//#include <PID_v1.h>

// Components
#define DEV_I2C Wire
#define SerialPort Serial

#define MEASUREMENT_INTERVAL_MS 5
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define SERVO_PIN 6
#define KP_PIN A0
#define KI_PIN A1
#define KD_PIN A2
#define KP_MAX 3
#define KI_MAX 3
#define KD_MAX 10

double ref_distance_mm = 200;
double ball_distance_mm;
double platform_angle_deg;


class PID {
  private:
    double minOutput, maxOutput; // Output limits
    double integral;           // Accumulated integral term
    double previousError;      // Error from the previous step
    unsigned long lastUpdate; // Time of the last update
    double previousDerivativeAvg;
    const unsigned long derivativeAvgCount = 50;
  
  public:
    double kp, ki, kd;         // PID constants
    PID(double kp, double ki, double kd, double minOutput = -255, double maxOutput = 255) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      this->minOutput = minOutput;
      this->maxOutput = maxOutput;
      this->integral = 0;
      this->previousError = 0;
      this->previousDerivativeAvg = 0;
      this->lastUpdate = millis();
    }

    // Compute PID output
    double compute(double setpoint, double currentValue) {
      double dt = (millis() - this->lastUpdate) / 1000.0;
      this->lastUpdate = millis();

      if (dt == 0) {
        dt = 0.0001;
      }
      
      // Calculate error
      double error = setpoint - currentValue;

      // Proportional term
      double proportional = this->kp * error;

      // Integral term
      this->integral += error * dt;
      double integralTerm = constrain(this->ki * this->integral, -this->maxOutput, this->maxOutput);

      // Derivative term
      double derivative = (error - this->previousError) / dt;

      double derivativeAvg = (derivativeAvgCount * previousDerivativeAvg + derivative) / (derivativeAvgCount + 1);

      this->previousDerivativeAvg = derivativeAvg;
      
      double derivativeTerm = this->kd * derivativeAvg;

      // Calculate output
      double output = proportional + integralTerm + derivativeTerm + 90.0;

      // Clamp output
      output = constrain(output, this->minOutput, this->maxOutput);

      Serial.print(" | ");
      Serial.print(proportional);
      Serial.print(" + ");
      Serial.print(integralTerm);
      Serial.print(" + ");
      Serial.print(derivativeTerm);
      Serial.print(" = ");
      Serial.print(output);

      // Store current error for next derivative calculation
      this->previousError = error;
      return output;
    }
};

VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, -1); // -1 means no XSHUT pin used
UltraSonicDistanceSensor distanceSensor(TRIGGER_PIN, ECHO_PIN);
Servo lifterServo;
PID pid(1, 1, 1, 0, 180);


void setup() {
  // Initialize serial communication and I2C bus
  SerialPort.begin(250000);

  if (lifterServo.attach(SERVO_PIN) != 0) {
    Serial.println("Servo failed to attach");
  }

  liftPlatform(0);

  DEV_I2C.begin();

  // Initialize the VL53L4CD sensor
  if (sensor_vl53l4cd_sat.begin() != 0 || sensor_vl53l4cd_sat.InitSensor() != VL53L4CD_ERROR_NONE) {
    SerialPort.println("Failed to initialize VL53L4CD sensor.");
    while (1); // Halt if initialization fails
  }

  // Configure sensor range timing for accurate measurements
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(MEASUREMENT_INTERVAL_MS, 0);

  // Start ranging measurements
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();
}

double getBallDistance_mm() {
  uint8_t NewDataReady = 0;
  static double last_dist = 0;

  VL53L4CD_Result_t results;

  // Wait for new data to be ready
  double started = millis();

  sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);

  if (!NewDataReady) {
    return last_dist;
  }

  // Clear hardware interrupt to prepare for the next measurement
  sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

  // Retrieve measurement results
  if (sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results) == VL53L4CD_ERROR_NONE) {
    last_dist = static_cast<double>(results.distance_mm);
    return last_dist; // Convert to double
  }

  return 0.0; // Return 0.0 if there is an error
}

void liftPlatform(int degree) {
  lifterServo.write(degree);
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double readPIDParameters(double& kp, double& ki, double& kd) {
  double kp_value = analogRead(KP_PIN);
  double ki_value = analogRead(KI_PIN);
  double kd_value = analogRead(KD_PIN);

  kp = mapDouble(kp_value, 0.0, 1023.0, 0.0, KP_MAX);
  ki = mapDouble(ki_value, 0.0, 1023.0, 0.0, KI_MAX);
  kd = mapDouble(kd_value, 0.0, 1023.0, 0.0, KD_MAX);
}

double getReferenceDistance_mm() {
  unsigned long currentTime = millis();

  static unsigned long lastUpdate = 0;
  static double lastValidDistanceCm = 0;
  const double windowSize = 100;

  if (currentTime - lastUpdate > MEASUREMENT_INTERVAL_MS) {
    unsigned long distanceCm = distanceSensor.measureDistanceCm(); 

    if (distanceCm > 0) {
      lastValidDistanceCm = (windowSize * lastValidDistanceCm + distanceCm) / (windowSize + 1);
    }
    
    lastUpdate = currentTime;
  }

  return round(lastValidDistanceCm * 10);
}

void loop() {
  double kp, ki, kd;
  readPIDParameters(kp, ki, kd);

  pid.kp = kp;
  pid.ki = ki;
  pid.kd = kd;

  SerialPort.print("kp: ");
  SerialPort.print(kp);
  SerialPort.print(" ki: ");
  SerialPort.print(ki);
  SerialPort.print(" kd: ");
  SerialPort.print(kd);
  
  ref_distance_mm = getReferenceDistance_mm();
  ball_distance_mm = getBallDistance_mm();
  platform_angle_deg = pid.compute(ref_distance_mm, ball_distance_mm);

  SerialPort.print(" | Reference Distance: ");
  SerialPort.print(ref_distance_mm);
  SerialPort.print(" mm | ");

  SerialPort.print("Ball Distance: ");
  SerialPort.print(ball_distance_mm);
  SerialPort.print(" mm | ");

  SerialPort.print("Calculated angle: ");
  SerialPort.println(round(platform_angle_deg), DEC);

  liftPlatform(round(platform_angle_deg));

  delay(20);
}
