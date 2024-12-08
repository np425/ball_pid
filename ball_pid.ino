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
      this->lastUpdate = millis();
    }

    // Compute PID output
    double compute(double setpoint, double currentValue) {
      double dt = (millis() - this->lastUpdate) / 1000.0;
      this->lastUpdate = millis();
      
      //double dt = 0.1;
      Serial.print(" | dt: ");
      Serial.print(dt);

      if (dt == 0) {
        Serial.print("is zero!");
        dt = 0.0001;
      }
      
      // Calculate error
      double error = setpoint - currentValue;

      // Proportional term
      double proportional = this->kp * error;

      // Integral term
      this->integral += error * dt;
      // this->integral = constrain(this->integral, -250, 250); // Adjust limits as needed
      double integralTerm = this->ki * this->integral;

      Serial.print(" | integral: ");
      Serial.print(this->integral);

      // Derivative term
      double derivative = (error - this->previousError) / dt;
      double derivativeTerm = this->kd * derivative;
      // derivativeTerm = constrain(derivativeTerm, -150, 150); // Adjust limits as needed

      Serial.print(" | derivative: ");
      Serial.print(derivativeTerm);

      // Calculate output
      double output = proportional + integralTerm + derivativeTerm + 90.0;

      Serial.print(" | output: ");
      Serial.print(output);

      // Clamp output
      output = constrain(output, this->minOutput, this->maxOutput);



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

  //while (!NewDataReady) {
  //  sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
  //  Serial.println("Not receiving data");
  //}
  //Serial.println("Received data");

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
  unsigned long current_time = millis();

  static unsigned long last_update = 0;
  static double last_valid_distance = 0; // Use double for precision

  if (current_time - last_update > MEASUREMENT_INTERVAL_MS) {
    double distance_cm = distanceSensor.measureDistanceCm(); // Use double for distance

    if (distance_cm > 0) {
      if (last_valid_distance > 0) {
        last_valid_distance = (last_valid_distance + distance_cm) / 2.0;
      } else {
        last_valid_distance = distance_cm;
      }
    }

    last_update = current_time;
  }

  // Return distance in millimeters as a double
  return (last_valid_distance > 0) ? last_valid_distance * 10.0 : 0.0;
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
  
  //ref_distance_mm = getReferenceDistance_mm();
  ball_distance_mm = getBallDistance_mm();
  platform_angle_deg = pid.compute(ref_distance_mm, ball_distance_mm);

  /*
  if (platform_angle_deg > 180) {
    platform_angle_deg = 180;
  }
  if (platform_angle_deg < 0) {
    platform_angle_deg = 0;
  }
  */

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
