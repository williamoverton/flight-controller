#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <PPMReader.h>
#include <PID_v1.h>

/**
 * Stabilisation Modes
 */
#define NO_STABILISATION 0
#define CANCEL_OUT_MOVEMENT 1
#define HOLD_ATTITUDE 2

/**
 * RC Receiver Connection
 */
#define CONTROLLER_PPM_PIN 15
#define CHANNEL_AMOUNT 6
PPMReader ppm = PPMReader(CONTROLLER_PPM_PIN, CHANNEL_AMOUNT);
unsigned raw_channel_values[CHANNEL_AMOUNT] = {};

/**
 * Gyroscope Connection
 */
MPU6050 mpu(Wire);

/**
 * Pin setup
 */
#define ELEVATOR_INPUT_CHANNEL 1
#define ELEVATOR_OUTPUT_PIN 27
#define AILERON_INPUT_CHANNEL 0
#define AILERON_OUTPUT_PIN 26

#define MODESELECT_INPUT_CHANNEL 5

/**
 * TUNING VARIABLES
 */
static int hold_mode = CANCEL_OUT_MOVEMENT;
static float elevator_stab_gain = 1;
static float aileron_stab_gain = 1;

/**
 * Done with config!
 */

void setupPins()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void setupMpu()
{
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
  {
  }

  Serial.println(F("Calculating offsets, do not move!!"));
  delay(500);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial.println("Done!\n");
}

Servo elevatorServo;
Servo aileronServo;

void setupServos()
{
  elevatorServo.attach(ELEVATOR_OUTPUT_PIN);
  aileronServo.attach(AILERON_OUTPUT_PIN);
}

// Roll PID Controller
double rollPidSetpoint, rollPidInput, rollPidOutput;
double rollPidKp = 1, rollPidKi = 0.005, rollPidKd = 0.2;
PID rollPID(&rollPidInput, &rollPidOutput, &rollPidSetpoint, rollPidKp, rollPidKi, rollPidKd, DIRECT);

// Pitch PID Controller
double pitchPidSetpoint, pitchPidInput, pitchPidOutput;
double pitchPidKp = 1, pitchPidKi = 0.005, pitchPidKd = 0.2;
PID pitchPID(&pitchPidInput, &pitchPidOutput, &pitchPidSetpoint, pitchPidKp, pitchPidKi, pitchPidKd, DIRECT);

// Yaw PID Controller
double yawPidSetpoint, yawPidInput, yawPidOutput;
double yawPidKp = 1, yawPidKi = 0.005, yawPidKd = 0.2;
PID yawPID(&yawPidInput, &yawPidOutput, &yawPidSetpoint, yawPidKp, yawPidKi, yawPidKd, DIRECT);

void setupPids()
{
  rollPidSetpoint = 0;
  rollPID.SetOutputLimits(-1, 1);
  rollPID.SetMode(AUTOMATIC);

  pitchPidSetpoint = 0;
  pitchPID.SetOutputLimits(-1, 1);
  pitchPID.SetMode(AUTOMATIC);

  yawPidSetpoint = 0;
  yawPID.SetOutputLimits(-1, 1);
  yawPID.SetMode(AUTOMATIC);
}

void setup()
{
  Serial.begin(115200);

  // while (!Serial){
  //   delay(10);
  // }
  // delay(500);

  ppm.begin();
  setupPins();
  setupServos();
  setupMpu();
  setupPids();
}

long gyroPrintTimer = 0;
void printGyroInfo()
{
  if (millis() - gyroPrintTimer > 1000)
  { // print data every second
    Serial.print(F("TEMPERATURE: "));
    Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));
    Serial.print(mpu.getAccX());
    Serial.print("\tY: ");
    Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");
    Serial.println(mpu.getAccZ());

    Serial.print(F("GYRO      X: "));
    Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");
    Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");
    Serial.println(mpu.getGyroZ());

    Serial.print(F("ACC ANGLE X: "));
    Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");
    Serial.println(mpu.getAccAngleY());

    Serial.print(F("ANGLE     X: "));
    Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");
    Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");
    Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));

    gyroPrintTimer = millis();
  }

  //  Serial.print(F("ACC X: "));Serial.println(mpu.getAccX());
}

float elevatorStabData = 0;
float aileronStabData = 0;
float rcInputElevator = 0.5;
float rcInputAileron = 0.5;

void computeStabilization()
{
  if (hold_mode == NO_STABILISATION)
  {
    // Serial.println("NO STABILISATION");
  }
  else if (hold_mode == CANCEL_OUT_MOVEMENT)
  {
    // Serial.println("CANCEL OUT MOVEMENT");

    rollPidInput = mpu.getGyroX() / 90.0;
    rollPID.Compute();
    aileronStabData = min(1.0, max(0.0, rollPidOutput * aileron_stab_gain + 0.5));

    pitchPidInput = (mpu.getGyroY() / 90.0);
    pitchPID.Compute();
    elevatorStabData = min(1.0, max(0.0, pitchPidOutput * elevator_stab_gain + 0.5));
  }
  else if (hold_mode == HOLD_ATTITUDE)
  {
    // Serial.println("HOLD ATTITUDE");

    rollPidInput = (mpu.getAngleX() / 90.0);
    rollPID.Compute();
    aileronStabData = min(1.0, max(0.0, rollPidOutput * aileron_stab_gain + 0.5));

    pitchPidInput = (mpu.getAngleY() / 90.0);
    pitchPID.Compute();
    elevatorStabData = min(1.0, max(0.0, pitchPidOutput * elevator_stab_gain + 0.5));
  }
}

float readChannel(int channel, float minLimit, float maxLimit, float defaultValue)
{
  int in_min = 1000;
  int in_max = 2000;

  int pwm = raw_channel_values[channel];
  if (pwm < 100)
    return defaultValue;
  return (float)(pwm - in_min) * (float)(maxLimit - minLimit) / (float)(in_max - in_min) + minLimit;
}

void readController()
{
  for (byte channel = 0; channel < CHANNEL_AMOUNT; channel++)
  {
    raw_channel_values[channel] = ppm.rawChannelValue(channel + 1);
  }

  rcInputAileron = readChannel(AILERON_INPUT_CHANNEL, 0, 1, 0.5);
  rcInputElevator = readChannel(ELEVATOR_INPUT_CHANNEL, 0, 1, 0.5);

  float modeSelect = readChannel(MODESELECT_INPUT_CHANNEL, 0.0, 1.0, 0);
  if (modeSelect < 0.33)
  {
    hold_mode = CANCEL_OUT_MOVEMENT;
  }
  else if (modeSelect < 0.66)
  {
    hold_mode = HOLD_ATTITUDE;
  }
  else
  {
    hold_mode = NO_STABILISATION;
  }
}

void positionServos()
{
  int min_value = 0;
  int max_value = 180;

  elevatorServo.write(max(min_value, min(max_value, ((elevatorStabData - 0.5) + (rcInputElevator - 0.5) + 0.5) * (max_value - min_value * 0.5))));
  aileronServo.write(max(min_value, min(max_value, ((aileronStabData - 0.5) + (rcInputAileron - 0.5) + 0.5) * (max_value - min_value * 0.5))));
}

int timer;
void loopRate(int rate)
{
  float timeBetween = (float)1000 / (float)rate;
  if (millis() - timer > timeBetween)
  {
    Serial.println("WARNING: loop rate too low!");
  }

  while (millis() - timer < timeBetween)
  {
    delay(1);
  }
  timer = millis();
}

void userDefinedFunctions()
{
  if (hold_mode == NO_STABILISATION)
  {
  }
  else if (hold_mode == CANCEL_OUT_MOVEMENT)
  {
    elevator_stab_gain = 0.1;
    aileron_stab_gain = 0.1;
  }
  else if (hold_mode == HOLD_ATTITUDE)
  {
    elevator_stab_gain = 1;
    aileron_stab_gain = 1;
  }
}

int counter = 0;
int hzTimer = 0;
void loop()
{
  counter++;

  if (hzTimer < millis())
  {
    Serial.println(counter);
    counter = 0;
    hzTimer = millis() + 1000;
  }

  userDefinedFunctions();

  mpu.update();

  // printGyroInfo();
  computeStabilization();
  readController();
  positionServos();

  // loopRate(2000);
}

void loop1()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}