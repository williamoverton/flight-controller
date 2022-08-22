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
#define SERVO_OUT_ONE 16
#define SERVO_OUT_TWO 17
#define SERVO_OUT_THREE 18
#define SERVO_OUT_FOUR 19
#define SERVO_OUT_FIVE 20
#define SERVO_OUT_SIX 21

/**
 * Channel Setup
 */
#define ROLL_INPUT_CHANNEL 0
#define PITCH_INPUT_CHANNEL 1
#define YAW_INPUT_CHANNEL 2
#define THROTTLE_INPUT_CHANNEL 3
#define MOTOR_ENABLE_INPUT_CHANNEL 4
#define MODESELECT_INPUT_CHANNEL 5

/**
 * TUNING VARIABLES
 */
static int hold_mode = CANCEL_OUT_MOVEMENT;
static float pitch_stab_gain = 1;
static float roll_stab_gain = 1;
static float yaw_stab_gain = 1;

/**
 * Done with config!
 */

/**
 * Global vars
 */
static float output_one = 0;
static float output_two = 0;
static float output_three = 0;
static float output_four = 0;
static float output_five = 0;
static float output_six = 0;

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

Servo servo_one;
Servo servo_two;
Servo servo_three;
Servo servo_four;
Servo servo_five;
Servo servo_six;

void setupServos()
{
  servo_one.attach(SERVO_OUT_ONE);
  servo_two.attach(SERVO_OUT_TWO);
  servo_three.attach(SERVO_OUT_THREE);
  servo_four.attach(SERVO_OUT_FOUR);
  servo_five.attach(SERVO_OUT_FIVE);
  servo_six.attach(SERVO_OUT_SIX);
}

// Roll PID Controller
double rollPidSetpoint, rollPidInput, rollPidOutput;
double rollPidKp = 1, rollPidKi = 0, rollPidKd = 0.2;
PID rollPID(&rollPidInput, &rollPidOutput, &rollPidSetpoint, rollPidKp, rollPidKi, rollPidKd, DIRECT);

// Pitch PID Controller
double pitchPidSetpoint, pitchPidInput, pitchPidOutput;
double pitchPidKp = 1, pitchPidKi = 0, pitchPidKd = 0.2;
PID pitchPID(&pitchPidInput, &pitchPidOutput, &pitchPidSetpoint, pitchPidKp, pitchPidKi, pitchPidKd, DIRECT);

// Yaw PID Controller
double yawPidSetpoint, yawPidInput, yawPidOutput;
double yawPidKp = 1, yawPidKi = 0, yawPidKd = 0.2;
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

float pitchStabData = 0;
float rollStabData = 0;
float yawStabData = 0;

float rcInputPitch = 0;
float rcInputRoll = 0;
float rcInputYaw = 0;
float rcInputThrottle = 0;

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
    rollStabData = min(0.5, max(-0.5, rollPidOutput * roll_stab_gain));

    pitchPidInput = (mpu.getGyroY() / 90.0);
    pitchPID.Compute();
    pitchStabData = min(0.5, max(-0.5, pitchPidOutput * pitch_stab_gain));

    yawPidInput = (mpu.getGyroZ() / 90.0);
    yawPID.Compute();
    yawStabData = min(0.5, max(-0.5, yawPidOutput * yaw_stab_gain));
  }
  else if (hold_mode == HOLD_ATTITUDE)
  {
    // Serial.println("HOLD ATTITUDE");

    rollPidInput = (mpu.getAngleX() / 90.0);
    rollPID.Compute();
    rollStabData = min(0.5, max(-0.5, rollPidOutput * roll_stab_gain));

    pitchPidInput = (mpu.getAngleY() / 90.0);
    pitchPID.Compute();
    pitchStabData = min(0.5, max(-0.5, pitchPidOutput * pitch_stab_gain));

    /**
     * Always use Gyro not angle for yaw
     * because z drift is a mad ting init
     */
    yawPidInput = (mpu.getGyroZ() / 90.0);
    yawPID.Compute();
    yawStabData = min(0.5, max(-0.5, yawPidOutput * yaw_stab_gain));
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
  // Get raw data from receiver
  for (byte channel = 0; channel < CHANNEL_AMOUNT; channel++)
  {
    raw_channel_values[channel] = ppm.rawChannelValue(channel + 1);
  }

  // Read stick inputs
  rcInputRoll = readChannel(ROLL_INPUT_CHANNEL, -0.5, 0.5, 0);
  rcInputPitch = readChannel(PITCH_INPUT_CHANNEL, -0.5, 0.5, 0);
  rcInputYaw = readChannel(YAW_INPUT_CHANNEL, -0.5, 0.5, 0);
  rcInputThrottle = readChannel(THROTTLE_INPUT_CHANNEL, 0, 1, 0);

  // Mode select switch
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

  servo_one.write(max(min_value, min(max_value, (output_one + 0.5) * (max_value - min_value * 0.5))));
  servo_two.write(max(min_value, min(max_value, (output_two + 0.5) * (max_value - min_value * 0.5))));
  servo_three.write(max(min_value, min(max_value, (output_three + 0.5) * (max_value - min_value * 0.5))));
  servo_four.write(max(min_value, min(max_value, (output_four + 0.5) * (max_value - min_value * 0.5))));
  servo_five.write(max(min_value, min(max_value, (output_five + 0.5) * (max_value - min_value * 0.5))));
  servo_six.write(max(min_value, min(max_value, (output_six + 0.5) * (max_value - min_value * 0.5))));
}

int timer;
int counter = 0;
int hzTimer = 0;
void loopRate(int rate)
{
  counter++;

  if (hzTimer < millis())
  {
    Serial.print("Clock Counter: ");
    Serial.println(counter);
    counter = 0;
    hzTimer = millis() + 1000;
  }

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
  /**
   * Adjust gains for different modes
   */
  if (hold_mode == NO_STABILISATION)
  {
  }
  else if (hold_mode == CANCEL_OUT_MOVEMENT)
  {
    pitch_stab_gain = 0.2;
    roll_stab_gain = 0.2;
  }
  else if (hold_mode == HOLD_ATTITUDE)
  {
    pitch_stab_gain = 1;
    roll_stab_gain = 1;
  }

  /**
   * Configure Output Channels
   */
  output_one = pitchStabData + rcInputPitch;
  output_two = rollStabData + rcInputRoll;
}

void loop()
{
  userDefinedFunctions();

  mpu.update();

  // printGyroInfo();
  computeStabilization();
  readController();
  positionServos();

  loopRate(500);
}

void loop1()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}