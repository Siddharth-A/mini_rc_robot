#include <Arduino.h>

// #define MOTOR1_IN1 14
// #define MOTOR1_IN2 12
// #define MOTOR1_EN 13
// #define MOTOR2_IN3 2
// #define MOTOR2_IN4 5
// #define MOTOR2_EN 4
// #define CH3 15 // throttle
// #define CH1 16 // turn

#define CH1 16
#define CH3 15
#define pwmA 13
#define in1A 14
#define in2A 12
#define pwmB 4
#define in1B 2
#define in2B 5

// Motor Speed Values - Start at zero
int MotorSpeedA = 0;
int MotorSpeedB = 0;
// Motor Direction Values - 0 = backward, 1 = forward
int MotorDirA = 1;
int MotorDirB = 1;

int rcCH1 = 0;
int rcCH3 = 0;

// Control Motor A
void mControlA(int mspeed, int mdir)
{
  // Determine direction
  if (mdir == 0)
  {
    // Motor backward
    digitalWrite(in1A, LOW);
    digitalWrite(in2A, HIGH);
  }
  else
  {
    // Motor forward
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
  }
  // Control motor
  analogWrite(pwmA, mspeed);
}

// Control Motor B
void mControlB(int mspeed, int mdir)
{
  // Determine direction
  if (mdir == 0)
  {
    // Motor backward
    digitalWrite(in1B, LOW);
    digitalWrite(in2B, HIGH);
  }
  else
  {
    // Motor forward
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);
  }
  // Control motor
  analogWrite(pwmB, mspeed);
}

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue)
{
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100)
    return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue)
{
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup()
{
  // Start serial monitor for debugging
  Serial.begin(115200);

  // Set all the motor control pins to outputs
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  pinMode(CH1, INPUT);
  pinMode(CH3, INPUT);
}

void loop()
{
  // Get RC channel values
  rcCH1 = readChannel(CH1, -100, 100, 0);
  rcCH3 = readChannel(CH3, -100, 100, -100);

  // Print values to serial monitor for debugging
  Serial.print("Ch1 = ");
  Serial.print(rcCH1);
  Serial.print(" Ch3 = ");
  Serial.print(rcCH3);

  // Set speeds with channel 3 value
  MotorSpeedA = rcCH3;
  MotorSpeedB = rcCH3;

  // Set left/right offset with channel 1 value
  MotorSpeedA = MotorSpeedA - rcCH1;
  MotorSpeedB = MotorSpeedB + rcCH1;

  // Ensure that speeds are between 0 and 255
  MotorSpeedA = constrain(MotorSpeedA, 0, 255);
  MotorSpeedB = constrain(MotorSpeedB, 0, 255);

  // Drive Motors
  mControlA(MotorSpeedA, MotorDirA);
  mControlB(MotorSpeedB, MotorDirB);

  // Print speed values to serial monitor for debugging
  Serial.print("Motor A Speed = ");
  Serial.print(MotorSpeedA);
  Serial.print(" | Motor B Speed = ");
  Serial.println(MotorSpeedB);

  // Slight delay
  delay(50);
}