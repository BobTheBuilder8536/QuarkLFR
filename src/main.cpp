#include <Arduino.h>
#include <pins_arduino.h>
#include <time.h>

#define IR 5          // no. of IR sensors
#define KS 1000       // sensor factor used for calibration
#define SENS 450      // sensitivity of IR for line detection
#define SWITCHING_THRESHOLD 2000 //Change in error needed to trigger an inversion

#define CALSPEED 40   // calibration speed of motors
#define FWDSPEED 40   // forward speed of motors
#define MAXSPEED 80   // max speed of motors

// MotorA = LEFT
// MotorB = RIGHT
#define AIN1 8
#define AIN2 7       // pwm pin used to control speed of MotorA
#define BIN2 9       // pwm pin used to control speed of MotorA
#define BIN1 10
#define STARTSWITCH 12

#define KPID 1     // PID constant
#define KP 1.5      // proportional constant
#define KD 0        // differential constant
#define KI 0        // integral constant

int16_t P, I, D, PID, PrevErr = 0;

const uint8_t sensorPin[IR] = { A1, A2, A3, A6, A7 };     // IR sensor pins
int sensor[IR], sensorMin[IR], sensorMax[IR] = {0}, sensorRaw[IR];  // sensor value and limits array

int weightedSum = 0;
int sum = 0;

int posX = 0; // position of line wrt sensors

int16_t SpeedA, SpeedB = 0; // speed of both motors

bool lineDetected = false;  // boolean variable determining whether a line is detected or not
bool testing = false;       // boolean variable determining whether test suite is executed or not
uint8_t counter = 0;

//for testing purposes
char data[150];
char sensor_position[150];

bool b_line = true; //Black line  by default

// function prototypes
void calibrate(void);
void getSensorVal(void);
void motorDrive();
void calculatePID(void);
void motorStop(void);
void test_suite(void);

void setup() {

  Serial.begin(500000);
  delay(5000);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT); 
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STARTSWITCH, INPUT_PULLUP);

  Serial.println("Calibration Started");
  SpeedA = CALSPEED;
  SpeedB = -CALSPEED;
  motorDrive();
  calibrate();
  motorStop();
  
  Serial.println("Calibration Done");
  while( digitalRead(STARTSWITCH) ) delay(10);
}

void loop() {

  getSensorVal();

  // weightedSum = (sensor[0] * -2) + (sensor[4] * 2);
  // sum = sensor[0] + sensor[

//TEST SUITE
// UNCOMMENT THE LINES BELOW TO ENABLE THE TEST SUITE
// ----------------------------------------------------------------
//  test_suite();
//-----------------------------------------------------------------


//ACTUAL RUN
//UNCOMMENT THE LINES BELOW TO ACTIVATE THE ACTUAL RUN
//-----------------------------------------------------------------
  motorDrive();
  calculatePID();
//-----------------------------------------------------------------
  weightedSum = 0;
  sum = 0;
} 

void calibrate() {

  for (int i = 0; i < IR; i++) {
    sensor[i] = analogRead(sensorPin[i]);
    sensorMin[i] = sensor[i];
    sensorMax[i] = sensor[i];
  }
  
  for(int i = 0; i < 7000; i++) {
    for (int j = 0; j < IR; j++){
      sensor[j] = analogRead(sensorPin[j]);
      if ( sensor[j] < sensorMin[j] ) sensorMin[j] = sensor[j];
      if ( sensor[j] > sensorMax[j] ) sensorMax[j] = sensor[j];
  } }
}

void getSensorVal() {

  for (int i = 0; i < IR; i++) {
    sensorRaw[i] = analogRead(sensorPin[i]);
    if (b_line){
      sensor[i] = constrain( map( analogRead( sensorPin[i] ), sensorMin[i], sensorMax[i], KS, 0 ), 0, KS);
    } else {
      sensor[i] = constrain( map( analogRead( sensorPin[i] ), sensorMin[i], sensorMax[i], 0, KS ), 0, KS);
    }
    
    weightedSum += sensor[i] * (i-2);
    sum += sensor[i];
    if (sensor[i] <= SENS) lineDetected = true;
  }
  

  if ( lineDetected ) posX = (weightedSum / sum)*5;
  else if ( posX < 0) posX = 1;
  else posX = -1;

  if(abs(posX - PrevErr) > SWITCHING_THRESHOLD){
    b_line = !b_line;
  }

}

void calculatePID() {
  PID = KPID * (( KP * posX ) + ( KD * (posX - PrevErr)) + ( KI * (posX + PrevErr)));
  SpeedA = constrain( SpeedA - PID, -MAXSPEED, MAXSPEED );
  SpeedB = constrain( SpeedB + PID, -MAXSPEED, MAXSPEED );
  PrevErr = posX;
}

void motorDrive() {

  if (SpeedA < 0) {
    digitalWrite(AIN1, LOW);
    analogWrite(AIN2, abs(SpeedA));
  } else {
    digitalWrite(AIN1, HIGH);
    analogWrite(AIN2, 255 - abs(SpeedA));
  }

  if (SpeedB < 0) {
    digitalWrite(BIN1, LOW);
    analogWrite(BIN2, abs(SpeedB));
  } else {
    digitalWrite(BIN1, HIGH);
    analogWrite(BIN2, 255 - abs(SpeedB));
  }
}

void motorStop(void) {
  SpeedA = 0; SpeedB = 0;
  motorDrive();
  SpeedA = FWDSPEED;
  SpeedB = FWDSPEED;
}

void test_suite(){  

  if (counter == 0) {  
    sprintf(data, "Sensors:\t%d  %d  %d  %d  %d | PosX\tWeightedSum\tSum", sensor[0], sensor[1], sensor[2], sensor[3], sensor[4]);
    sprintf(sensor_position, "RawSensors:\t%d  %d  %d  %d  %d | %d\t    %d\t     %d ", sensorRaw[0], sensorRaw[1], sensorRaw[2], sensorRaw[3], sensorRaw[4], posX , weightedSum, sum);
    Serial.println("__________________________________________________________________");
    Serial.println(data);
    Serial.println(sensor_position);
    Serial.println("__________________________________________________________________");
    Serial.println();
  }

  counter++;
}
