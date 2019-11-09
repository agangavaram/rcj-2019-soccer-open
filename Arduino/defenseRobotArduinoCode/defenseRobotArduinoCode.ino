#include <Adafruit_MotorShield.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
int comp;
int targ;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *frmotor = AFMS.getMotor(3);
Adafruit_DCMotor *flmotor = AFMS.getMotor(2);
Adafruit_DCMotor *brmotor = AFMS.getMotor(1);
Adafruit_DCMotor *blmotor = AFMS.getMotor(4);

const byte interruptPinL = 2;
const byte interruptPinR = 3;
volatile boolean leftlight = false;
volatile boolean rightlight = false;

float frpwr;
float flpwr;
float brpwr;
float blpwr;

const int buttonPin = 10;
const int initPin = 11;

bool isInitialized = false;


void lineL()
{
  Serial.println("L");
  leftlight = true;
}

void lineR()
{
  Serial.println("R");
  rightlight = true;
}

int getHeading(){
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

bool initialize(){
  targ = getHeading();
  digitalWrite(initPin, HIGH);
  Serial.println("Initialized");
  return true;
}

void go(int degree, int spd){
  Serial.println(degree);
  float rad = degree / (180 / PI);
  
  float rad = degree / (180 / pi);
  float frpwr;
  float flpwr;
  float brpwr;
  float blpwr;
  float err = 0;
  static float kp = 0.3;
  float adjP;
  int curr;

  curr = getHeading();
  if ((targ - curr) > 180)
    err = ((targ - curr) - 360) * -1;
  else if ((targ - curr) < -180)
    err = ((targ - curr) + 360);
  else
    err = targ - curr;

  if (-2 < err && err < 2)
    err = 0;
  else
    err = err;
  adjP = -1 * (err * kp);

if(degree == 0 || degree == 180)
{
    flpwr = (sin(rad) * speed) * (sqrt(2) / 2) - ((cos(rad) * speed) / 2) + adjP; // front-left
    frpwr  = (-(sin(rad) * speed) * (sqrt(2) / 2)) - (cos(rad) * speed) / 2 + adjP; // front-right
    blpwr = (sin(rad) * speed) * (sqrt(2) / 2) - (-(cos(rad) * speed) / 2) + adjP; // front-left
    brpwr  = (-(sin(rad) * speed) * (sqrt(2) / 2)) - (-(cos(rad) * speed) / 2) + adjP;
}
else
{
    flpwr = (sin(rad) * speed) * (sqrt(2) / 2) - ((cos(rad) * speed) / 2) - adjP; // front-left
    frpwr  = (-(sin(rad) * speed) * (sqrt(2) / 2)) - (cos(rad) * speed) / 2 - adjP; // front-right
    blpwr = (sin(rad) * speed) * (sqrt(2) / 2) - (-(cos(rad) * speed) / 2) - adjP; // front-left
    brpwr  = (-(sin(rad) * speed) * (sqrt(2) / 2)) - (-(cos(rad) * speed) / 2) - adjP;
}
 
  if (frpwr < 0)
    frpwr = frpwr * -1;
  if (brpwr < 0)
    brpwr = brpwr * -1;
  if (flpwr < 0)
    flpwr = flpwr * -1;
  if (blpwr < 0)
    blpwr = blpwr * -1;

  Serial.print("FLPWR: ");
  Serial.println(flpwr);
  Serial.print("FRPWR: ");
  Serial.println(frpwr);
  Serial.print("BLPWR: ");
  Serial.println(blpwr);
  Serial.print("BRPWR: ");
  Serial.println(brpwr);
  Serial.print("current= ");
  Serial.println(curr);
  Serial.print("target= ");
  Serial.println(targ);
  Serial.print("error= ");
  Serial.println(err);
  Serial.println("-----------------------------");

  switch (degree)
  {
    case 0:
      frmotor->run(FORWARD);
      flmotor->run(FORWARD);
      brmotor->run(BACKWARD);
      blmotor->run(BACKWARD);
      break;
    case 45:
      frmotor->run(RELEASE);
      flmotor->run(FORWARD);
      brmotor->run(BACKWARD);
      blmotor->run(RELEASE);
      break;
    case 90:
      frmotor->run(BACKWARD);
      flmotor->run(FORWARD);
      brmotor->run(BACKWARD);
      blmotor->run(FORWARD);
      break;
    case 135:
      frmotor->run(BACKWARD);
      flmotor->run(RELEASE);
      brmotor->run(RELEASE);
      blmotor->run(FORWARD);
      break;
    case 180:
      frmotor->run(BACKWARD);
      flmotor->run(BACKWARD);
      brmotor->run(FORWARD);
      blmotor->run(FORWARD);
      break;
    case 225:
      frmotor->run(RELEASE);
      flmotor->run(BACKWARD);
      brmotor->run(FORWARD);
      blmotor->run(RELEASE);
      break;
    case 270:
      frmotor->run(FORWARD);
      flmotor->run(BACKWARD);
      brmotor->run(FORWARD);
      blmotor->run(BACKWARD);
      break;
    case 315:
      frmotor->run(FORWARD);
      flmotor->run(RELEASE);
      brmotor->run(RELEASE);
      blmotor->run(BACKWARD);
      break;
  }
  frmotor->setSpeed(frpwr);
  brmotor->setSpeed(brpwr);
  flmotor->setSpeed(flpwr);
  blmotor->setSpeed(blpwr);
}

void setup() {
  Serial.begin(9600);
  Serial.println("STARTING");
  AFMS.begin();
  Serial.println("MOTOR SETUP SUCCESSFUL");
  while (!Serial) {
  }
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  pinMode(buttonPin, INPUT);
}

void loop() {
  if(!isInitialized){
    while(digitalRead(buttonPin) == 0){
      Serial.println("Waiting for Button Press");
      getHeading();
      delay(100);
    }
    isInitialized = initialize();
  }
  else{
    if(digitalRead() == ){
      
    }
  } 
}
