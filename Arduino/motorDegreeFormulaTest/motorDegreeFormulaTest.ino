#include <Adafruit_MotorShield.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
const int button = 2;
int buttonState = 0;
int comp;
int targ;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *frmotor = AFMS.getMotor(3);
Adafruit_DCMotor *flmotor = AFMS.getMotor(2);
Adafruit_DCMotor *brmotor = AFMS.getMotor(1);
Adafruit_DCMotor *blmotor = AFMS.getMotor(4);

float frpwr;
float flpwr;
float brpwr;
float blpwr;

int getHeading(){
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}
void go(float degree, int speed)
{
  Serial.println(degree);
  static float pi = 3.1415;
  float rad = degree / (180 / pi);
  float frpwr;
  float flpwr;
  float brpwr;
  float blpwr;
  float err = 0;
  static float kp = .8;
  float adjP;
  int curr;
  

  curr = getHeading();
  if ((targ - curr) > 180)
    err = ((targ - curr) - 360) * -1;
  else if ((targ - curr) < -180)
    err = ((targ - curr) + 360);
  else
    err = targ - curr;

  if (-5 < err && err < 5)
    err = 0;
  else
    err = err;
adjP = -1 * (err * kp);
Serial.println(err);
Serial.println(adjP);
  flpwr = (sin(rad) * speed) * (sqrt(2) / 2) - ((cos(rad) * speed) / 2) - adjP; // front-left
  frpwr  = (-(sin(rad) * speed) * (sqrt(2) / 2)) - (cos(rad) * speed) / 2 - adjP; // front-right
  blpwr = (sin(rad) * speed) * (sqrt(2) / 2) - (-(cos(rad) * speed) / 2) - adjP; // front-left
  brpwr  = (-(sin(rad) * speed) * (sqrt(2) / 2)) - (-(cos(rad) * speed) / 2) - adjP;
  Serial.print("FLPWR: ");
  Serial.println(flpwr);
  Serial.print("FRPWR: ");
  Serial.println(frpwr);
  Serial.print("BLPWR: ");
  Serial.println(blpwr);
  Serial.print("BRPWR: ");
  Serial.println(brpwr);
  Serial.println("-----------------------------");

  if (frpwr > 0)
    frmotor ->run(FORWARD);
  else
  {
    frmotor -> run(BACKWARD);
    frpwr = frpwr * -1;
  }

  if (brpwr > 0)
    brmotor ->run(FORWARD);
  else
  {
    brmotor -> run(BACKWARD);
    brpwr = brpwr * -1;
  }

  if (flpwr > 0)
    flmotor ->run(FORWARD);
  else {
    flmotor -> run(BACKWARD);
    flpwr = flpwr * -1;
  }


  if (blpwr > 0)
    blmotor ->run(FORWARD);
  else {
    blmotor -> run(BACKWARD);
    blpwr = blpwr * -1;
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

  while(!Serial){
  }
  if(!bno.begin())
  {    
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);   
  targ = getHeading();
}

void loop() {
  go(90, 150);
}
