#include <Adafruit_MotorShield_MOD_CB.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);


Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *frmotor = AFMS.getMotor(3);
Adafruit_DCMotor *flmotor = AFMS.getMotor(2);
Adafruit_DCMotor *brmotor = AFMS.getMotor(1);
Adafruit_DCMotor *blmotor = AFMS.getMotor(4);

float frpwr;
float flpwr;
float brpwr;
float blpwr;
const int proximity = 3;

int comp;
int targ;

int p0 = 5; //p0, p1 from camera -> digitals pins 5, 4
int p1 = 4; 
int p2 = 6; //output digital pin 6 -> p3
int p0Value;
int p1Value;
int p2Value;
bool isInitialized = false; //checks whether initial values of heading + goals are set

int getHeading() { //obtains current heading from imu
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

bool initialize() {
  targ = getHeading(); //setting initial orientation as target heading
  Serial.println(targ);
  digitalWrite(p2, HIGH); //sets p3 high, initializing the camera
  return true;
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
  static float kp = 0.38;
  float adjP;
  int curr;
  adjP = -1 * (err * kp);

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

  flpwr = (sin(rad) * speed) * (sqrt(2) / 2) - ((cos(rad) * speed) / 2) + adjP; // front-left
  frpwr  = (-(sin(rad) * speed) * (sqrt(2) / 2)) - (cos(rad) * speed) / 2 + adjP; // front-right
  blpwr = (sin(rad) * speed) * (sqrt(2) / 2) - (-(cos(rad) * speed) / 2) + adjP; // front-left
  brpwr  = (-(sin(rad) * speed) * (sqrt(2) / 2)) - (-(cos(rad) * speed) / 2) + adjP;
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

void rotate(int pwr) {
  frmotor->run(FORWARD);
  flmotor->run(FORWARD);
  brmotor->run(FORWARD);
  blmotor->run(FORWARD);

  frmotor->setSpeed(pwr);
  brmotor->setSpeed(pwr);
  flmotor->setSpeed(pwr);
  blmotor->setSpeed(pwr);
}

void setup() {
  Serial.begin(9600);
  Serial.println("STARTING");
  AFMS.begin();
  Serial.println("MOTOR SETUP SUCCESSFUL");
  //pinMode(button, INPUT);
  pinMode(proximity, INPUT);
  while (!Serial) {
  }
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  pinMode(p0, INPUT);
  pinMode(p1, INPUT);
  pinMode(p2, OUTPUT);
}


void loop() {
  if (!isInitialized) {
    while (digitalRead(proximity) == 1) {
    }
    isInitialized = initialize();
  }
  else
  {
    p0Value = digitalRead(p0);
    p1Value = digitalRead(p1);
    Serial.print(p0Value);
    Serial.print(p1Value);
    while (p0Value == 1 || p1Value == 1)
    {
      p0Value = digitalRead(p0);
      p1Value = digitalRead(p1);
      if (p0Value == 1 && p1Value == 0)
      {
        Serial.println("Going Left");
        go(0, 200);
      }
      else if (p0Value == 0 && p1Value == 1)
      {
        go(180, 200);
        Serial.println("Going Right");
      }
      else if (p0Value == 1 && p1Value == 1)
      {
        go(90, 100);
        Serial.println("Going Straight");
      }
    }
    if (digitalRead(proximity) == 0)
    {
      go(90, 100);
      Serial.println("Going Straight w p");
    }
    else
    {
      rotate(150);
      Serial.print(p0Value);
      Serial.print(p1Value);
      Serial.println("Rotate");
    }
  }
}


