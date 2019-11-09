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
const int proximity = 4;

int comp;
int forwardTarg;
int tempTarg;

int lastSeenBallTime = 0;
int lastSeenBallPosition; //0 - left, 1 - right, 2 - straight

int p0 = 9; //p0, p1 from camera -> digitals pins 5, 4
int p1 = 10;
int initPin = 11; //output digital pin 6 -> p3
int proxValuePin = 12;
int seeBallPin = 6;
int diagPin = 5;

int p0Value;
int p1Value;
int initPinValue;
int seeBallPinValue;
int diagPinValue;

bool isInitialized = false; //checks whether initial values of heading + goals are set

const byte interruptPinL = 2;
const byte interruptPinR = 3;
volatile boolean leftlight = false;
volatile boolean rightlight = false;

int getHeading() { //obtains current heading from imu
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

bool initialize() {
  forwardTarg = getHeading(); //setting initial orientation as target heading
  tempTarg = forwardTarg;
  Serial.println(forwardTarg);
  digitalWrite(initPin, HIGH); //sets p3 high, initializing the camera
  return true;
}

void setDribbler(bool state){
  if(state){
    digitalWrite(13, HIGH);
  }
  else{
    digitalWrite(13, LOW);
  }
}

void go(int degree, int speed, int targ)
{
  Serial.println(degree);
  static float pi = 3.1415;
  float rad = degree / (180 / pi);
  float frpwr;
  float flpwr;
  float brpwr;
  float blpwr;
  float err = 0;
  static float kp = 0.3;
  float adjP = 0;
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

  if (degree == 0 || degree == 180)
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

void goToBall(int p0Value, int p1Value) {
  Serial.println("Going To Ball");
  Serial.print(p0Value);
  Serial.println(p1Value);
  if (p0Value == 1 && p1Value == 0) {
    Serial.println("Going Left");
    go(180, 200, tempTarg);
    lastSeenBallPosition = 0;
  }
  else if (p0Value == 0 && p1Value == 1) {
    go(0, 200, tempTarg);
    Serial.println("Going Right");
    lastSeenBallPosition = 1;
  }
  else if (p0Value == 1 && p1Value == 1) {
    go(90, 200, tempTarg);
    Serial.println("Going Straight");
    lastSeenBallPosition = 2;
  }
  lastSeenBallTime = millis();
}

void findBall() {
  if ((millis() - lastSeenBallTime) < 300) {
    switch (lastSeenBallPosition) {
      case 0:
        go(180, 250, tempTarg);
        break;
      case 1:
        go(0, 250, tempTarg);
        break;
      case 2:
        go(90, 250, tempTarg);
        break;
    }
  }
  else {
    rotate(60);
  }
}

void setHeading() {
  if ((tempTarg - getHeading()) > 2 || (tempTarg - getHeading() < -2)) {
    tempTarg = getHeading();
  }
  else {
    tempTarg = tempTarg;
  }
}

void goToGoal() {
  delay(500);
  while (digitalRead(seeBallPin) == 0 && digitalRead(proximity) == 0)
  {
    if ((getHeading() - forwardTarg) > 2)
    {
      frmotor->run(FORWARD);
      flmotor->run(FORWARD);
      brmotor->run(FORWARD);
      blmotor->run(FORWARD);

      frmotor->setSpeed(150);
      brmotor->setSpeed(150);
      flmotor->setSpeed(150);
      blmotor->setSpeed(150);
    }
    else if ((getHeading() - forwardTarg) < -2) {
      frmotor->run(BACKWARD);
      flmotor->run(BACKWARD);
      brmotor->run(BACKWARD);
      blmotor->run(BACKWARD);

      frmotor->setSpeed(150);
      brmotor->setSpeed(150);
      flmotor->setSpeed(150);
      blmotor->setSpeed(150);
    }
    else {
      if (digitalRead(diagPin) == 1) {
        p0Value = digitalRead(p0);
        p1Value = digitalRead(p1);
        if (p0Value == 1 && p1Value == 0) {
          Serial.println("Going Left Diag");
          go(45, 75, forwardTarg);
        }
        else if (p0Value == 0 && p1Value == 1) {
          go(135, 75, forwardTarg);
          Serial.println("Going Right");
        }
        else if (p0Value == 1 && p1Value == 1) {
          go(90, 75, forwardTarg);
          Serial.println("Going Straight");
        }
      }
      else {
        if (digitalRead(p0) == 1 && digitalRead(p1) == 1)
        {
          kick();
        }
        else {
          goToBall(digitalRead(p0), digitalRead(p1));
        }
      }
    }
  }
}

void kick() {
  setDribbler(false);
  Serial.println("Kicking");
  digitalWrite(8, HIGH);
  delay(100);
  digitalWrite(8, LOW);
  delay(500);
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
  pinMode(initPin, OUTPUT);
  pinMode(proxValuePin, OUTPUT);
  pinMode(seeBallPin, INPUT);
  pinMode(diagPin, INPUT);
  pinMode(13, OUTPUT);
  pinMode(interruptPinL, INPUT);
  pinMode(interruptPinR, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinL), lineL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinR), lineR, CHANGE);
  setDribbler(true);
}

void loop() {
  int x = 0;
  Serial.println(digitalRead(proximity));
  if (!isInitialized) {
    while (digitalRead(proximity) == 1) {
    }
    isInitialized = initialize();
  }
  else
  {
    Serial.print("Proximity, SeeBall: ");
    Serial.print(digitalRead(proximity)); //Reads input from proximity sensor
    Serial.println(digitalRead(seeBallPin)); //Reads input from Camera, if high, sees ball, if low, doesnt see ball
    if (digitalRead(proximity) == 1) { //if proximity sensor doesn't sense anything
      digitalWrite(proxValuePin, LOW); //tell camera value of prox sensor
      if (digitalRead(seeBallPin) == 1) { //if camera sees ball
        setHeading();
        while (digitalRead(seeBallPin) == 1) {
          Serial.print("X: ");
          Serial.println(x);
          goToBall(digitalRead(p0), digitalRead(p1));
        }
        x++;
      }
      else if (digitalRead(seeBallPin) == 0) { //if camera doesn't see ball
        Serial.println("Don't See Ball");
        findBall();
      }
    }
    else {
      digitalWrite(proxValuePin, HIGH);
      if (digitalRead(seeBallPin) == 1) {
        Serial.println("Going to Ball");
        setHeading();
        goToBall(digitalRead(p0), digitalRead(p1));
      }
      else {
        goToGoal();
      }
    }
  }
}





