// Include libraries for RF control
#include <SPI.h>
#include <RF24.h>

// Include libraries for line array
#include <QTRSensors.h>
#include "Wire.h"

// Include libraries for Adafruit TCS34725
#include "Adafruit_TCS34725.h"

// Include libraries Servo
#include <Servo.h>
int pos = 0;  // variable to store the servo position
int servoU;
int servoD;

//Pin initialisation for colour sensor
#define redpin 3
#define greenpin 5
#define bluepin 6

// set to false if using a common cathode LED
#define commonAnode true

//RGB -> eye-recognized gamma color
byte gammatable[256];

//Initialise Adafruit TCS34725 RGB Color Sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

RF24 radio(7, 8);  // CE, CSN
const byte address[6] = "00001";



//---MOTOR---//
// Motor 1 control pins
#define MOTOR_R1 2  //pwm 480hZ
#define MOTOR_R2 3  //pwm 480hZ
// Motor 2 control pins
#define MOTOR_L1 5   //pwm 480hZ
#define MOTOR_L2 10  //pwm 480hZ

#define neg1 35
#define pos1 31

#define neg2 45
#define pos2 41
//flag for motor
int lefttrn = 0;
int righttrn = 0;
int home = 0;
int speed = 200;

Servo myservo;  // create servo object to control a servo

QTRSensors qtr;
int n = 8;
int r = 0;


const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int sensorVal[8];

 int array[] = { sensorVal[0], sensorVal[1], sensorVal[2], sensorVal[3], sensorVal[4], sensorVal[5], sensorVal[6], sensorVal[7] };
  int fwd[] = { 0, 0, 0, 1, 1, 0, 0, 0 };
  //////////////// L3 L2 L1 L  R  R1 R2 R3//////////////////
  int rgdlft[] = { 1, 1, 1, 1, 1, 0, 0, 0 };  //rigid left turn
  int lft0[] = { 1, 0, 0, 0, 0, 0, 0, 0 };
  int lft1[] = { 1, 1, 0, 0, 0, 0, 0, 0 };
  int lft2[] = { 0, 1, 1, 0, 0, 0, 0, 0 };
  int lft3[] = { 0, 0, 1, 1, 0, 0, 0, 0 };
  int lft4[] = { 0, 0, 1, 1, 1, 0, 0, 0 };
  int lft5[] = { 0, 1, 1, 1, 0, 0, 0, 0 };
  int lft6[] = { 1, 1, 1, 0, 0, 0, 0, 0 };

  //////////////// L3 L2 L1 L  R  R1 R2 R3//////////////////
  int rgdrgt[] = { 0, 0, 0, 1, 1, 1, 1, 1 };  //rigid right turn
  int rgt0[] = { 0, 0, 0, 0, 0, 0, 0, 1 };
  int rgt1[] = { 0, 0, 0, 0, 0, 0, 1, 1 };
  int rgt2[] = { 0, 0, 0, 0, 0, 1, 1, 0 };
  int rgt3[] = { 0, 0, 0, 0, 1, 1, 0, 0 };
  int rgt4[] = { 0, 0, 0, 1, 1, 1, 0, 0 };
  int rgt5[] = { 0, 0, 0, 0, 1, 1, 1, 0 };
  int rgt6[] = { 0, 0, 0, 0, 0, 1, 1, 1 };

  int blank[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  int ref[] = { 1, 1, 1, 1, 1, 1, 1, 1 };

  // pickup points
  int X1[] = { 0, 1, 0, 1, 1, 0, 1, 0 };
  int X2[] = { 0, 1, 1, 0, 0, 1, 1, 0 };
  //drop points
  int Y1[] = { 0, 1, 1, 0, 0, 1, 1, 0 };
  int Y2[] = { 0, 1, 0, 1, 0, 1, 1, 0 };
  // initialising pick and drop array
  int pick[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  int drop[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

//Function call to compare array elements efficiently
bool ArraysMatch(int a[], int b[], int size) {
  for (int i = 0; i < size; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}
void rightturn() {
  while (1) {
    analogWrite(MOTOR_L1, 170);
    analogWrite(MOTOR_L2, 0);
    analogWrite(MOTOR_R1, 170);
    analogWrite(MOTOR_R2, 0);
    delay(350);

    analogWrite(MOTOR_L1, 180);
    analogWrite(MOTOR_L2, 0);
    analogWrite(MOTOR_R1, 0);
    analogWrite(MOTOR_R2, 180);
    delay(500);
    break;
  }
}
void leftturn() {
  while (1) {
    analogWrite(MOTOR_L1, 170);
    analogWrite(MOTOR_L2, 0);
    analogWrite(MOTOR_R1, 170);
    analogWrite(MOTOR_R2, 0);
    delay(350);

    analogWrite(MOTOR_L1, 0);
    analogWrite(MOTOR_L2, 180);
    analogWrite(MOTOR_R1, 180);
    analogWrite(MOTOR_R2, 0);
    delay(500);
    break;
  }
}

void pick_Returnseq() {
  analogWrite(MOTOR_L1, 0);
  analogWrite(MOTOR_L2, 190);
  analogWrite(MOTOR_R1, 190);
  analogWrite(MOTOR_R2, 0);
  delay(1700);
}

void drop_Returnseq() {
  analogWrite(MOTOR_L1, 0);
  analogWrite(MOTOR_L2, 180);
  analogWrite(MOTOR_R1, 0);
  analogWrite(MOTOR_R2, 180);
  delay(900);
  analogWrite(MOTOR_L1, 0);
  analogWrite(MOTOR_L2, 190);
  analogWrite(MOTOR_R1, 190);
  analogWrite(MOTOR_R2, 0);
  delay(1700);
}
void pickfwd() {
  analogWrite(MOTOR_L1, 170);
  analogWrite(MOTOR_L2, 0);
  analogWrite(MOTOR_R1, 170);
  analogWrite(MOTOR_R2, 0);
  delay(50);
}

void motorreset() {
  digitalWrite(MOTOR_L1, 0);
  digitalWrite(MOTOR_L2, 0);
  digitalWrite(MOTOR_R1, 0);
  digitalWrite(MOTOR_R2, 0);
}

void setup()

{
  Serial.begin(9600);
  myservo.attach(9);
  myservo.write(0);
  //setting pinmode for motors and LED's
  pinMode(MOTOR_L1, OUTPUT);
  pinMode(MOTOR_L2, OUTPUT);
  pinMode(MOTOR_R1, OUTPUT);
  pinMode(MOTOR_R2, OUTPUT);

  pinMode(neg1, OUTPUT);
  pinMode(pos1, OUTPUT);

  pinMode(neg2, OUTPUT);
  pinMode(pos2, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){
                      36, 34, 32, 30, 28, 26, 24, 22 },
                    SensorCount);
  //qtr.setSensorPins((const uint8_t[]){32, 30, 28, 26}, SensorCount);
  qtr.setEmitterPin(38);
  //delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);  // turn off Arduino's LED to indicate we are through with calibration
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;  // halt!
  }

#if defined(ARDUINO_ARCH_ESP32)
  ledcAttach(redpin, 12000, 8);
  ledcAttach(greenpin, 12000, 8);
  ledcAttach(bluepin, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif

  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
  }
  //Radio Receive initialisaiton
  radio.begin();
  Serial.println("checking if chip connected");
  bool check = radio.isChipConnected();
  Serial.print("check-");
  Serial.println(check);

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

}
void loop() {


  // Receiving iputs from nrf module
  if (radio.available()) {
    int message[4];
    radio.read(&message, sizeof(message));
    int i;
    for (i = 0; i < 4; i = i + 1) {
      Serial.print(message[i]);
    }
    Serial.println();
    int sum = message[0] + message[1] + message[2] + message[3];
    if (sum == 6) {
      //stop
      Serial.println("STOP");
    }
    if (sum == 3) {
      if (message[0] == 0 && message[1] == 1 && message[2] == 1 && message[3] == 1) {
        //Pickup location 1
        Serial.println("Pickup: X1");
        for (i = 0; i < 8; i++) {
          pick[i] = X1[i];
          Serial.print(pick[i]);
          Serial.print('\t');
        }
      }
      if (message[0] == 1 && message[1] == 0 && message[2] == 1 && message[3] == 1) {
        //Pickup location 2
        Serial.println("Pickup: X2");
        for (i = 0; i < 8; i++) {
          pick[i] = X2[i];
          Serial.print(pick[i]);
          Serial.print('\t');
        }
      }
      if (message[0] == 1 && message[1] == 1 && message[2] == 0 && message[3] == 1) {
        //Drop location 1
        Serial.println("Drop: Y1");

        Serial.print('\t');
        for (i = 0; i < 8; i++) {
          drop[i] = Y1[i];
          Serial.print(drop[i]);
          Serial.print('\t');
        }
      }
      if (message[0] == 1 && message[1] == 1 && message[2] == 1 && message[3] == 0) {
        //Drop location 2
        Serial.print("drop: Y2");
        for (i = 0; i < 8; i++) {
          drop[i] = Y2[i];
          Serial.print(drop[i]);
          Serial.print('\t');
        }
      }
    }
  }
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED
  delay(60);                // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED

  int R = Serial.print("R:\t");
  Serial.print(int(red));
  int G = Serial.print("\tG:\t");
  Serial.print(int(green));
  int B = Serial.print("\tB:\t");
  Serial.print(int(blue));

  Serial.print("\n");
#if defined(ARDUINO_ARCH_ESP32)
  ledcWrite(1, gammatable[(int)red]);
  ledcWrite(2, gammatable[(int)green]);
  ledcWrite(3, gammatable[(int)blue]);
#else
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);
#endif

 // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  //displays '0' for white
  if (sensorValues[i]<qtr.calibrationOn.minimum[i]+200)
  {
    sensorVal[i]=0;
    Serial.print(0);
    Serial.print('\t');
  
  }
  ////displays '1' for black
  else
  {
    sensorVal[i]=1;
    Serial.print(1);
    Serial.print('\t');
  }

  //Go straight
  if (ArraysMatch(array, fwd, n)) {
    analogWrite(MOTOR_L1, 160);
    analogWrite(MOTOR_L2, 0);
    analogWrite(MOTOR_R1, 160);
    analogWrite(MOTOR_R2, 0);
  }
  //left correction
  else if (ArraysMatch(array, lft0, n) || ArraysMatch(array, lft1, n) || ArraysMatch(array, lft2, n) || ArraysMatch(array, lft3, n) || ArraysMatch(array, lft4, n) || ArraysMatch(array, lft5, n) || ArraysMatch(array, lft6, n)) {

    analogWrite(MOTOR_L1, 0);
    analogWrite(MOTOR_L2, 0);
    analogWrite(MOTOR_R1, 160);
    analogWrite(MOTOR_R2, 0);
  }
  //right correction
  else if (ArraysMatch(array, rgt0, n) || ArraysMatch(array, rgt1, n) || ArraysMatch(array, rgt2, n) || ArraysMatch(array, rgt3, n) || ArraysMatch(array, rgt4, n) || ArraysMatch(array, rgt5, n) || ArraysMatch(array, rgt6, n)) {

    analogWrite(MOTOR_L1, 160);
    analogWrite(MOTOR_L2, 0);
    analogWrite(MOTOR_R1, 0);
    analogWrite(MOTOR_R2, 0);
  }
  //Turn right
  else if (ArraysMatch(array, rgdrgt, n)) {
    rightturn();
    righttrn = 1;
    lefttrn = 0;
  }
  //Turn Left
  else if (ArraysMatch(array, rgdlft, n)) {
    leftturn();
    lefttrn = 1;
    righttrn = 0;
  }
  // Move forward and stop for sometime after reaching destination
  else if (ArraysMatch(array, ref, n)) {
    while (1) {

      analogWrite(MOTOR_L1, 100);
      analogWrite(MOTOR_L2, 0);
      analogWrite(MOTOR_R1, 100);
      analogWrite(MOTOR_R2, 0);
      delay(90);
      analogWrite(MOTOR_L1, 0);
      analogWrite(MOTOR_L2, 0);
      analogWrite(MOTOR_R1, 0);
      analogWrite(MOTOR_R2, 0);
      break;
    }
    r = +1;
  }
  // Conditions to be executed after reaching pick or drop point
  else if (r == 1) {
    //checking sensor readings matching with the given pick location
    if (ArraysMatch(array, pick, n) && ((R > 85 && R < 80) && (G > 110 && G < 130) && (B > 45 && B < 55)) && servoU == 0) {
      //Turn ON Green LED
      digitalWrite(pos2, HIGH);
      digitalWrite(neg2, LOW);
      //Pickup sequence
      for (pos = 0; pos <= 80; pos += 1) {
        myservo.write(pos);  // tell servo to go to position in variable 'pos'
        delay(15);
      }
      servoU = 1;
      pick_Returnseq();
      digitalWrite(pos2, LOW);
      digitalWrite(neg2, LOW);
    }
    //Take rightturn if previous turn is right
    else if (ArraysMatch(array, blank, n) && righttrn == 1) {
      rightturn();
      righttrn = 0;
    }
    //Take lefttturn if previous turn is left
    else if (ArraysMatch(array, blank, n) && lefttrn == 1) {
      leftturn();
      lefttrn = 0;

    }
    //checking sensor readings matching with the given drop location
    else if (ArraysMatch(array, drop, n) && servoU == 1) {
      digitalWrite(pos1, HIGH);
      digitalWrite(neg1, LOW);
      //delay(1000);
      for (pos = 80; pos >= 0; pos -= 1) {
        myservo.write(pos);  // tell servo to go to position in variable 'pos'
        delay(15);
      }
      //break;
      servoU = 0;
      drop_Returnseq();
      home = 1;
      digitalWrite(MOTOR_L1, LOW);
      digitalWrite(MOTOR_L2, LOW);
      digitalWrite(MOTOR_R1, LOW);
      digitalWrite(MOTOR_R2, LOW);
      digitalWrite(pos1, LOW);
      digitalWrite(neg1, LOW);
    }
    //   No drop or pick location take return sequence executed for drop
    else {
      drop_Returnseq();
    }
    r = 0;
  }

  else if (ArraysMatch(array, blank, n)) {
    digitalWrite(MOTOR_L1, LOW);
    digitalWrite(MOTOR_L2, LOW);
    digitalWrite(MOTOR_R1, LOW);
    digitalWrite(MOTOR_R2, LOW);
  }
}
