#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

SoftwareSerial gsmSerial(3,4);
SoftwareSerial gpsSerial(5,6);
TinyGPS gps;

#define INTERRUPT_PIN 2
#define WHITE 13
#define RED 12
#define BUZZER 11

#define ACC_X_LIMIT 2000
#define ACC_Y_LIMIT 2000
#define YAW_LIMIT 60
#define YAW_LIMIT_MAX 180
#define PITCH_LIMIT 40
#define ROLL_LIMIT 60

MPU6050 mpu;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
float lat = 28.5458,lon = 77.1703;  // create variable for latitude and longitude object
String apn = "TELETALK INTERNET";                       //APN
String url = "http://knocktodo.com/theme/add.php";  //URL for HTTP-POST-REQUEST
int acc_x;    //x axis acceleration
int acc_y;    //y axis acceleration
int yaw;      //yaw orientation
int pitch;    //pitch orientation
int roll;     //roll orientation
int yaw_pre_value;
int pitch_pre_value;
int roll_pre_value;
int acc_x_pre_value = 0;
int acc_y_pre_value = 0;
int yaw_diff = 0;
int pitch_diff = 0;
int roll_diff = 0;
int acc_x_diff = 0;
int acc_y_diff = 0;
bool is_accident = false;
String latitude;
String longitude;
bool active = true;
int value = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);
  gsmSerial.begin(9600);
  while(!Serial);

  pinMode(WHITE, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  Accelerometer_Gyroscope_Setup();
  delay(20000);
  digitalWrite(WHITE, HIGH);
  delay(1000);
  digitalWrite(WHITE, LOW);
  delay(1000);
}

void Accelerometer_Gyroscope_Setup(){
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  Accelerometer_Gyroscope();

  if(is_accident) {
    get_location();
    send_message();
    
    while(1) {
      digitalWrite(RED, LOW);
      Serial.println("Program Terminated");
      delay(1000);
      digitalWrite(RED, HIGH);
      digitalWrite(WHITE, LOW);
      delay(1000);
      digitalWrite(RED, LOW);
      digitalWrite(WHITE, HIGH);
    }
  }
  delay(200);
}

void Accelerometer_Gyroscope() {
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    break;
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw = ypr[0] * 180/M_PI;
    pitch = ypr[1] * 180/M_PI;
    roll = ypr[2] * 180/M_PI;

    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    acc_x = aaWorld.x;
    acc_y = aaWorld.y;

    send_data();
    decision();
  }
}

void send_data() {
  digitalWrite(WHITE, HIGH);
  Serial.print("ypr\t");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(roll);

  Serial.print("aworld\t");
  Serial.print(acc_x);
  Serial.print("\t");
  Serial.println(acc_y);
}

void decision() {
  if(active) {
    //ini_velocity = 0;
    yaw_pre_value = yaw;
    pitch_pre_value = pitch;
    roll_pre_value = roll;
    active = false;
  }
  yaw_diff = yaw_pre_value - yaw;
  pitch_diff = pitch_pre_value - pitch;
  roll_diff = roll_pre_value - roll;
  acc_x_diff = acc_x_pre_value - acc_x;
  acc_y_diff = acc_x_pre_value - acc_y;

  if(yaw_diff < 0) yaw_diff = -1 * yaw_diff;
  if(pitch_diff < 0) pitch_diff = -1 * pitch_diff;
  if(roll_diff < 0) roll_diff = -1 * roll_diff;
  if(acc_x_diff < 0) acc_x_diff = -1*acc_x_diff;
  if(acc_y_diff < 0) acc_y_diff = -1*acc_y_diff;

  unsigned long t = millis();
  Serial.print(yaw_diff);
  Serial.print("\t");
  Serial.print(pitch_diff);
  Serial.print("\t");
  Serial.print(roll_diff);
  Serial.print("\t");
  Serial.print(acc_x_diff);
  Serial.print("\t");
  Serial.println(t);

  /*if(yaw_diff > YAW_LIMIT && yaw_diff < YAW_LIMIT_MAX) {
    is_accident = true;
    digitalWrite(WHITE, LOW);
    digitalWrite(RED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(3000);
    digitalWrite(BUZZER, LOW);
    Serial.println("Yaw");
  }*/

  if(pitch_diff > PITCH_LIMIT) {
    digitalWrite(WHITE, LOW);
    is_accident = true;
    digitalWrite(RED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(3000);
    digitalWrite(BUZZER, LOW);
    Serial.println("Pitch");
  }

  if(roll_diff > ROLL_LIMIT) {
    digitalWrite(WHITE, LOW);
    is_accident = true;
    digitalWrite(RED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(3000);
    digitalWrite(BUZZER, LOW);
    Serial.println("Roll");
  }

  if(acc_x_diff > ACC_X_LIMIT) {
    digitalWrite(WHITE, LOW);
    is_accident = true;
    digitalWrite(RED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(3000);
    digitalWrite(BUZZER, LOW);
    Serial.println("X");
  }

  /*if(acc_y_diff > ACC_Y_LIMIT) {
    digitalWrite(WHITE, LOW);
    is_accident = true;
    digitalWrite(RED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(5000);
    digitalWrite(BUZZER, LOW);
    Serial.println("Y");
  }*/
  
  yaw_pre_value = yaw;
  //pitch_pre_value = pitch;
  //roll_pre_value = roll;
  acc_x_pre_value = acc_x;
  acc_y_pre_value = acc_y;
}

void get_location() {
  if(gps.encode(gpsSerial.read())) { 
    gps.f_get_position(&lat,&lon); // get latitude and longitude 

    latitude = String(lat,6); 
    longitude = String(lon,6);
    Serial.println("Get Location!"); 
    delay(1000);
    }
}

void send_message() {

  String message;
  //String la = String(23.6980117, 7);
  //String lo = String(90.4027336, 7);

  if(gsmSerial.available() > 0) {
    Serial.write(gsmSerial.read());
    message = gsmSerial.read();
  }
    
  gsmSerial.println("AT+CMGF=1");                      //Sets the gsmSerial Module in Text Mode
  delay(1000);

  gsmSerial.println("AT+CMGS=\"+8801521584412\"\r");   //Send message to this number
  delay(1000);

  gsmSerial.println("Your car has been crashed. Location: http://maps.google.com/?q=" + latitude +"," + longitude);

  gsmSerial.println(message);
  delay(100);
  
  gsmSerial.println((char)26);                         // ASCII code of CTRL+Z
  Serial.println("Message Sent!");
  delay(1000);
}

void runsl() {
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
}

void dmpDataReady() {
  mpuInterrupt = true;
}
