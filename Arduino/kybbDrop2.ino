//switch
const float highSet = 0.41;
const float lowSet = 0.208;

//PID constants
const float Pup   = 200;
const float Iup   = 8;
const float Dup   = 0;
const float Pdown = 30;
const float Idown = 2;
const float Ddown = 0;

const float toZeroPwr = 0.39;
const float maxUp = 70;
const float minUp = 50;
const float maxDown = 60;
const float minDown = 18;

int count = 0;
int countLast = 0;
float targ = highSet;
bool stopping = false;
bool last = false;

float maxPwr = maxUp;
float minPwr = minUp;

float kp = Pup;
float ki = Iup;
float kd = Dup;

//-----------------communication--------------------
#include <HardWire.h>
#include <VL53L0X.h>
#include <I2C_MPU6886.h>

#include <Ethernet.h>
#include <EthernetUdp.h>

VL53L0X range_sensor;
I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire);

unsigned long time1 = millis();

IPAddress ip(192, 168, 10, 240);
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

EthernetUDP udp_server;

char packet_buffer[UDP_TX_PACKET_MAX_SIZE];
//-----------------motor controll--------------------
#include <AVR_RTC.h>

//*************************************
// DC Motor PID position control example
// By Øystein Bjelland, IIR, NTNU
// Based on this example: https://curiores.com/dc-motor-control/
//**************************************

#include <util/atomic.h>

//**************************************

#define ENCA 2      //Encoder pinA
#define ENCB 3      //Encoder pinB
#define PWM 11       //motor PWM pin
#define IN2 6       //motor controller pin2
#define IN1 7       //motor controller pin1
#define CHANGESP    // pin for setting of setpoint

volatile int posi = 0; // position variable. https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
unsigned long prevT = 0;
float eprev = 0;
float eintegral = 0;


//---------------------------------------------------
void setup()
{
  //-----------------communication--------------------
  Serial.begin(115200);
  Wire.begin();
  Ethernet.begin(mac, ip);
  delay(500);

  if (!initialize())
    while (true)
      delay(10);

  imu.begin();
  range_sensor.startContinuous();

  udp_server.begin(8888);
  Serial.println("Setup complete");
  //-----------------motor controll--------------------
  // ENCODER
  pinMode (ENCA, INPUT);
  pinMode (ENCB, INPUT);
  //  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING); //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

  // DC MOTOR
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //---------------------------------------------------
}


void loop()
{
  float pos = 0;
  //-----------------communication--------------------
  int packet_size = udp_server.parsePacket();
  if (packet_size)
  {
    float accel[3];
    float gyro[3];
    float t;
    float d;

    imu.getAccel(&accel[0], &accel[1], &accel[2]);
    imu.getGyro(&gyro[0], &gyro[1], &gyro[2]);
    imu.getTemp(&t);
    d = range_sensor.readRangeContinuousMillimeters();


    String sensor_values;
    sensor_values.concat(accel[0]); sensor_values.concat(",");
    sensor_values.concat(accel[1]); sensor_values.concat(",");
    sensor_values.concat(accel[2]); sensor_values.concat(",");
    sensor_values.concat(d);

    udp_server.read(packet_buffer, UDP_TX_PACKET_MAX_SIZE);
    pos = String(packet_buffer).toFloat();
    Serial.print("|");
    //Serial.print("sending: "); Serial.println(sensor_values);
    udp_server.beginPacket(udp_server.remoteIP(), udp_server.remotePort());
    udp_server.write(sensor_values.c_str(), sensor_values.length());
    udp_server.endPacket();
    //}
    if (stopping == false) {
      //-----------------motor controll--------------------

      //time diference
      unsigned long currT = millis();
      float deltaT = ((float) (currT - prevT)) / (1.0e3);
      prevT = currT;

      // int pos = 0;
      // ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      //   pos = posi;
      // }

      //Error calculation
      float e = (pos - targ);
      //Serial.println(e);

      //PID Calculation
      float dedt = (e - eprev) / (deltaT); //Derivative
      eintegral = eintegral + e * deltaT; //Integral
      float u = kp * e + kd * dedt + ki * eintegral; //Control signal

      //Motor power

      float pwr = 0;
      if (u > 0) {
        minPwr = minDown;
        maxPwr = maxDown;
      }
      else {
        minPwr = minUp;
        maxPwr = maxUp;
      }
      if (fabs(u) > toZeroPwr) {
        pwr = fabs(u) + minPwr;  //fabs == floating point, absolute value
      }
      if (pwr > maxPwr) {
        pwr = maxPwr; //Capping
      }

      //Motor direction
      int dir = (u < 0) ? -1 : 1;

      //Send signal to motor
      setMotor(dir, pwr, PWM, IN1, IN2);

      //Store previous error
      eprev = e;

      Serial.print("\tPos: "); Serial.print(pos);
      Serial.print("\te: "); Serial.print(e);
      Serial.print(" \tpwr = "); Serial.print(pwr);
      Serial.print("\tu = "); Serial.print(u);
      Serial.print("\tcount= "); Serial.println(count);
      //---------------------------------------------------
      if (pos > targ - 0.01 && pos < targ + 0.01 && last == false) {
        if (targ == lowSet) {
          targ = highSet;
          kp = Pup;
          ki = Iup;
          kd = Dup;
          maxPwr = maxUp;
          minPwr = minUp;
        }
        else {
          targ = lowSet;
          kp = Pdown;
          ki = Idown;
          kd = Ddown;
          maxPwr = maxDown;
          minPwr = minDown;
          count++;
          if (count >= 5) {
            last = true;
          }
        }

        e = (pos - targ);
        eprev = e;
        eintegral = 0;
        Serial.print("-----------------------------------------------------\n");
      }
      else if (pos > targ - 0.005 && pos < targ + 0.005 && last == true) {
        Serial.print("-");
        countLast++;
        if (countLast > 30) {
          setMotor(dir, 0, PWM, IN1, IN2);
          stopping = true;
        }
      }
      else if (last == true) {
        countLast = 0;
      }
    }
  }
}


//-----------------communication--------------------
bool initialize()
{
  range_sensor.setTimeout(500);
  if (!range_sensor.init())
  {
    Serial.println("Failed to detect and initialize range sensor!");
    return false;
  }

  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet shield was not found.");
    return false;
  }
  else if (Ethernet.hardwareStatus() == EthernetW5500)
    Serial.println("Found W5500 ethernet shield");

  if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("Ethernet::LinkOff: is the cable connected?");
    return false;
  }
  else if (Ethernet.linkStatus() == LinkON)
    Serial.println("Ethernet::LinkOn");
  return true;
}


void printVector3(char label, float *vector)
{
  Serial.print(label); Serial.print(" = ");
  Serial.print(vector[0] > 0.0 ? " " : ""); Serial.print(vector[0]); Serial.print(",    \t");
  Serial.print(vector[1] > 0.0 ? " " : ""); Serial.print(vector[1]); Serial.print(",    \t");
  Serial.print(vector[2] > 0.0 ? " " : ""); Serial.println(vector[2]);
}

void printScalar(char label, float scalar)
{
  Serial.print(label); Serial.print(" = ");
  Serial.print(scalar > 0.0 ? " " : ""); Serial.println(scalar);
}

void printPackageMetaInfo(int packet_size)
{
  Serial.print("Received packet of size ");
  Serial.println(packet_size);
  Serial.print("From ");
  IPAddress remote = udp_server.remoteIP();
  for (int i = 0; i < 4; i++)
  {
    Serial.print(remote[i], DEC);
    if (i < 3)
      Serial.print(".");
  }
  Serial.print(", port ");
  Serial.println(udp_server.remotePort());
}
//-----------------motor controll--------------------
//MOTOR
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
//---------------------------------------------------
