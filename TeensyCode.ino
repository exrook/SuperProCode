#include <SPI.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include <NS73M.h>

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

//const int ledPin = 11;
const short azPin = 23;
const short ayPin = 22;
const short axPin = 21;

const short latchPin = 0;
const short dataPin = 1;
const short clockPin = 2;
const short ninePin = 3;

const short in1 = 4;
const short in2 = 5;

short state_1 = 0;

double gx = 0, gy = 0, gz = 0;
int16_t gz_drift;
double dgx, dgy, dgz;
double ax, ay, az;
double mx, my, mz;
double dt;

double hx,hy;
double heading;

elapsedMicros time;

int lel;

byte i = 0;


void setup() {
  analogWriteRes(10);
  analogWriteFrequency(0,46875);
  analogWriteFrequency(1,46875);
  analogWriteFrequency(2,46875);
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(axPin, OUTPUT);
  pinMode(ayPin, OUTPUT);
  pinMode(azPin, OUTPUT);
  
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(ninePin, OUTPUT);
  
  digitalWrite(13, HIGH);
  analogWrite(axPin, 128);
  delay(1000);
  digitalWrite(13, LOW);
  digitalWrite(axPin, HIGH);
  //radio.setup(88900000);
  uint16_t status = dof.begin();
  Serial.println(status);
  digitalWrite(axPin, LOW);
  gz_drift = dof.calcGyro(dof.gz);
}

void loop() {
  time = 0;
  while (true) {
    double accel = 0;
    switch (state_1) {
      case 0:
        dof.readGyro();
        dof.readAccel();
        dof.readMag();
        //gx += dof.calcGyro(dof.gx)*dt;
        //gy += dof.calcGyro(dof.gy)*dt;
        gz += (dof.calcGyro(dof.gz)-gz_drift)*dt;
        ax = dof.calcAccel(dof.ax);
        ay = dof.calcAccel(dof.ay);
        az = dof.calcAccel(dof.az);
        mx = dof.calcMag(dof.mx);
        my = dof.calcMag(dof.my);
        mz = dof.calcMag(dof.mz);
        accel = sqrt(powf(fabs(ax),2) + powf(fabs(ay),2) + powf(fabs(az),2));
        if (accel > 0.5 && accel < 2.0) {
          dgx = atan2f(ay,az) * (180/M_PI);
          gx = gx * 0.98 + dgx * 0.02;
          dgy = atan2f(ax,az) * (180/M_PI);
          gy = gy * 0.98 + dgy * 0.02;
          //dgz = atan2f(dof.calcAccel(dof.a),dof.calcAccel(dof.a)) * (180/M_PI);
        }
        if (accel > 1.1 || accel < .9) {
          digitalWrite(13, HIGH);
        } else {
          digitalWrite(13, LOW);
        }
        hy = (double)dof.my;
        hx = (double)dof.mx;
        float heading;
        if (hy > 0) {
          heading = 90 - (atan(hx / hy) * (180 / PI));
        } else if (hy < 0) {
          heading = - (atan(hx / hy) * (180 / PI));
        } else {// hy = 0
          if (hx < 0) heading = 180;
          else heading = 0;
        }
        
        Serial.print("Heading: ");
        Serial.println(heading, 2);
        // Write to LEDs
        //analogWrite(axPin, powf(1*fabs(ax),2)*800.0);
        //analogWrite(ayPin, powf(1*fabs(ay),2)*800.0);
        //analogWrite(azPin, powf(1*fabs(az),2)*800.0);
        analogWrite(axPin, fabs(mx)*1024);
        analogWrite(ayPin, fabs(my)*1024);
        analogWrite(azPin, fabs(mz)*1024);
        digitalWrite(latchPin, LOW);
        
        /*if (digitalRead(in1)) {
          if (digitalRead(in2)) {
            lel = (int)(gx+128); //11
          } else {
            lel = (int)(gz+128); //10
          }
        } else {
          if (digitalRead(in2)) {
            lel = (int)(gy+128); //01
          } else {
            lel = (int)(gx+128); //00
          }
        }*/
        lel = (int)(gz+128);
        // 512 = 0b000000000
        if (lel<256) {
          digitalWrite(ninePin, LOW);
        } else {
          digitalWrite(ninePin, HIGH);
        }
        shiftOut(dataPin, clockPin, MSBFIRST, lel);
        digitalWrite(latchPin, HIGH);
        break;
      default:
        break;
    }
    Serial.print("A");
    Serial.print(gx);
    Serial.println("E");
    Serial.print("B");
    Serial.print(gy);
    Serial.println("E");
    Serial.print("C");
    Serial.print(gz);
    Serial.println("E");
    Serial.print("X");
    Serial.print(ax);
    Serial.println("E");
    Serial.print("Y");
    Serial.print(ay);
    Serial.println("E");
    Serial.print("Z");
    Serial.print(az);
    Serial.println("E");
//    Serial.println(mx);
//    Serial.println(my);
//    Serial.println(mz);
    Serial.println(heading);
    //Serial.println(gz);
    dt = ((double)time)/1000000.0; //convert microseconds to seconds
    time = 0;
  }
}

