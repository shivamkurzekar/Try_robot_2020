//Edited on 16/3/2020
//Robocon 2020 kick-bot
//Shivam Kurzekar

//Mechanisms - Catch, Place, Kick
//  Algorithms implemented :
//  1. Manually operated
//  2. Angle alignment
//  3. Kicking Mechanism (without f/b)
//  4. Self align at TRY SPOT

//Remaining:
//  TEE Alingment
//  Kicking (closed loop)
//  Catching
//  Placing
//  Indication
  
// b >> wheels B and D || a >> wheels A and C
/*
  // C  B
  // D  A
*/
// 45,43,41,39 ultra2

#include<I2C_Anything.h>
#include<Wire.h>
#include <XBOXRECV.h>

#include <MPU6050.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>


const byte slave_address_one = 8, slave_address_two = 11, slave_address_three = 10, slave_address_four = 12;   // 8,11,10,12

boolean try_spot_flag = 0, tee_align_flag = 0, tee_align_flag_2 = 0;     // status flags

/////////////////////////ultrasonic_PID////////////////////////////
float  ultrasonic_rot_error = 0, ultrasonic_str_error = 0;
float ultrasonic_average;
float ultrasonic_rot_error_diff = 0, ultrasonic_str_error_diff = 0;
float last_ultrasonic_rot_error = 0, last_ultrasonic_str_error = 0;
float ultra_str_kp = 10, ultra_str_kd = 50;
float ultra_rot_kp = 10, ultra_rot_kd = 50;
///////////////////////////////////////////////////////////////////

//////////////////////MANUAL_CONTROL//////////////////////////////
int pa = 0, pb = 0, pc = 0, pd = 0;
int x = 0, y = 0;
float a = 0, b = 0, c = 0, d = 0, abs_a = 0, abs_b = 0, abs_c = 0, abs_d = 0;
int max_value = 0;
int pwm = 0;
int max_pwm = 7000;
float magnitude = 0;
int rotational = 0, rotational_pwm = 0, max_rotational_pwm = 2000 ;
float theta = 0;
///////////////////////////////////////////////////////////////////


///////////////////////ultrasonic_sensor///////////////////////////
long distance_1 = 0, duration_1 = 0, distance_2 = 0, duration_2 = 0;
const int trig1 = 23, trig2 = 41, echo1 = 25, echo2 = 43;
//const int trig1 = 23, trig2 = 51, echo1 = 25, echo2 = 53;
int dis_1 = 0, dis_2 = 0;
///////////////////////////////////////////////////////////////////

/////////////////////////////Kick//////////////////////////////////
const int kick_m1 = 3, kick_m2 = 7, kick_pwm_pin = 5;
boolean kick_flag = 0;
const int kick_relay_1 = A0, kick_relay_2 = A1;
int k = 0;
///////////////////////////////////////////////////////////////////

//////////////////////ANGLE_ALIGNMENT//////////////////////////////
unsigned long timer = 0;
float timeStep = 0.01;
boolean align_status = 0, wall_status = 0;
int MPU_rotational_pwm = 0;
float error = 0, last_error = 0, d_error = 0, i_error = 0;
float kp = 0, ki = 0, kd = 0;            // for rotation
float KP = 75, KD = 105, KI = 0;
unsigned long i_timer = 0;
// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
float angle = 10;       //Desired predefined angle
///////////////////////////////////////////////////////////////////

///////////////////////////
MPU6050 mpu;
///////////////////////////

///////////////////////////
USB Usb;
XBOXRECV Xbox(&Usb);
///////////////////////////

void setup() {

  ////////////KICK////////////////////
  pinMode(kick_m1, OUTPUT);
  pinMode(kick_m2, OUTPUT);
  pinMode(kick_pwm_pin, OUTPUT);

  pinMode(kick_relay_1, OUTPUT);
  pinMode(kick_relay_2, OUTPUT);
  /////////////////////////////////////

  /////////////ULTRASONIC//////////////
  pinMode(echo1, INPUT);
  pinMode(trig1, OUTPUT);

  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);

  pinMode(45, OUTPUT);   //ultrasonic 2 Gnd
  pinMode(39, OUTPUT);   //ultrasonic 2 Vcc
  ///////////////////////////////////////
  digitalWrite(39, HIGH);  // Ultra2 VCC
  digitalWrite(45, LOW);   // Ultra2 GND
  ///////////////////////////////////////

  Serial.begin(115200);
  ///////////////////////////XBOX////////////////////////////////////
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
  ///////////////////////////////////////////////////////////////////

  //////////////////INITIALIZE_I2C///////////////////////////////////
  Wire.begin(slave_address_one);
  Wire.begin(slave_address_two);
  Wire.begin(slave_address_three);
  Wire.begin(slave_address_four);
  ////////////////////////////////////////////////////////////////////


  /////////////////////////MPU6050////////////////////////////////////
  //    while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  //    {
  //      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
  //      delay(500);
  //    }
  //    // Calibrate gyroscope. The calibration must be at rest.
  //    // If you don't want calibrate, comment this line.
  //    mpu.calibrateGyro();
  //    // Set threshold sensivty. Default 3.
  //    // If you don't want use threshold, comment this line or set 0.
  //    mpu.setThreshold(1);
  /////////////////////////////////////////////////////////////////////
}

void loop() {
  /*
    //  if (align_status)
    //  {
    //    align();
    //  }
    //  else
    //  {
    //    MPU_rotational_pwm = 0;
    //  }
  */

  calculate_vector();
  calculate_pwm();

  if(kick_flag)
  {
    initiate_kick();
  }
  send_values_to_slaves();

    print_values();
}

void print_values()
{
  Serial.print(pa);
  Serial.print("   ");
  Serial.print(pb);
  Serial.print("   ");
  Serial.print(pc);
  Serial.print("   ");
  Serial.println(pd);

//  Serial.print("  d1:");
//  Serial.print(dis_1);
//  Serial.print("  d2:");
//  Serial.print(dis_2);
//  Serial.print("x: ");
//  Serial.print(x);
//  Serial.print("  y: ");
//  Serial.println(y);
//  Serial.print(" Rot: ");
//  Serial.println(rotational_pwm);
//
//  Serial.print("   ");
//  Serial.print(ultrasonic_1());
//  Serial.print(" ");
//  Serial.println(ultrasonic_2());
//  Serial.println(y);
}

int ultrasonic_1()
{
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  duration_1 = pulseIn(echo1, HIGH);
  distance_1 = duration_1 * 0.017;
  return distance_1;
}

int ultrasonic_2()
{
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);
  duration_2 = pulseIn(echo2, HIGH);
  distance_2 = duration_2 * 0.017;
  return distance_2;
}

void calculate_vector()
{
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    Serial.print("c");
    for (uint8_t i = 0; i < 4; i++) {
      if (Xbox.Xbox360Connected[i]) {
        Serial.println("C: ");
        if (Xbox.getAnalogHat(LeftHatX, i) > 7500 || Xbox.getAnalogHat(LeftHatX, i) < -7500 || Xbox.getAnalogHat(LeftHatY, i) > 7500 || Xbox.getAnalogHat(LeftHatY, i) < -7500 || Xbox.getAnalogHat(RightHatX, i) > 7500 || Xbox.getAnalogHat(RightHatX, i) < -7500 || Xbox.getAnalogHat(RightHatY, i) > 7500 || Xbox.getAnalogHat(RightHatY, i) < -7500) {
          if (Xbox.getAnalogHat(LeftHatX, i) > 7500 ) {
            x = Xbox.getAnalogHat(LeftHatX, i);
            x = map(Xbox.getAnalogHat(LeftHatX, i), 7500, 32768, 0, 100);
          }
          else if ( Xbox.getAnalogHat(LeftHatX, i) < -7500)
          {
            x = Xbox.getAnalogHat(LeftHatX, i);
            x = map(Xbox.getAnalogHat(LeftHatX, i), -7500, -32768, 0, -100);
          }

          if (Xbox.getAnalogHat(LeftHatY, i) > 7500  ) {
            y = Xbox.getAnalogHat(LeftHatY, i);
            y = map(Xbox.getAnalogHat(LeftHatY, i), 7500, 32768, 0, 100);
          }
          else if (Xbox.getAnalogHat(LeftHatY, i) < -7500)
          {
            y = (Xbox.getAnalogHat(LeftHatY, i));
            y = map(Xbox.getAnalogHat(LeftHatY, i), -7500, -32768, 0, -100);
          }
        }
        if (Xbox.getAnalogHat(RightHatX, i) > 7500 )
        {
          rotational = (Xbox.getAnalogHat(RightHatX, i));
          rotational_pwm = map(rotational, 7500, 32768, 0, max_rotational_pwm);
        }
        else if (Xbox.getAnalogHat(RightHatX, i) < - 7500 )
        {
          rotational = (Xbox.getAnalogHat(RightHatX, i));
          rotational_pwm = map(rotational, -7500, -32768, 0, -max_rotational_pwm);
        }
        else if (Xbox.getAnalogHat(RightHatX, i) > - 7500 && Xbox.getAnalogHat(RightHatX, i) <  7500 )
        {
          rotational_pwm = 0;
        }

        if (Xbox.getAnalogHat(LeftHatY, i) > -7500 && Xbox.getAnalogHat(LeftHatY, i) < 7500)
        {
          y = 0;
        }

        if (Xbox.getAnalogHat(LeftHatX, i) > -7500 && Xbox.getAnalogHat(LeftHatX, i) < 7500)
        {
          x = 0;
        }

        if (Xbox.getAnalogHat(RightHatX, i) > - 7500 && Xbox.getAnalogHat(RightHatX, i) <  7500 )
        {
          rotational_pwm = 0;
        }

        if (Xbox.getButtonClick(A, i))
        {
          kick_flag = !kick_flag;
          Serial.println("kick");
          digitalWrite(kick_m1, LOW);
          digitalWrite(kick_m2, LOW);
          analogWrite(kick_pwm_pin, 0);
          digitalWrite(kick_relay_1, LOW);
          digitalWrite(kick_relay_2, LOW);
          k = 0;
        }

        if (Xbox.getButtonClick(B, i))
        {
          try_spot_flag = !try_spot_flag;
          Serial.println("B - TRYSPOT");
        }

        if (Xbox.getButtonClick(Y, i))
        {
          align_status = !align_status;
          yaw = 0;
          Serial.println("MPU align");
        }
      }
    }
  }
}

void calculate_pwm()
{
  magnitude = sqrt(sq(x) + sq(y));
  theta = atan2(y, x);    //tan inv of (y/x)
  b = magnitude * (cos(theta + (PI / 4)));   // b and d
  a = magnitude * (sin(theta + (PI / 4)));   // a and c

  pa = map(a, -142, 142, -max_pwm, max_pwm);
  pc = pa;
  pb = map(b, -142, 142, -max_pwm, max_pwm);
  pd = pb;

  pa = pa - rotational_pwm - MPU_rotational_pwm;
  pb = pb + rotational_pwm + MPU_rotational_pwm;
  pc = pc + rotational_pwm + MPU_rotational_pwm;
  pd = pd - rotational_pwm - MPU_rotational_pwm;
}
//////////////////////MPU////////////////////////////
int get_MPU_values()
{
  timer = millis();
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + (norm.ZAxis ) * timeStep;
  //  Serial.print(" Yaw = ");
  //  Serial.println(yaw);
  delay((timeStep * 1000) - (millis() - timer));
}

int align()                                        //// MPU align
{
  Serial.println("ENTERED");
  get_MPU_values();

  rotational_pwm = 0, x = 0, y = 0;

  Serial.print("Yaw = ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.println(error);
  error = yaw - angle;
  if (error <= 0.5 && error >= -0.5)
  {
    kp = 0;
    kd = 0;
    ki = 0;
  }
  else
  {
    kp = KP;
    kd = KD;
    ki = KI;
  }

  d_error = error - last_error;
  i_error = i_error + error;
  if (millis() - i_timer >= 1000)
  {
    i_error = 0;
    i_timer = millis();
  }
  MPU_rotational_pwm = kp * error + kd * d_error + ki * i_error;
  if (MPU_rotational_pwm >= 1000)
  {
    MPU_rotational_pwm = 1000;
  }
  else if (MPU_rotational_pwm <= -1000)
  {
    MPU_rotational_pwm = -1000;
  }

  last_error = error;
}
///////////////////////////////////////END//////////////////

void align_at_try_spot()
{
  last_ultrasonic_str_error = ultrasonic_str_error;
  last_ultrasonic_rot_error = ultrasonic_rot_error;
  //ultra_str_kp = 8, ultra_rot_kd = 10;

  dis_1 = ultrasonic_1();
  dis_2 = ultrasonic_2();
  dis_1 = constrain(dis_1, 0, 50);
  dis_2 = constrain(dis_2, 0, 50);
  ultrasonic_rot_error = dis_2 - dis_1;
  ultrasonic_average = (dis_1 + dis_2) / 2;
  ultrasonic_str_error = ultrasonic_average - 15; //10>>distance to maintain
  ultrasonic_str_error_diff = ultrasonic_str_error - last_ultrasonic_str_error;
  ultrasonic_rot_error_diff = ultrasonic_rot_error - last_ultrasonic_rot_error;


  if (dis_1 > 35 && dis_2 <= 35)
  {
    y = 10;
    x = 0;
  }

  else if (dis_1 < 35 && dis_2 >= 35)
  {
    y = -10;
    x = 0;
  }
  else if (dis_1 <= 35 && dis_2 <= 35)
  {
    y = 0;
    x = ((ultra_str_kp * ultrasonic_str_error) + ultra_str_kd * ultrasonic_str_error_diff);
    x = constrain(x, -25, 25);
    rotational_pwm = -(ultra_rot_kp * ultrasonic_rot_error) + (ultra_rot_kd * ultrasonic_rot_error_diff);
    rotational_pwm = constrain(rotational_pwm, -300, 300);
  }
  else
  {
    x = 0;
    y = 0;
    rotational_pwm = 0;
  }
  //  Serial.print(dis_1);
  //  Serial.print("  ");
  //  Serial.print(dis_2);
  //  Serial.print(" x:");
  //  Serial.print(x);
  //  Serial.print(" y:");
  //  Serial.print(y);
  //  Serial.print(" r:");
  //  Serial.println(rotational_pwm);
}

int send_values_to_slaves()
{
  Wire.beginTransmission(slave_address_one);
  I2C_writeAnything(pa);
  Wire.endTransmission(slave_address_one);

  Wire.beginTransmission(slave_address_two);
  I2C_writeAnything(pb);
  Wire.endTransmission(slave_address_two);

  Wire.beginTransmission(slave_address_three);
  I2C_writeAnything(pc);
  Wire.endTransmission(slave_address_three);

  Wire.beginTransmission(slave_address_four);
  I2C_writeAnything(pd);
  Wire.endTransmission(slave_address_four);
}

void initiate_kick()
{
  digitalWrite(kick_m1, HIGH);
  digitalWrite(kick_m2, LOW);
  analogWrite(kick_pwm_pin, k);
  delay(20);
  k++;
   if(k == 200)
   {
      delay(100);
      digitalWrite(kick_relay_1, HIGH);
      digitalWrite(kick_relay_2, LOW);
      delay(400);
      digitalWrite(kick_relay_1, LOW);
      digitalWrite(kick_relay_2, LOW);
      
      digitalWrite(kick_m1, LOW);
      digitalWrite(kick_m2, LOW);
      analogWrite(kick_pwm_pin, 0);
      k = 0;
      kick_flag = 0;
   }
}
