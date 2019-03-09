
#include <Servo.h>
#include <Wire.h>

#define SERVO_L 3
#define SERVO_R 2

#define ESC_L 5
#define ESC_R 4 

int DELAY = 1000;
int YAW;

Servo servoL,servoR;
Servo escL,escR;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[3];
float Total_angle[3];

float angleSetPoint = 0;
float angRes = 1.0;

double channel[4];

float time,timePrev, elapsedTime;
float timePrevDisarm=0;
float timeDisarm=0;
float yawTimeDisarm=0;
float yawTime =0;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float yawPID, yaw_err, yaw_perr;
float pid_p=0;
float pid_i=0;
float pid_d=0;

float ypid_p=0;
float ypid_i=0;
float ypid_d=0;

double y_p=4;//3
double y_i=0.005;
double y_d=10.0;//6
/////////////////PID CONSTANTS/////////////////
double kp=3.55;//3.55
double ki=0.005;//0.005
double kd=2.05;//2.05
///////////////////////////////////////////////

double throttle=1030; //initial value of throttle to the motors
double pitch = 0;
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady
bool firstTime=true;



void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //Serial.begin(250000);

  
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);

  servoL.attach(SERVO_L);
  servoR.attach(SERVO_R);

  escL.attach(ESC_L);
  escR.attach(ESC_R);

  time = millis();
  escL.writeMicroseconds(1000);
  escR.writeMicroseconds(1000);
}

void loop() {
timePrev = time;
time = millis();
elapsedTime = (time - timePrev) / 1000; 

//ARMING PHASE
while(yawTime < 2000){ 
  escL.writeMicroseconds(1000);
  escR.writeMicroseconds(1000);
  channel[0] = pulseIn(10, HIGH);
  
  if( (channel[0]<1800) && (channel[0]>1200)  ){
    timePrev = time;
    time = millis();
    yawTime = time - timePrev;
  }else{   
    time = millis();
    yawTime = time - timePrev;
  }
  
}

if(firstTime){
  firstTime = false;
  timePrev = 0;
  time = millis();
  timeDisarm = millis();
  timePrev=time;
  timePrevDisarm=timeDisarm;

}

if( (channel[3]<1700) && (channel[3]>1300)  ){
  timePrevDisarm = timeDisarm;
  timeDisarm = millis();
  yawTimeDisarm = timeDisarm - timePrevDisarm;
}else{   
  timeDisarm = millis();
  yawTimeDisarm = timeDisarm - timePrevDisarm;
}
  
if(yawTimeDisarm>2000){
  delay(1000);
  yawTimeDisarm = 0;
  yawTime = 0;
  firstTime = true;
}

  // put your main code here, to run repeatedly:
  channel[0] = pulseIn(10, HIGH); // Yaw
  channel[1] = pulseIn(11, HIGH); // Throttle
  channel[2] = pulseIn(12, HIGH); // Pitch
  channel[3] = pulseIn(13, HIGH); // Roll 


 Wire.beginTransmission(0x68);
 Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
 Wire.endTransmission(false);
 Wire.requestFrom(0x68,6,true); 

 Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
 Acc_rawY=Wire.read()<<8|Wire.read();
 Acc_rawZ=Wire.read()<<8|Wire.read();

     /*---X---*/
 Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
 Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;


 Wire.beginTransmission(0x68);
 Wire.write(0x43); //Gyro data first adress
 Wire.endTransmission(false);
 Wire.requestFrom(0x68,6,true); //Just 4 registers
  
 Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
 Gyr_rawY=Wire.read()<<8|Wire.read();
 Gyr_rawZ=Wire.read()<<8|Wire.read();

    /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;
   /*---Z---*/
   Gyro_angle[2] = Gyr_rawZ/131.0;
     /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   /*---Z axis angle---*/
   Total_angle[2] =       (Total_angle[2] + Gyro_angle[2]*elapsedTime);

   
   error = Total_angle[1] - desired_angle;
  if((channel[0]<1400)||(channel[0]>1500)){
    angleSetPoint+= angRes*(0.0025*channel[0]-1)-2.625;
  }
   yaw_err = angleSetPoint - Total_angle[2];


   ypid_p = y_p*yaw_err;
   ypid_d = y_d*((yaw_err-yaw_perr)/elapsedTime);

   if(-10 <yaw_err <10)
    {
      ypid_i = ypid_i+(y_i*yaw_err);
      yawPID = ypid_p + ypid_i + ypid_d;  
    }else{
      yawPID = pid_p + pid_d;
    }
if(yawPID < -250)
{
  yawPID=-250;
}
if(yawPID > 250)
{
  yawPID=250;
}
   pid_p = kp*error;
   pid_d = kd*((error - previous_error)/elapsedTime);
   if(-3 <error <3)
    {
      pid_i = pid_i+(ki*error);
      PID = pid_p + pid_i + pid_d;  
    }else{
      PID = pid_p + pid_d;
    }
if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}

throttle = channel[1]*1.175 - 297.75;
pitch = (channel[2]-1470)*0.5;

if(pitch<10&&pitch>-10){
  pitch =0;  
}

if (throttle<1130){
  throttle = 1130;
}
if (throttle>1900){
  throttle = 1900;
}

pwmLeft =  throttle - PID;
pwmRight = throttle + PID;
  
//Right
if(pwmRight < 1030)
{
  pwmRight= 1030;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}
//Left
if(pwmLeft < 1030)
{
  pwmLeft= 1030;
}
if(pwmLeft > 2000)
{
  pwmLeft=2000;
}
//Estos son los ESCs de los motores
escR.writeMicroseconds( pwmRight );

escL.writeMicroseconds( pwmLeft  );


//Estos son los servos para controlar el giro o Yaw y el Pitch
servoL.writeMicroseconds(1500- pitch + yawPID );
servoR.writeMicroseconds(1500+ pitch + yawPID );


previous_error = error;
yaw_perr = yaw_err;


  
}
