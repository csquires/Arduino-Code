#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
/* Axes dimensions( mm ) */
#define L0 50.00
#define L1 155.00      
#define L2 180.00        
#define L3 95.00

#define pi 3.14159
 
/* Servo names and PIN number */
#define B_SERVO 0
#define S_SERVO 1
#define E_SERVO 2
#define W_SERVO 3
#define WR_SERVO 4
#define G_SERVO 5

// Naming pins used here
#define xswitch A0    
#define yswitch A1
#define zswitch A2
#define xsignage A3
#define ysignage A4
#define zsignage A5

// to be used for switching modes
#define modeswitch 't'
#define killswitch 'k'
#define killz 'z'

const float minmove = .2;
const float minmove2 = .5;

//variables for the readings of each pin via AnalogRead
int xValue = 0;
int yValue = 0;
int zValue = 0;
int xsignValue = 0;
int ysignValue = 0;
int zsignValue = 0;

int xsigncounter = 0;
int ysigncounter = 0;
int zsigncounter = 0;

boolean xinc = false;
boolean yinc = false;
boolean zinc = false;
boolean xpressed = false;
boolean ypressed = false;
boolean zpressed = false;
boolean zdead = false;

// control velocities
float dx = minmove;
float dy = minmove;
float dz = minmove;
float dgad = minmove;
float dwr = minmove2;
float dg = minmove2;
float dyprime = minmove;
float dzprime = minmove;

int mode = 1;
int countermax = 10;
int xtimesthrough = 0;
int ytimesthrough = 0;
int ztimesthrough = 0;

const int arraysize = 10;
int myXvalues[arraysize];
int myYvalues[arraysize];
int myZvalues[arraysize];

/* Pre-calculation */
float L1sq = L1*L1;
float L2sq = L2*L2;

float x = 0;
float y = 120;
float z = 120;
float gad = 0;
float wr = 190;
float g = 300;
 
int minXvoltage = 150;
int minYvoltage = 150;
int minZvoltage = 150;
unsigned long counter = 0;
 
 
 
 
 
 
 
 
 
void setup()
{
  pinMode(xswitch, INPUT);  //all pins set to accept input
  pinMode(yswitch, INPUT);
  pinMode(zswitch, INPUT);
  pinMode(xsignage, INPUT);
  pinMode(ysignage, INPUT);
  pinMode(zsignage, INPUT);
  Serial.begin(9600);
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  x = 120;
  y = 120;
  z = 120;
  gad = 0;
}// end of setup

void loop() 
{
  //counter++;
  //sets servos
  IK (x, y, z, gad);
  set_wrist_rotate(wr);
  set_gripper(g);
  
  //prints coordinates
  //if(counter%5==0)
  //{
    Serial.print("Coordinates: ");
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print("\t");
    Serial.print(gad);
    Serial.print("\tWR: ");
    Serial.print(wr);
    Serial.print("\tG: ");
    Serial.print(g);
    Serial.print("\t Mode: ");
    Serial.print(mode);
    Serial.print("\n");
  //}
  delay (30);
  char byte = 0;
  
  // gets values from each sensor
  xValue = analogRead(xswitch);
  delay(1);
  yValue = analogRead(yswitch);
  delay(1);
  zValue = analogRead(zswitch);
  delay(1);
  xsignValue = analogRead(xsignage);
  //delay(1);
  ysignValue = analogRead(ysignage);
  //delay(1);
  zsignValue = analogRead(zsignage);
  //delay(1);
  
  if (xsignValue != 1023){xpressed = true;}else{xpressed=false;}
  if (ysignValue != 1023){ypressed = true;}else{ypressed=false;}
  if (zsignValue != 1023){zpressed = true;}else{zpressed=false;}
  
  //updates arrays
  for(int i=0;i<arraysize-1;i++)
  {
    myXvalues[i+1] = myXvalues[i];
  }
  myXvalues[0] = xValue;
  for(int i=0;i<arraysize-1;i++)
  {
    myYvalues[i+1] = myYvalues[i];
  }
  myYvalues[0] = yValue;
  for(int i=0;i<arraysize-1;i++)
  {
    myZvalues[i+1] = myZvalues[i];
  }
  myZvalues[0] = zValue;
  
  // keeps track of times through the loop since last impulse
  xtimesthrough++;
  ytimesthrough++;
  ztimesthrough++;
  
  // checks arrays to see if 7 out of the 10 values are above 100
  // checking times through allows computer to distinguish between signals
  if(xtimesthrough >40 && more_than_number(myXvalues,4,minXvoltage))
  {
    xinc = !xinc;
    xtimesthrough = 0;
    dx = minmove;
    dgad = minmove;
  }
  if(xtimesthrough >40 && more_than_number(myYvalues,4,minYvoltage))
  {
    yinc = !yinc;
    ytimesthrough = 0;
    dy = minmove;
    dwr = minmove2;
  }
  if(xtimesthrough >40 && more_than_number(myZvalues,4,minZvoltage))
  {
    zinc = !zinc;
    ztimesthrough = 0;
    dz = minmove;
    dg = minmove2;
  }
  
  // changes values to be sent to servos when loop starts over
  if (xinc)
  {
    int a = 1;
    if(xtimesthrough>80){a = 4;}
    else if(xtimesthrough>40){a=2;}
    if (xpressed){a *= -1;}
    if (mode == 1){x += a*dx;}
    else {gad += a*dgad;}
    //else if (mode == 2){gad += a*dgad;}
    //else if (mode == 3){x+=a*dx;}
  }
  if (yinc)
  {
    int a = 1;
    if(ytimesthrough>80){a=4;}
    else if(ytimesthrough>40){a=2;}
    if (ypressed){a *= -1;}
    if (mode == 1){y += a*dy;}
    else {wr += a*dwr;}
    //else if (mode == 2){wr += a*dwr;}
    //else if (mode == 3){y+=a*dyprime*cos(gad); z-=a*dyprime*sin(gad);}
    
    // boundary checks
    if (y<30){y=30;}
    if (wr<70){wr=70;}
    else if (wr>540){wr=540;}
  }
  if (zinc)
  {
    int a = 1;
    if(ztimesthrough>80){a = 4;}
    else if(ztimesthrough>40){a = 2;}
    if (zpressed){a *= -1;}
    if (mode == 1 && !zdead){z += a*dz;}
    else {g += a*dg;}
    //else if (mode == 2){g += a*dg;}
    //else if (mode == 3){y+=a*dzprime*sin(gad);z+=a*dzprime*cos(gad);}
    
    // boundary checks
    if (z<-60){z=-60;}
    if (g>600){g=600;}
    else if (g<230){g=230;}
  }
  
  
  
  // allows user to switch between modes
  while (Serial.available()>0)
  {
      Serial.readBytes(&byte,1);
      if (byte == modeswitch){switch_modes(mode);}
      if (byte == killswitch)//in case we fuck up
      {
        x=120;
        y=120;
        z=120;
        gad=0;
        g = 230;
        xinc = yinc = zinc = false;
        dx = dy = dz = dgad = minmove;
        dwr = dg = minmove2;
      }
      if (byte == killz){zdead = !zdead;}
  }
}// end of loop














void IK( float x, float y, float z, float gad ) //gad = angle (in degree) between the hand and horizontal axis (theta 3 prime)
{

    float gar = radians( gad );    //convert to radians for use in calculations
 
   /* Calculating base angle and radial distance */
    float theta0r = atan2( y, x );
    float r= sqrt(( x * x ) + ( y * y ));
  
//    Serial.print("Base angle: ");
//    Serial.print(theta0r*180/pi);
//    Serial.print("\n");
  
   /* Calculating position of the wrist without considering the base height */
    float a = z - L0 + L3*sin(gar);
    float b = r - L3*cos(gar);
 
   /* Calculating L4 */
   float L4sq = a*a + b*b;
   float L4 = sqrt(L4sq);
 
   /* Shoulder angle */
   float theta1r = acos((L2sq-L1sq-L4sq)/(-2*L1*L4)) - asin(b/L4);
//   Serial.print("Theta1: ");
//   Serial.print(theta1r*180/pi);
//   Serial.print("\n");
 
   /* Elbow angle */
   float theta2r = pi - acos((L4sq-L1sq-L2sq)/(-2*L1*L2));
//   Serial.print("Theta2: ");
//   Serial.print(theta2r*180/pi);
//   Serial.print("\n");
 
   /* Wrist angle */
   float theta3r = gar + pi/2 - acos((L1sq-L2sq-L4sq)/(-2*L2*L4)) - asin(b/L4);
//   Serial.print("Theta3: ");
//   Serial.print(theta3r*180/pi);
//   Serial.print("\n\n");
 
   // sets servos to desired angles
   set_base_servo(theta0r);
   set_shoulder_servo(theta1r);
   set_elbow_servo(theta2r);
   set_wrist_servo(theta3r);
}// end of IK function





// functions to set servos
void set_base_servo(float theta)
{
  float t = pi - theta;
  float BPWM;
  if (t <= pi/4){BPWM = mapp(t,0,pi/4,150,240);}
  if (t > pi/4 && t <= pi/2){BPWM = mapp(t,pi/4,pi/2,240,360);}
  if (t > pi/2 && t<= 3*pi/4){BPWM = mapp(t,pi/2,3*pi/4,360,485);}
  if (t > 3*pi/4){BPWM = mapp(t,3*pi/4,pi,485,600);}
  pwm.setPWM(B_SERVO, 0, BPWM);
}

void set_shoulder_servo(float theta)
{
  float t = theta + pi/2;
  float SPWM;
  if (t <= pi/4){SPWM = mapp(t,0,pi/4,150,260);}
  if (t > pi/4 && t <= pi/2){SPWM = mapp(t,pi/4,pi/2,260,360);}
  if (t > pi/2 && t<= 3*pi/4){SPWM = mapp(t,pi/2,3*pi/4,360,460);}
  if (t > 3*pi/4){SPWM = mapp(t,3*pi/4,pi,460,560);}
  pwm.setPWM(S_SERVO, 0, SPWM);
}

void set_elbow_servo(float t)
{
  float EPWM;
  if (t <= pi/4){EPWM = mapp(t,0,pi/4,200,285);}
  if (t > pi/4 && t <= pi/2){EPWM = mapp(t,pi/4,pi/2,285,370);}
  if (t > pi/2 && t<= 3*pi/4){EPWM = mapp(t,pi/2,3*pi/4,370,475);}
  if (t > 3*pi/4){EPWM = mapp(t,3*pi/4,pi,475,545);}
  pwm.setPWM(E_SERVO, 0, EPWM);
}

void set_wrist_servo(float theta)
{
  float t = pi/2 - theta;
  float WPWM;
  if (t <= pi/4){WPWM = mapp(t,0,pi/4,130,240);}
  if (t > pi/4 && t <= pi/2){WPWM = mapp(t,pi/4,pi/2,240,345);}
  if (t > pi/2 && t<= 3*pi/4){WPWM = mapp(t,pi/2,3*pi/4,345,440);}
  if (t > 3*pi/4){WPWM = mapp(t,3*pi/4,pi,440,565);}
  pwm.setPWM(W_SERVO, 0, WPWM);
}

void set_wrist_rotate(float wr)
{
  pwm.setPWM( WR_SERVO, 0, wr);
}

void set_gripper(float g)
{
  pwm.setPWM( G_SERVO, 0, g);
}
// end of functions to set servos











float mapp(float val, float fromLow, float fromHigh, float toLow, float toHigh)
{
  float fromRange = fromHigh - fromLow;
  float toRange = toHigh - toLow;
  float fromDif = val - fromLow;
  float toDif = fromDif*(toRange/fromRange);
  return toLow + toDif;
}

void switch_modes(int m)
{
  if (m == 1){mode = 2;}
  if (m == 2){mode = 1;}
  //if (m == 2){mode = 3;}
  //if (m == 3){mode = 1;}
}

boolean more_than_number(int array[], int number, int voltage)
{
  int count = 0;
  for(int i=0;i<10;i++)
  {
    if (abs(array[i]-300) > voltage){count++;}
  }
  if (count>=number){return true;}
  else {return false;}
}

/* Set home position for servos */
void servo_home()
{
   pwm.setPWM( B_SERVO, 0, 550 );
   pwm.setPWM( S_SERVO, 0, 450);
   pwm.setPWM( E_SERVO, 0, 350 );
   pwm.setPWM( W_SERVO, 0, 130 );
   pwm.setPWM( WR_SERVO, 0, 600);
   pwm.setPWM( G_SERVO, 0, 600 );
   return;
}

