//Mini throttle and 2-axis joystick for USB Joystick Flight Sim Teensy 3.2
// Notes at https://www.pjrc.com/teensy/td_joystick.html

#define JOYSTICK_MODE
//#define DEBUG

// Mapping
#define FIRE_AUX_2 0
#define COUNTER 1
#define SHIELD_FOCUS 3
#define BOOST 2
#define PING 10
#define TARGET 11

// Joystick
#define THROTTLE A2 
#define PITCH A3
#define ROLL A1
#define POV A0
#define YAW A4
#define FIRE_AUX 9 
#define FIRE_PRIMARY 8

// POV Hat
#define POWER_BALANCE 7
#define POWER_ENGINE 6
#define POWER_WEAPONS 5
#define POWER_SHIELDS 4

#define MOVING_AVG_FILTER_NUM 3
int smooth_array_throttle[MOVING_AVG_FILTER_NUM];
int index_throttle = 0;
int sum_throttle = 0;
int smooth_throttle = 0;

char outputArray[256*4];

//  int converted_throttle = mapVal(smooth_throttle,687,1023,0,1023);
int mapVal(int curr_val, int minIn, int maxIn, int minOut, int maxOut)
{
  return (minOut + ((maxOut - minOut) / (maxIn - minIn) * (curr_val - minIn)));
}

int getRoll()
{
  int test_roll = analogRead(ROLL); 
  if (test_roll < 0) return 0;
  else return test_roll;
}

int getYaw()
{
  int test_yaw = mapVal(analogRead(YAW),0,448,0,1023); 
  if (test_yaw < 0) return 0;
  else if (test_yaw > 1023) return 1023;
  return test_yaw;
}

int getPitch()
{
  int test_pitch = analogRead(PITCH); 
  if (test_pitch < 0) return 0;
  else if (test_pitch > 1023) return 1023;
  else return test_pitch;
}

int getThrottle()
{  
  int test_throttle = mapVal(analogRead(THROTTLE),690,1023,0,1023); 
  if (test_throttle < 0) return 0;
  else if (test_throttle > 1023) return 1023;
  else return test_throttle;
}

int getsmoothThrottle()
{
  sum_throttle = sum_throttle - smooth_array_throttle[index_throttle];  
  uint8_t temp_value = mapVal(analogRead(THROTTLE),687,1023,0,1023);
  smooth_array_throttle[index_throttle] = temp_value;
  sum_throttle += temp_value;
  index_throttle = (index_throttle+1) % MOVING_AVG_FILTER_NUM;
  smooth_throttle = sum_throttle / MOVING_AVG_FILTER_NUM;
  return smooth_throttle;
} 

// The hat switch can be moved in 45 degree increments. Use -1 to return the hat to center resting position.
// Pushbuttons used to activate balance option
int getFlightBalance()
{
  if (!digitalRead(POWER_BALANCE)) return 0;
  else if (!digitalRead(POWER_ENGINE) && !digitalRead(POWER_WEAPONS)) return 125; //top left
  else if (!digitalRead(POWER_ENGINE) && !digitalRead(POWER_BALANCE)) return 45; //bottom left
  else if (!digitalRead(POWER_WEAPONS) && !digitalRead(POWER_SHIELDS)) return 225; //top right
  else if (!digitalRead(POWER_BALANCE) && !digitalRead(POWER_SHIELDS)) return 315; //bottom right
  else if (!digitalRead(POWER_ENGINE)) return 90;
  else if (!digitalRead(POWER_WEAPONS)) return 180;
  else if (!digitalRead(POWER_SHIELDS)) return 270;
  else return -1;
}

void setup() {

  #ifdef JOYSTICK_MODE
    Joystick.useManualSend(true);
  #endif

  pinMode(THROTTLE,INPUT);
  pinMode(ROLL,INPUT);
  pinMode(PITCH,INPUT);  
  pinMode(YAW,INPUT);  
  pinMode(13,OUTPUT);
  pinMode(FIRE_AUX, INPUT);
  pinMode(FIRE_PRIMARY, INPUT);
  pinMode(PING,INPUT);  
  pinMode(TARGET,INPUT);  

  pinMode(POV,INPUT);  

  pinMode(COUNTER,INPUT);
  pinMode(SHIELD_FOCUS,INPUT);
  pinMode(FIRE_AUX_2,INPUT);
  pinMode(BOOST,INPUT);

  pinMode(POWER_BALANCE,INPUT);
  pinMode(POWER_ENGINE,INPUT);
  pinMode(POWER_WEAPONS,INPUT);
  pinMode(POWER_SHIELDS,INPUT);

  digitalWrite(13,HIGH);

  Serial.begin(115200);
  Serial.println("test");

  delay(1000);
}

void loop()
{        
 
  #ifdef JOYSTICK_MODE

    // Analog Sticks
    Joystick.X(getPitch()); // on vintage joystick
    Joystick.Y(getRoll());  // on vintage joystick 
    Joystick.Zrotate(getThrottle());
    Joystick.Z(getYaw());

    //POV Select - POV South, POV North, POV West, POV East
    Joystick.hat(getFlightBalance());

    // Buttons`
    Joystick.button(1,!digitalRead(FIRE_PRIMARY)); // on vintage joystick
    Joystick.button(3,!digitalRead(FIRE_AUX)); // on vintage joystick
    Joystick.button(11,!digitalRead(PING)); // on throttle body
    Joystick.button(2,!digitalRead(TARGET)); // on throttle body


    if (analogRead(POV) >= 180 && analogRead(POV) < 255)
    {
      Joystick.button(12,HIGH); //FREE LOOK
      Joystick.button(5,LOW);
      Joystick.button(9,LOW);
      Joystick.button(10,LOW);
    } 
    else if (analogRead(POV) >= 255 & analogRead(POV) < 512)
    {
      Joystick.button(5,HIGH); //CYCLE_TARGET
      Joystick.button(12,LOW);       
      Joystick.button(9,LOW);
      Joystick.button(10,LOW);
    } 
    else if (analogRead(POV) >= 512 & analogRead(POV) < 768)
    {
      Joystick.button(9,HIGH); //TARGET_MENU
      Joystick.button(12,LOW);       
      Joystick.button(5,LOW);
      Joystick.button(10,LOW);
    } 
    else if (analogRead(POV) >= 768) 
    {
      Joystick.button(10,HIGH); //MENU
      Joystick.button(12,LOW);       
      Joystick.button(9,LOW);             
      Joystick.button(5,LOW);
    }
    else
    {
      Joystick.button(10,LOW);
      Joystick.button(5,LOW);
      Joystick.button(9,LOW);
      Joystick.button(12,LOW);      
    }
    
    Joystick.button(4,digitalRead(FIRE_AUX_2)); // SPST Switch
    Joystick.button(6,digitalRead(COUNTER)); // SPST Switch
    Joystick.button(7,digitalRead(SHIELD_FOCUS)); // SPST Switch
    Joystick.button(8,digitalRead(BOOST)); // SPST Switch
      
    Joystick.send_now();
  #endif

  #ifdef DEBUG
    sprintf(outputArray,"ROLL: %d, PITCH: %d, YAW: %d\n",getRoll(), getPitch(), getYaw());    
    //sprintf(outputArray,"PRIM: %d, AUX1 %d, AUX2 %d, TAR: %d, PING: %d, COUNT: %d, SHIELD: %d, BOOST: %d\n",digitalRead(FIRE_PRIMARY), digitalRead(FIRE_AUX), digitalRead(FIRE_AUX_2), digitalRead(TARGET),digitalRead(PING),
    //digitalRead(COUNTER),digitalRead(SHIELD_FOCUS),digitalRead(BOOST));
    Serial.print(outputArray);
  #endif
    
  delay(50);
   
}
