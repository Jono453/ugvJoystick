/*

Driver to send vintage joystick inputs to 915 MHz telemetry link
to send to RC car

References:
http://www.built-to-spec.com/blog/2009/09/10/using-a-pc-joystick-with-the-arduino/
https://old.pinouts.ru/InputCables/GameportPC_pinout.shtml
https://baremetalmicro.com/tutorial_avr_digital_io/04-Outputs.html

*/

//#define DEBUG_MODE //debug_mode shows raw ADC 
#define STEER_ADC A0
#define THROTTLE_ADC A1
#define LED 6
#define TRIGGER 2
#define BLINK_TIME 250

char tx_array[24] = {};
unsigned long blink_ctr = 0;

#define MOVING_AVG_FILTER_NUM 5
int smooth_array_steer[MOVING_AVG_FILTER_NUM];
int smooth_array_throttle[MOVING_AVG_FILTER_NUM];
int index_steer = 0;
int sum_steer = 0;
int index_throttle = 0;
int sum_throttle = 0;
int smooth_steer = 0;
int smooth_throttle = 0;

void setup()
{

  Serial.begin(57600);
  pinMode(STEER_ADC,INPUT);
  pinMode(THROTTLE_ADC,INPUT);
  pinMode(LED,OUTPUT);
  pinMode(TRIGGER,INPUT);
}

int mapVal(int curr_val, int minIn, int maxIn, int minOut, int maxOut)
{
  return (minOut + ((maxOut - minOut) / (maxIn - minIn) * (curr_val - minIn)));
}

// moving average filter for ADC (steer)
int tx_steer()
{
//  sum_steer = sum_steer - smooth_array_steer[index_steer];
//  uint8_t temp_value_steer = analogRead(STEER_ADC);
//  smooth_array_steer[index_steer] = temp_value_steer;
//  sum_steer += temp_value_steer;    
//  index_steer = (index_steer+1) % MOVING_AVG_FILTER_NUM;
//  smooth_steer = sum_steer / MOVING_AVG_FILTER_NUM;
  return mapVal(analogRead(STEER_ADC),960,670,1000,2000);
}

// moving average filter for ADC (throttle)
int tx_throttle()
{
  sum_throttle = sum_throttle - smooth_array_throttle[index_throttle];
  uint8_t temp_value = analogRead(THROTTLE_ADC);
  smooth_array_throttle[index_throttle] = temp_value;
  sum_throttle += temp_value;
  index_throttle = (index_throttle+1) % MOVING_AVG_FILTER_NUM;
  smooth_throttle = sum_throttle / MOVING_AVG_FILTER_NUM;
  return mapVal(smooth_throttle,27,252,1000,2000);
}

void loop()
{
  
  if (digitalRead(TRIGGER) == LOW)
  {
    if (millis() - blink_ctr > BLINK_TIME)
    {
      blink_ctr = millis();
      digitalWrite(LED,HIGH);     
    }
    else
    {
      digitalWrite(LED,LOW);
    }
     
    #ifdef DEBUG_MODE
      sprintf(tx_array,"S: %d T: %d\n",analogRead(STEER_ADC),analogRead(THROTTLE_ADC));
    #else
      sprintf(tx_array,"%d,%d#\n",tx_steer(),tx_throttle());
    #endif
    Serial.print(tx_array);  
  }
  
  delay(50);  
}
