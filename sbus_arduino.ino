// Adapted from https://quadmeup.com/generate-s-bus-with-arduino-in-a-simple-way/

//#define DEBUG 

HardwareSerial SerialSBUS(PA3,PA2); //syntax for STM32Duino
//HardwareSerial SerialSBUS(PA10,PA9); //syntax for STM32Duino

#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010
#define RC_RAW_MIN 1000
#define RC_RAW_MAX 2000
#define LED PB4 //D12
#define LED_TIME 250
#define FLIGHT_SWITCH_A PA12 //D2
#define FLIGHT_SWITCH_B PB7 //D4
#define ARM_SWITCH PA11 //D10
#define GIMBAL_STEER PA0
#define GIMBAL_THROTTLE PA1
#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 15 //ms

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){

    static int output[SBUS_CHANNEL_NUMBER] = {0};

    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;
uint32_t ledTime = 0;
bool ledState = 0;

void setup() {

    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        rcChannels[i] = 0;
    }
    
    #ifdef DEBUG
      Serial.begin(115200);
    #else
      SerialSBUS.begin(100000, SERIAL_8E2);    
    #endif

    pinMode(LED, OUTPUT);
    pinMode(FLIGHT_SWITCH_A,INPUT);
    pinMode(FLIGHT_SWITCH_B,INPUT);
    pinMode(ARM_SWITCH,INPUT);
}

int getMode()
{
  if (!digitalRead(FLIGHT_SWITCH_A)) return 1815;
  if (!digitalRead(FLIGHT_SWITCH_B)) return 1165;
  else return 1425;
}

int getArm()
{
  if (!digitalRead(D10)) return 1950;
  else return 1150;
}

void loop() {
    
    uint32_t currentMillis = millis();    
    uint32_t ledMillis = millis();

    /*   
    ArduRover RC Channel Mapping 
    RC1 - servo steer (PA0)
    RC3 - throttle (PA1)
    RC4 - Unused
    RC5 is Mode Switch (PB6 and PB1)
      Hold - 1165
      Manual - 1425
      Auto - 1815
    RC6 is Arming Switch (PA11)
    */
    
    if (currentMillis > sbusTime) {
      
        sbusPreparePacket(sbusPacket, rcChannels, false, false);
        
        #ifdef DEBUG
          Serial.print("RC1 ");
          Serial.print(map(analogRead(GIMBAL_STEER),0,1015,RC_RAW_MIN,RC_RAW_MAX));
          //Serial.print(analogRead(GIMBAL_STEER));
          Serial.print(" | ");          
          Serial.print("RC3 ");          
          Serial.print(map(analogRead(GIMBAL_THROTTLE),0, 1015,RC_RAW_MIN,RC_RAW_MAX));
          //Serial.print(analogRead(GIMBAL_THROTTLE));
          Serial.print(" | ");          
          Serial.print(getMode());
          Serial.print(" | ");          
          Serial.println(getArm());
        #else
          // Here you can modify values of rcChannels while keeping it in 1000:2000 range     
          rcChannels[0] = map(analogRead(GIMBAL_STEER),0,1015,RC_RAW_MIN,RC_RAW_MAX);   
          rcChannels[1] = 1500;
          rcChannels[2] = map(analogRead(GIMBAL_THROTTLE),0,1015,RC_RAW_MIN,RC_RAW_MAX);  
          rcChannels[3] = 1500;
          rcChannels[7] = getMode();
          rcChannels[5] = getArm();
          SerialSBUS.write(sbusPacket, SBUS_PACKET_LENGTH);
        #endif

        sbusTime = currentMillis + SBUS_UPDATE_RATE;
    }

  if( ledState == HIGH )
  {
    if( (millis()- ledTime) >= LED_TIME){   
      ledState = LOW;// change the state of LED
      ledTime=millis();// remember Current millis() time
    }
  }
  else
  {   
    if( (millis()- ledTime) >= LED_TIME){     
      ledState =HIGH;// change the state of LED
      ledTime=millis();// remember Current millis() time
    }
  }

  digitalWrite(LED,ledState);
      
}