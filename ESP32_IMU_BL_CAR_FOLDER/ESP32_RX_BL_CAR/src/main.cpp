/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include "ControlMotors/MotorManager.h"
#include "BluetoothSerial.h"

#define PIN_PWM2_1  13
#define PIN_DIR2_1  12
#define PIN_PWM1_1  14
#define PIN_DIR1_1  27

#define PIN_PWM2_2  25
#define PIN_DIR2_2  33
#define PIN_PWM1_2  32
#define PIN_DIR1_2  16

#define PIN_LED 2
#define SIZE_FILTER 6

MotorManager *myMotors;
BluetoothSerial SerialBT;

int8_t i8DutyCycle = 30;
int8_t i8_STOP = 0;
int8_t i8_facteur =3;
bool connected = false;
bool connected2 = false;
char frame[SIZE_FILTER];
int8_t indexFrame;

void MotorsSTOP(void);
void Forward_LEFT(void);
void Forward_RIGHT(void);
void Backward_DOWN(void);
void Forward_UP(void);
void Turn_RIGHT(void);
void Turn_LEFT(void);
void MotorsDirection1(void);
void MotorsDirection2(void);
void MotorANALOG(int8_t i8DutyCycle1, int8_t i8DutyCyvle2);



void setup()
{
    Serial.begin(115200);
     
    SerialBT.begin("ESP32 IMU master",true); 

    /*Led to know connexion was good*/
    pinMode(PIN_LED,OUTPUT);
    digitalWrite(PIN_LED,LOW);

    myMotors = new MotorManager();

    myMotors->Begin(PIN_DIR1_1,PIN_DIR2_1,PIN_PWM1_1,PIN_PWM2_1,PIN_DIR1_2, PIN_DIR2_2, PIN_PWM1_2, PIN_PWM2_2);

    frame[0] = 'x';
    frame[1] = 0;
    frame[2] = 'y';
    frame[3] = 0;
    frame[4] = '\r';
    frame[5] = '\n';

    Serial.println("Ready.");
}

void loop()
{
    digitalWrite(PIN_LED,LOW);
    MotorsSTOP();

    if(false == connected){        
        connected = SerialBT.connect("ESP32 IMU HAND");
        Serial.println("waiting the conexion 1");
    }

    while(connected){
           
        digitalWrite(PIN_LED,HIGH);

        if (SerialBT.available()) {
            char incommingData = SerialBT.read();

            if('S'==incommingData){
                indexFrame = 0;
            }
            else{
                if(indexFrame < 7)
                    indexFrame++;
                if(1==indexFrame)
                    frame[indexFrame] = incommingData;
                if(3==indexFrame)
                    frame[indexFrame] = incommingData;
            }

            switch (incommingData)
            {
                case 'S':
                    frame[0] = 'x';
                break;
                case 'T':
                    frame[1] = 0;

                break;
                case 'O':
                    frame[2] = 'y';
                break;
                case 'P':
                    frame[3] = 0;
                break;

                break;
                default:

                break;
            }



            Serial.print(frame[0]);
            Serial.print((int8_t)frame[1]);
            Serial.print(frame[2]);
            Serial.print((int8_t)frame[3]);
            Serial.print(frame[4]);
            Serial.print(frame[5]);

            /*Conversion*/
            int8_t i8DataCycle1 = (int8_t)frame[1];
            
            int8_t i8DataCycle2 = (int8_t)frame[3];

            MotorANALOG((i8DataCycle1)*1.5, (i8DataCycle2)*1.5);

        }
        else{
            
            if(!SerialBT.connected(100)){
                connected = false;
                Serial.println("disconnexion Bluetooth");
            }
        }
     
        
       
    }
 

}


void Forward_LEFT(void){
   
    //myMotors->PWM1(i8_STOP);
    myMotors->PWM2(i8DutyCycle);     /*forward LEFT*/
    myMotors->PWM3(i8DutyCycle);    /*forward LEFT*/
    //myMotors->PWM4(i8_STOP);
}

void Forward_RIGHT(void){

    //myMotors->PWM1(i8_STOP);
    myMotors->PWM2(-i8DutyCycle);    /* forward RIGHT */
    myMotors->PWM3(-i8DutyCycle);     /* forward RIGHT */
    //myMotors->PWM4(i8_STOP);
}

void Backward_DOWN(void){
   
    myMotors->PWM1(i8DutyCycle);      /* backward DOWN*/
    //myMotors->PWM2(i8_STOP);
    //myMotors->PWM3(i8_STOP);
    myMotors->PWM4(i8DutyCycle);     /* backward DOWN*/
}

void Forward_UP(void){
   
    myMotors->PWM1(-i8DutyCycle);    /* forward UP */
    //myMotors->PWM2(i8_STOP);
    //myMotors->PWM3(i8_STOP);
    myMotors->PWM4(-i8DutyCycle);      /* forward UP */
}

void Turn_RIGHT(void){
   
    /* TURN RIGHT */
    myMotors->PWM1(-i8DutyCycle);       /* forward UP */
    myMotors->PWM2(i8DutyCycle);        /*forward LEFT*/
    myMotors->PWM3(-i8DutyCycle);       /* forward RIGHT */
    myMotors->PWM4(i8DutyCycle);        /* backward DOWN*/
}

void Turn_LEFT(void){
   
    /* TURN LEFT */
    myMotors->PWM1(i8DutyCycle);        /* backward DOWN*/
    myMotors->PWM2(-i8DutyCycle);       /* forward RIGHT */
    myMotors->PWM3(i8DutyCycle);        /*forward LEFT*/
    myMotors->PWM4(-i8DutyCycle);       /* forward UP */
}

void MotorsDirection1(void){
   
    /* TURN LEFT */
    myMotors->PWM1(-i8DutyCycle);        /* forward UP */   
    myMotors->PWM2(-i8DutyCycle);       /* forward RIGHT */
    myMotors->PWM3(-i8DutyCycle);        /* forward RIGHT */
    myMotors->PWM4(-i8DutyCycle);       /* forward UP */
}

void MotorsDirection2(void){
   
    /* TURN LEFT */
    myMotors->PWM1(i8DutyCycle);        /* backward DOWN*/
    myMotors->PWM2(i8DutyCycle);       /*forward LEFT*/
    myMotors->PWM3(i8DutyCycle);        /*forward LEFT*/
    myMotors->PWM4(i8DutyCycle);       /* backward DOWN*/
}
void MotorsSTOP(void){
   
    myMotors->PWM1(i8_STOP);
    myMotors->PWM2(i8_STOP);
    myMotors->PWM3(i8_STOP);
    myMotors->PWM4(i8_STOP);

}

void MotorANALOG(int8_t i8DutyCycle1, int8_t i8DutyCyvle2){
        myMotors->PWM1(i8DutyCyvle2);
        myMotors->PWM2(i8DutyCycle1);    /* forward RIGHT */
        myMotors->PWM3(i8DutyCycle1);     /* forward RIGHT */
        myMotors->PWM4(i8DutyCyvle2);
}