/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include "BNO080/BNO080.h"
#include "BluetoothSerial.h"
#include "ControlMotors/MotorManager.h"

/*******************************************************************************************************************************************
 *  												GLOBAL VARIABLES
 *******************************************************************************************************************************************/

#define SIZE_FILTER 6

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define IMU_CSPIN 32
#define IMU_WAKPIN 18
#define IMU_INTPIN 26
#define IMU_RSTPIN 27
#define IMU_SPISPEED 3000000
#define IMU_CLK 14
#define IMU_MISO 25
#define IMU_MOSI 13
#define IMU_PS0 18
#define IMU_PS1 12


//IMU instance
BNO080 *myIMU;
BluetoothSerial SerialBT;
char frame[SIZE_FILTER];

int8_t s8Roll;
int8_t s8Pitch;

// Task declaration
TaskHandle_t TaskCore0, TaskCore1;

// Timer declaration
hw_timer_t * timer0 = NULL;

portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

// Timer Flags
bool flagTimer0 = false;



void sendFRameBluetooth(int8_t s8Roll, int8_t s8Pitch);
int8_t ConvertFrom16To8_IMU(int16_t s16value);
/*******************************************************************************************************************************************
 *  												CORE LOOPS
 *******************************************************************************************************************************************/
// Loop of core 0
void LoopCore0( void * parameter ){

    while(true) {
    /****************************************************/
    /*      Manager TASK IMU*/
    /****************************************************/
    if(RC_e::SUCCESS == myIMU->Run()){  

        s8Roll = ConvertFrom16To8_IMU((int16_t)myIMU->m_Roll);
        s8Pitch = ConvertFrom16To8_IMU((int16_t)myIMU->m_Pitch);

        Serial.print("16Roll =");
        Serial.print(s8Roll);
        Serial.print(", 16Pitch =");
        Serial.println(s8Pitch);          


    } else {
        Serial.println("IMU not connected!!!");
    }


    delay(10);

    }
}

// Loop of core 1
void LoopCore1( void * parameter ){
    while(true) {
        sendFRameBluetooth(s8Roll, s8Pitch);
    delay(100);

    }
}
/*******************************************************************************************************************************************
 *  												TIMERS
 *******************************************************************************************************************************************/
// TIMER 0
void IRAM_ATTR onTimer0(){
    portENTER_CRITICAL_ISR(&timerMux0);

    flagTimer0 = true;

    portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup()
{
    Serial.begin(115200);

    SerialBT.begin("ESP32 IMU HAND"); //Bluetooth device name

    /*****************************************************************************************/
    /* Manager INIT IMU*/
    myIMU = new BNO080();
    myIMU->configure(IMU_CSPIN, IMU_WAKPIN, IMU_INTPIN, IMU_RSTPIN, IMU_SPISPEED, IMU_CLK, IMU_MISO, IMU_MOSI, DIRECT_ROLL, REVERSE_PITCH, REVERSE_YAW, IMU_PS0,IMU_PS1);
    myIMU->enableRotationVector(5);
    /*****************************************************************************************/

    Serial.println("Ready.");

    frame[0] = 'S';
    frame[1] = 'T';
    frame[2] = 'O';
    frame[3] = 'P';
    frame[4] = '\r';
    frame[5] = '\n';

      // Task of core 0
    xTaskCreatePinnedToCore(
        LoopCore0,  /* Function to implement the task */
        "LoopCore0",    /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        1,          /* Priority of the task */
        &TaskCore0, /* Task handle. */
        0);         /* Core where the task should run */

    delay(500);  // needed to start-up task1

    // Task of core 1
    xTaskCreatePinnedToCore(
        LoopCore1,  /* Function to implement the task */
        "LoopCore1",    /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        1,          /* Priority of the task */
        &TaskCore1, /* Task handle. */
        1);         /* Core where the task should run */

    // Timer0
    Serial.println("start timer 0");
    timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
    timerAttachInterrupt(timer0, &onTimer0, true); // edge (not level) triggered 
    timerAlarmWrite(timer0, 5000000, true); // 1000000 * 1 us = 1 s, autoreload true


    // Enable the timer alarms
    timerAlarmEnable(timer0); // enable


}

void loop()
{
    vTaskDelete(NULL);

}


void sendFRameBluetooth(int8_t s8Roll, int8_t s8Pitch){
    if(0 == s8Roll)  /*the value zero by bluetooth stop the comm*/
        s8Roll = 'T';  /*the value 0 => stop the motor*/
    
    if(0 == s8Pitch) /*the value zero by bluetooth stop the comm*/
        s8Pitch = 'P'; /*the value 0 => stop the motor*/
    frame[1] = s8Roll;
    frame[3] = s8Pitch;

    SerialBT.printf(frame);

    delay(200);
    SerialBT.flush();
    delay(200);
}

int8_t ConvertFrom16To8_IMU(int16_t s16value){
    
    int8_t value = 0;

    if(s16value <= -90)
    {
        value = -90;
    }
    else if (s16value > 90)
    {
        value = 90;
    }else{
        value =(int8_t)s16value;
    }

    return value;
}