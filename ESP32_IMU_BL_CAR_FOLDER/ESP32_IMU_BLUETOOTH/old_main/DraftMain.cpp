/*======================================================== INCLUDES =========================================================================*/
#include <Arduino.h>
#include "../lib/SBR_Global/Definition/GlobalDef.h"
#include "Ps3Controller.h"
/*======================================================= DEFINITION ========================================================================*/

/*======================================================== GLOBAL VARIABLES =================================================================*/

/*Test [JSA] to remove*/
#define LED_BUILTIN 2

// Task declaration
TaskHandle_t TaskCore0, TaskCore1;
// Timer declaration
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

// Timer Flags
bool flagTimer0 = false;




//======================================================== CORE LOOPS =======================================================================

// Loop of core 0
void LoopCore0( void * parameter ){
    while(true) {
        // Code for Timer 0 interruption
        if (flagTimer0){
            //Serial.printf("G1F2\r\n");

        }
        

        delay(1);

    }
}

// Loop of core 1
void LoopCore1( void * parameter ){
    while(true) {

        delay(1);
    }
}

//======================================================== TIMERS ===========================================================================

// TIMER 0
void IRAM_ATTR onTimer0(){
    portENTER_CRITICAL_ISR(&timerMux0);

    flagTimer0 = true;

    portEXIT_CRITICAL_ISR(&timerMux0);
}


//======================================================== SETUP ==============================================================================

void setup() {

    Serial.begin(115200);

    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("01:02:03:04:05:06");

    Serial.println("Ready.");
    // Serial Port
    Serial2.begin(9600);


    /*Test [JSA] to remove*/
    pinMode(LED_BUILTIN, OUTPUT);       // Config Output LED on board ESP32

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
    timerAlarmWrite(timer0, 1000000, true); // 1000000 * 1 us = 1 s, autoreload true

    // Enable the timer alarms
     timerAlarmEnable(timer0); // enable
}

//======================================================== MAIN LOOP ============================================================================
// Loop not used
void loop() {
    vTaskDelete(NULL);  
}
