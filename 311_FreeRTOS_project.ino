//Filename: 311_FreeRTOS_project.ino
//Author: Shariq Moghees (OL41895)
//Date: 12/9/25
//Class: CMPE 311 Dr. Kidd Fall 2025
//Desc: A program that takes the Asynchronous LED + motor driver circuit from the last project and converts it into a 
// FreeRTOS task scheduling system using an imported Library instead of using a Cyclic Executive.
//------------------------------------------------------------------------------------------------------------------------

#include <Arduino_FreeRTOS.h>
#include "task.h"

#define PIN_LED1   13   //led 1
#define PIN_LED2   12   //led 2
#define PIN_PWM     9   //PWM signal will be coming from pin 9
#define PIN_BUTTON  2   // Using internal pull-up. active LOW push button

// global variables for LED and PWM dirver circuit tasks

volatile uint32_t intervalLed[3] = {0, 1000, 500};  // default intervals
volatile int ledOn1 = 0;
volatile int ledOn2 = 0;

volatile int brightness = 0;     // 0–5
volatile bool decreasing = false;

//TASK DECLARATIONS
void taskLED1(void *pv);
void taskLED2(void *pv);
void taskPWM(void *pv);
void taskButton(void *pv);
void taskSerial(void *pv);

//SETUP
void setup() {

  Serial.begin(9600);

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP); //internal arduino pull-up resistor for button

  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  analogWrite(PIN_PWM, 0); //we initialize the Motor to be off initially (level 0)

  Serial.println("FreeRTOS Integrated System Ready");
  Serial.println("Format: <LED#> <interval_ms>");

  //creat the tasks using FreeRTOS library syntax
  xTaskCreate(taskLED1,   "LED1",   110, NULL, 1, NULL);
  xTaskCreate(taskLED2,   "LED2",   110, NULL, 1, NULL);
  xTaskCreate(taskPWM,    "PWM",    90,  NULL, 1, NULL);
  xTaskCreate(taskButton, "BTN",    90,  NULL, 2, NULL); //set the serial and button input tasks at a higher priority
  xTaskCreate(taskSerial, "SER",    180, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop() {} //loop dosnt need anything in it becasue FreeRTOS does all our task scheduling for us


//LED TASKS

void taskLED1(void *pv) {

  TickType_t lastWake = xTaskGetTickCount();
  pinMode(PIN_LED1, OUTPUT);

  for (;;) {
    ledOn1 = !ledOn1;
    digitalWrite(PIN_LED1, ledOn1);

    // EXACT periodic timing
    vTaskDelayUntil(&lastWake, intervalLed[1] / portTICK_PERIOD_MS);
  }
}

void taskLED2(void *pv) {

  TickType_t lastWake = xTaskGetTickCount();
  pinMode(PIN_LED2, OUTPUT);

  for (;;) {
    ledOn2 = !ledOn2;
    digitalWrite(PIN_LED2, ledOn2);

    vTaskDelayUntil(&lastWake, intervalLed[2] / portTICK_PERIOD_MS);
  }
}

//PWM TASK
void taskPWM(void *pv) {
  pinMode(PIN_PWM, OUTPUT);

  for (;;) {
    analogWrite(PIN_PWM, brightness * 51);  // 0–255
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

//PUSH BUTTON INPUT TASK

void taskButton(void *pv) {

  for (;;) {

    if (digitalRead(PIN_BUTTON) == LOW) {  // pressed (active LOW)

      if (!decreasing) { //step up
        brightness++;
        if (brightness >= 5) {
          brightness = 5;
          decreasing = true;
        }
      } else {    //step down
        brightness--;
        if (brightness <= 0) {
          brightness = 0;
          decreasing = false;
        }
      }

      Serial.print("Brightness: ");
      Serial.println(brightness);

      vTaskDelay(200 / portTICK_PERIOD_MS); // debounce/edge detect for the push button
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

//SERIAL INPUT TASK (for changing the asynchronous LED blinking rate)
//I'm using chars intead of strings here to reduce the amount of memory used. I was running into a memory issue with stack space and RTOS tasks.
void taskSerial(void *pv) {
  
  char buffer[16];
  uint8_t idx = 0;

  for (;;) {

    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        buffer[idx] = 0;  // null-terminate string

        int led = atoi(strtok(buffer, " "));
        int val = atoi(strtok(NULL, " "));

        if (led >= 1 && led <= 2 && val > 0) {
          intervalLed[led] = val;

          Serial.print("LED");
          Serial.print(led);
          Serial.print(" rate updated to ");
          Serial.print(val);
          Serial.println(" ms");
        }

        idx = 0;
      }
      else if (idx < 15) {
        buffer[idx++] = c;
      }
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}
