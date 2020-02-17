
//==============================================================================//
//                                                                              //
//  Alerto! - Portable and Connected Motion Alert Device                         //
//                                                                              //
//  Filename : Motion-Alert-Device.ino                                          //
//  Description : Arduino sketch file.                                          //
//  Author : Vishnu M Aiea                                                      //
//  Version : 0.1                                                               //
//  Initial Release : -                                                         //
//  License : MIT                                                               //
//                                                                              //
//  Last Modified : 11:16 PM 17-02-2020, Monday                                 //
//                                                                              //
//==============================================================================//

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"

uint32_t pulseCount = 0;

//==============================================================================//

void setup() {
  Serial.begin(115200);
  Serial.println("KEMET PIR Test");
  pinMode(2, OUTPUT);
  pinMode(4, INPUT);
  attachInterrupt(4, countUp, RISING);
}

//==============================================================================//

void loop() {

}

//==============================================================================//

void countUp() {
  pulseCount++;
  Serial.println(pulseCount);
}