
//==============================================================================//
//                                                                              //
//  Alerto! - Portable and Connected Motion Alert Device                        //
//                                                                              //
//  Filename : Motion-Alert-Device.ino                                          //
//  Description : Arduino sketch file.                                          //
//  Author : Vishnu M Aiea                                                      //
//  Version : 1.0.2                                                             //
//  Initial Release : - 04:17 PM 02-03-2020, Monday                             //
//  Project page : https://www.hackster.io/vishnumaiea/f7323c                   //
//  License : MIT                                                               //
//                                                                              //
//  Last Modified : 04:37 PM 28-03-2020, Saturday                               //
//                                                                              //
//==============================================================================//

//due to time constraints I could not include all documentation in the project
//page, like setting time etc. hope my comments here can help you understand
//the code

//==============================================================================//
//includes

#include <WiFi.h>
#include <ISL1208_RTC.h>  //search on Arduino IDE for this library and install it
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

//==============================================================================//
//defines

#define TIME_WINDOW       1000  //time window within which multiple pulses are ignored
#define PULSE_THRESHOLD   2     //how many pulses should be counted within the time window
#define MOTION_THRESHOLD  20    //how many motion you want to detect before triggering notification event

//serial ports
#define gpsPort Serial1 //hardware serial port for GPS
#define gsmPort Serial2 //hardware serial port for GSM
#define debugPort Serial  //this is the ESP32's default serial port

//gsm specific parameters
#define DEFAULT_GSM_BAUD 115200 //gsm module baud rate
#define CRLF_ON         //comment if you do not want to terminate commands with <CRLF>
#define CARRIAGE_RETURN 0x0D
#define NEWLINE         0x0A
#define CRLF_STRING     String("\r\n")
#define CRLF            "\r\n"

#define OP_OK        100  //operation okay
#define OP_NOTOK     101  //operation not okay
#define OP_TIMEOUT   103  //operation times out

//debug flags
#define GPS_DEBUG
#define GSM_DEBUG
#define RTC_DEBUG
#define WIFI_DEBUG
#define GLOBAL_DEBUG

//pins
#define BATTERY_PIN 19
#define SWITCH_PIN  5
#define LED_R_PIN 0
#define LED_G_PIN 12
#define LED_B_PIN 13
#define PIR_PIN 4
#define REED_PIN 18

//led states
#define ON 1
#define OFF 0

//led colors
#define R 1 //red
#define G 2 //green
#define B 3 //blue
#define RGB 4

//==============================================================================//
//globals

uint32_t pulseCount = 0;  //all pulses coming out from PIR
uint32_t pulseCountTotal = 0; //total pulses detected
uint32_t motionCount = 0;  //all pulses coming out from PIR
uint32_t motionCountTotal = 0; //total pulses detected
uint32_t lastEvent = 0; //last time point a pulse was detected
uint32_t elapsedTime = 0; //returned by millis()

//-----------------------------------------------------------------------------//
//create a custom scenario in Pushing box and paste the deviceID and API

String deviceId = "vC75E90DC5352BFE";
const char* logServer = "api.pushingbox.com";

//-----------------------------------------------------------------------------//
//Wi-Fi credentials

const char* ssid = "Magnimous";
const char* password = "Magcompany1$";

//-----------------------------------------------------------------------------//

//RTC object
ISL1208_RTC myRtc = ISL1208_RTC();

//hardware UART ports of ESP32
HardwareSerial Serial1(1);
HardwareSerial Serial2(2);

//gsm global variables
String gsmReply = ""; //response from GSM module
int gsmBaud = DEFAULT_GSM_BAUD; //GSM module baud rate
bool simStatus = false;   //if SIM available or not

String locationString = "NULL"; //current location in latitude and longitude
String lastLocationString = "NULL"; //last known location
bool locationSet = false; //whether a location fix was acquired
bool notificationTrigger = false;
bool smsNotification = false;
bool enableNotification = true; //enable or disable motion notification
bool enableGlobalLED = false;
uint8_t ledArray[3] = {0};  //RGB led status array

//==============================================================================//
//function protos

void IRAM_ATTR countUp();

//==============================================================================//
//setup function runs once

void setup() {
  debugPort.begin(115200);
  Wire.begin(); //initialize I2C
  myRtc.begin();  //initialize RTC

  debugPort.println();
  debugPort.println("===========================================");
  debugPort.println("                Alerto!");
  debugPort.println("-------------------------------------------");
  debugPort.println("Portable and Connected Motion Alert Device");
  debugPort.println("===========================================");
  debugPort.println();
  debugPort.println("Type \"cmd\" to enter command mode.");

  pinMode(BATTERY_PIN, INPUT);  //1.5V battery input
  pinMode(PIR_PIN, INPUT);  //PIR input
  pinMode(SWITCH_PIN, INPUT);  //Switch
  pinMode(LED_R_PIN, OUTPUT); //LED
  pinMode(LED_G_PIN, OUTPUT); //LED
  pinMode(LED_B_PIN, OUTPUT); //LED
  pinMode(REED_PIN, INPUT); //reed switch

  gpsPort.begin(9600, SERIAL_8N1, 14, 15);  //RX, TX
  gsmPort.begin(DEFAULT_GSM_BAUD, SERIAL_8N1, 16, 17);  //RX, TX

  led(G, OFF);  //turn off green and blue LEDs
  led(B, OFF);

  //check statuses of battery and spdt switch
  if(digitalRead(BATTERY_PIN) == HIGH) {  //check if secondary AA battery is present
    debugPort.println("Battery not found. LED_R will be OFF.");
    enableGlobalLED = false;  //disable all LEDs
    led(RGB, OFF);
  }
  else {
    if(digitalRead(SWITCH_PIN) == LOW)  {
      debugPort.println("Battery found. Switch is ON. LED_R is ON.");
      enableGlobalLED = true; //enable LEDs
      led(R, ON);
      led(G, OFF);
      led(B, OFF);
    }
    else {
      debugPort.println("Battery found. Switch is OFF. LED_R is OFF.");
      enableGlobalLED = false;  //disable LEDs
      led(RGB, OFF);
    } 
  }

  //check status of reed switch
  if(digitalRead(REED_PIN) == LOW) {
    enableNotification = false;
    debugPort.println();
    debugPort.println("Notifications are disabled.");
  }
  else {
    debugPort.println();
    debugPort.println("Notifications are enabled.");
  }

  //to set time use the command mode by sending "cmd"
  //"settime T20022810304213# "will set time
  if(myRtc.isRtcActive()) {
    debugPort.println();
    debugPort.println("RTC found on the bus.");
    myRtc.setTime("T20022810304213#");  //sample time update string
  }

  debugPort.println();
  debugPort.println("Initializing GSM module..");
  debugPort.println();
  gsmInitialize();


  //create freeRTOS tasks
  xTaskCreatePinnedToCore(
                    gpsTask,   //Function to implement the task
                    "gpsTask", //Name of the task
                    10000,        //Stack size in words
                    NULL,         //Task input parameter
                    0,            //Priority of the task
                    NULL,         //Task handle.
                    0);           //Core where the task should run

  xTaskCreatePinnedToCore(
                    gsmTask,   //Function to implement the task
                    "gsmTask", //Name of the task
                    10000,        //Stack size in words
                    NULL,         //Task input parameter
                    0,            //Priority of the task
                    NULL,         //Task handle.
                    0);           //Core where the task should run

  //attach interrupts
  attachInterrupt(PIR_PIN, countUp, RISING);  //isr is executed for all rising edge of pulses
  // attachInterrupt(SWITCH_PIN, switchInterrupt, CHANGE);
  // attachInterrupt(BATTERY_PIN, batteryInterrupt, CHANGE);
  // attachInterrupt(REED_PIN, reedInterrupt, CHANGE);
}

//==============================================================================//
//infinite loop

void loop() {
  //check if motion status is above the threshold
  if(motionCount >= MOTION_THRESHOLD) {
      debugPort.println("Motion activity detected.");
      debugPort.print("Total Activity Detected : ");
      debugPort.println(pulseCountTotal);
    if(enableNotification) {  //only when enabled, or reed switch is open
      detachInterrupt(PIR_PIN); //do not detect motion while proceesing is going on
      debugPort.println();
      debugPort.println("Notifications are enabled.");
      debugPort.println("Sending notification..");
      debugPort.print("Message : ");
      debugPort.print(String(pulseCount));
      debugPort.print(", ");
      debugPort.print(myRtc.getTimeDateDayString());
      debugPort.print(", ");
      debugPort.print(lastLocationString);
      debugPort.println();
      debugPort.println();
      int status = sendNotification();  //send notification via internet

      if(status != 0) { //check is it was a success
        debugPort.println();
        debugPort.println("Sending notification failed!");
        debugPort.println("Trying to send SMS..");
        smsNotification = true; //send sms if it was not
      }
      motionCount = 0; //reset pulse count so that we can count from fresh
      attachInterrupt(PIR_PIN, countUp, RISING);  //re-attach interrupt
    }
    else {  //do nothing if notification are disabled
      debugPort.println("Notifications are disabled.");
    }
  }

  //------------------------------------------------------------------------------//
  //check serial ports for command inputs

  if(debugPort.available() > 0) {
    bool cmdMode = false;
    String inputString = "";

    inputString = debugPort.readString();  //read the contents of serial buffer as string
    if(inputString.indexOf("cmd") > -1) { //check is it does have "cmd" in it
      detachInterrupt(PIR_PIN); //no need to detect motion in command mode
      debugPort.println();
      debugPort.println("Command mode activated.");
      inputString = "";
      cmdMode = true;
    }

    //continuously monitor serial port
    while(cmdMode) {  //until when command mode is deactivated
      //this is to feed the watchdog timer
      TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
      TIMERG0.wdt_feed=1;
      TIMERG0.wdt_wprotect=0;

      if(debugPort.available() > 0) {
        String commandString = "";  //initialize strings
        String firstParam = "";
        String secondParam = "";
        String thirdParam = "";

        inputString = debugPort.readString();
        debugPort.println();
        debugPort.print("Input String : ");
        debugPort.println(inputString);

        //-------------------------------------------------------------------------//
        //the follwing loop extracts the commands and parameters separated by whitespace

        uint8_t posCount = 0;  //the position token of each whitespace
        int indexOfSpace = 0;  //locations of the whitespaces

        while(inputString.indexOf(" ") != -1) { //loop until all whitespace chars are found
          indexOfSpace = inputString.indexOf(" ");  //get the position of first whitespace
          if(indexOfSpace != -1) {  //if a whitespace is found
            if(posCount == 0) //the first one will be command string
              commandString = inputString.substring(0, indexOfSpace); //end char is exclusive
            if(posCount == 1) //second will be second param
              firstParam = inputString.substring(0, indexOfSpace);
            if(posCount == 2) //and so on
              secondParam = inputString.substring(0, indexOfSpace);
            else if(posCount == 3)
              thirdParam = inputString.substring(0, indexOfSpace);
            inputString = inputString.substring(indexOfSpace+1);  //trim the input string
            posCount++;
          }
        }

        //saves the last part of the string if no more whitespace is found
        if(posCount == 0) //means there's just the command
          commandString = inputString;
        if(posCount == 1)
          firstParam = inputString;
        if(posCount == 2)
          secondParam = inputString;
        if(posCount == 3)
          thirdParam = inputString;

        debugPort.print("Command string = ");
        debugPort.println(commandString);

        if(firstParam.length() > 0) {  //only print if there's a valid first parameter
          debugPort.print("First param = ");
          debugPort.println(firstParam);
        }

        if(secondParam.length() > 0) {  //same for other parameters
          debugPort.print("Second param = ");
          debugPort.println(secondParam);
        }

        if(thirdParam.length() > 0) {
          debugPort.print("Third param = ");
          debugPort.println(thirdParam);
        }

        debugPort.println();

        //-------------------------------------------------------------------------//
        //commands and their actions
        //add any of your custom commands here
        //commands and parameters are separated by whitespace
        //you can send up to 3 parameters

        if(commandString == "exit") {
          debugPort.println("Exiting command mode.");
          cmdMode = false;
        }

        //prints time
        else if(commandString == "time") {
          debugPort.println(myRtc.getTimeDateDayString());
        }

        //sets time. send valid time string as first parameter
        else if(commandString == "settime") {
          myRtc.setTime(firstParam);
        }

        else if(commandString == "gps") {
          debugPort.print("Current Location : ");
          debugPort.println(locationString);
          debugPort.print("Last Known Location : ");
          debugPort.println(lastLocationString);
        }

        //test the push notification
        else if(commandString == "push") {
          sendNotification();
        }

        //test sms
        else if(commandString == "sms") {
          if(firstParam.length() > 0) {
            sendSMS(firstParam);
          }
          else {
            sendSMS(String("Test SMS from Alerto! ") + String(millis()));
          }
        }
        
        //turns on the red LED. 1 = ON,, everything else = OFF
        else if(commandString == "ledr") {
          if(firstParam == "1") {
            led(R, ON);
          }
          else {
            led(R, OFF);
          }
        }

        //turns on the green LED. 1 = ON,, everything else = OFF
        else if(commandString == "ledg") {
          if(firstParam == "1") {
            led(G, ON);
          }
          else {
            led(G, OFF);
          }
        }

        //turns on the blue LED. 1 = ON,, everything else = OFF
        else if(commandString == "ledb") {
          if(firstParam == "1") {
            led(B, ON);
          }
          else {
            led(B, OFF);
          }
        }

        //override global LED flag
        else if(commandString == "enableled") {
          debugPort.println("Global LED notifications are enabled.");
          enableGlobalLED = true;
        }

        //unknown commands
        else {
          debugPort.println("Unknown command. Send \"exit\" command mode.");
          inputString = "";
        }
      }
    }
    attachInterrupt(PIR_PIN, countUp, RISING);  //re-enable interrupt at exit
  }
}

//==============================================================================//
//counts the pulses from PIR at every rising edge

void IRAM_ATTR countUp() {
  pulseCountTotal++;

  if(pulseCount == 0) {   //start of a new detection window
    lastEvent = millis(); //save the starting point
  }

  if((millis() - lastEvent) > TIME_WINDOW) {  //if the detection window overruns
    pulseCount = 0; //reset pulse count
    lastEvent = 0;
  }
  else {  //if the new pulse arrives withing the detection time window
    pulseCount++;
    
    if(pulseCount >= PULSE_THRESHOLD) { //check how many pulses are detected within time window
      motionCount++;
      motionCountTotal++;

      led(R, OFF);  //blink the green LED
      led(G, ON);
      unsigned long int startTime = millis(); //generate delay
      while((millis() - startTime) < 100) {
      }
      led(G, OFF);
      led(R, ON);
      debugPort.println(motionCount);
      pulseCount = 0; //so that we can start a new detection window
    }
  }
}

//==============================================================================//
//turns LEDs ON/OFF
//color can be R, G, B, or RGB
//status can be ON or OFF

void led(int color, int status) {
  if(enableGlobalLED) {
    // ledArray[0] = 0;
    // ledArray[1] = 0;
    // ledArray[2] = 0;

    if(color == RGB) {
      ledArray[0] = status;
      ledArray[1] = status;
      ledArray[2] = status;

      if(status == ON) {
        digitalWrite(LED_R_PIN, LOW);
        digitalWrite(LED_G_PIN, LOW);
        digitalWrite(LED_B_PIN, LOW);
      }
      else {
        digitalWrite(LED_R_PIN, HIGH);
        digitalWrite(LED_R_PIN, HIGH);
        digitalWrite(LED_R_PIN, HIGH);
      }
      return;
    }

    if(digitalRead(LED_R_PIN) == HIGH) {
      ledArray[0] = 0;
    }
    else {
      ledArray[0] = 1;
    }

    if(digitalRead(LED_G_PIN) == HIGH) {
      ledArray[1] = 0;
    }
    else {
      ledArray[1] = 1;
    }

    if(digitalRead(LED_B_PIN) == HIGH) {
      ledArray[2] = 0;
    }
    else {
      ledArray[2] = 1;
    }

    if(color == R) {
      if(status == ON) {
        ledArray[0] = 1;
      }
      else {
        ledArray[0] = 0;
      }
    }

    if(color == G) {
      if(status == ON) {
        ledArray[1] = 1;
      }
      else {
        ledArray[1] = 0;
      }
    }

    if(color == B) {
      if(status == ON) {
        ledArray[2] = 1;
      }
      else {
        ledArray[2] = 0;
      }
    }

    if(ledArray[0] == ON) {
      digitalWrite(LED_R_PIN, LOW);
    }
    else {
      digitalWrite(LED_R_PIN, HIGH);
    }

    if(ledArray[1] == ON) {
      digitalWrite(LED_G_PIN, LOW);
    }
    else {
      digitalWrite(LED_G_PIN, HIGH);
    }

    if(ledArray[2] == ON) {
      digitalWrite(LED_B_PIN, LOW);
    }
    else {
      digitalWrite(LED_B_PIN, HIGH);
    }
  }
  else {
    if(status != OFF) {
      debugPort.println("Global LED notifications are disabled.");
    }
    digitalWrite(LED_R_PIN, HIGH);
    digitalWrite(LED_G_PIN, HIGH);
    digitalWrite(LED_B_PIN, HIGH);
  }

}

//==============================================================================//
//triggered when reed switch state is changed, or when you bring a magnet near to the reed switch

void reedInterrupt() {
  vTaskDelay(100 / portTICK_PERIOD_MS); //wait for 100ms
  if(digitalRead(REED_PIN) == LOW) {  //this is an active low input
    enableNotification = false;
    debugPort.println();
    debugPort.println("Notifications are disabled.");
  }
  else {
    enableNotification = true;
    debugPort.println();
    debugPort.println("Notifications are enabled.");
  }
}

//==============================================================================//
//spdt switch interrupt
//turning this switch ON simply turns on red LED and all other LED notifications
//the intruder would mistake it for a power ON/OFF switch

void switchInterrupt() {
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  if(digitalRead(SWITCH_PIN) == LOW) {  //active low
    debugPort.println("Switch turned ON.");
    if(digitalRead(BATTERY_PIN) == LOW) {  //if battery is present
      debugPort.println("LED is ON.");
      enableGlobalLED = true;
      led(R, ON); //turn on LED
    }
    else {
      debugPort.println("No battery. LED is off.");
      enableGlobalLED = false;
      led(R, OFF); //else keep it off
    }
  }
  else {
    debugPort.println("Switch turned OFF.");
    debugPort.println("LED is OFF.");
    enableGlobalLED = false;
    led(R, OFF);
  }
}

//==============================================================================//
//triggered when AA secondary battery is inserted or removed

void batteryInterrupt() {
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  if(digitalRead(BATTERY_PIN) == LOW) { //active low
    enableGlobalLED = true; //enable all LEDs
    if(digitalRead(SWITCH_PIN) == LOW) {
      debugPort.println("Switch is ON.");
      debugPort.println("LED is ON.");
      enableGlobalLED = true;
      led(R, ON);
    }
    else {
      debugPort.println("Switch is off.");
      debugPort.println("LED is off.");
      enableGlobalLED = false;
      led(R, OFF);
    }
  }
  else {
    enableGlobalLED = false;
    debugPort.println("Battery removed.");
    debugPort.println("LED is off.");
    led(R, OFF);
  }
}

//==============================================================================//
//infinite loop for processing AT commands

void gsmTask(void* pvParameters) {
  while(1) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if(smsNotification) { //send SMS when notification is triggered
      String smsString;
      smsString = "Alerto! ";
      smsString += String(pulseCount);
      smsString += " motion detected : ";
      smsString += myRtc.getTimeDateDayString();
      smsString += ", ";
      smsString += lastLocationString;
      if(sendSMS(smsString)) {
        debugPort.println("SMS has been sent");
        debugPort.println("Message : ");
        debugPort.println(smsString);
      }
      else {
        // debugPort.println("Sending SMS failed");
        debugPort.println("Message : ");
        debugPort.println(smsString);
      }
      smsNotification = false;
    }
  }
}

//==============================================================================//
//read location coordinates from GPS module

void gpsTask(void* pvParameters) {
  char gpsBuffer[600] = {0};

  while(1) {  //infinite loop
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if(gpsPort.available()) {
      gpsPort.readBytes(gpsBuffer, 600);  //read 600 bytes
      String gpsString = String(gpsBuffer);
      int gprmcIndex = gpsString.indexOf("GPRMC");  //find GPRMC header
      if(gprmcIndex > -1) {
        int delimiterIndex = gpsString.indexOf('*', gprmcIndex);  //extract GPRMC sentence
        gpsString = gpsString.substring(gprmcIndex, delimiterIndex);
        if(gpsString.endsWith("A")) { //if the sentence has valid coordinates
          int commaPos[12] = {0};
          int commaCount = 0;
          for(int i=0; i<gpsString.length(); i++) { //count commas
            if(gpsString[i] == ',') {
              commaPos[commaCount] = i;
              commaCount++;
            }
          }
          locationString = gpsString.substring(commaPos[2]+1, commaPos[6]); //extract latitude and longitude
          lastLocationString = locationString;  //update location strings
          locationSet = true;
        }
        else {
          locationSet = false;
          locationString = "NA";
        }
      }
    }
  }
}

//==============================================================================//
//connects to Wi-Fi and sends notification to the phone

int sendNotification(){
  led(R, OFF);
  delay(100);
  led(B, ON);
  debugPort.println("Connecting to Home Router SID: " + String(ssid));
  
  int retryCount = 0;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    led(B, ON);
    delay(500);
    led(B, OFF);
    retryCount++;
    if(retryCount >= 10) {  //up to 10 retries
      debugPort.println("Could not connect to Wi-Fi.");
      led(B, OFF);
      delay(300);
      led(B, ON);
      delay(300);
      led(B, OFF);
      delay(300);
      led(B, ON);
      delay(300);
      led(B, OFF);
      return 200;
    }
    debugPort.print(".");
  }
  delay(200);
  //some flash sequence
  led(B, OFF);
  led(G, ON);
  delay(100);
  led(G, OFF);
  delay(100);
  led(G, ON);
  delay(100);
  led(G, OFF);
  delay(100);
  led(G, ON);
  delay(100);
  led(G, OFF);

  debugPort.println();
  debugPort.println("Successfully connected");
  debugPort.println("Starting client");
  
  WiFiClient client;

  //connect to server
  debugPort.println("Connecting to pushing server: " + String(logServer));
  if (client.connect(logServer, 80)) {
    debugPort.println("Successfully connected");
    
    //prepare message string
    String postStr = "devid=";
    postStr += String(deviceId);
    postStr += "&motion_count=";
    postStr += String(pulseCount);
    postStr += "&total_motion_count=";
    postStr += String(pulseCountTotal);
    postStr += "&time_string=";
    postStr += String(myRtc.getTimeDateDayString());
    postStr += "&location_string=";
    postStr += String(lastLocationString);
    postStr += "\r\n\r\n";
    
    debugPort.println("Sending data...");
    
    client.print("POST /pushingbox HTTP/1.1\n");
    client.print("Host: api.pushingbox.com\n");
    client.print("Connection: close\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);
  }
  else {
    debugPort.println("Could not connect to server.");
    return 201;
  }

  client.stop();
  debugPort.println("Stopping the client");

  //another flashing sequence
  led(B, ON);
  led(G, OFF);
  delay(300);
  led(B, OFF);
  led(G, ON);
  delay(300);
  led(B, ON);
  led(G, OFF);
  delay(300);
  led(B, OFF);
  led(G, ON);
  delay(300);
  led(B, OFF);
  led(G, OFF);
  delay(500);
  led(R, ON);

  return 0;
}

//==============================================================================//
//initializes GSM module

int gsmInitialize() {
  int initStatus = -1;
  do {
    initStatus = gsmBegin();
    if(initStatus != OP_OK) {
      debugPort.print("Error initializing GSM module - ");
      debugPort.println(initStatus);
      debugPort.println("Retrying...");
    }
    delay(1000);
  }
  while(initStatus != OP_OK);

  if(initStatus == OP_OK) {
    debugPort.println("GSM module initialized.");
    debugPort.println();
    // debugPort.println("Waiting for command");
    // debugPort.println();

    if(gsmCommand("AT+CPIN?", "OK", "READY", 4000, 1) != OP_OK) {
      debugPort.println("SIM card is not available.");
      simStatus = false;
    }
    else {
      debugPort.println("SIM card available.");
      simStatus = true;
    }
    return OP_OK;
  }

  return OP_OK;
}

//==============================================================================//
//initialize the GSM module

int gsmBegin() {
  gsmCommand("AT", "OK", 2000, 3);
  gsmCommand("AT", "OK", 2000, 3);
  // gsmCommand("AT&F0", "OK", 3000, 2);
  gsmCommand("AT+CREG?", "OK", 3000, 3);
  // int networkStatus = gsmWaitfor("0,", "1,", 1500);

  // while (networkStatus != OP_OK) {
  //   gsmSend("AT+CREG?");
  //   networkStatus = gsmWaitfor("0,", "1,", 1500);
  // }

  if (gsmCommand("AT+CMEE=2", "OK", 5000, 2) == OP_OK) {  //detailed error messages
    if (gsmCommand("ATE0&W", "OK", 5000, 2) == OP_OK) // disable Echo
      return OP_OK;  // enable better error messages
    else
      return OP_NOTOK;
  }

  // if (gsmCommand("AT&F0", "OK", "yy", 5000, 2) == OP_OK) {   // Reset to factory settings
  //   if (gsmCommand("ATE0", "OK", "yy", 5000, 2) == OP_OK) {  // disable Echo
  //     if (gsmCommand("AT+CMEE=2", "OK", "yy", 5000, 2) == OP_OK)
  //       return OP_OK;  // enable better error messages
  //     else
  //       return OP_NOTOK;
  //   }
  // }

  return OP_NOTOK;
}

//==============================================================================//
//send a string to the GSM module
//if CRLF_ON is defined, println() will be used
//print() otherwise

int gsmSend (String inputString) {
  #ifdef CRLF_ON
    gsmPort.println(inputString);
  #else
    gsmPort.print(inputString);
  #endif

  return OP_OK;
}

//==============================================================================//
//wait for a reply from GSM module

int gsmWaitfor (String response1, String response2, unsigned long timeOut) {
  unsigned long entryTime = millis(); //starting time
  int returnValue = 255;
  bool response1Found = false;
  bool response2Found = false;

  do {
    gsmReply = gsmRead();  //read the response from the GSM module if any
    if(gsmReply != "") { //if the reply is not empty
      if(findInResponse("ERROR") || findInResponse("+CME:")) {
        #ifdef GLOBAL_DEBUG && GSM_DEBUG
          debugPort.println();
          debugPort.print("Error (");
          debugPort.print(gsmReply.length());
          debugPort.println(") --");
          debugPort.print(gsmReply);
          debugPort.println("-- end");
          debugPort.println();
        #endif
        entryTime = 100;
      }
     
      else if(findInResponse(response1) && findInResponse(response2)) {
        #ifdef GLOBAL_DEBUG && GSM_DEBUG
          debugPort.print("Finished in ");
          debugPort.print((millis() - entryTime));
          debugPort.print(" ms -- ");
          debugPort.print(gsmReply);
          debugPort.println("-- end");
          debugPort.println();
        #endif
        response1Found = true;
        response2Found = true;
      }

      else if(findInResponse(response1)) {
        response1Found = true;

        #ifdef GLOBAL_DEBUG && GSM_DEBUG
          if(response1Found && response2Found) {
            debugPort.print("Finished in ");
            debugPort.print((millis() - entryTime));
            debugPort.print(" ms -- ");
            debugPort.print(gsmReply);
            debugPort.println("-- end");
            debugPort.println();
          }
          else {
            debugPort.print(gsmReply);
          }
        #endif
      }

      else if(findInResponse(response2)) {
        response2Found = true;

        #ifdef GLOBAL_DEBUG && GSM_DEBUG
          if(response1Found && response2Found) {
            debugPort.print("Finished in ");
            debugPort.print((millis() - entryTime));
            debugPort.print(" ms -- ");
            debugPort.print(gsmReply);
            debugPort.println("-- end");
            debugPort.println();
          }
          else {
            debugPort.print(gsmReply);
          }
        #endif
      }

      else {
        #ifdef GLOBAL_DEBUG && GSM_DEBUG
          debugPort.println();
          debugPort.print("Unsolicited (");
          debugPort.print(gsmReply.length());
          debugPort.println(") --");
          debugPort.print(gsmReply);
          debugPort.println("-- end");
          debugPort.println();
        #endif
      }
    }
  } while (((response1Found == false) || (response2Found == false)) && ((millis() - entryTime) < timeOut));

  // debugPort.print("GSM Reply = ");
  // debugPort.println(gsmReply);
  // debugPort.println("--");

  if ((millis() - entryTime) >= timeOut) {
    returnValue = OP_TIMEOUT;
  }
  else {
    if (response1Found && response2Found) { //indexOf returns -1 if a string is not found
      // debugPort.print("Index sum = ");
      // debugPort.println(gsmReply.indexOf(response1) + gsmReply.indexOf(response2));
      returnValue = OP_OK;
    }
    else {
      returnValue = OP_NOTOK;
    }
  }
  //  debugPort.print("retVal = ");
  //  debugPort.println(retVal);
  return returnValue;
}

//------------------------------------------------------------------------//

int gsmWaitfor (String response1, unsigned long timeOut) {
  unsigned long entryTime = millis(); //starting time
  int returnValue = 255;
  bool response1Found = false;

  do {
    gsmReply = gsmRead();  //read the response from the GSM module if any

    if(gsmReply != "") { //if the reply is not empty
      if(findInResponse("ERROR") || findInResponse("+CME:")) {
        #ifdef GLOBAL_DEBUG && GSM_DEBUG
          debugPort.println();
          debugPort.print("Error (");
          debugPort.print(gsmReply.length());
          debugPort.println(") --");
          debugPort.print(gsmReply);
          debugPort.println("-- end");
          debugPort.println();
        #endif
        entryTime = 100;
      }
      else if(findInResponse(response1)) {
        #ifdef GLOBAL_DEBUG && GSM_DEBUG
          debugPort.print("Finished in ");
          debugPort.print((millis() - entryTime));
          debugPort.print(" ms -- ");
          debugPort.print(gsmReply);
          debugPort.println("-- end");
          debugPort.println();
        #endif
        response1Found = true;
      }
      else {
        debugPort.println();
        debugPort.print("Unsolicited (");
        debugPort.print(gsmReply.length());
        debugPort.println(") --");
        debugPort.print(gsmReply);
        debugPort.println("-- end");
        debugPort.println();
      }
    }
  } while ((response1Found == false) && ((millis() - entryTime) < timeOut));

  // debugPort.print("GSM Reply = ");
  // debugPort.println(gsmReply);
  // debugPort.println("--");

  if ((millis() - entryTime) >= timeOut) {
    returnValue = OP_TIMEOUT;
  }
  else {
    if (gsmReply.indexOf(response1) > -1) { //indexOf returns -1 if a string is not found
      // debugPort.print("Index sum = ");
      // debugPort.println(gsmReply.indexOf(response1) + gsmReply.indexOf(response2));
      returnValue = OP_OK;
    }
    else {
      returnValue = OP_NOTOK;
    }
  }
  //  debugPort.print("retVal = ");
  //  debugPort.println(retVal);
  return returnValue;
}

//==============================================================================//
//send AT command

int gsmCommand (String command, String response1, String response2, unsigned long timeOut, int repetitions) {
  int returnValue = OP_NOTOK;
  uint8_t count = 0;

  while ((count < repetitions) && (returnValue != OP_OK)) {
    gsmSend(command);

    #ifdef GLOBAL_DEBUG && GSM_DEBUG
      debugPort.print("Command: ");
      debugPort.println(command);
    #endif

    if(gsmWaitfor(response1, response2, timeOut) == OP_OK) {
      // debugPort.println("OK");
      returnValue = OP_OK;
    }
    else {
      returnValue = OP_NOTOK;
    }
    count++;
  }

  return returnValue;
}

//------------------------------------------------------------------------//

int gsmCommand (String command, String response1, unsigned long timeOut, int repetitions) {
  int returnValue = OP_NOTOK;
  uint8_t count = 0;

  while ((count < repetitions) && (returnValue != OP_OK)) {
    gsmSend(command);

    #ifdef GLOBAL_DEBUG && GSM_DEBUG
      debugPort.print("Command: ");
      debugPort.println(command);
    #endif

    if (gsmWaitfor(response1, timeOut) == OP_OK) {
      // debugPort.println("OK");
      returnValue = OP_OK;
    }
    else {
      returnValue = OP_NOTOK;
    }
    count++;
  }

  return returnValue;
}

//==============================================================================//
//reads the reply from GSM module

String gsmRead() {
  if (gsmPort.available())  {
    String readString = gsmPort.readString();

    if(readString.indexOf("AST_POWERON") != -1) {
      #ifdef GLOBAL_DEBUG && GSM_DEBUG
        debugPort.println();
        debugPort.println("= = = = = = = = = = = = = = = =");
        debugPort.println("Module restarted. Reinitializing...");
        debugPort.println("= = = = = = = = = = = = = = = =");
        debugPort.println();
      #endif

      gsmInitialize();
      return "";
    }
    else {
      return readString;
    }
  }
  else {
    return "";
  }
}

//==============================================================================//
//get time string from GSM module

String getGSMTime () {
  if(gsmCommand("AT+CCLK?", "OK", 2000, 1) == OP_OK) {
    String timeString = gsmReply.substring((gsmReply.indexOf("\"")+1), (gsmReply.lastIndexOf("\"")));
    timeString.replace("/", "-");

    #ifdef GLOBAL_DEBUG && GSM_DEBUG
      debugPort.print("Time string : ");
      debugPort.println(timeString);
    #endif

    return timeString;
  }
  return "NULL";
}

//==============================================================================//
//send SMS text message

bool sendSMS(String smsString) {
  gsmCommand("AT+CMGF=1", "OK", 2000, 2); //Because we want to send the SMS in text mode
  delay(100);
  gsmCommand("AT+CMGS=\"+919567205051\"", ">", 3000, 2); //send sms message, be careful need to add a country code before the cellphone number
  delay(100);
  gsmPort.print(smsString);//the content of the message
  gsmPort.write(26);//the ASCII code of the ctrl+z is 26

  if(gsmWaitfor("+CMGS:", 15000) == OP_OK) {
    #ifdef GLOBAL_DEBUG && GSM_DEBUG
      debugPort.println("Sending SMS success!");
      debugPort.println();
    #endif
    return true;
  }
  #ifdef GLOBAL_DEBUG && GSM_DEBUG
    debugPort.println("Sending SMS failed!");
    debugPort.println();
  #endif
  return false;
}

//==============================================================================//
//search the GSM response for a string

bool findInResponse (String s) {
  if(gsmReply.indexOf(s) != -1)
    return true;
  else
    return false;
}

//==============================================================================//






