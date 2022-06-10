#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#define sensor1 2
#define sensor2 3
#define proximity  4
#define IN1	7
#define IN2	6
#define MAX_SPEED 255 
#define MIN_SPEED 0
#define Time_To_Open 3000 //ms
#define Waiting_Time 5000 //ms
#define SPEED 50

void door_state_checking(void* pvParameters);
void door_control(void* pvParameters);
void detecting(void* pvParameters);
void close_door(int);
void open_door(int);
void stop();

volatile bool closed = false;
volatile bool detected = false;

void setup() {
  pinMode(proximity, INPUT);  
  pinMode(5, OUTPUT);         // Enable pin for sensor 1-2
  digitalWrite(5, HIGH);
  pinMode(sensor1, INPUT);    
  pinMode(sensor2, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  xTaskCreate(door_state_checking, "door_state_checking", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(door_control, "door_control", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(detecting, "detecting", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  
}

void loop() {}

void door_state_checking(void* pvParameters) {
  (void) pvParameters;
  
  while(1) {
    while(digitalRead(proximity) == LOW) {
      closed = true;
    }
      closed = false;
  }
}

void detecting(void* pvParameters) {
  (void) pvParameters;
  
  while(1) {
    while(digitalRead(sensor1) == LOW || digitalRead(sensor2) == LOW) {
      detected = true;
    }
      detected = false;
  }
}

void door_control(void* pvParameters) {
  (void) pvParameters;

  while(1) {
    stop();
    if(detected){
      open_door(SPEED);
      vTaskDelay(Time_To_Open / portTICK_PERIOD_MS);
      stop();
    }
    while(detected){};
    while(!detected){
      if(!closed){
        vTaskDelay(Waiting_Time / portTICK_PERIOD_MS);
        close_door(SPEED);
        while(!closed){};
        stop();
      }else stop();
    }
  }
}

void stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void open_door(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(IN1, HIGH);// chân này không có PWM
	analogWrite(IN2, 255 - speed);
}

void close_door(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(IN1, LOW);// chân này không có PWM
	analogWrite(IN2, speed);
}