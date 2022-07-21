#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include <FastLED.h>

#include "vector3f.h"

// #define CAN_ds

typedef int16_t md_data_typedef[4];

typedef struct{
  uint16_t state1;
  uint16_t state2;
  uint32_t delay_time;
}sol_pattern_typedef;

typedef struct{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
}RGB_typedef;

typedef uint8_t CAN_msg_data[8];

QueueHandle_t queueHandle_run;
QueueHandle_t queueHandle_md;
// QueueHandle_t queueHandle_sol;
QueueHandle_t queueHandle_solenoids[10];
QueueHandle_t queueHandle_triggers[5];    // 0 = spotA, 1 = spotB, 2 = leftrunway/baseA, 3 = rightrunway/baseA, 4 = baseB
QueueHandle_t queueHandle_led;

//task
void task_md_update(void *id) {
  int run = 0;
  md_data_typedef output = {0, 0, 0, 0};
  delay(50);
  while (1) {
    xQueueReceive(queueHandle_md,&output,100);
    xQueuePeek(queueHandle_run, &run,0);

    if(run){
      CAN.beginPacket(*(int*)id, 8);
      for (int i = 0; i < 4; i++) {
        CAN.write(output[i] & 0xFF);
        CAN.write(output[i] >> 8 & 0xFF);
      }
      CAN.endPacket();
      Serial.printf("MD updated\t id:%d\t 1:%d\t 2:%d\t 3:%d\t 4:%d\r\n", *(int*)id, output[0], output[1], output[2], output[3]);
    }

    delay(5);
  }
}

// void task_sol_update(void *id) {
//   int run = 0;
//   sol_pattern_typedef output = {0x0000, 0x0000, 0};
//   delay(50);
//   while (1) {
//     xQueueReceive(queueHandle_sol,&output,portMAX_DELAY);
//     xQueuePeek(queueHandle_run, &run,0);
//
//     if(run){
//       CAN.beginPacket(*(int*)id, 1);
//       CAN.write(0xFF & output.state1);
//       CAN.write(0xFF & output.state1 >> 8);
//       CAN.endPacket();
//       Serial.printf("sol updated\t id:%d\t data:%x\r\n", *(int*)id, output.state1);
//
//       delay(output.delay_time);
//
//       CAN.beginPacket(*(int*)id, 1);
//       CAN.write(0xFF & output.state2);
//       CAN.write(0xFF & output.state2 >> 8);
//       CAN.endPacket();
//       Serial.printf("sol updated\t id:%d\t data:%x\r\n", *(int*)id, output.state2);
//     }
//
//     delay(5);
//   }
// }

typedef struct{
  uint32_t id;
  QueueHandle_t &run;              //it must be int
  QueueHandle_t *solenoids;       //it must be unsigned int queue
  uint8_t num_solenoids;          //up to 10 solenoids
  uint32_t refreash_time;         //refreash time in ms
}solenoid_task_arg;

void solenoid_task(void *arg) {
  solenoid_task_arg params = *(solenoid_task_arg*)arg;
  int run = 0;
  unsigned int sol_state = 0;
  delay(50);
  while (1) {
    xQueuePeek(params.run, &run,0);
    if(run){
      for(int i = 0; i < params.num_solenoids; i++){
        unsigned int rcv_buf = 0;
        xQueueReceive(params.solenoids[i],&rcv_buf,0);
        sol_state |= rcv_buf << i;
      }
      CAN.beginPacket(params.id, 2);
      CAN.write(0xFF & sol_state);
      CAN.write(0xFF & sol_state >> 8);
      CAN.endPacket();
      Serial.printf("sol updated\t id:%d\t data:%x\r\n", params.id, sol_state);
    }

    delay(params.refreash_time);
  }
}

typedef struct{
  QueueHandle_t &trigger;          //it msut be int queue
  QueueHandle_t &solenoid1;
  QueueHandle_t &solenoid2;
  uint32_t t1;
  uint32_t t2;
  uint32_t t3;
}launch_task_arg;

void launcher_task(void *arg) {
  launch_task_arg params = *(launch_task_arg*)arg;
  while(1){
    int trigger = 0;
    xQueueReceive(params.trigger,&trigger,portMAX_DELAY);

    int s1 = 1;
    int s2 = 0;
    xQueueOverwrite(params.solenoid1, &s1);
    xQueueOverwrite(params.solenoid2, &s2);
    delay(params.t1);
    s1 = 0;
    s2 = 0;
    xQueueOverwrite(params.solenoid1, &s1);
    xQueueOverwrite(params.solenoid2, &s2);
    delay(params.t2);
    s1 = 0;
    s2 = 1;
    xQueueOverwrite(params.solenoid1, &s1);
    xQueueOverwrite(params.solenoid2, &s2);
    delay(params.t3);
    s1 = 0;
    s2 = 0;
    xQueueOverwrite(params.solenoid1, &s1);
    xQueueOverwrite(params.solenoid2, &s2);

    delay(5);
  }
}

void task_stamp_led(void *arg) {
  const uint8_t led_pin = 27;
  RGB_typedef rgb = {0, 0, 0};
  CRGB led[1];

  //LED init
  FastLED.addLeds<SK6812, led_pin, RGB>(led, 1); //GRB order
  led[0] = rgb.green << 16 | rgb.red << 8 | rgb.blue;
  FastLED.show();

  while(1){
    xQueueReceive(queueHandle_led, &rgb, portMAX_DELAY);
    led[0] = rgb.green << 16 | rgb.red << 8 | rgb.blue;
    FastLED.show();
    delay(50);
  }
}


void setup() {}

void loop() {
  const uint32_t MD_ID = 0x01;
  const uint32_t sol_ID = 0x06;

  TaskHandle_t taskHandle_md_update;
  // TaskHandle_t taskHandle_sol_update;
  TaskHandle_t taskHandle_solenoid[1];
  TaskHandle_t taskHandle_launcher[5];
  TaskHandle_t taskHandle_stamp_led;

  //enable core1 WTD
  enableCore1WDT();

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("reseted");

  //queue create
  queueHandle_run = xQueueCreate(1, sizeof(int));
  queueHandle_md  = xQueueCreate(1, sizeof(md_data_typedef));
  // queueHandle_sol = xQueueCreate(1, sizeof(sol_pattern_typedef));
  for(int i = 0; i < 10; i++) {
    queueHandle_solenoids[i] = xQueueCreate(1, sizeof(unsigned int));
  }
  for(int i = 0; i < 5; i++) {
    queueHandle_triggers[i] = xQueueCreate(1, sizeof(int));
  }
  queueHandle_led = xQueueCreate(1, sizeof(RGB_typedef));

  //task create
  xTaskCreateUniversal(task_md_update,  "task_md_update",   8192, (void*)&MD_ID,  1,  &taskHandle_md_update,  APP_CPU_NUM);
  // xTaskCreateUniversal(task_sol_update, "task_sol_update",  8192, (void*)&sol_ID, 1,  &taskHandle_sol_update, APP_CPU_NUM);
  for(int i = 0; i < 1; i++){
    solenoid_task_arg sol_arg = {sol_ID, queueHandle_run, queueHandle_solenoids, 10, 50};
    xTaskCreateUniversal(solenoid_task, "solenoid_task",  8192, (void*)&sol_arg, 1,  &taskHandle_solenoid[i], APP_CPU_NUM);
  }
  for(int i = 0; i < 5; i++){
    launch_task_arg launch_arg = {queueHandle_triggers[i], queueHandle_solenoids[i*2], queueHandle_solenoids[i*2+1], 500, 500, 500};
    xTaskCreateUniversal(launcher_task, "launcher_task",  8192, (void*)&launch_arg, 1,  &taskHandle_launcher[i], APP_CPU_NUM);
  }
  xTaskCreateUniversal(task_stamp_led,  "task_stamp_led",   8192, NULL,           1,  &taskHandle_stamp_led,  APP_CPU_NUM);

  //CAN init
  CAN.setPins(19, 22);
  CAN.begin(1e6);
  
  //CAN event callback def
  CAN.onReceive([](int packetSize){
  });

  //bluetooth for PS4 init
  uint8_t btmac[6];
  esp_read_mac(btmac, ESP_MAC_BT);
  Serial.printf("[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", btmac[0], btmac[1], btmac[2], btmac[3], btmac[4], btmac[5]);
  PS4.begin("4C:75:25:92:31:52");   //set bluetooth MAC address(see this↑ with serial monitor) here
  
  //PS4 event callback def
  ps4SetEventCallback([](ps4_t ps4, ps4_event_t event){
    if(event.analog_move.stick.rx || event.analog_move.stick.ry || event.analog_move.stick.lx){
      const double k_max_speed = 0.5;
      const double k_max_turn_speed = 0.5;
      
      md_data_typedef data;
      int16_t in_data[3];

      in_data[0] = ps4.analog.stick.rx << 8;
      in_data[1] = ps4.analog.stick.ry << 8;
      in_data[2] = ps4.analog.stick.lx << 8;

      data[0] = k_max_speed * (in_data[0] * cos(PI*2.0/3.0) + in_data[1] * sin(PI*2.0/3.0))
        + k_max_turn_speed * in_data[2];
      data[1] = k_max_speed * (in_data[0] * cos(PI*4.0/3.0) + in_data[1] * sin(PI*4.0/3.0))
        + k_max_turn_speed * in_data[2];
      data[2] = k_max_speed * in_data[0]
        + k_max_turn_speed * in_data[2];
      xQueueOverwrite(queueHandle_md, data);
    }
    if(event.button_down.circle){
      int trig = 1;
      xQueueOverwrite(queueHandle_triggers[0], &trig);
    }
    if(event.button_down.cross){
      int trig = 1;
      xQueueOverwrite(queueHandle_triggers[1], &trig);
    }
    if(event.button_down.left){
      int trig = 1;
      xQueueOverwrite(queueHandle_triggers[2], &trig);
    }
    if(event.button_down.right){
      int trig = 1;
      xQueueOverwrite(queueHandle_triggers[3], &trig);
    }
    if(event.button_down.triangle){
      int trig = 1;
      xQueueOverwrite(queueHandle_triggers[4], &trig);
    }
    if(event.button_down.ps){
      int run = 1;
      xQueuePeek(queueHandle_run, &run, 0);
      run = !run;
      xQueueOverwrite(queueHandle_run, &run);

      ps4_cmd_t controller = {0, 0, 0, 0, 0, 0, 0};
      if(run){
        controller.r = 0;
        controller.g = 0;
        controller.b = 255;
      }else{
        controller.r = 255;
        controller.g = 0;
        controller.b = 0;
        controller.flashOn = 16;
        controller.flashOff = 16;
      }
      ps4SetOutput(controller);
      Serial.printf("is run : %d\r\n", run);
    }
  });
  ps4SetConnectionCallback([](uint8_t isConnected){
    if(isConnected){
      RGB_typedef blue = {0, 0, 128};
      xQueueOverwrite(queueHandle_led, &blue);

      int run = 0;
      xQueueOverwrite(queueHandle_run, &run);
      ps4_cmd_t controller = {0, 0, 0, 0, 0, 0, 0};
      controller.r = 255;
      controller.g = 0;
      controller.b = 0;
      controller.flashOn = 16;
      controller.flashOff = 16;
      ps4SetOutput(controller);
      Serial.printf("is run : %d\r\n", run);
    }else{
      RGB_typedef red = {128, 0, 0};
      md_data_typedef data = {0, 0, 0, 0};
      xQueueOverwrite(queueHandle_md, data);
      xQueueOverwrite(queueHandle_led, &red);
    }
  });
  Serial.println("Ready.");

  //set LED green if succese all initialize
  RGB_typedef green = {0, 128, 0};
  xQueueOverwrite(queueHandle_led, &green);

  while(1){
    // put your main code here, to run repeatedly:
    if (PS4.isConnected()) {
      int run = 1;
      uint8_t bat_lev = PS4.Battery();
      Serial.printf("Battery Level : %d\r\n", bat_lev);
      xQueuePeek(queueHandle_run, &run, 0);
      if(run){
        ps4_cmd_t controller = {0, 0, 0, 0, 0, 0, 0};
        if(bat_lev > 1){
          controller.r = 0;
          controller.g = 0;
          controller.b = 255;
        }else{
          controller.r = 255;
          controller.g = 128;
          controller.b = 0;
        controller.flashOn = 64;
        controller.flashOff = 32;
        }
        ps4SetOutput(controller);
      }
    }else{
      Serial.println("Controller not connected.");
    }
    //plese keep this delay bigger than 1. if not, main task stopped by WTD
    //(WTD is enabled to restart when CAN message writing failue)
    delay(500);
  }
}