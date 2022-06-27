#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include <FastLED.h>

// #define CAN_ds

#define LED_PIN 27

typedef int16_t md_data_typedef[4];

typedef struct{
  float x;
  float y;
  float z;
}vector3f_typedef;

typedef struct{
  uint8_t state1;
  uint8_t state2;
  uint32_t delay_time;
}sol_pattern_typedef;

typedef struct{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
}RGB_typedef;

QueueHandle_t queueHandle_run;
QueueHandle_t queueHandle_md;
QueueHandle_t queueHandle_sol;
QueueHandle_t queueHandle_led;
QueueHandle_t queueHandle_speed_cur;
QueueHandle_t queueHandle_speed_tar;

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

void task_sol_update(void *id) {
  int run = 0;
  sol_pattern_typedef output = {0x00, 0x00, 0};
  delay(50);
  while (1) {
    xQueueReceive(queueHandle_sol,&output,portMAX_DELAY);
    xQueuePeek(queueHandle_run, &run,0);

    if(run){
      CAN.beginPacket(*(int*)id, 1);
      CAN.write(output.state1);
      CAN.endPacket();
      Serial.printf("sol updated\t id:%d\t data:%x\r\n", *(int*)id, output.state1);

      delay(output.delay_time);

      CAN.beginPacket(*(int*)id, 1);
      CAN.write(output.state2);
      CAN.endPacket();
      Serial.printf("sol updated\t id:%d\t data:%x\r\n", *(int*)id, output.state2);
    }

    delay(5);
  }
}

void task_stamp_led(void *arg) {
  RGB_typedef rgb = {0, 0, 0};
  CRGB led[1];

  //LED init
  FastLED.addLeds<SK6812, LED_PIN, RGB>(led, 1); //GRB order
  led[0] = rgb.green << 16 | rgb.red << 8 | rgb.blue;
  FastLED.show();

  while(1){
    xQueueReceive(queueHandle_led, &rgb, portMAX_DELAY);
    led[0] = rgb.green << 16 | rgb.red << 8 | rgb.blue;
    FastLED.show();
    delay(50);
  }
}

void task_speed_pid(void *arg) {
  vector3f_typedef speed_current;
  vector3f_typedef speed_target;
  while(1){
    xQueueReceive(queueHandle_speed_cur, &speed_current, 100);
    xQueueReceive(queueHandle_speed_tar, &speed_target, 0);
    delay(1);
  }
}

void setup() {}

void loop() {
  const uint32_t MD_ID = 0x01;
  const uint32_t sol_ID = 0x06;

  TaskHandle_t taskHandle_md_update;
  TaskHandle_t taskHandle_sol_update;
  TaskHandle_t taskHandle_stamp_led;

  //enable core1 WTD
  enableCore1WDT();

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("reseted");

  //task create
  xTaskCreateUniversal(task_md_update,  "task_md_update",   8192, (void*)&MD_ID,  1,  &taskHandle_md_update,  APP_CPU_NUM);
  xTaskCreateUniversal(task_sol_update, "task_sol_update",  8192, (void*)&sol_ID, 1,  &taskHandle_sol_update, APP_CPU_NUM);
  xTaskCreateUniversal(task_stamp_led,  "task_stamp_led",   8192, NULL,           1,  &taskHandle_stamp_led,  APP_CPU_NUM);

  //queue create
  queueHandle_md  = xQueueCreate(1, sizeof(md_data_typedef));
  queueHandle_sol = xQueueCreate(1, sizeof(sol_pattern_typedef));
  queueHandle_run = xQueueCreate(1, sizeof(int));
  queueHandle_led = xQueueCreate(1, sizeof(RGB_typedef));

  //CAN init
  CAN.setPins(19, 22);
  CAN.begin(1e6);
  
  //CAN event callback def
  CAN.onReceive([](int packetSize){
    const long id_sensor_board0_0 = 9;
    const long id_sensor_board0_1 = 10;
    // if(CAN.packet_id == id_sensor_board0_0){
      
    // }
    // if(CAN.packet_id == id_sensor_board0_1){

    // }
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
      sol_pattern_typedef data = {0b10101010, 0b00000000, 500};
      xQueueOverwrite(queueHandle_sol, &data);
    }
    if(event.button_down.cross){
      sol_pattern_typedef data = {0b01010101, 0b00000000, 500};
      xQueueOverwrite(queueHandle_sol, &data);
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
      PS4.setLed(0, 0, 255);
      xQueueOverwrite(queueHandle_led, &blue);
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