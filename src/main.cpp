#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include <FastLED.h>

// #define CAN_ds

#define LED_PIN 27

typedef int16_t md_data_typedef[4];
typedef struct{
  uint8_t state1;
  uint8_t state2;
  uint32_t delay_time;
}sol_pattern_typedef;

QueueHandle_t queueHandle_run;
QueueHandle_t queueHandle_md;
QueueHandle_t queueHandle_sol;

CRGB led[1];

void update_led(CRGB* led, uint8_t R, uint8_t G, uint8_t B){
  led[0] = G << 16 | R << 8 | B;
  FastLED.show();
  return;
}

int update_md(uint32_t id, int16_t data[4]){
  
  //debug
  Serial.printf("id : %d\n", id);
  Serial.printf("data : %d, %d, %d, %d\n", data[0], data[1], data[2], data[3]);
  #if defined(CAN_ds)
  return 1;
  #else
  if(!CAN.beginPacket(id, 8)){
    return 0;
  }
  for (int i = 0; i < 4; i++) {
    CAN.write(data[i] & 0xFF);
    CAN.write(data[i] >> 8 & 0xFF);
  }

  return CAN.endPacket();
  #endif

}

int update_sol(uint32_t id, uint8_t data){
  #if defined(CAN_ds)
  return 1;
  #else
  if(!CAN.beginPacket(id, 1)){
    return 0;
  }
  CAN.write(data);
  return CAN.endPacket();
  #endif
}

//task
void task_md_update(void *id) {
  int run = 0;
  md_data_typedef output = {0, 0, 0, 0};
  delay(500);
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
  delay(500);
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

void setup() {}

void loop() {
  const uint32_t MD_ID = 0x01;
  const uint32_t sol_ID = 0x06;

  TaskHandle_t taskHandle_md_update;
  TaskHandle_t taskHandle_sol_update;

  //enable core1 WTD
  enableCore1WDT();

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("reseted");

  //LED init
  FastLED.addLeds<SK6812, LED_PIN, RGB>(led, 1); //GRB order
  update_led(led, 0, 0, 0);

  //CAN init
  CAN.setPins(19, 22);
  CAN.begin(1e6);

  //task create
  xTaskCreateUniversal(
    task_md_update,
    "task_md_update",
    8192,
    (void*)&MD_ID,
    1,
    &taskHandle_md_update,
    APP_CPU_NUM
  );
  xTaskCreateUniversal(
    task_sol_update,
    "task_sol_update",
    8192,
    (void*)&sol_ID,
    1,
    &taskHandle_sol_update,
    APP_CPU_NUM
  );

  //queue create
  queueHandle_md = xQueueCreate(1, sizeof(md_data_typedef));
  queueHandle_sol = xQueueCreate(1, sizeof(sol_pattern_typedef));
  queueHandle_run = xQueueCreate(1, sizeof(int));

  //bluetooth for PS4 init
  uint8_t btmac[6];
  esp_read_mac(btmac, ESP_MAC_BT);
  Serial.printf("[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", btmac[0], btmac[1], btmac[2], btmac[3], btmac[4], btmac[5]);
  PS4.begin("4C:75:25:92:31:52");
  
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
    if(event.button_up.circle){
      sol_pattern_typedef data = {0b10101010, 0b00000000, 500};
      xQueueOverwrite(queueHandle_sol, &data);
    }
    if(event.button_up.cross){
      sol_pattern_typedef data = {0b01010101, 0b00000000, 500};
      xQueueOverwrite(queueHandle_sol, &data);
    }
    if(event.button_up.ps){
      int run = 1;
      xQueuePeek(queueHandle_run, &run, 0);
      run = !run;
      xQueueOverwrite(queueHandle_run, &run);
    }
  });
  ps4SetConnectionCallback([](uint8_t isConnected){
    if(isConnected){
      PS4.setLed(0, 0, 255);
      update_led(led, 0, 0, 128);
    }else{
      md_data_typedef data = {0, 0, 0, 0};
      xQueueOverwrite(queueHandle_md, data);
      update_led(led, 0, 128, 0);
    }
  });
  Serial.println("Ready.");

  //set LED green if succese all initialize
  update_led(led, 0, 128, 0);

  while(1){
    // put your main code here, to run repeatedly:
    if (PS4.isConnected()) {
      Serial.printf("Battery Level : %d\n", PS4.Battery());
      Serial.println();
    }else{
      Serial.println("Controller not connected.");
    }
    //plese keep this delay bigger than 1. if not, main task stopped by WTD
    //(WTD is enabled in src\main.cpp:87 to restart when CAN message writing failue)
    delay(500);
  }
}