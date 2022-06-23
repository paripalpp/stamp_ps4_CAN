#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include <FastLED.h>

// #define CAN_ds

#define LED_PIN 27

const double k_max_speed = 0.5;
const double k_max_turn_speed = 0.5;

CRGB led[1];

int16_t in_data[3];
const uint32_t MD_ID = 0x01;
const uint32_t sol_ID = 0x06;
int16_t md_data[4] = {0, 0, 0, 0};

void update_led(uint8_t R, uint8_t G, uint8_t B){
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  FastLED.addLeds<SK6812, LED_PIN, RGB>(led, 1); //GRB order
  update_led(0, 0, 0);
  uint8_t btmac[6];
  esp_read_mac(btmac, ESP_MAC_BT);
  Serial.printf("[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", btmac[0], btmac[1], btmac[2], btmac[3], btmac[4], btmac[5]);
  PS4.begin("4C:75:25:92:31:52");
  Serial.println("Ready.");
  update_led(128, 0, 0);
  CAN.setPins(19, 22);
  CAN.begin(1e6);
  update_led(0, 128, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (PS4.isConnected()) {
    update_led(0, 0, 128);

    in_data[0] = PS4.RStickX() << 8;
    in_data[1] = PS4.RStickY() << 8;
    in_data[2] = PS4.LStickX() << 8;

    md_data[0] = k_max_speed * (in_data[0] * cos(PI*2.0/3.0) + in_data[1] * sin(PI*2.0/3.0))
      + k_max_turn_speed * in_data[2];
    md_data[1] = k_max_speed * (in_data[0] * cos(PI*4.0/3.0) + in_data[1] * sin(PI*4.0/3.0))
      + k_max_turn_speed * in_data[2];
    md_data[2] = k_max_speed * in_data[0]
      + k_max_turn_speed * in_data[2];
    update_md(MD_ID, md_data);

    if(PS4.Circle()){
      update_sol(sol_ID, 0b10101010);
    }else if(PS4.Cross()){
      update_sol(sol_ID, 0b01010101);
    }else{
      update_sol(sol_ID, 0b00000000);
    }

    // if(PS4.PSButton()){
    //   PS4.attachOnDisconnect([](){
    //     Serial.println("Disconnected.");
    //     update_led(0, 0, 0);
    //   });
    // }

    Serial.printf("Battery Level : %d\n", PS4.Battery());
    Serial.println();
    // This delay is to make the output more human readable
    // Remove it when you're not trying to see the output
  }else{
    md_data[0] = 0;
    md_data[1] = 0;
    md_data[2] = 0;
    md_data[3] = 0;
    update_md(MD_ID, md_data);
    update_led(0, 128, 0);
    Serial.println("Controller not connected.");
  }
  delay(10);
}