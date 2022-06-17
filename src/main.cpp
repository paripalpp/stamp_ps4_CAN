#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include <FastLED.h>

#define LED_PIN 27

const double k_max_speed = 0.5;
const double k_max_turn_speed = 0.5;

CRGB led[1];

int16_t in_data[3];
const uint32_t MD_ID = 0x01;
int16_t md_data[4] = {0, 0, 0, 0};

void update_led(uint8_t R, uint8_t G, uint8_t B){
  led[0] = G << 16 | R << 8 | B;
  FastLED.show();
  return;
}

int update_md(uint32_t id, int16_t data[4]){
  CAN.beginPacket(id, 8);
  for (int i = 0; i < 4; i++) {
    CAN.write(data[i] >> 8 & 0xFF);
    CAN.write(data[i] & 0xFF);
  }
  return CAN.endPacket();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  FastLED.addLeds<SK6812, LED_PIN, RGB>(led, 1); //GRB order
  update_led(0, 0, 0);
  PS4.begin("1a:2b:3c:01:01:01");
  Serial.println("Ready.");
  update_led(0, 128, 0);
  CAN.setPins(19, 22);
  CAN.begin(1000e3);
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
    md_data[1] = k_max_speed * (in_data[0] * cos(PI*4.0/3.0) + in_data[1] * sin(PI*2.0/3.0))
      + k_max_turn_speed * in_data[2];
    md_data[2] = k_max_speed * in_data[0]
      + k_max_turn_speed * in_data[2];
    update_md(MD_ID, md_data);

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