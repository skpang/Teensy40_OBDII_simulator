/*
Teensy 4.0 OBDII CAN-BUS ECU simulator.
www.skpang.co.uk
December 2022

For use with:
https://www.skpang.co.uk/collections/teensy/products/teensy-4-0-obdii-can-bus-ecu-simulator-with-teensy-4-0

Other PIDs can be added, more info on PID:
http://en.wikipedia.org/wiki/OBD-II_PIDs

*/

#include "ecu_sim.h"

IntervalTimer timer;

ecu_t ecu;
uint16_t led_tick = 0;
uint16_t pot_tick = 0;
uint16_t flash_led_tick = 0;

int led = 13;   // red LED on Teensy

void setup() {
  pinMode(led,OUTPUT);
  pinMode(LED_red,OUTPUT);
  pinMode(LED_green,OUTPUT);

  digitalWrite(LED_red, HIGH);
  delay(1000);
  digitalWrite(LED_red, LOW);
  digitalWrite(LED_green, HIGH);
  delay(1000);
  digitalWrite(LED_green, LOW);

  
  Serial.println("****** Teensy 4.0 OBDII simulator skpang.co.uk 2022");
  
  ecu_sim.init(500000);
  timer.begin(tick, 1000);    //1ms tick
}

void tick(void)
{
    led_tick++;
    pot_tick++;
    flash_led_tick++;
}

void loop() {
  
  ecu_sim.update();

  if(led_tick > 1000)
  {
      led_tick = 0;
      digitalToggle(led);
  }

  if(pot_tick>10)
  {
      pot_tick = 0;
      ecu_sim.update_pots();
  }
  if(flash_led_tick > 50)
  {
      flash_led_tick =0;
      digitalWrite(LED_green, LOW);
  }
}
