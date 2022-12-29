#include "Print.h"
/* CAN Bus ECU Simulator
 *  
 * www.skpang.co.uk
 *
 * 
 */

#include <Bounce.h>
#include "ecu_sim.h"
#include <FlexCAN_T4.h>

Bounce pushbuttonSW1 = Bounce(SW1, 10);
Bounce pushbuttonSW2 = Bounce(SW2, 10);

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // can1 port 
extern uint16_t flash_led_tick;

ecu_simClass::ecu_simClass() {
 
}

uint8_t ecu_simClass::init(uint32_t baud) {
  pinMode(SW1,INPUT_PULLUP);
  pinMode(SW2,INPUT_PULLUP);
  can1.begin();
  can1.setBaudRate(500000);
  can1.setMBFilter(ACCEPT_ALL);
  can1.distribute();
  can1.mailboxStatus();

  ecu.dtc = 0;
  return 0;
}
void ecu_simClass::update_pots(void) 
{
  uint16_t temp;
  ecu.engine_rpm = 0xffff - map(analogRead(AN1), 0, 1023, 0, 0xffff);
  ecu.vehicle_speed = 0xff - map(analogRead(AN3), 0, 1023, 0, 0xff);
  ecu.coolant_temp =  0xff - map(analogRead(AN2), 0, 1023, 0, 0xff);
  ecu.maf_airflow = 0xffff - map(analogRead(AN4), 0, 1023, 0, 0xffff);
  ecu.throttle_position = 0xff - map(analogRead(AN5), 0, 1023, 0, 0xff);
  ecu.o2_voltage = 0xffff - map(analogRead(AN6), 0, 1023, 0, 0xffff);
 
  if (pushbuttonSW1.update()) 
  {
    if (pushbuttonSW1.fallingEdge()) 
    {
      if(ecu.dtc == 0)
      {
          ecu.dtc = 1;
          digitalWrite(LED_red, HIGH);

      }else 
      {
          ecu.dtc = 0;
          digitalWrite(LED_red, LOW);
      }
    }
  }
}


uint8_t ecu_simClass::update(void) 
{
  CAN_message_t can_MsgRx,can_MsgTx;

  if(can1.readMB(can_MsgRx)) 
  {
     Serial.print(can_MsgRx.id,HEX);Serial.print(" len:");
     Serial.print(can_MsgRx.len);Serial.print(" ");
     Serial.print(can_MsgRx.buf[0]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[1]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[2]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[4]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[5]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[6]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[7]);Serial.println(" ");
     
     if (can_MsgRx.id == PID_REQUEST) 
     {
       digitalWrite(LED_green, HIGH);
       flash_led_tick = 0;

        if(can_MsgRx.buf[1] == MODE3) // Request trouble codes
        {
            if(ecu.dtc == false){
                can_MsgTx.buf[0] = 0x02; 
                can_MsgTx.buf[1] = MODE3_RESPONSE;    
                can_MsgTx.buf[2] = 0x00;  
             }else{
                can_MsgTx.buf[0] = 0x06; 
                can_MsgTx.buf[1] = MODE3_RESPONSE;    
                can_MsgTx.buf[2] = 0x02;  
                can_MsgTx.buf[3] = 0x01;  
                can_MsgTx.buf[4] = 0x00;                
                can_MsgTx.buf[5] = 0x02;
                can_MsgTx.buf[6] = 0x00;                
             }
             can_MsgTx.id = PID_REPLY;  //7E8
             can_MsgTx.len = 8; 
             can1.write(can_MsgTx);
        }
      
        if(can_MsgRx.buf[1] == MODE4) // Clear trouble codes, clear Check engine light
        {
            ecu.dtc = false;  
            digitalWrite(LED_red, LOW);
            can_MsgTx.buf[0] = 0x00; 
            can_MsgTx.buf[1] = MODE4_RESPONSE; 
            can_MsgTx.id = PID_REPLY;
            can_MsgTx.len = 8; 
            can1.write(can_MsgTx);  
        }
        
        if(can_MsgRx.buf[1] == MODE1)
        {
            can_MsgTx.id = PID_REPLY;
            can_MsgTx.len = 8; 
            can_MsgTx.buf[1] = MODE1_RESPONSE;
            
            switch(can_MsgRx.buf[2])
            {   /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
                case PID_SUPPORTED:
                    can_MsgTx.buf[0] = 0x06;  
                    can_MsgTx.buf[2] = PID_SUPPORTED; 
                    can_MsgTx.buf[3] = 0xE8;
                    can_MsgTx.buf[4] = 0x19;
                    can_MsgTx.buf[5] = 0x30;
                    can_MsgTx.buf[6] = 0x12;
                    can_MsgTx.buf[5] = 0x00;
                    can1.write(can_MsgTx);  
                    break;
                
                case MONITOR_STATUS:
                    can_MsgTx.buf[0] = 0x05;  
                    can_MsgTx.buf[2] = MONITOR_STATUS; 
                    if(ecu.dtc == 1) can_MsgTx.buf[3] = 0x82;
                        else can_MsgTx.buf[3] = 0x00;
                    
                    can_MsgTx.buf[4] = 0x07;
                    can_MsgTx.buf[5] = 0xFF;
                    can1.write(can_MsgTx);      
                    break;
                        
                case ENGINE_RPM:              //   ((A*256)+B)/4    [RPM]
                    can_MsgTx.buf[0] = 0x04;  
                    can_MsgTx.buf[2] = ENGINE_RPM; 
                    can_MsgTx.buf[3] = (ecu.engine_rpm & 0xff00) >> 8;
                    can_MsgTx.buf[4] = ecu.engine_rpm & 0x00ff;
                    can1.write(can_MsgTx);
                    break;
                               
                case ENGINE_COOLANT_TEMP:     //     A-40              [degree C]
                    can_MsgTx.buf[0] = 0x03;  
                    can_MsgTx.buf[2] = ENGINE_COOLANT_TEMP; 
                    can_MsgTx.buf[3] = ecu.coolant_temp;
                    can1.write(can_MsgTx);
                    break;
                               
                case VEHICLE_SPEED:         // A                  [km]
                    can_MsgTx.buf[0] = 0x03;  
                    can_MsgTx.buf[2] = VEHICLE_SPEED; 
                    can_MsgTx.buf[3] = ecu.vehicle_speed;
                    can1.write(can_MsgTx);
                    break;
    
                case MAF_SENSOR:               // ((256*A)+B) / 100  [g/s]
                    can_MsgTx.buf[0] = 0x04;  
                    can_MsgTx.buf[2] = MAF_SENSOR; 
                    can_MsgTx.buf[3] = (ecu.maf_airflow & 0xff00) >> 8;
                    can_MsgTx.buf[4] =  ecu.maf_airflow & 0x00ff;
                    can1.write(can_MsgTx);
                    break;
    
                case O2_VOLTAGE:            // A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
                    can_MsgTx.buf[0] = 0x04;  
                    can_MsgTx.buf[2] = O2_VOLTAGE; 
                    can_MsgTx.buf[3] = ecu.o2_voltage & 0x00ff;
                    can_MsgTx.buf[4] = (ecu.o2_voltage & 0xff00) >> 8;
                    can1.write(can_MsgTx);
                    break;;
                   
                case THROTTLE:            //
                    can_MsgTx.buf[0] = 0x03;  
                    can_MsgTx.buf[2] = THROTTLE; 
                    can_MsgTx.buf[3] = ecu.throttle_position;
                    can1.write(can_MsgTx);
                    Serial.print("Throttle: ");
                    Serial.println(ecu.throttle_position,HEX);
                    break;
              }//switch
          }
       }
    }
   return 0;
}
     
ecu_simClass ecu_sim;
