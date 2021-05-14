#include "HIB.h"
#include "timerConfig.h"
#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include "SO.h"

HIB hib;
MCP_CAN CAN(SPI_CS_PIN);
SO so;

/********************************************
  Declaration of global shared vars and cons
*********************************************/

volatile uint8_t key = 0;

/********************************
  Declaration of flags and masks
*********************************/
  // flag (set of bits) for external events
Flag fExtEvent;
const unsigned char maskKeyEvent = 0x01;
volatile uint16_t  keyValue = 0; 


/*****************
  KEYPAD hook
******************/ 
void KeyPadHook(uint8_t newKey)
{
  keyValue = newKey;

  hib.ledToggle(5); // for debugging

  // Print the new adc adquired value
  // Awake task A by setting to '1' the bits of fExtEvent indicated by maskAdcEvent
  so.setFlag(fKeyEvent, maskKeyEvent);
  Serial.println(key);  // This is just for debugging
                        // in a "real program" we sholdn't
                        // print within an ISR/hook
}



/******************************************
  TASKS declarations and implementations
*******************************************/ 

 /* Task states

  DESTROYED / uninitalized
  BLOCKED / WAITING FOR SEMAPHORE-MAILBOX-FLAG
  AUTO-SUSPENDED BY TIME
  ACTIVE / ELIGIBLE / READY
  RUNNING

*/

void PanelPulsado() {
  uint16_t auxKeyValue;

  while(1)
  {
          // Wait until any of the bits of the flag fExtEvent
          // indicated by the bits of maskAdcEvent are set to '1'        
    so.waitFlag(fKeyEvent, maskKeyEvent);
          // Clear the maskAdcEvent bits flag fExtEvent to not process the same event twice
    so.clearFlag(fKeyEvent, maskKeyEvent);

    auxKeyValue = adcValue; // latch adquired sensor value in a local var

    // Update auxSampled Sensor (shared with TaskState)
    so.waitSem(sSampledSensor);

          sampledSensor = auxSampledSensor;
    
    so.signalSem(sSampledSensor);
  }
}



/*****************
  MAIN PROGRAM
******************/

void setup() {  // SETUP

  Serial.begin(115200); // SPEED
  hib.begin();
}


void loop()
{ 
  char buffKetPad[10];
  Serial.println(" ");
  Serial.println(" MAIN ");

  // Set up keypad interrupt
  // expected time between keystrokes is set to 100 ms
  // you can play with this first parameter to adjust
  // the 'sensitivity' of the keypad interrupt
  hib.keySetIntDriven(100, KeyPadHook);

  // Print the 
  while(1)
  {
    delay(500);
  }
}
