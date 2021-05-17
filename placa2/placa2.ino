#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>
#include "Terminal.h"
#include "HIB.h"
#include "timerConfig.h"
#include "SO.h"


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

/******************************************************************************/
/** Global variables **********************************************************/
/******************************************************************************/

HIB hib;
MCP_CAN CAN(SPI_CS_PIN);
SO so;
Terminal term;

const uint32_t ID_PANEL_PULSADO =0x00022449; 
const uint32_t ID_INCENDIO = 0x00022450; 
const uint32_t ID_BASCULA = 0x00022451;  

Flag fCANEvent;

const unsigned char maskControl = 0x01;

volatile uint32_t rx_id;
volatile char rx_tecla;

/******************************************************************************/
/** Additional functions prototypes *******************************************/
/******************************************************************************/

void print_rx_msg();

/******************************************************************************/
/** TASKS *********************************************************************/
/******************************************************************************/

void TaskControl(){

    while(1){
      
      so.waitFlag(fCANEvent, maskControl);
      
      so.clearFlag(fCANEvent, maskControl);
       switch(rx_id){
          case ID_PANEL_PULSADO:
            print_rx_msg();
          break;
          case ID_INCENDIO:
          break;
          case ID_BASCULA: 
          break;
        } 
    }
}


/******************************************************************************/
/** Hooks *********************************************************************/
/******************************************************************************/



// Handle reception interrupt
//
// Note that all CAN interrupt sources/events invoque the same ISR.
// 
// If more than one CAN interrupt is enabled, then
// some CAN interrupt events could occur quasi simultaneously.
//
// If this happens and the ISR has not finished yet,
// then the new CAN interrupt events will not trigger an additional ISR.
//
// Therefore, for dealing with those cases, we have to check within the ISR
// if more than one CAN event has happened.
//
// However, since we have only enabled the RX interrupt, we do not need to check this
// kind of things in this example program!
//
// Anyway, just to show how to use the library to check what events have triggered the ISR,
// in this ISR we check that the ISR has been actually triggered by an RX event.




/*****************
  CANISR
******************/ 

void isrCAN()
{
  char auxSREG;

  // Save the AVR Status Register by software
  // since the micro does not do it automatically
  auxSREG = SREG;

  if (CAN.rxInterrupt()) {
    hib.ledToggle(3); // for debugging

    rx_id = CAN.getRxMsgId();
    switch(rx_id) {
      case ID_PANEL_PULSADO:
       
        CAN.getRxMsgData((char*) &rx_tecla);
        Serial.println(rx_tecla);
        so.setFlag(fCANEvent, maskControl);
        break;

      default:
        break;
    }
  }

  // Restore the AVR Status Register by software
  // since the micro does not do it automatically
  SREG = auxSREG;
}


/******************************************************************************/
/** Setup *********************************************************************/
/******************************************************************************/

void setup()
{
  // Init terminal
  Serial.begin(115200);
  term.clear();
  Serial.println("Soy Placa 2 imprimo lo que me pasa placa 1 ");

  // Init HIB
  hib.begin();

  // Clear LCD
  hib.lcdClear();
  
  // Init can bus : baudrate = 500k, loopback mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }

  

  Serial.println("CAN BUS Shield init ok!");

  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.
  attachInterrupt(0, isrCAN, FALLING);
}


/******************************************************************************/
/** Main loop *****************************************************************/
/******************************************************************************/

void loop()
{
  fCANEvent= so.defFlag();
  // Rx vars
  uint16_t rx_msg_count = 0;
  
  so.defTask(TaskControl, 2);
  // Start mutltasking (program does not return to 'main' from hereon)
  so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
}


/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/

void print_rx_msg()
{  
  Serial.println("-----RESULTADO-----");
  hib.ledToggle(0);
  switch(rx_tecla){
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          Serial.println("Vamos al piso ");
          Serial.println(rx_tecla);
          break;
          case '#':
          Serial.println("Cerrando puertas");
          break;
          case '*':
          Serial.println("Abriendo puertas");
          break;
          case '0':
          Serial.println("A L A R M A");
          break;
          default:
          break;
   }
  
  Serial.println("");
}
