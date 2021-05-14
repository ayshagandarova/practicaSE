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

volatile bool tx_timer_expired;

volatile uint32_t rx_id;
volatile uint8_t rx_dlc;
volatile char rx_tecla;

/******************************************************************************/
/** Additional functions prototypes *******************************************/
/******************************************************************************/

void print_tx_msg(int tx_sensor_to_print);
void print_rx_msg();

/******************************************************************************/
/** TASKS *********************************************************************/
/******************************************************************************/

void TaskControl(){
  
   so.waitFlag(fCANEvent, maskControl);
   term.print("HOLAA ");
   so.clearFlag(fCANEvent, maskControl);
   
   switch(rx_id){
      case ID_PANEL_PULSADO:
        CAN.getRxMsgData((byte*) &rx_tecla);
        print_rx_msg();
       
      break;
      case ID_INCENDIO:
      break;
      case ID_BASCULA: 
      break;
  }
}


/******************************************************************************/
/** Hooks *********************************************************************/
/******************************************************************************/

// Handle Timer 5
void timer5Hook()
{
  tx_timer_expired = true;
}


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

void MCP2515_ISR()
{
  char auxSREG;

  // Save the AVR Status Register by software
  // since the micro does not do it automatically
  auxSREG = SREG;


  if (CAN.rxInterrupt())
  {
    // Function readRxMsg() does the following actions:
    // (1) Reads received message from RX buffer and places it
    //     in a software buffer within the CAN object ('within the library')
    // (2) Clears and releases the RX buffer
    // (3) Clears the reception interrupt flag placed at the CAN controller
    CAN.readRxMsg();
    rx_id = CAN.getRxMsgId();
    rx_dlc = CAN.getRxMsgDlc();
    so.setFlag(fCANEvent, maskControl);
    switch(rx_id){
      case ID_PANEL_PULSADO:
        CAN.getRxMsgData((char*) &rx_tecla);
        print_rx_msg();
       
      break;
      case ID_INCENDIO:
      break;
      case ID_BASCULA: 
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
  term.begin(115200);
  term.clear();
  term.println("Soy Placa 2 imprimo lo que me pasa placa 1 ");

  // Init HIB
  hib.begin();

  // Clear LCD
  hib.lcdClear();
  
  // Init can bus : baudrate = 500k, loopback mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
    term.println("CAN BUS Shield init fail");
    term.println(" Init CAN BUS Shield again");
    delay(100);
  }

  term.println("CAN BUS Shield init ok!");

  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.
  attachInterrupt(0, MCP2515_ISR, FALLING);
}


/******************************************************************************/
/** Main loop *****************************************************************/
/******************************************************************************/

void loop()
{
  
  // Rx vars
  uint16_t rx_msg_count = 0;

  hib.setUpTimer5(TIMER_TICKS_FOR_1s, TIMER_PSCALER_FOR_1s, timer5Hook);
  fCANEvent= so.defFlag();
  so.defTask(TaskControl, 2);
  while (true)
  {
      
  }
}


/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/

// -- RX -----------------------
// TIME: 172077
// COUNT: 172
// ID: 22449
// DLC: size of rx_sensor
// SENSOR: rx_sensor

void print_rx_msg()
{  
   
  term.println("-----RESULTADO-----");
  
  switch(rx_tecla){
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          term.print("Vamos al piso ");
          term.print(rx_tecla);
          break;
          case '#':
          term.print("Cerrando puertas");
          break;
          case '*':
          term.print("Abriendo puertas");
          break;
          case '0':
          term.print("A L A R M A");
          break;
          default:
          break;
   }
  
  term.println("");
}
