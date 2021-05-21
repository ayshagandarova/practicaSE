#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>
#include "Terminal.h"
#include "HIB.h"
#include "timerConfig.h"
#include "SO.h"

/********************************************
  Declaration of global shared vars and cons
*********************************************/
#define PRIO_TASK_CONTROL 4

volatile uint32_t rx_id;
volatile uint8_t rx_tecla;
volatile uint16_t rx_peso;

// RX consts and vars 
const uint32_t ID_PANEL_PULSADO =0x00022449; 
const uint32_t ID_INCENDIO = 0x00022450; 
const uint32_t ID_BASCULA = 0x00022451;  
const uint32_t ID_SIMULADOR_CAMBIO_PISO = 0x00022452; 

const int SPI_CS_PIN = 9;

// TX consts and vars
const uint32_t ID_CONTROL = 0x00022453;  

HIB hib;
MCP_CAN CAN(SPI_CS_PIN);
SO so;

/***************************
  Declaration of semaphores
***************************/ 


/********************************
  Declaration of flags and masks
*********************************/
Flag fCANEvent;

// Define masks for adc and keypad event respectively
const unsigned char maskControl = 0x01;



/******************************************************************************/
/** Additional functions prototypes *******************************************/
/******************************************************************************/




/*****************
  TICK hook
******************/ 

// Hook FOR TICKISR
void timer5Hook ()
{
  so.updateTime(); // Call SO to update time (tick) and manage tasks
}


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
    CAN.readRxMsg();
    rx_id = CAN.getRxMsgId();
    
    switch(rx_id){
      case ID_PANEL_PULSADO:
         CAN.getRxMsgData((INT8U *) &rx_tecla);
      break;
      case ID_INCENDIO:
         // CAN.getRxMsgData((INT8U *) &rx_tecla);
      break;
      case ID_BASCULA: 
          CAN.getRxMsgData((INT8U *) &rx_peso);
      break;
      case ID_SIMULADOR_CAMBIO_PISO:
      break;
    }
    so.setFlag(fCANEvent, maskControl);
  }
  // Restore the AVR Status Register by software
  // since the micro does not do it automatically
  SREG = auxSREG;
}


/******************************************
  TASKS declarations and implementations
*******************************************/ 

void TaskControl(){
    uint8_t auxKey;
    while(1){
      
      so.waitFlag(fCANEvent, maskControl);
      so.clearFlag(fCANEvent, maskControl);
     
       switch(rx_id){
          case ID_PANEL_PULSADO:
            Serial.println("------ ASCENSOR -----");
            
                switch(rx_tecla){
                        case 0:
                        case 1:
                        case 2:
                        case 3:
                        case 4:
                        case 5:
                        Serial.println("Vamos al piso ");
                        Serial.println(rx_tecla);
                        if (CAN.checkPendingTransmission() != CAN_TXPENDING){
                          //envíamos por bus CAN la primera tecla almazenada en el array lastKeys 
                          auxKey= rx_tecla + 1;
                          CAN.sendMsgBufNonBlocking(ID_CONTROL, CAN_EXTID, sizeof(INT8U), (INT8U *) &auxKey);  
                        }
                        break;
                        case 11:
                        Serial.println("Cerrando puertas");
                        break;
                        case 9:
                        Serial.println("Abriendo puertas");
                        break;
                        case 10:
                        Serial.println("A L A R M A");
                        break;
                        default:
                        break;
                 }
                
                Serial.println("");
          break;
          case ID_INCENDIO:
          break;
          case ID_BASCULA: 
          break;
        } 
    }
}



/*****************
  MAIN PROGRAM
******************/

void setup()
{
  Serial.begin(115200);
  so.begin();
  hib.begin();
  hib.lcdClear();

  // Init can bus : baudrate = 500k, normal mode, enable reception and transmission interrupts
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


void loop()
{
      Serial.println("Placa 2: tareas de control del ascensor");
      
      // Definition and initialization of semaphores
     
      
      // Definition and initialization of flags
      fCANEvent = so.defFlag();
      
      // Definition and initialization of tasks
      so.defTask(TaskControl, PRIO_TASK_CONTROL);
      //Set up timer 5 so that the SO can regain the CPU every tick
      hib.setUpTimer5(TIMER_TICKS_FOR_50ms, TIMER_PSCALER_FOR_50ms, timer5Hook);

      // Start mutltasking (program does not return to 'main' from here on)
      so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
}


/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
