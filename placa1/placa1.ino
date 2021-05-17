#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>
#include "Terminal.h"
#include "HIB.h"
#include "timerConfig.h"
#include "SO.h"

const int SPI_CS_PIN = 9;

HIB hib;
MCP_CAN CAN(SPI_CS_PIN);
SO so;

Terminal term;

volatile bool tx_timer_expired;
volatile char  keyValue= 0; 
// TX consts and vars
const uint32_t id_panel_pulsado = 0x00022449; 
const uint32_t id_incendio = 0x00022450; 
const uint32_t id_bascula = 0x00022451;  

/***************************
  Declaration of semaphores
***************************/ 
Sem sCANControl;

Flag fKeyEvent;

const unsigned char maskKeyEvent = 0x01;




/*****************
  TICK hook
******************/ 

// Hook FOR TICKISR
void timer5Hook ()
{
  so.updateTime(); // Call SO to update time (tick) and manage tasks
}



void KeyHook(uint8_t newKey){
  
  char charBuff[20];
      switch(newKey){
          case 0:
          case 1:
          case 2:
          case 3:
          case 4:
          case 5:
            //term.print(keyValue);
            keyValue = 49 + newKey; // el 49 es el valor ASCII de 1, por lo tanto se guarda en
                                    // KeyValue el valor en char de cada piso
            sprintf(charBuff, "Vamos al piso %c",keyValue);
            break;
          case 9:
          sprintf(charBuff, "Abriendo puertas");
          keyValue = '*'; // abrir puertas
          break;
          case 11:
          sprintf(charBuff, "Cerrando puertas");
          keyValue = '#'; //cerrar puertas
          break;
          case 10:
            sprintf(charBuff, "A L A R M A");
            keyValue = '0'; // alarma
            hib.buzzPlay(200, 3500);
            break;
          default:
           break;
        }
        hib.lcdClear();
        hib.lcdSetCursorFirstLine();
        hib.lcdPrint(charBuff);
        // Check whether or not the TX buffer is available (no Tx still pending)
        // to request transmission of sensor
  
        // Update auxSampled Sensor (shared with TaskState)
      so.waitSem(sCANControl);
  
       if (CAN.checkPendingTransmission() != CAN_TXPENDING){
            // en el sizeof le indicamos cuantos bytes queremos leer de la dirección de memoria 
            //y el último párametro es la direccion de la variable de tx_sensor &tx_sensor  
            // (INT8U *) es un casting para nuestro compilador no se lie, 
            //convertimos un puntero a un puntero de 8 bits
            
          CAN.sendMsgBufNonBlocking(id_panel_pulsado, CAN_EXTID, sizeof(char), (char *) &keyValue);
          
        }
      so.signalSem(sCANControl);
      
   /*switch(newKey){
          case 0:
          case 1:
          case 2:
          case 3:
          case 4:
          case 5:
            keyValue = 49 + newKey; // el 49 es el valor ASCII de 1, por lo tanto se guarda en
                                    // KeyValue el valor en char de cada piso
            break;
          case 9:
           keyValue = '*'; // abrir puertas
           break;
          case 11:
           keyValue = '#'; //cerrar puertas
           break;
          case 10:
            keyValue = '0'; // alarma
            break;
        }*/
  so.setFlag(fKeyEvent, maskKeyEvent);
}

void KeyEvent(){
  
  while(true){
      so.waitFlag(fKeyEvent, maskKeyEvent);
      so.clearFlag(fKeyEvent, maskKeyEvent);
      
      //escribe una trama en el buffer (transmisión)
      
    } 
}



void setup() {
  // Init terminal
  Serial.begin(115200);
  term.clear();
  //Serial.println("Voy a transmitir la tecla pulsada a la placa 2!");

  // Init HIB
  hib.begin();

  // Clear LCD
  hib.lcdClear();
  so.begin();

  // Init can bus : baudrate = 500k, loopback mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, false, false) != CAN_OK) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }

  //Serial.println("CAN BUS Shield init ok!");

  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.
  //attachInterrupt(0, MCP2515_ISR, FALLING);
  //cuando ocurra un flanco descendiente en el pin 0 que llame a ...
  
}

void loop() {
    hib.keySetIntDriven(100, KeyHook); 
    //Definition and initialization of semaphores
      sCANControl = so.defSem(1); // intially accesible
      fKeyEvent= so.defFlag();
      so.defTask(KeyEvent, 2);
      //Set up timer 5 so that the SO can regain the CPU every tick
      hib.setUpTimer5(TIMER_TICKS_FOR_50ms, TIMER_PSCALER_FOR_50ms, timer5Hook);
      
      so.enterMultiTaskingEnvironment();
}
