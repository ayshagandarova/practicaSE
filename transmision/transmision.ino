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
const unsigned char maskKeyEvent = 0x01;
volatile char  keyValue = 0; 
// TX consts and vars
const uint32_t id_panel_pulsado = 0x00022449; 
const uint32_t id_incendio = 0x00022450; 
const uint32_t id_bascula = 0x00022451;  

/***************************
  Declaration of semaphores
***************************/ 
Sem sCANControl;

void print_consola(char to_print);

void KeyPadHook(uint8_t newKey)
{
  hib.ledToggle(5); // for debugging

   // TX by polling based on timer
    

     
      switch(newKey){
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
          keyValue = 49 + newKey; // el 49 es el valor ASCII de 1, por lo tanto se guarda en
                                  // KeyValue el valor en char de cada piso
          term.print(keyValue);
          break;
        case 9:
         keyValue = '*'; // abrir puertas
         break;
        case 11:
         keyValue = '#'; //cerrar puertas
         break;
        case 10:
          keyValue = '0';
          break;
         
        
      }
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
    //escribe una trama en el buffer (transmisión)
    print_consola(keyValue);
      
    
}



void setup() {
  // Init terminal
  term.begin(115200);
  term.clear();
  term.println("Voy a transmitir la tecla pulsada a la placa 2!");

  // Init HIB
  hib.begin();

  // Clear LCD
  hib.lcdClear();

  // Init can bus : baudrate = 500k, loopback mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, false, false) != CAN_OK) {
    term.println("CAN BUS Shield init fail");
    term.println(" Init CAN BUS Shield again");
    delay(100);
  }

  term.println("CAN BUS Shield init ok!");

  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.
  //attachInterrupt(0, MCP2515_ISR, FALLING);
  //cuando ocurra un flanco descendiente en el pin 0 que llame a ...
  
}

void loop() {

  // Definition and initialization of semaphores
    sCANControl = so.defSem(1); // intially accesible
    hib.keySetIntDriven(100, KeyPadHook);
       // Definition and initialization of tasks
    
    while (true) {}
}
void print_consola(char to_print)
{
  char charBuff[20];
  sprintf(charBuff, "KeyValue: %i     ",to_print);
  hib.lcdSetCursorFirstLine();
  hib.lcdPrint(charBuff);
}
