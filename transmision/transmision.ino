#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>
#include "HIB.h"
#include "Terminal.h"
#include "timerConfig.h"
#include "SO.h"

const int SPI_CS_PIN = 9;

HIB hib;
MCP_CAN CAN(SPI_CS_PIN);

Terminal term;

volatile bool tx_timer_expired;
const unsigned char maskKeyEvent = 0x01;
volatile uint16_t  keyValue = 0; 
  // TX consts and vars
  const uint8_t TX_LED = 0;
  const uint32_t tx_id = 0x00022449;
  int tx_sensor = -3276; // int is 2-bytes long in arduino, thus is range in C2 is [-32768, 32767]




void print_tx_msg(int tx_sensor_to_print);

void KeyPadHook(uint8_t newKey)
{
  hib.ledToggle(5); // for debugging

   // TX by polling based on timer
    
      keyValue = newKey;
      // Check whether or not the TX buffer is available (no Tx still pending)
      // to request transmission of sensor
      if (CAN.checkPendingTransmission() != CAN_TXPENDING)
      {
          // en el sizeof le indicamos cuantos bytes queremos leer de la dirección de memoria 
          //y el último párametro es la direccion de la variable de tx_sensor &tx_sensor  
          // (INT8U *) es un casting para nuestro compilador no se lie, 
          //convertimos un puntero a un puntero de 8 bits
        CAN.sendMsgBufNonBlocking(tx_id, CAN_EXTID, sizeof(int), (INT8U *) &keyValue);
        //escribe una trama en el buffer (transmisión)
        print_tx_msg(keyValue);
      }
    
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

    hib.keySetIntDriven(100, KeyPadHook);

    while (true) {
  }
}
void print_tx_msg(int tx_sensor_to_print)
{
  char charBuff[20];
  sprintf(charBuff, "KeyValue: %i     ",tx_sensor_to_print);
  hib.lcdSetCursorFirstLine();
  hib.lcdPrint(charBuff);
}
