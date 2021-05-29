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
#define PRIO_TASK_CONTROL 6
#define PRIO_TASK_PANEL 5
#define PRIO_TASK_COMANDOS 4
#define PRIO_TASK_LUCES 3

#define PERIOD_TASK_COMAND 4
#define PERIOD_TASK_LUCES 50

volatile uint8_t pisoActual = 1;
volatile uint32_t rx_id;
volatile uint8_t rx_tecla;
volatile uint16_t rx_peso;
volatile uint8_t rx_temp;
volatile char comando;
// RX consts and vars
const uint32_t ID_PANEL_PULSADO = 0x00022449;
const uint32_t ID_INCENDIO = 0x00022450;
const uint32_t ID_BASCULA = 0x00022451;
const uint32_t ID_SIMULADOR_CAMBIO_PISO = 0x00022452;

const int SPI_CS_PIN = 9;

// TX consts and vars
const uint32_t ID_CONTROL = 0x00022453;


HIB hib;
MCP_CAN CAN(SPI_CS_PIN);
SO so;
Terminal term;
/***************************
  Declaration of semaphores
***************************/


/********************************
  Declaration of flags and masks
*********************************/
Flag fControl;

// Define masks for adc and keypad event respectively
const unsigned char maskCANEvent = 0x01;
const unsigned char maskMantenimiento = 0x02;
const unsigned char maskFuegoApagado = 0x04;
const unsigned char maskReparado = 0x08;

Flag fLuces;
const unsigned char maskIncendio = 0x01;

/********************************
  Declaration of mailBoxes
*********************************/

MBox mbPanel;


// Posibles estados en los que se puede encontrar el ascensor
enum ascensor {Detenido, EnMovimiento, Bloqueado, Incendio};

ascensor estado;

struct infoAscensor
{
  uint8_t pisoAct;
  uint8_t pisoDestino;
  ascensor estado; // para ponerlo en palabras .name()
  uint8_t temperatura;
  char causa [25];
};

typedef infoAscensor informacion;



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

    switch (rx_id) {
      case ID_PANEL_PULSADO:
        CAN.getRxMsgData((INT8U *) &rx_tecla);
        break;
      case ID_INCENDIO:
        CAN.getRxMsgData((INT8U *) &rx_temp);
        break;
      case ID_BASCULA:
        CAN.getRxMsgData((INT8U *) &rx_peso);
        break;
      case ID_SIMULADOR_CAMBIO_PISO:
        // si ha llegado al piso destino:
        CAN.getRxMsgData((INT8U *) &pisoActual);
        break;
    }
    so.setFlag(fControl, maskCANEvent);
  }
  // Restore the AVR Status Register by software
  // since the micro does not do it automatically
  SREG = auxSREG;
}

/*
   switch(rx_tecla){
                case 0:case 1: case 2: case 3: case 4:case 5:
                Serial.print("Vamos al piso ");
                Serial.println(auxKey);
                break;
                case 11:  Serial.println("Cerrando puertas");  break;
                case 9:   Serial.println("Abriendo puertas");  break;
                case 10:  Serial.println("A L A R M A");       break;
                default:  break;
*/

/******************************************
  TASKS declarations and implementations
*******************************************/

void TaskControl() {
  const uint16_t PESO_MAX = 255;
  uint8_t auxKey;
  uint8_t actuacion;
  informacion info;
  info.pisoAct = 1;
  info.estado = estado;
  info.temperatura = 25;
  unsigned char flagValue;
  const unsigned char maskComandos = (maskMantenimiento | maskFuegoApagado | maskReparado);
  const unsigned char mask = (maskCANEvent | maskComandos);
  boolean enviar = false;
  while (1)
  {
    so.waitFlag(fControl, mask);
    flagValue = so.readFlag(fControl);
    so.clearFlag(fControl, mask);

    switch (estado)
    {
      case Detenido:


        memset(&info.causa, 0, sizeof(info.causa));
        /*
           1-6 -> subir/bajar al piso N
           7 -> cerrar puertas
           8 -> abrir puertas
        */
        if (flagValue == maskMantenimiento) {
          estado = Bloqueado;
          info.estado = estado;
          sprintf(info.causa, "Mantenimiento en curso");
          so.signalMBox(mbPanel, (byte*) &info);
        } else if (rx_id != 0) {

          switch (rx_id)
          {
            case ID_PANEL_PULSADO: // keyPad pulsado
              auxKey = rx_tecla + 1;
              if (auxKey <= 6 && auxKey >= 1) // si elige un piso
              {
                if (auxKey == pisoActual)
                {
                  actuacion = 8;
                } else
                {
                  actuacion = auxKey;
                  estado = EnMovimiento;
                  info.estado = estado;
                  info.pisoDestino = auxKey;
                  so.signalMBox(mbPanel, (byte*) &info);
                }
              } else if (auxKey == 10) // si se pulsa '*' Abrimos puertas
              {
                actuacion = 8;
              } else if (auxKey == 12) // su se pulsa '#' Cerramos puertas
              {
                actuacion = 7;

              }
              enviar = true;
              break;

            case ID_INCENDIO:  // añadir una tarea mixta que controle los leds, para que parpadeen, hasta que se arregle :)
              estado = Incendio;
              info.estado = estado;
              info.temperatura = rx_temp;
              so.signalMBox(mbPanel, (byte*) &info);
              Serial.println("activamos los leds ");
              so.setFlag(fLuces, maskIncendio);
              break;
            case ID_BASCULA:
              if (rx_peso >= PESO_MAX) {
                estado = Bloqueado;
                info.estado = estado;
                sprintf(info.causa, "Peso actual: %i", rx_peso);
                so.signalMBox(mbPanel, (byte*) &info);
              }
              break;
            case ID_SIMULADOR_CAMBIO_PISO:
              estado = Detenido;
              info.pisoAct = pisoActual;
              info.estado = estado;
              so.signalMBox(mbPanel, (byte*) &info);
              break;
          }
        }

        break;
      case EnMovimiento:
        if (rx_id == ID_SIMULADOR_CAMBIO_PISO) {
          estado = Detenido;
          info.pisoAct = pisoActual;
          info.estado = estado;
          enviar = true;
          actuacion = 8; // abrir puertas
          so.signalMBox(mbPanel, (byte*) &info);

        }

        break;
      case Bloqueado:
        if (flagValue == maskReparado) {
          estado = Detenido;
          info.estado = estado;
          sprintf(info.causa, "Mantenimiento acabado :)");
          so.signalMBox(mbPanel, (byte*) &info);
        } else if (rx_id == ID_BASCULA && rx_peso < PESO_MAX ) {
          estado = Detenido;
          info.estado = estado;
          sprintf(info.causa, "Peso adecuado :)");
          so.signalMBox(mbPanel, (byte*) &info);
        }

        // mantenimientoAcabado
        break;
      case Incendio:
        if (flagValue == maskFuegoApagado) {
          estado = Detenido;
          info.estado = estado;
          info.temperatura = 25;
          so.signalMBox(mbPanel, (byte*) &info);
        }
        break;
    }
    if (enviar && CAN.checkPendingTransmission() != CAN_TXPENDING) {
      //envíamos por bus CAN
      CAN.sendMsgBufNonBlocking(ID_CONTROL, CAN_EXTID, sizeof(INT8U), (INT8U *) &actuacion);
      enviar = false;

    }
    rx_id = 0; // :)

  }
}

/*
   TAREA PANEL
*/

void TaskPanel()
{
  informacion * rx_infoAscens;
  informacion info;
  while (1)
  {
    so.waitMBox(mbPanel, (byte**) &rx_infoAscens);
    info = *rx_infoAscens;
    term.println("+++++++++++++++++++++++++++++++++");
    Serial.println();
    Serial.print("Piso Actual: ");
    Serial.println(info.pisoAct);

    Serial.print("Estado Actual: ");
    switch (info.estado) {
      case Detenido:
        Serial.println("Detenido");
        if (info.causa[0] != 0) {
          Serial.println(info.causa);
        }
        break;
      case EnMovimiento:
        Serial.println("En movimiento");
        Serial.print("Piso destino: ");
        Serial.println(info.pisoDestino);
        break;
      case Bloqueado:
        Serial.print("Bloqueado: ");
        Serial.println(info.causa);
        break;
      case Incendio:
        Serial.println("Incendio");
        break;
    }
    Serial.print("Temperatura: ");
    Serial.println(info.temperatura);
  }
}


void TaskComandos()
{
  const char EN_MANTENIMIENTO = 'm';  // si está en detenido
  const char MANTENIMIENTO_ACABADO = 'a';  // solo si está bloqueado
  const char FUEGO_APAGADO = 'f';   // en caso de incendio
  unsigned long nextActivationTick;

  while (1) {
    // Autosuspend until time
    nextActivationTick = so.getTick();
    comando = term.getChar(true);

    switch (comando) {
      case EN_MANTENIMIENTO:
        so.setFlag(fControl, maskMantenimiento);
        break;
      case MANTENIMIENTO_ACABADO:
        so.setFlag(fControl, maskReparado);
        break;
      case FUEGO_APAGADO:
        so.setFlag(fControl, maskFuegoApagado);
        break;

    }


    nextActivationTick = nextActivationTick + PERIOD_TASK_COMAND; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);
  }
}


void TaskLucesIncendio()
{
  unsigned long nextActivationTick;
  const uint8_t num_leds = 6;
  boolean incendioON = false;
  while (1)
  {
    if (!incendioON) {
      so.waitFlag(fLuces, maskIncendio);
      so.clearFlag(fLuces, maskIncendio);
      Serial.println("LUCEEEES");
      incendioON = true;
    } else {
      for (int i = 0; i < num_leds; i++) {
        hib.ledToggle(i);
      }

      if(estado != Incendio){
        for (int i = 0; i < num_leds; i++) {
          hib.ledOff(i);
        }
        incendioON = false;
      }
      nextActivationTick = so.getTick();
      nextActivationTick = nextActivationTick + PERIOD_TASK_LUCES; // Calculate next activation time;
      so.delayUntilTick(nextActivationTick);
    }
  }
}


/*****************
  MAIN PROGRAM
******************/

void setup()
{
  term.begin(115200);
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
  estado = Detenido;
  // Definition and initialization of semaphores


  // Definition and initialization of flags
  fControl = so.defFlag();
  fLuces = so.defFlag();

  // Definition and initialization of mailBoxes
  mbPanel = so.defMBox();


  // Definition and initialization of tasks
  so.defTask(TaskControl, PRIO_TASK_CONTROL);
  so.defTask(TaskPanel, PRIO_TASK_PANEL);
  so.defTask(TaskComandos, PRIO_TASK_COMANDOS);
  so.defTask(TaskLucesIncendio, PRIO_TASK_LUCES);

  //Set up timer 5 so that the SO can regain the CPU every tick
  hib.setUpTimer5(TIMER_TICKS_FOR_50ms, TIMER_PSCALER_FOR_50ms, timer5Hook);

  // Start mutltasking (program does not return to 'main' from here on)
  so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
}


/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
