#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>
#include "Terminal.h"
#include "HIB.h"
#include "timerConfig.h"
#include "SO.h"
#include <math.h>
/********************************************
  Declaration of global shared vars and cons
*********************************************/
#define PRIO_TASK_CONTROL 2
#define PRIO_TASK_PANEL 3
#define PRIO_TASK_COMANDOS 1
#define PRIO_TASK_LUCES 2

#define PERIOD_TASK_COMAND 4
#define PERIOD_TASK_LUCES 10

#define PERIOD_TASK_QUART 2

volatile uint8_t pisoActual = 1; 
volatile uint32_t rx_id;
volatile uint8_t rx_tecla;
volatile uint16_t rx_peso;
volatile uint8_t rx_temp;
volatile char comando; // el comando que se introduce por terminal

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


/********************************
  Declaration of flags and masks
*********************************/
Flag fControl;

const unsigned char maskCANEvent = 0x01;
// Máscaras referentes a los comandos introducidos por la terminal:
const unsigned char maskMantenimiento = 0x02;
const unsigned char maskFuegoApagado = 0x04;
const unsigned char maskReparado = 0x08;

Flag fEmergencia;
const unsigned char maskEmergencia = 0x01;

Flag fLuces;
const unsigned char maskIncendio = 0x01;

/********************************
  Declaration of mailBoxes
*********************************/
MBox mbPanel;

// Posibles estados en los que se puede encontrar el ascensor
enum ascensor {Detenido, EnMovimiento, Bloqueado, Incendio};

ascensor estado;

// Struct que almacena el la información que se imprimirá por la terminal:
struct infoAscensor{
  uint8_t pisoAct;
  uint8_t pisoDestino;
  uint8_t temp;
  ascensor estado; 
  char causa [25];
};

typedef infoAscensor informacion;

void playNote(uint8_t note, uint8_t octave, uint16_t duration);

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
//   volatile uint8_t rx_temp;

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
          so.setFlag(fControl, maskCANEvent); // activamos la tarea de control 
  }
  
  // Restore the AVR Status Register by software
  // since the micro does not do it automatically
  SREG = auxSREG;
}


/******************************************
  TASKS declarations and implementations
*******************************************/

/*
 * Controla el funcionamiento del ascensor.
 * Se activa periódicamente por flag.
 * Determina la actuación en base al estado actual del ascensor, y de la máscara 
 * que ha activado la tarea.
 */
void TaskControl() {
  const uint8_t ABRIR_PUERTAS=8, CERRAR_PUERTAS=7;
  const uint8_t ALMOHADILLA=12, ASTERISCO=10;
  uint8_t auxKey;
  uint8_t actuacion;
  informacion info;
  info.pisoAct = 1;
  info.estado = estado;
  unsigned char flagValue;
  const uint8_t NUM_LAST_KEYS = 5;
  int8_t lastKeys[NUM_LAST_KEYS]; //array con utilización FIFO para guardar las teclas pulsadas             
  for (int i=0; i<NUM_LAST_KEYS; i++){
      lastKeys[i] = -1;
  }
  uint8_t posKey = 0;
  const unsigned char mask = (maskCANEvent | maskMantenimiento | maskFuegoApagado | maskReparado);
  boolean enviar = false;
  unsigned long nextCANAwakeTick;
  while (1){
      // Wait until any of the bits of the flag fControl
      // indicated by the bits of mask are set to '1'
      so.waitFlag(fControl, mask);
      flagValue = so.readFlag(fControl);
      // Clear the flag fControl to not process the same event twice
      so.clearFlag(fControl, mask);

    switch (estado){
            case Detenido:
                     memset(&info.causa, 0, sizeof(info.causa));  
                    
                    if (flagValue == maskMantenimiento) { // si es el comando de en mantenimiento
                          estado = Bloqueado;
                          info.estado = estado;
                          sprintf(info.causa, "Mantenimiento en curso");
                          so.signalMBox(mbPanel, (byte*) &info);
                    } else if (flagValue == maskCANEvent) {  // si es un valor de CAN
                          switch (rx_id){
                                case ID_PANEL_PULSADO: // keyPad pulsado
                                        
                                        auxKey = rx_tecla + 1;
                                        if (auxKey <= 6 && auxKey >= 1) {  // es un piso 
                                              //rotamos a la izquierda los valores de lastKeys, eliminando así el primero
                                              if(lastKeys[0] != -1){
                                                    for(int i=0;i<NUM_LAST_KEYS-1;i++){
                                                         lastKeys[i]=lastKeys[i+1];
                                                     }
                                                     lastKeys[NUM_LAST_KEYS] = -1;
                                              }
                                              if (auxKey == pisoActual){   // si se ha pulsado el mismo piso, solo abrimos puertas
                                                    actuacion = ABRIR_PUERTAS;
                                              } else {  
                                                    actuacion = auxKey;
                                                    estado = EnMovimiento;
                                                    info.estado = estado;
                                                    info.pisoDestino = auxKey;
                                                    so.signalMBox(mbPanel, (byte*) &info);
                                              }
                                        } else if (auxKey == ASTERISCO){   // si se pulsa '*' Abrimos puertas
                                              actuacion = ABRIR_PUERTAS;
                                        } else if (auxKey == ALMOHADILLA) {  // su se pulsa '#' Cerramos puertas
                                              actuacion = CERRAR_PUERTAS;
                                        } 
                                        
                                        if (auxKey == 11){
                                              so.setFlag(fEmergencia, maskEmergencia);
                                        }else{ // si se ha pulsado un piso o abrir/cerrar puertas
                                              enviar = true;
                                        }
                                break;
                                case ID_INCENDIO:
                                        estado = Incendio;
                                        info.estado = estado;
                                        info.temp = rx_temp;
                                        so.signalMBox(mbPanel, (byte*) &info);
                                        so.setFlag(fLuces, maskIncendio);
                                break;
                                case ID_BASCULA:
                                        estado = Bloqueado;
                                        info.estado = estado;
                                        sprintf(info.causa, "Peso actual: %i", rx_peso);
                                        so.signalMBox(mbPanel, (byte*) &info);
                                break;
                          }
                    }
      
            break;
            case EnMovimiento:
                    // Cuando el ascensor ha llegado al pisoDestino
                    if (rx_id == ID_SIMULADOR_CAMBIO_PISO && flagValue == maskCANEvent) {
                          //Serial.println("Entro en enMovimiento");
                          estado = Detenido;
                          info.pisoAct = pisoActual;
                          info.estado = estado;
                          enviar = true;
                          actuacion = ABRIR_PUERTAS; // abrir puertas
                          so.signalMBox(mbPanel, (byte*) &info);
                          
                          if (lastKeys[0] != -1){   //simulamos que en ese momento han vuelto a pulsar una tecla, pero en verdad coge la primera del buffer
                            rx_id = ID_PANEL_PULSADO;
                            rx_tecla = lastKeys[0];
                            so.setFlag(fControl, maskCANEvent);
                          }
                    }
                    //Si pulsan un piso mientras está en movimiento
                    if (rx_id == ID_PANEL_PULSADO && flagValue == maskCANEvent && (rx_tecla <= 5 && rx_tecla >= 0)) {
                        
                       //si la tecla no está en el buffer de teclas la añadimos en la siguiente posición disponible
                        posKey = 0;
                        for (int i=0; i<NUM_LAST_KEYS; i++){ 
                            if (lastKeys[i] == rx_tecla){          // piso ya estaba en el array
                                  break;
                            } 
                            if (lastKeys[i] == -1){  //si posición vacía guarda ahí
                                  lastKeys[i] = rx_tecla;
                                  break;
                            }
                            if(info.pisoDestino >= 3){  //atiende los pisos de mayor a menor
                                  if(lastKeys[i] < rx_tecla){  //desplazar a la derecha
                                          for(int j=NUM_LAST_KEYS;j>=i;j--){
                                              lastKeys[j]=lastKeys[j-1];
                                          }
                                          lastKeys[i] = rx_tecla;
                                          break;
                                  }
                            } else {  
                                    if(lastKeys[i] > rx_tecla){  //desplazar a la derecha
                                          for(int j=NUM_LAST_KEYS;j>=i;j--){
                                                lastKeys[j]=lastKeys[j-1];
                                           }
                                           lastKeys[i] = rx_tecla;
                                           break;
                                    }
                            }
                        }
                    } else if(rx_id == ID_PANEL_PULSADO && flagValue == maskCANEvent && (rx_tecla == 10)) {
                        so.setFlag(fEmergencia, maskEmergencia);
                    }
            break;
            case Bloqueado:
                    if (flagValue == maskReparado) {   // acabamos el mantenimiento 
                          estado = Detenido;
                          info.estado = estado;
                          
                          so.signalMBox(mbPanel, (byte*) &info);
                    } else if (rx_id == ID_BASCULA && flagValue == maskCANEvent) {  // el peso es adecuado
                          estado = Detenido;
                          info.estado = estado;
                          sprintf(info.causa, "Peso adecuado :)");
                          so.signalMBox(mbPanel, (byte*) &info);
                    }
            break;
            case Incendio:
                    if (flagValue == maskFuegoApagado) {  // se ha apagado el incendio 
                          estado = Detenido;
                          info.estado = estado;
                          sprintf(info.causa, "Fuego apagado :)");
                          so.signalMBox(mbPanel, (byte*) &info);
                    }
            break;
    }
    
    // solo envía si debe realizar alguna actuación sobre la placa1
    if (enviar){
        while (CAN.checkPendingTransmission() == CAN_TXPENDING){
            nextCANAwakeTick = so.getTick();
            so.delayUntilTick(nextCANAwakeTick + 1);
        }
         //envíamos por bus CAN la actuación 
        CAN.sendMsgBufNonBlocking(ID_CONTROL, CAN_EXTID, sizeof(INT8U), (INT8U *) &actuacion);
        enviar = false;
    }
  }
}

/*
 * Es esporádica por mailbox, se activa cada vez que recibe por mailBox procedente de la tareas de control
 * Imprime por la terminal la información del ascensor
 */

void TaskPanel(){
  informacion * rx_infoAscens;
  informacion info;
  while (1) {
        so.waitMBox(mbPanel, (byte**) &rx_infoAscens);
        info = *rx_infoAscens;
        term.println("");
        term.println("+++++++++++++++++++++++++++++++++");
        term.println("");
        term.print("Piso Actual: ");
        term.println(info.pisoAct);
        term.print("Estado Actual: ");
        switch (info.estado) {
              case Detenido:
                    term.println("Detenido");
                    if (info.causa[0] != 0) {
                      term.println(info.causa);
                    }
              break;
              case EnMovimiento:
                    term.println("En movimiento");
                    term.print("Piso destino: ");
                    term.println(info.pisoDestino);
              break;
              case Bloqueado:
                    term.print("Bloqueado: ");
                    term.println(info.causa);
              break;
              case Incendio:
                    term.print("Incendio con temperatura:");
                    term.println(info.temp);
              break;
        }
  }
}

/*
 * Se activa periódicamente (TICK). 
 * Cada vez que recibe uno de los 3 comandos (m, a, f)activa la tarea de control.
 */
void TaskComandos()
{
  const char EN_MANTENIMIENTO = 'm';  
  const char MANTENIMIENTO_ACABADO = 'a';  
  const char FUEGO_APAGADO = 'f';   
  unsigned long nextActivationTick;

  while (1) {
        nextActivationTick = so.getTick();
        comando = term.getChar(true);  // 
        switch (comando) {
                case EN_MANTENIMIENTO:
                      Serial.println("Comando m: En mantenimiento introducido");
                      so.setFlag(fControl, maskMantenimiento);
                      
                break;
                case MANTENIMIENTO_ACABADO:
                      Serial.println("Comando a: Mantenimiento acabado introducido");
                      so.setFlag(fControl, maskReparado);
                break;
                case FUEGO_APAGADO:
                      Serial.println("Comando f: Fuego apagado introducido");
                      so.setFlag(fControl, maskFuegoApagado);
                break;
        }
        // Autosuspend until time
        nextActivationTick = nextActivationTick + PERIOD_TASK_COMAND; // Calculate next activation time;
        so.delayUntilTick(nextActivationTick);
  }
}

/*
 *  Es una tarea mixta, se activa esporádicamente cuando la activan por flag, 
 *  y una vez activada se ejecuta periodicamente hasta que el estado del 
 *  ascensor no sea Incendio.
 *  Parpadea los LEDS.
 */
void TaskLucesIncendio(){
  unsigned long nextActivationTick;
  const uint8_t num_leds = 6;
  boolean incendioON = false;
  while (1){
        if (!incendioON) {
              
              so.waitFlag(fLuces, maskIncendio);
              so.clearFlag(fLuces, maskIncendio);
              incendioON = true;
              for (int i = 0; i < num_leds; i++) {
                  if (i % 2 == 0) {
                      hib.ledOn(i);
                  }
              }
        } else {
              for (int i = 0; i < num_leds; i++) {
                    hib.ledToggle(i);
              }
              hib.buzzPlay(200, 3500);
              if(estado != Incendio){  // descativamos
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

void TaskLlamadaEmergencia(){
  #define DO   1
  #define RE   3
  #define MI   5
  #define FA   6
  #define SOL  8
  #define LA  10
  #define SI  12
  const uint8_t NUMERO_NOTAS = 24;

  uint8_t numNota;
  unsigned long nextActivationTick;

  const uint8_t partitura[NUMERO_NOTAS] = {SOL, FA, FA, SOL, FA, FA, FA, MI, SOL, FA, FA, SOL, FA, FA, FA, MI, SOL, FA, FA, SOL, FA, FA, FA, MI};
  
  while(1){
       so.waitFlag(fEmergencia, maskEmergencia);
       so.clearFlag(fEmergencia, maskEmergencia);

       // Tocar escala
 
      term.println("");
      term.print("Llamando a emergencias");
      for (numNota = 0; numNota < NUMERO_NOTAS; numNota++){
          playNote(partitura[numNota], 4, 197);
          nextActivationTick = so.getTick();

          nextActivationTick = nextActivationTick + PERIOD_TASK_QUART; // Calculate next activation time;
          so.delayUntilTick(nextActivationTick);
          term.print(".");
      }
      term.println("");
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
    term.println("CAN BUS Shield init fail");
    term.println(" Init CAN BUS Shield again");
    delay(100);
  }

  term.println("CAN BUS Shield init ok!");

  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.
  attachInterrupt(0, isrCAN, FALLING);
}


void loop()
{

  term.println("Nodo2: cabina de control");
  estado = Detenido;
  // Definition and initialization of flags
  fControl = so.defFlag();
  fLuces = so.defFlag();
  fEmergencia = so.defFlag();

  // Definition and initialization of mailBoxes
  mbPanel = so.defMBox();

  // Definition and initialization of tasks
  so.defTask(TaskLucesIncendio, PRIO_TASK_LUCES);
  so.defTask(TaskControl, PRIO_TASK_CONTROL);
  so.defTask(TaskPanel, PRIO_TASK_PANEL);
  so.defTask(TaskComandos, PRIO_TASK_COMANDOS);
  so.defTask(TaskLlamadaEmergencia, PRIO_TASK_LUCES);

  //Set up timer 5 so that the SO can regain the CPU every tick
  hib.setUpTimer5(TIMER_TICKS_FOR_50ms, TIMER_PSCALER_FOR_50ms, timer5Hook);

  // Start mutltasking (program does not return to 'main' from here on)
  so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
}

/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/

// Plays a note in the buzzer
// note: code of the note to play
// octave: octave of the note
// duration: time in miliseconds the sound is on
void playNote(uint8_t note, uint8_t octave, uint16_t duration){
  float v1 = (note-10.0)/12.0;
  float v2 = octave-4;
  unsigned int freq = 400 * pow(2, v1 + v2);  

  hib.buzzPlay(duration, freq);
}
