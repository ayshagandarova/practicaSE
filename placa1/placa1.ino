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
#define PRIO_TASK_ADC 1
#define PRIO_TASK_KEY 6
#define PRIO_TASK_SIM_PISOS 4
#define PRIO_TASK_TEM 3
#define PRIO_TASK_LDR 3
#define PRIO_TASK_PP 5
#define PRIO_TASK_INC 3
#define PRIO_TASK_SIM_PUERT 3

#define PERIOD_TASK_TEM 4
#define PERIOD_TASK_LDR 6
#define PERIOD_TASK_SIM 50 // simular el tiempo que tarda en subir los pisos o abrir y cerrar puertas

  // stores every new adc adquired value
  // shared between: adchook (writes) and TaskBascula (reads)
volatile uint16_t adcValue = 0; // critical region beetween adcHook and TaskBascula
const uint8_t PESO_MAX = 250;

  // guarda el piso de destino recibido por el bus CAN
  // shared between: isrCAN (writes) and TaskSimuladorCambioPiso (reads)
volatile uint8_t pisoDest = 0;

  // stores last pressed keypad's key
  // shared between: keyHook (writes) and TaskPanelPulsado (reads)
volatile uint8_t key = 0;

  // TaskIncendio
volatile uint8_t temp=0;
volatile uint16_t light=0;




// TX consts and vars
const uint32_t ID_PANEL_PULSADO = 0x00022449; 
const uint32_t ID_INCENDIO = 0x00022450; 
const uint32_t ID_BASCULA = 0x00022451;  
const uint32_t ID_SIMULADOR_CAMBIO_PISO = 0x00022452; 

const int SPI_CS_PIN = 9;

HIB hib;
MCP_CAN CAN(SPI_CS_PIN);
SO so;

/***************************
  Declaration of semaphores
***************************/ 
Sem sCANControl;
Sem sLCD;
Sem sPisoDest;


/********************************
  Declaration of flags and masks
*********************************/

Flag fExtEvent;
 // Define masks for adc and keypad event respectively
const unsigned char maskAdcEvent = 0x01; // represents new adc value adquired
const unsigned char maskkeyEvent = 0x02; //representa subir o bajar de piso

Flag fActControl;
const unsigned char maskUpDown = 0x01; //representa subir o bajar de piso
const unsigned char maskCerrarPuertas = 0x02; 
const unsigned char maskAbrirPuertas = 0x04;  

Flag fIncendio;
const unsigned char maskTemp = 0x01; //representa subir o bajar de piso
const unsigned char maskLight = 0x02; //representa subir o bajar de piso


/*****************
  ADC hook
******************/ 

void adcHook(uint16_t newAdcAdquiredValue)
{
  // Awake TaskBascula by setting to '1' the bits of fExtEvent indicated by maskAdcEvent
 
    adcValue = newAdcAdquiredValue;
      so.setFlag(fExtEvent, maskAdcEvent);
}

/*****************
  KEYPAD hook
******************/ 
/*key [0-5] --> pisos
  key ==  9 --> abrir puertas
  key == 11 --> cerrar puertas
  key == 10 --> alarma
*/
void keyHook(uint8_t newKey)
{
  key = newKey;
  switch(key)
  {
       case 0:
       case 1:
       case 2:
       case 3:
       case 4:
       case 5:
          hib.ledOn(key);
          break;
       case 10:
          hib.buzzPlay(200, 3500);
          break;
       default:
          break;
    }
   so.setFlag(fExtEvent, maskkeyEvent);
}

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
  uint8_t rxAct; 
  char auxSREG;
  // Save the AVR Status Register by software
  // since the micro does not do it automatically
  auxSREG = SREG;
////////

  if (CAN.rxInterrupt())
  { 
    CAN.readRxMsg();
    //el case depende del mensaje (actuación) no del ID, pq siempre la envía TaskControl
    CAN.getRxMsgData((char*) &rxAct);
    /*
     * 1-6 -> subir/bajar al piso N
     * 7 -> cerrar puertas
     * 8 -> abrir puertas
     */

    switch(rxAct)
    {
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
        pisoDest = rxAct;
        so.setFlag(fActControl, maskUpDown);
        break;
      case 7:
        so.setFlag(fActControl, maskCerrarPuertas);
        break;
      case 8:
        so.setFlag(fActControl, maskAbrirPuertas);
        break;
      default:
        break;
    }
  }


/////
  // Restore the AVR Status Register by software
  // since the micro does not do it automatically
  SREG = auxSREG;
}


/******************************************
  TASKS declarations and implementations
*******************************************/ 
  
void TaskPanelPulsado()
{
  unsigned char flagValue; 
  while(1)
  {
      // Check whether or not the TX buffer is available (no Tx still pending)
      // to request transmission of key
        so.waitFlag(fExtEvent, maskkeyEvent);
        flagValue = so.readFlag(fExtEvent);
        so.clearFlag(fExtEvent, maskkeyEvent);
        so.waitSem(sCANControl);
            if (CAN.checkPendingTransmission() != CAN_TXPENDING)
            {
              //envíamos por bus CAN la tecla
              CAN.sendMsgBufNonBlocking(ID_PANEL_PULSADO, CAN_EXTID, sizeof(INT8U), (INT8U *) &key);
            }
        so.signalSem(sCANControl);
  }
}

// Autosuspends itself until the event: "kestroke" (key pressed)
// When awake, then print the actual floor on the right-handed 7-seg display

void TaskSimuladorCambioPiso()
{
  uint8_t pisoAct = 1;
   unsigned long nextActivationTick;
   const unsigned char PARADO = 0;
   const unsigned char SUBIENDO = 1;
   const unsigned char BAJANDO = 2;
   const unsigned char LLEGADA = 3;
   unsigned char state = PARADO;
   char mensaje[16];
   while(1)
   {
      
      switch(state){
              case PARADO:
                      // Wait until any of the bits of the flag fActControl
                     // indicated by the bits of maskUpDown are set to '1'        
                     so.waitFlag(fActControl, maskUpDown);
                     // Clear the flag fActControl to not process the same event twice
                     so.clearFlag(fActControl, maskUpDown);
                    if (pisoDest > pisoAct){
                      state = SUBIENDO;
                      sprintf(mensaje,"Subiendo...");
                    }
                    if (pisoDest < pisoAct){
                      state = BAJANDO;
                      sprintf(mensaje,"Bajando...");
                    }

        /// mejorar if para si pulsoAct== piso Destino abrir puertas
       
        break;
              case SUBIENDO:
                       // Autosuspend until time
                       nextActivationTick = so.getTick();
                       nextActivationTick = nextActivationTick + PERIOD_TASK_SIM; // Calculate next activation time;
                       so.delayUntilTick(nextActivationTick);
            
                       pisoAct++;
            
                       
                       if (pisoDest == pisoAct) 
                       {
                          state= LLEGADA;
                       } 
        break;
              case BAJANDO:
                      // Autosuspend until time
                       nextActivationTick = so.getTick();
                       nextActivationTick = nextActivationTick + PERIOD_TASK_SIM; // Calculate next activation time;
                       so.delayUntilTick(nextActivationTick);
            
                       pisoAct--;
                       if (pisoDest == pisoAct) 
                       {
                          state= LLEGADA;
                       } 
         break;
               case LLEGADA:
                     Serial.println("EN LLEGADA");
                     Serial.print("+++++");
                     Serial.print("Antes del CAN pisoAct:");
                     Serial.println(pisoAct);
                     state = PARADO;
                     sprintf(mensaje,"Piso %i alcanzado", pisoDest);
                     so.waitSem(sCANControl);
                         while (CAN.checkPendingTransmission() == CAN_TXPENDING);
                         CAN.sendMsgBufNonBlocking(ID_SIMULADOR_CAMBIO_PISO, CAN_EXTID, sizeof(INT8U), (INT8U *) &pisoAct);
                     so.signalSem(sCANControl);
                     Serial.print("-----");
                     Serial.print("Despues del CAN pisoAct:");
                     Serial.println(pisoAct);
                     hib.ledOff(pisoAct-1);  // apagamos el led del piso 
                     so.clearFlag(fActControl, maskUpDown);
               break;
      }
     // Actualizamos el LCD:
     so.waitSem(sLCD);
       hib.lcdClear();
       hib.lcdSetCursorFirstLine();
       hib.lcdPrint(mensaje);
     so.signalSem(sLCD);
     hib.d7sPrintDigit((uint8_t) pisoAct, hib.RIGHT_7SEG_DIS);

   }
 }


// TAREA SIMULAR PUERTAS

void TaskSimuladorPuertas() 
{
  const unsigned char mask = (maskCerrarPuertas | maskAbrirPuertas);
  unsigned char flagValue;
  char mensaje[16];
   unsigned long nextActivationTick;
  
  while(1){
     // Wait until any of the bits of the flag fActControl
     // indicated by the bits of maskUpDown are set to '1' 
     so.waitFlag(fActControl, mask);
     
     flagValue = so.readFlag(fActControl);

     so.clearFlag(fActControl, mask);
    
     switch(flagValue) {
      
              case maskAbrirPuertas:
                
                //Serial.print("abriendo   ");
                // Clear the flag fActControl to not process the same event twice
                sprintf(mensaje,"Abriendo puertas");
                so.waitSem(sLCD);
                   hib.lcdSetCursorSecondLine();
                   hib.lcdPrint(mensaje);
                so.signalSem(sLCD);

        
                 // Autosuspend until time
              
                 nextActivationTick = so.getTick();
                 nextActivationTick = nextActivationTick + PERIOD_TASK_SIM; // Calculate next activation time;
                 so.delayUntilTick(nextActivationTick);
                 
                so.setFlag(fActControl, maskCerrarPuertas);
      break;
                case maskCerrarPuertas:
                  sprintf(mensaje,"Cerrando puertas");
                  so.waitSem(sLCD);
                     hib.lcdSetCursorSecondLine();
                     hib.lcdPrint(mensaje);
                  so.signalSem(sLCD);
          
                  // Autosuspend until time
                   nextActivationTick = so.getTick();
                   nextActivationTick = nextActivationTick + PERIOD_TASK_SIM; // Calculate next activation time;
                   so.delayUntilTick(nextActivationTick);
          
                   so.waitSem(sLCD);
                      hib.lcdClear();
                   so.signalSem(sLCD);
                break;
               }
  }
}
    
          
  
// When awake, then print the new adc adquired value on LCD

// preguntar a manuel si lo del adcHook esta bien :) (no queremos que envíe por CAN todo el rato el peso, solo cuando cambia)
void TaskBascula()
{
  boolean superado = false;
  while(1)
  {
    
    // Wait until any of the bits of the flag fExtEvent
    // indicated by the bits of maskAdcEvent are set to '1'        
    so.waitFlag(fExtEvent, maskAdcEvent);
    
    // Clear the flag fExtEvent to not process the same event twice
    so.clearFlag(fExtEvent, maskAdcEvent);
    
    if(!superado && adcValue >= PESO_MAX)
    {
      Serial.println("peso max superado");
        superado = true;
        so.waitSem(sLCD);
          hib.lcdClear();
          hib.lcdSetCursorFirstLine();
          hib.lcdPrint("¡PESO MAXIMO ");
          hib.lcdSetCursorSecondLine();
          hib.lcdPrint("ALCANZADO! ");
        so.signalSem(sLCD);
        so.waitSem(sCANControl);
          while (CAN.checkPendingTransmission() == CAN_TXPENDING);
            //envíamos por bus CAN la primera tecla almazenada en el array lastKeys 
            CAN.sendMsgBufNonBlocking(ID_BASCULA, CAN_EXTID, sizeof(adcValue), (INT8U *) &adcValue);
        so.signalSem(sCANControl);
    }else if(superado && adcValue < PESO_MAX){
      Serial.println("ya no hay peso maximo");
        superado = false;
        so.waitSem(sLCD);
          hib.lcdClear();
        so.signalSem(sLCD);
        so.waitSem(sCANControl);
          while (CAN.checkPendingTransmission() == CAN_TXPENDING);
            //envíamos por bus CAN la primera tecla almazenada en el array lastKeys
            
            CAN.sendMsgBufNonBlocking(ID_BASCULA, CAN_EXTID, sizeof(adcValue), (INT8U *) &adcValue);
        so.signalSem(sCANControl);
    }

    if(adcValue == 0){
      Serial.println("peso=0 Luces apagadas");
      so.waitSem(sLCD);
        hib.lcdClear();
        hib.lcdSetCursorFirstLine();
        hib.lcdPrint("Apagando luces");
      so.signalSem(sLCD);
    }
    
  }
}


// Periodically sample both temperature sensors
// and print their values on the LCD
void TaskTEM()
{
  unsigned long nextActivationTick;
  float leftTemperature, rightTemperature;
  const uint8_t TEMP_MAX = 35;
  nextActivationTick = so.getTick();
  while(1)
  {
    // Sample left and right handed temperature sensors
    
    leftTemperature = hib.temReadCelsius(hib.LEFT_TEM_SENS);
    rightTemperature = hib.temReadCelsius(hib.RIGHT_TEM_SENS);

    temp = (uint8_t) ((leftTemperature + rightTemperature)/2);
    // Autosuspend until time
    nextActivationTick = nextActivationTick + PERIOD_TASK_TEM; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);  
    if (temp>TEMP_MAX ){
      so.setFlag(fIncendio, maskTemp);
    }
  }
}

// Periodically sample both LDR sensors,
// map each LDR value to a single digit,
// print the left-handed one on the left-handed 7-seg display and
// print the right-handed one on the right-handed 7-seg display
void TaskLDR()
{
  unsigned long nextActivationTick;
  uint16_t leftLDR, rightLDR;
  float mapedRealValue;
  uint16_t LIGHT_MAX = 1000; 
  
  nextActivationTick = so.getTick();
  
  while(1)
  {
    // Sample left and right handed temperature sensors
    
    leftLDR = hib.ldrReadAdc(hib.LEFT_LDR_SENS);
    rightLDR = hib.ldrReadAdc(hib.RIGHT_LDR_SENS);

    // Map each LDR value to one digit
    // and print each on the corresponding 7-seg display

    light = (leftLDR + rightLDR)/2;
    // Autosuspend until time
    nextActivationTick = nextActivationTick + PERIOD_TASK_LDR; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);    

    if (light>LIGHT_MAX ){
      so.setFlag(fIncendio, maskLight);
    }
  }
}


/*****************
  TaskIncendio
*****************/

void TaskIncendio() 
{
  const unsigned char mask = (maskTemp && maskLight);
  boolean en_incendio = false;
  
   while(1)
  {     
    so.waitFlag(fIncendio, mask);
    so.clearFlag(fIncendio, mask);

     if(!en_incendio){
        so.waitSem(sCANControl);
            while (CAN.checkPendingTransmission() == CAN_TXPENDING);
              //envíamos por bus CAN la primera tecla almazenada en el array lastKeys 
              //preguntar
              CAN.sendMsgBufNonBlocking(ID_INCENDIO, CAN_EXTID, sizeof(INT8U), (INT8U *) &temp);
        so.signalSem(sCANControl);
        en_incendio = true;
     }
     
     // De momento solo para un incendio en la ejecución de las placas
    
  }
}

/*****************
  MAIN PROGRAM
******************/

void setup() {
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


void loop() {
      Serial.println("Placa 1: tareas de sensorización y actuación del ascensor");

      
      //el ascensor se encuentra en el piso 1 inicialmente
      hib.d7sPrintDigit((uint8_t) 1, hib.RIGHT_7SEG_DIS);
      
      // Definition and initialization of semaphores
      sCANControl = so.defSem(1); // intially accesible
      sLCD = so.defSem(1); // intially accesible
      sPisoDest = so.defSem(1); // intially accesible
      
      // Definition and initialization of flags
      fExtEvent = so.defFlag();
      fActControl = so.defFlag();
      fIncendio = so.defFlag();
      
      // Definition and initialization of tasks
      so.defTask(TaskBascula, PRIO_TASK_ADC);
      so.defTask(TaskPanelPulsado, PRIO_TASK_PP);
      so.defTask(TaskSimuladorCambioPiso, PRIO_TASK_SIM_PISOS);
      so.defTask(TaskTEM, PRIO_TASK_TEM);
      so.defTask(TaskIncendio, PRIO_TASK_INC);
      so.defTask(TaskLDR, PRIO_TASK_LDR);
      so.defTask(TaskSimuladorPuertas, PRIO_TASK_SIM_PUERT);

      // Set up keypad interrupt
      // expected time between keystrokes is set to 100 ms
      hib.keySetIntDriven(100, keyHook); 
      
      //Set up adc
      hib.adcSetTimerDriven(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, adcHook);
  
      //Set up timer 5 so that the SO can regain the CPU every tick
      hib.setUpTimer5(TIMER_TICKS_FOR_50ms, TIMER_PSCALER_FOR_50ms, timer5Hook);

      // Start mutltasking (program does not return to 'main' from here on)
      so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
}
