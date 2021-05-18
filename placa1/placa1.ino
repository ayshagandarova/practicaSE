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
#define PRIO_TASK_ADC 4
#define PRIO_TASK_KEY 3
#define PRIO_TASK_SIM 3
#define PRIO_TASK_TEM 2
#define PRIO_TASK_LDR 1


#define PERIOD_TASK_PP 3
#define PERIOD_TASK_TEM 4
#define PERIOD_TASK_LDR 6
#define PERIOD_TASK_LDR 3000 //no se si esto es lokura

  // stores every new adc adquired value
  // shared between: adchook (writes) and TaskBascula (reads)
volatile uint16_t adcValue = 0; // critical region beetween adcHook and TaskBascula

  // guarda el piso de destino recibido por el bus CAN
  // shared between: isrCAN (writes) and TaskSimuladorCambioPiso (reads)
volatile uint8_t pisoDest = 0;

  // stores last pressed keypad's key
  // shared between: keyHook (writes) and TaskPanelPulsado (reads)
volatile uint8_t key = 0;
#define NUM_LAST_KEYS 10;
uint8_t lastKeys[NUM_LAST_KEYS]; //array con utilización FIFO para guardar las teclas pulsadas

// TX consts and vars
const uint32_t ID_PANEL_PULSADO = 0x00022449; 
const uint32_t ID_INCENDIO = 0x00022450; 
const uint32_t ID_BASCULA = 0x00022451;  

const int SPI_CS_PIN = 9;

HIB hib;
MCP_CAN CAN(SPI_CS_PIN);
SO so;

/***************************
  Declaration of semaphores
***************************/ 
Sem sCANControl;
Sem sLCD;
Sem sKeyBuffer; //para acceder al buffer de teclas pulsadas
Sem sPisoDest;

/********************************
  Declaration of flags and masks
*********************************/
Flag fExtEvent;
Flag fActControl;

 // Define masks for adc and keypad event respectively
const unsigned char maskAdcEvent = 0x01; // represents new adc value adquired
const unsigned char maskUpDown = 0x01; //representa subir o bajar de piso


/*****************
  ADC hook
******************/ 

void adcHook(uint16_t newAdcAdquiredValue)
{
  adcValue = newAdcAdquiredValue;

  // Awake TaskBascula by setting to '1' the bits of fExtEvent indicated by maskAdcEvent
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
  uint8_t posKey = 0;
  boolean keyFound = false;
  
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
    
  //si la tecla no está en el buffer de teclas la añadimos en la siguiente posición disponible
  posKey = 0;
  so.waitSem(sKeyBuffer);
  for (int i=0; i<NUM_LAST_KEYS; i++){
     if (lastKeys[i] == key){
        keyFound = true;
        break;
     } 
     if (lastKeys[i] == -1){
        posKey = i;
        break;
     }
   }

   if(keyFound != true){
     lastKeys[posKey] = key;
   }
   so.signalSem(sKeyBuffer);
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
        so.waitSem(sPisoDest);
        pisoDest = rxAct;
        so.signalSem(sPisoDest);
        so.setFlag(fActControl, maskUpDown);
        break;
      case 7:
        so.waitSem(sLCD);
        hib.lcdClear();
        hib.lcdSetCursorFirstLine();
        hib.lcdPrint("Cerrando puertas");
        so.signalSem(sLCD);
        break;
      case 8:
        so.waitSem(sLCD);
        hib.lcdClear();
        hib.lcdSetCursorFirstLine();
        hib.lcdPrint("Abriendo puertas");
        so.signalSem(sLCD);
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
  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();
  
  while(1)
  {
    
      // Check whether or not the TX buffer is available (no Tx still pending)
      // to request transmission of key
      if(lastKeys[0] != -1){ //tengo que usar semáforo aquí también???
        so.waitSem(sCANControl);
        if (CAN.checkPendingTransmission() != CAN_TXPENDING){
          //envíamos por bus CAN la primera tecla almazenada en el array lastKeys 
          CAN.sendMsgBufNonBlocking(ID_PANEL_PULSADO, CAN_EXTID, sizeof(INT8U), (INT8U *) &lastKeys);
          
          //rotamos a la izquierda los valores de lastKeys, eliminando así el primero
          so.waitSem(sKeyBuffer);
          for(int i=0;i<NUM_LAST_KEYS-1;i++)
          {
            lastKeys[i]=lastKeys[i+1];
          }
          lastKeys[NUM_LAST_KEYS] = -1;
          so.signalSem(sKeyBuffer);
        }
        so.signalSem(sCANControl);
      }
      
      // Autosuspend until time
      nextActivationTick = nextActivationTick + PERIOD_TASK_PP; // Calculate next activation time;
      so.delayUntilTick(nextActivationTick);
  }
}

// Autosuspends itself until the event: "kestroke" (key pressed)
// When awake, then print the actual floor on the right-handed 7-seg display

void TaskSimuladorCambioPiso(){
  uint8_t pisoAct = 0;
  uint8_t difPisos = 0;
  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();
  
  while(1)
  {
    // Wait until any of the bits of the flag fActControl
    // indicated by the bits of maskUpDown are set to '1'        
    so.waitFlag(fActControl, maskUpDown);
    // Clear the flag fActControl to not process the same event twice
    so.clearFlag(fActControl, maskUpDown);

    difPisos = abs(pisoDest-pisoAct);
    
    if(pisoDest > pisoAct){
        so.waitSem(sLCD);
        hib.lcdClear();
        hib.lcdSetCursorFirstLine();
        hib.lcdPrint("Subiendo...");
        so.signalSem(sLCD);
        pisoAct++;
        hib.d7sPrintDigit((uint8_t) pisoAct, hib.RIGHT_7SEG_DIS);
    } else if (pisoDest < pisoAct){
        so.waitSem(sLCD);
        hib.lcdClear();
        hib.lcdSetCursorFirstLine();
        hib.lcdPrint("Bajando...");
        so.signalSem(sLCD);
        pisoAct--;
        hib.d7sPrintDigit((uint8_t) pisoAct, hib.RIGHT_7SEG_DIS);
    } else {
        so.waitSem(sLCD);
        hib.lcdClear();
        hib.lcdSetCursorFirstLine();
        hib.lcdPrint("Abriendo puertas");
        so.signalSem(sLCD);
    }
  
    //hace tantas activaciones periódicas como pisos tiene que subir/bajar
    if(difPisos > 0){
      // Autosuspend until time
      nextActivationTick = nextActivationTick + PERIOD_TASK_SIM; // Calculate next activation time;
      so.delayUntilTick(nextActivationTick);
    }
  }
}


// Autosuspends itself until the event: "new adc adquired value"
// When awake, then print the new adc adquired value on LCD

void TaskBascula()
{
  char str[16];
  uint16_t auxAdcValue;
  float auxSampledSensor;
  const PESO_MAX = 400; //cual es el max del adc???????
  while(1)
  {
    // Wait until any of the bits of the flag fExtEvent
    // indicated by the bits of maskAdcEvent are set to '1'        
    so.waitFlag(fExtEvent, maskAdcEvent);
    // Clear the flag fExtEvent to not process the same event twice
    so.clearFlag(fExtEvent, maskAdcEvent);

    auxAdcValue = adcValue; // latch adquired sensor value in a local var
    auxSampledSensor = ((float) auxAdcValue) / 2.0;

    so.waitSem(sCANControl);
        if (CAN.checkPendingTransmission() != CAN_TXPENDING){
          //envíamos por bus CAN la primera tecla almazenada en el array lastKeys 
          CAN.sendMsgBufNonBlocking(ID_BASCULA, CAN_EXTID, sizeof(float), (float *) &auxSampledSensor);
        }
    so.signalSem(sCANControl);
    
    if(auxSampledSensor >= PESO_MAX){
      so.waitSem(sLCD);
        hib.lcdClear();
        hib.lcdSetCursorFirstLine();
        hib.lcdPrint("¡PESO MAXIMO ");
        hib.lcdSetCursorSecondLine();
        hib.lcdPrint("ALCANZADO! ");
      so.signalSem(sLCD);
    }

    if(auxSampledSensor == 0){
      so.waitSem(sLCD);
        hib.lcdClear();
        hib.lcdSetCursorFirstLine();
        hib.lcdPrint("Apagando luces");
      so.signalSem(sLCD);
    }
  }
}


/*
// Periodically sample both temperature sensors
// and print their values on the LCD
void TaskTEM()
{
  unsigned long nextActivationTick;
  float leftTemperature, rightTemperature;
  char str[8];
  char fStr[6];
  
  nextActivationTick = so.getTick();
  
  while(1)
  {
    // Sample left and right handed temperature sensors
    
    leftTemperature = hib.temReadCelsius(hib.LEFT_TEM_SENS);
    rightTemperature = hib.temReadCelsius(hib.RIGHT_TEM_SENS);

    // Print temperatures
    
    // "LT:___._ RT:___._"
    so.waitSem(sLCD);
    
      hib.lcdSetCursor(1,3);
      dtostrf(leftTemperature,4,1,fStr);
      sprintf(str,"%s", fStr);
      hib.lcdPrint(str);

      hib.lcdSetCursor(1,11);
      dtostrf(rightTemperature,4,1,fStr);
      sprintf(str,"%s", fStr);
      hib.lcdPrint(str);
      
    so.signalSem(sLCD);

    // Autosuspend until time

    nextActivationTick = nextActivationTick + PERIOD_TASK_TEM; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);    
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
  
  nextActivationTick = so.getTick();
  
  while(1)
  {
    // Sample left and right handed temperature sensors
    
    leftLDR = hib.ldrReadAdc(hib.LEFT_LDR_SENS);
    rightLDR = hib.ldrReadAdc(hib.RIGHT_LDR_SENS);

    // Map each LDR value to one digit
    // and print each on the corresponding 7-seg display

    mapedRealValue = ( ((float)leftLDR) * 10 ) / 1024;
    leftLDR = (uint16_t) mapedRealValue;
    hib.d7sPrintDigit((uint8_t) leftLDR, hib.LEFT_7SEG_DIS);

    mapedRealValue = ( ((float)rightLDR) * 10 ) / 1024;
    rightLDR = (uint16_t) mapedRealValue;
    hib.d7sPrintDigit((uint8_t) rightLDR, hib.RIGHT_7SEG_DIS);

    // Autosuspend until time
    nextActivationTick = nextActivationTick + PERIOD_TASK_LDR; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);    
  }
}
*/

/*****************
  MAIN PROGRAM
******************/

void setup() {
  Serial.begin(115200);
  so.begin();
  hib.begin();
  hib.lcdClear();

  // Init can bus : baudrate = 500k, normal mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, false, false) != CAN_OK) {
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

      //inicializamos array de teclas pulsadas indicando que todas las posiciones están libres
      for (int i=0; i<NUM_LAST_KEYS; i++){
        lastKeys[i] = -1;
      }

      //el ascensor se encuentra en el piso 1 inicialmente
      hib.d7sPrintDigit((uint8_t) 1, hib.RIGHT_7SEG_DIS);
      
      // Definition and initialization of semaphores
      sCANControl = so.defSem(1); // intially accesible
      sLCD = so.defSem(1); // intially accesible
      sKeyBuffer = so.defSem(1); // intially accesible
      Sem sPisoDest = so.defSem(1); // intially accesible
      
      // Definition and initialization of flags
      fExtEvent = so.defFlag();
      fActControl = so.defFlag();
      
      // Definition and initialization of tasks
      so.defTask(TaskBascula, PRIO_TASK_ADC);
      so.defTask(TaskPanelPulsado, PRIO_TASK_PP);
      so.defTask(TaskSimuladorCambioPiso, PRIO_TASK_SIM);
      //so.defTask(TaskTEM, PRIO_TASK_TEM);
      //so.defTask(TaskLDR, PRIO_TASK_LDR);

      // Set up keypad interrupt
      // expected time between keystrokes is set to 100 ms
      hib.keySetIntDriven(100, KeyHook); 
      
      //Set up adc
      hib.adcSetTimerDriven(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, adcHook);
  
      //Set up timer 5 so that the SO can regain the CPU every tick
      hib.setUpTimer5(TIMER_TICKS_FOR_50ms, TIMER_PSCALER_FOR_50ms, timer5Hook);

      // Start mutltasking (program does not return to 'main' from here on)
      so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
}
