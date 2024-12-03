#include <math.h>
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define MICROS_PER_SEC		 INT32_C(1000000)
#define MICROS_PER_MIN 		 INT32_C(MICROS_PER_SEC*60U)
#define MICROS_PER_HOUR 	 INT32_C(MICROS_PER_MIN*60U)
#define CAM_SPEED    		 1U
#define CRANK_SPEED    		 0U
#define SEC_TRIGGER_SINGLE 	 0
#define MICROS_PER_DEG_1_RPM INT32_C(166667)
#define MAX_RPM INT16_C(18000)
#define TOOTH_LOG_SIZE      128
#define TOOTH_LOG_BUFFER    256
#define BIT_SQUIRT_TOOTHLOG1READY 6  //Used to flag if tooth log 1 is ready

//Handy bitsetting macros
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : bitClear((var), (pos)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))

#define IGN_MODE_WASTED    				0U
#define INJ_PAIRED 						0

#define TRIGGER_FILTER_OFF              0
#define TRIGGER_FILTER_LITE             1

// Definir os terminais do LCD
#define TFT_CS   05
#define TFT_DC   26
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18
#define TFT_RST -1 // ligar ao 3V3

// Criar um objeto tft com indicação dos terminais CS e DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,
		TFT_RST, TFT_MISO);

/* The tasks to be created. */
void calculateRPM( void *pvParameters );
void vInitDisplay( void *pvParameters );

/* The service routine for the interrupt.  This is the interrupt that the task
will be synchronized with. */
void IRAM_ATTR onPulse( void );

/*-----------------------------------------------------------*/

/* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
SemaphoreHandle_t xBinarySemaphore;

// pin to generate interrupts
const uint8_t interruptPin = 4;

const uint8_t teethNum = 35;
volatile unsigned long pulseCount = 0; // Counts pulses
volatile unsigned long lapCount = 0; // Counts pulses
unsigned long lastTime = 0; // Last time RPM was calculated
unsigned int rpm = 0; // RPM value
unsigned int maxrpm = 0; // RPM value
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long mediumInterruptTime = 0;
volatile unsigned long nowTime = 0;
volatile uint32_t toothHistory[TOOTH_LOG_SIZE]; ///< Tooth trigger history - delta time (in uS) from last tooth (Indexe…
volatile unsigned int toothHistoryIndex = 0; ///< Current index to @ref toothHistory array

//const uint8_t interruptPin = 4;
volatile uint8_t decoderState;

//const uint8_t teethNum = 35;
const uint8_t triggerMissingTeeth = 1;
byte sparkMode = IGN_MODE_WASTED;
byte injLayout = INJ_PAIRED;
byte strokes;
byte triggerTeeth = 36;  ///< The full count of teeth on the trigger wheel if there were no gaps
int CRANK_ANGLE_MAX_IGN = 360;
volatile unsigned long targetGap;
int16_t triggerAngle;
volatile unsigned long TrigSpeed = CRANK_SPEED;
volatile unsigned long trigPatternSec = SEC_TRIGGER_SINGLE;

static unsigned long MAX_STALL_TIME = MICROS_PER_SEC/2U; //The maximum time (in uS) that the system will continue to function before the engine is considered stalled/stopped. This is unique to each decoder, depending on the number of teeth etc. 500000 (half a second) is used as the default value, most decoders will be much less.
volatile uint16_t toothCurrentCount = 0; //The current number of teeth (Once sync has been achieved, this can never actually be 0
volatile unsigned long toothOneTime = 0; //The time (micros()) that tooth 1 last
volatile unsigned long toothOneMinusOneTime = 0; //The 2nd to last time (micros()) that tooth 1 last triggered
volatile bool revolutionOne = 0; // For sequential operation, this tracks whether the current revolution is 1 or 2 (not 1)
volatile bool revolutionLastOne = 0; // used to identify in the rover pattern which has a non unique primary trigger something unique - has the secondary tooth changed.
static volatile unsigned int secondaryToothCount; //Used for identifying the current secondary (Usually cam) tooth for patterns with multiple secondary teeth
volatile bool perToothIgn = false;
volatile unsigned long triggerFilterTime; // The shortest time (in uS) that pulses will be accepted
volatile unsigned int thirdToothCount; //Used for identifying the current third
extern uint16_t fixedCrankingOverride = 0;
volatile uint16_t triggerToothAngle; //The number of crank degrees that elapse per tooth
uint16_t triggerActualTeeth;
volatile uint32_t startRevolutions = 0; /**< A counter for how many revolutions have been completed since sync*/

volatile unsigned long curTime;
volatile unsigned long curGap;
extern volatile unsigned long curTime2;
extern volatile unsigned long curGap2;
extern volatile unsigned long lastGap;

volatile unsigned long toothLastMinusOneToothTime = 0; //The time (micros()) that the tooth before the last tooth was registered
static volatile unsigned long toothLastToothTime = 0; //The time (micros()) that the last tooth was registered

static void setFilter(unsigned long);
bool engineIsRunning(uint32_t);
/* The tasks to be created. */
void calculateRPM( void *pvParameters );
void vInitDisplay( void *pvParameters );

struct cStatus{
	bool hasSync;
	uint16_t RPM;
	bool status3;
	uint16_t startRevolutions;
	int engine;
	int syncLossCounter;
	byte squirt;
};

/*
Name: Missing tooth wheel
Desc: A multi-tooth wheel with one of more 'missing' teeth. The first tooth after the missing one is considered number 1 and isthe basis for the trigger angle
Note: This does not currently support dual wheel (ie missing tooth + single tooth on cam)
*/
void triggerSetup_missingTooth()
{
  triggerToothAngle = 360 / triggerTeeth; //The number of degrees that passes from tooth to tooth
  triggerActualTeeth = triggerTeeth - triggerMissingTeeth; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerFilterTime = (int)(1000000 / (MAX_RPM / 60 * triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  //secondDerivEnabled = false;
  //decoderIsSequential = false;
  MAX_STALL_TIME = (3333UL * triggerToothAngle * (triggerMissingTeeth + 1)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void setup( void )
{
	// Set loopTask max priority before deletion
	vTaskPrioritySet(NULL, configMAX_PRIORITIES-1);

	struct cStatus currentStatus;

	  // Init USART and set Baud-rate to 115200
	  Serial.begin(115200);

	  // Inicializar o tft
	 tft.begin();
	 // Colocar fundo preto
	 tft.fillScreen(ILI9341_BLACK);
	 // Definir orientação da escrita
	 tft.setRotation(0);

	 triggerSetup_missingTooth();

  /* Before a semaphore is used it must be explicitly created.  In this example a binary semaphore is created. */
  vSemaphoreCreateBinary( xBinarySemaphore );

  //Interrupção Hall Sensor
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onPulse, FALLING);

  /* Check the semaphore was created successfully. */
  if( xBinarySemaphore != NULL )
  {
    /* Create the 'handler' task.  This is the task that will be synchronized
    with the interrupt.  The handler task is created with a high priority to
    ensure it runs immediately after the interrupt exits.  In this case a
    priority of 3 is chosen. */
    xTaskCreatePinnedToCore( calculateRPM, "Calculate RPM", 1024, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore( vInitDisplay, "Display Boot", 2048, NULL, 1, NULL, 1);

  }

}

// Task to calculate RPM
void calculateRPM(void *pvParameters) {
    while (true) {
        // Calculate RPM every second
        unsigned long currentTime = millis();
        unsigned long interval = currentTime - lastTime;

        if (interval >= 1000) { // Calculate every xms (x second)



            //rpm = ((pulseCount * 60 * 1000) / teethNum) / interval; // RPM calculation
        	rpm = (toothLastToothTime - toothLastMinusOneToothTime) * triggerTeeth;
            /*pulseCount = 0; // Reset pulse count
            lastTime = currentTime;*/

            if (maxrpm < rpm){
            	maxrpm = rpm;
            }

            Serial.print("RPM: ");
            Serial.println(rpm); // Output RPM

            Serial.print("Max RPM: ");
            Serial.println(maxrpm); // Output RPM~

            Serial.print("time between pulses: ");
            Serial.println(mediumInterruptTime); // Output RPM

            /*Serial.print("Voltas de Cambota: ");
            Serial.println(lapCount); // Output RPM*/
        }

        vTaskDelay( 200 / portTICK_PERIOD_MS); // Delay for task scheduling
    }
}

void vInitDisplay(void *pvParameters) {

	for( ;; )
	  {
		tft.setCursor(0, 10);
		tft.setTextColor(ILI9341_WHITE);
		tft.setTextSize(2);
		tft.println(" Microcontroladores ");
		tft.println(" Sistemas Embebidos ");
		tft.println("     2016/2017      ");

		vTaskDelay( 500 / portTICK_PERIOD_MS);
	  }
}

static inline void addToothLogEntry(unsigned long time)
{
	cStatus currentStatus;
  //High speed tooth logging history
  toothHistory[toothHistoryIndex] = time;
  if(toothHistoryIndex == (TOOTH_LOG_BUFFER-1))
  { toothHistoryIndex = 0; BIT_CLEAR(currentStatus.squirt, BIT_SQUIRT_TOOTHLOG1READY); } //The tooth log ready bit is cleared to ensure that we only get a set of concurrent values.
  else
  { toothHistoryIndex++; }
}

// Interrupção gerada na transição do sinal do sensor de Hall
void IRAM_ATTR onPulse( void )
{
	// http://www.msextra.com/forums/viewtopic.php?f=94&t=22976
	   // http://www.megamanual.com/ms2/wheel.htm
	cStatus currentStatus;
	   curTime = micros();
	   curGap = curTime - toothLastToothTime;
	   if ( curGap < triggerFilterTime ) { return; } //Debounce check. Pulses should never be less than triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
	   toothCurrentCount++; //Increment the tooth counter

	   addToothLogEntry(curGap);

	   //Begin the missing tooth detection
	   //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
	   if(triggerMissingTeeth == 1) { targetGap = (3 * (toothLastToothTime - toothLastMinusOneToothTime)) >> 1; } //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one) (Uses bitshift to multiply by 3 then divide by 2. Much faster than multiplying by 1.5)
	   else { targetGap = ((toothLastToothTime - toothLastMinusOneToothTime)) * 2; } //Multiply by 2 (Checks for a gap 2x greater than the last one)

	   if ( curGap > targetGap || toothCurrentCount > triggerActualTeeth)
	   {
	     toothCurrentCount = 1;
	     toothOneMinusOneTime = toothOneTime;
	     toothOneTime = curTime;
	     currentStatus.hasSync = true;
	     startRevolutions++; //Counter
	   }
	   else
	   {
	     //Filter can only be recalc'd for the regular teeth, not the missing one.
	     setFilter(curGap);
	   }

	   toothLastMinusOneToothTime = toothLastToothTime;
	   toothLastToothTime = curTime;
}

static inline void setFilter(unsigned long curGap)
{
   /*if(configPage2.triggerFilter == 1) { triggerFilterTime = curGap >> 2; } //Lite filter level is 25% of previous gap
   else if(configPage2.triggerFilter == 2) { triggerFilterTime = curGap >> 1; } //Medium filter level is 50% of previous gap
   else if (configPage2.triggerFilter == 3) { triggerFilterTime = (curGap * 3) >> 2; } //Aggressive filter level is 75% of previous gap
   else { triggerFilterTime = 0; } //trigger filter is turned off.*/
   triggerFilterTime = curGap >> 2;
}

//------------------------------------------------------------------------------
void loop()
{
  vTaskDelete( NULL );
}
