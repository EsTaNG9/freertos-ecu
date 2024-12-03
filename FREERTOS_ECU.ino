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
//#include <SimplyAtomic.h>

#define BIT_DECODER_2ND_DERIV           0 //The use of the 2nd derivative calculation is limited to certain decoders. This is set to either true or false in each decoders setup routine
#define BIT_DECODER_IS_SEQUENTIAL       1 //Whether or not the decoder supports sequential operation
#define BIT_DECODER_UNUSED1             2
#define BIT_DECODER_HAS_SECONDARY       3 //Whether or not the decoder supports fixed cranking timing
#define BIT_DECODER_HAS_FIXED_CRANKING  4
#define BIT_DECODER_VALID_TRIGGER       5 //Is set true when the last trigger (Primary or secondary) was valid (ie passed filters)
#define BIT_DECODER_TOOTH_ANG_CORRECT   6 //Whether or not the triggerToothAngle variable is currently accurate. Some patterns have times when the triggerToothAngle variable cannot be accurately set.
#define BIT_STATUS3_HALFSYNC            4 //shows if there is only sync from primary trigger, but not from secondary.
#define BIT_ENGINE_CRANK    			1   // Engine cranking
#define TRIGGER_FILTER_OFF              0
#define TRIGGER_FILTER_LITE             1
#define TWO_STROKE          			1U
#define IGN_MODE_WASTED    				0U
#define IGN_MODE_SINGLE     			1U
#define IGN_MODE_WASTEDCOP  			2U
#define IGN_MODE_SEQUENTIAL 			3U
#define IGN_MODE_ROTARY     			4U

#define INJ_PAIRED 						0
#define INJ_SEMISEQUENTIAL 				1
#define INJ_BANKED         				2
#define INJ_SEQUENTIAL      			3

#define TRIGGER_FILTER_MEDIUM           2
#define TRIGGER_FILTER_AGGRESSIVE       3
#define SEC_TRIGGER_POLL 				2
#define SEC_TRIGGER_TOYOTA_3  4

#define MICROS_PER_SEC		 INT32_C(1000000)
#define MICROS_PER_MIN 		 INT32_C(MICROS_PER_SEC*60U)
#define MICROS_PER_HOUR 	 INT32_C(MICROS_PER_MIN*60U)
#define CAM_SPEED    		 1U
#define CRANK_SPEED    		 0U
#define SEC_TRIGGER_SINGLE 	 0
#define MICROS_PER_DEG_1_RPM INT32_C(166667)
#define MAX_RPM INT16_C(18000)

//Handy bitsetting macros
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : bitClear((var), (pos)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))

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

//fnctions
static void setFilter(unsigned long);
bool engineIsRunning(uint32_t);

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

//220 bytes free
extern volatile uint8_t decoderState;

const uint8_t teethNum = 35;
const uint8_t triggerMissingTeeth = 1;
volatile unsigned long pulseCount = 0; // Counts pulses
volatile unsigned long lapCount = 0; // Counts pulses
unsigned long lastTime = 0; // Last time RPM was calculated
unsigned int rpm = 0; // RPM value
unsigned int maxrpm = 0; // RPM value
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long mediumInterruptTime = 0;
volatile unsigned long nowTime = 0;
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
static volatile byte toothSystemCount = 0; //Used for decoders such as Audi 135 where not every tooth is used for calculating crank angle. This variable stores the actual number of teeth, not the number being used to calculate crank angle
volatile unsigned long toothSystemLastToothTime = 0; //As below, but used for decoders where not every tooth count is used for calculation
static volatile unsigned long toothLastToothTime = 0; //The time (micros()) that the last tooth was registered
static volatile unsigned long toothLastSecToothTime = 0; //The time (micros()) that the last tooth was registered on the secondary input
volatile unsigned long toothLastThirdToothTime = 0; //The time (micros()) that the last tooth was registered on the second cam input
volatile unsigned long toothLastMinusOneToothTime = 0; //The time (micros()) that the tooth before the last tooth was registered
volatile unsigned long toothLastMinusOneSecToothTime = 0; //The time (micros()) that the tooth before the last tooth was registered on secondary input
volatile unsigned long toothLastToothRisingTime = 0; //The time (micros()) that the last tooth rose (used by special decoders to determine missing teeth polarity)
volatile unsigned long toothLastSecToothRisingTime = 0; //The time (micros()) that the last tooth rose on the secondary input (used by special decoders to determine missing teeth polarity)
volatile unsigned long targetGap2;
volatile unsigned long targetGap3;
volatile unsigned long toothOneTime = 0; //The time (micros()) that tooth 1 last triggered
volatile unsigned long toothOneMinusOneTime = 0; //The 2nd to last time (micros()) that tooth 1 last triggered
volatile bool revolutionOne = 0; // For sequential operation, this tracks whether the current revolution is 1 or 2 (not 1)
volatile bool revolutionLastOne = 0; // used to identify in the rover pattern which has a non unique primary trigger something unique - has the secondary tooth changed.
static volatile unsigned int secondaryToothCount; //Used for identifying the current secondary (Usually cam) tooth for patterns with multiple secondary teeth
volatile bool perToothIgn = false;
extern volatile unsigned long triggerFilterTime; // The shortest time (in uS) that pulses will be accepted
volatile unsigned int thirdToothCount; //Used for identifying the current third
extern uint16_t fixedCrankingOverride = 0;

uint16_t triggerActualTeeth;
//volatile unsigned long triggerFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
volatile unsigned long triggerSecFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the secondary input
volatile unsigned long triggerThirdFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the Third input

volatile uint8_t decoderState = 0;

unsigned int triggerSecFilterTime_duration; // The shortest valid time (in uS) pulse DURATION
volatile uint16_t triggerToothAngle; //The number of crank degrees that elapse per tooth
byte checkSyncToothCount; //How many teeth must've been seen on this revolution before we try to confirm sync (Useful for missing tooth type decoders)
byte triggerFilter;
unsigned long elapsedTime;
unsigned long lastCrankAngleCalc;

extern volatile unsigned long curTime;
extern volatile unsigned long curGap;
extern volatile unsigned long curTime2;
extern volatile unsigned long curGap2;
extern volatile unsigned long lastGap;

uint16_t ignition1EndTooth = 0;
uint16_t ignition2EndTooth = 0;
uint16_t ignition3EndTooth = 0;
uint16_t ignition4EndTooth = 0;
uint16_t ignition5EndTooth = 0;
uint16_t ignition6EndTooth = 0;
uint16_t ignition7EndTooth = 0;
uint16_t ignition8EndTooth = 0;

int16_t toothAngles[24]; //An array for storing fixed tooth angles. Currently sized at 24 for the GM 24X decoder, but may grow later if there are other decoders that use this style

//estruturas
/**< The master global "live" status struct. Contains all values that are updated frequently and used across modules */
struct cStatus{
	bool hasSync;
	uint16_t RPM;
	bool status3;
	uint16_t startRevolutions;
	int engine;
	int syncLossCounter;
};

void setup_decoder (void){
	BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
	  triggerToothAngle = 360 / triggerTeeth; //The number of degrees that passes from tooth to tooth

	  triggerActualTeeth = triggerTeeth - triggerMissingTeeth; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
	  triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise

	  triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U));
	  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
	  checkSyncToothCount = (triggerTeeth) >> 1; //50% of the total teeth.
	  toothLastMinusOneToothTime = 0;
	  toothCurrentCount = 0;
	  secondaryToothCount = 0;
	  thirdToothCount = 0;
	  toothOneTime = 0;
	  toothOneMinusOneTime = 0;
	  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerToothAngle * (triggerMissingTeeth + 1U)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)

	  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
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

        if (interval >= 200) { // Calculate every xms (x second)



            rpm = ((pulseCount * 60 * 1000) / teethNum) / interval; // RPM calculation
            pulseCount = 0; // Reset pulse count
            lastTime = currentTime;

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

		vTaskDelay( 500 / portTICK_PERIOD_MS);
	  }
}

bool engineIsRunning(uint32_t curTime) {
  // Check how long ago the last tooth was seen compared to now.
  // If it was more than MAX_STALL_TIME then the engine is probably stopped.
  // toothLastToothTime can be greater than curTime if a pulse occurs between getting the latest time and doing the comparison
  //ATOMIC() {
    return (toothLastToothTime > curTime) || ((curTime - toothLastToothTime) < MAX_STALL_TIME);
  //}
  //return false; // Just here to avoid compiler warning.
}

// Interrupção gerada na transição do sinal do sensor de Hall
void IRAM_ATTR onPulse( void )
{
	cStatus currentStatus;
	curTime = micros();
	   curGap = curTime - toothLastToothTime;
	   if ( curGap >= triggerFilterTime ) //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
	   {
	     toothCurrentCount++; //Increment the tooth counter
	     BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

	     //if(toothCurrentCount > checkSyncToothCount || currentStatus.hasSync == false)
	      if( (toothLastToothTime > 0) && (toothLastMinusOneToothTime > 0) )
	      {
	        bool isMissingTooth = false;

	        /*
	        Performance Optimisation:
	        Only need to try and detect the missing tooth if:
	        1. WE don't have sync yet
	        2. We have sync and are in the final 1/4 of the wheel (Missing tooth will/should never occur in the first 3/4)
	        3. RPM is under 2000. This is to ensure that we don't interfere with strange timing when cranking or idling. Optimisation not really required at these speeds anyway
	        */
	        if( (currentStatus.hasSync == false) || (currentStatus.RPM < 2000) || (toothCurrentCount >= (3 * triggerActualTeeth >> 2)) )
	        {
	          //Begin the missing tooth detection
	          //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
	          if(triggerMissingTeeth == 1) { targetGap = (3 * (toothLastToothTime - toothLastMinusOneToothTime)) >> 1; } //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one) (Uses bitshift to multiply by 3 then divide by 2. Much faster than multiplying by 1.5)
	          else { targetGap = ((toothLastToothTime - toothLastMinusOneToothTime)) * triggerMissingTeeth; } //Multiply by 2 (Checks for a gap 2x greater than the last one)

	          if( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { curGap = 0; }

	          if ( (curGap > targetGap) || (toothCurrentCount > triggerActualTeeth) )
	          {
	            //Missing tooth detected

	            isMissingTooth = true;
	            if( (toothCurrentCount < triggerActualTeeth) && (currentStatus.hasSync == true) )
	            {
	                //This occurs when we're at tooth #1, but haven't seen all the other teeth. This indicates a signal issue so we flag lost sync so this will attempt to resync on the next revolution.
	                currentStatus.hasSync = false;
	                BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //No sync at all, so also clear HalfSync bit.
	                currentStatus.syncLossCounter++;
	            }
	            //This is to handle a special case on startup where sync can be obtained and the system immediately thinks the revs have jumped:
	            //else if (currentStatus.hasSync == false && toothCurrentCount < checkSyncToothCount ) { triggerFilterTime = 0; }
	            else
	            {
	                if((currentStatus.hasSync == true) || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC))
	                {
	                  currentStatus.startRevolutions++; //Counter
	                  if ( TrigSpeed == CAM_SPEED ) { currentStatus.startRevolutions++; } //Add an extra revolution count if we're running at cam speed
	                }
	                else { currentStatus.startRevolutions = 0; }

	                toothCurrentCount = 1;
	                /*if (trigPatternSec == SEC_TRIGGER_POLL) // at tooth one check if the cam sensor is high or low in poll level mode
	                {
	                  if (PollLevelPolarity == READ_SEC_TRIGGER()) { revolutionOne = 1; }
	                  else { revolutionOne = 0; }
	                }
	                else {revolutionOne = !revolutionOne;} //Flip sequential revolution tracker if poll level is not used*/
	                revolutionOne = !revolutionOne; //Extraido do if comentado pq n temos cam sensor
	                toothOneMinusOneTime = toothOneTime;
	                toothOneTime = curTime;

	                //if Sequential fuel or ignition is in use, further checks are needed before determining sync
	                if( (sparkMode == IGN_MODE_SEQUENTIAL) || (injLayout == INJ_SEQUENTIAL) )
	                {
	                  //If either fuel or ignition is sequential, only declare sync if the cam tooth has been seen OR if the missing wheel is on the cam
	                  if( (secondaryToothCount > 0) || (TrigSpeed == CAM_SPEED) || (trigPatternSec == SEC_TRIGGER_POLL) || (strokes == TWO_STROKE) )
	                  {
	                    currentStatus.hasSync = true;
	                    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //the engine is fully synced so clear the Half Sync bit
	                  }
	                  else if(currentStatus.hasSync != true) { BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If there is primary trigger but no secondary we only have half sync.
	                }
	                else { currentStatus.hasSync = true;  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If nothing is using sequential, we have sync and also clear half sync bit
	                if(trigPatternSec == SEC_TRIGGER_SINGLE || trigPatternSec == SEC_TRIGGER_TOYOTA_3) //Reset the secondary tooth counter to prevent it overflowing, done outside of sequental as v6 & v8 engines could be batch firing with VVT that needs the cam resetting
	                {
	                  secondaryToothCount = 0;
	                }

	                triggerFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
	                toothLastMinusOneToothTime = toothLastToothTime;
	                toothLastToothTime = curTime;
	                BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //The tooth angle is double at this point
	            }
	          }
	        }

	        if(isMissingTooth == false)
	        {
	          //Regular (non-missing) tooth
	          setFilter(curGap);
	          toothLastMinusOneToothTime = toothLastToothTime;
	          toothLastToothTime = curTime;
	          BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
	        }
	      }
	      else
	      {
	        //We fall here on initial startup when enough teeth have not yet been seen
	        toothLastMinusOneToothTime = toothLastToothTime;
	        toothLastToothTime = curTime;
	      }


	      //NEW IGNITION MODE
	      if( (perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
	      {
	        int16_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + triggerAngle;

	        crankAngle = ignitionLimits(crankAngle);
	        checkPerToothTiming(crankAngle, toothCurrentCount);
	      }
	   }
}

#define MIN_CYCLES_FOR_ENDCOMPARE 6

inline void adjustCrankAngle(IgnitionSchedule &schedule, int endAngle, int crankAngle) {

	cStatus currentStatus;

  if( (schedule.Status == RUNNING) ) {
    SET_COMPARE(schedule.compare, schedule.counter + uS_TO_TIMER_COMPARE( angleToTimeMicroSecPerDegree( ignitionLimits( (endAngle - crankAngle) ) ) ) );
  }
  else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) {
    schedule.endCompare = schedule.counter + uS_TO_TIMER_COMPARE( angleToTimeMicroSecPerDegree( ignitionLimits( (endAngle - crankAngle) ) ) );
    schedule.endScheduleSetByDecoder = true;
  }
}

/**
On decoders that are enabled for per tooth based timing adjustments, this function performs the timer compare changes on the schedules themselves
For each ignition channel, a check is made whether we're at the relevant tooth and whether that ignition schedule is currently running
Only if both these conditions are met will the schedule be updated with the latest timing information.
If it's the correct tooth, but the schedule is not yet started, calculate and an end compare value (This situation occurs when both the start and end of the ignition pulse happen after the end tooth, but before the next tooth)
*/
static inline void checkPerToothTiming(int16_t crankAngle, uint16_t currentTooth){

	cStatus currentStatus;

  if ( (fixedCrankingOverride == 0) && (currentStatus.RPM > 0) )
  {
    if ( (currentTooth == ignition1EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule1, ignition1EndAngle, crankAngle);
    }
  }
}

/**
 * Sets the new filter time based on the current settings.
 * This ONLY works for even spaced decoders.
 */
static void setFilter(unsigned long curGap)
{

  switch(triggerFilter)
  {
    case TRIGGER_FILTER_OFF:
      triggerFilterTime = 0;
      break;
    case TRIGGER_FILTER_LITE:
      triggerFilterTime = curGap >> 2;
      break;
    case TRIGGER_FILTER_MEDIUM:
      triggerFilterTime = curGap >> 1;
      break;
    case TRIGGER_FILTER_AGGRESSIVE:
      triggerFilterTime = (curGap * 3) >> 2;
      break;
    default:
      triggerFilterTime = 0;
      break;
  }
}

static inline int16_t nudge(int16_t min, int16_t max, int16_t value, int16_t nudgeAmount)
{
    if (value<min) { return value + nudgeAmount; }
    if (value>max) { return value - nudgeAmount; }
    return value;
}

static inline int16_t ignitionLimits(int16_t angle)
{
    return nudge(0, CRANK_ANGLE_MAX_IGN, angle, CRANK_ANGLE_MAX_IGN);
}

//------------------------------------------------------------------------------
void loop()
{
  vTaskDelete( NULL );
}
