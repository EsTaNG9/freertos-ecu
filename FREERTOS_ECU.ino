#include <math.h>
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

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
//void calculateRPM( void *pvParameters );
//void calculateRPMandAngleTask( void *pvParameters );
void displayTask( void *pvParameters );
//void vInitDisplay( void *pvParameters );

/* The service routine for the interrupt.  This is the interrupt that the task
will be synchronized with. */
void IRAM_ATTR onPulse( void );

/*-----------------------------------------------------------*/

// Mutex and semaphore
SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t pulse_semaphore;
SemaphoreHandle_t rpm_mutex;

// pin to generate interrupts
const uint8_t interruptPin = 4;

const uint8_t teethNum = 35;
volatile unsigned long pulseCount = 0; // Counts pulses
volatile unsigned long lapCount = 0; // Counts pulses
unsigned int maxrpm = 0; // RPM value
unsigned long pulseInterval = 0; //Interval between pulses
volatile unsigned long lastPulseTime = 0; // Time of the last pulse
unsigned long lastTime = 0; //Variable to attribute the time before mesuring rpm
unsigned long lastPulse = 0; //Variable to attribute the time at the pulse interrupt
volatile uint32_t pulse_count = 0;
volatile float rpm = 0.0; // RPM value
volatile float angle = 0.0;
const int total_teeth = 36 - 1;  // 36 teeth with one missing (36-1)
const float angle_per_tooth = 360.0 / total_teeth;  // Angle per tooth
volatile uint32_t last_pulse_tick = 0;  // Tick count of the last pulse
volatile uint32_t current_pulse_tick = 0;


void setup( void )
{
	// Set loopTask max priority before deletion
	vTaskPrioritySet(NULL, configMAX_PRIORITIES-1);

	  // Init USART and set Baud-rate to 115200
	  Serial.begin(115200);

  /* Before a semaphore is used it must be explicitly created.  In this example a binary semaphore is created. */
  vSemaphoreCreateBinary( xBinarySemaphore );
  pulse_semaphore = xSemaphoreCreateBinary();
  rpm_mutex = xSemaphoreCreateMutex();

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
	//xTaskCreatePinnedToCore(calculateAngleTask, "CalculateAngleTask", 2048, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(displayTask, "displayTask", 2048, NULL, 1, NULL, 1);
	//xTaskCreatePinnedToCore(calculateRPMandAngleTask, "calculateRPMandAngleTask", 2048, NULL, 1, NULL, 1);
    //xTaskCreatePinnedToCore( calculateRPM, "Calculate RPM", 1024, NULL, 3, NULL, 1);
    //xTaskCreatePinnedToCore( vInitDisplay, "Display Boot", 1024, NULL, 1, NULL, 1);
  }

}

/*void displayTask(void *pvParameters) {
    while (1) {
        // Lock mutex to safely read rpm and angle
        if (xSemaphoreTake(rpm_mutex, portMAX_DELAY) == pdTRUE) {
            Serial.printf("RPM: %.2f, Angle: %.2f degrees\n", rpm, angle);
            xSemaphoreGive(rpm_mutex);
        }

        // Update display every 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}*/

// Task to calculate RPM
/*void calculateRPMandAngleTask(void *pvParameters) {
    uint32_t current_tick, tick_difference;

    while (1) {
        // Wait for the pulse semaphore (signal from ISR)
        if (xSemaphoreTake(pulse_semaphore, portMAX_DELAY) == pdTRUE) {
            current_tick = xTaskGetTickCount();  // Current tick count

            // Calculate the time difference
            tick_difference = current_tick - last_pulse_tick;
            last_pulse_tick = current_tick;

            // Calculate RPM and angle
            float time_difference_seconds = (tick_difference * portTICK_PERIOD_MS) / 1000.0;  // Convert to seconds
            if (time_difference_seconds > 0) {
                float calculated_rpm = 60.0 / (time_difference_seconds * total_teeth);

                // Lock mutex and update shared variables
                if (xSemaphoreTake(rpm_mutex, portMAX_DELAY) == pdTRUE) {
                    rpm = calculated_rpm;
                    angle = (360.0 / total_teeth) * pulse_count;
                    xSemaphoreGive(rpm_mutex);
                }
            }
        }

        // Optional: Delay for a stable loop if desired
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}*/

void displayTask(void *pvParameters) {
    while (1) {
        // Lock mutex to safely read rpm and angle
        if (xSemaphoreTake(rpm_mutex, portMAX_DELAY) == pdTRUE) {
            Serial.printf("RPM: %.2f, Angle: %.2f degrees\n", rpm, angle);
            xSemaphoreGive(rpm_mutex);
        }

        // Update display every 100 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
/*void calculateRPM(void *pvParameters) {
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
            Serial.println(maxrpm); // Output RPM

            /*Serial.print("Voltas de Cambota: ");
            Serial.println(lapCount); // Output RPM
        }

        vTaskDelay( 200 / portTICK_PERIOD_MS); // Delay for task scheduling
    }
}*/


/*void vInitDisplay(void *pvParameters) {

	  // Inicializar o tft
	 tft.begin();
	 // Colocar fundo preto
	 tft.fillScreen(ILI9341_BLACK);
	 // Definir orientação da escrita
	 tft.setRotation(0);

	vTaskDelete( NULL );

}*/

void IRAM_ATTR onPulse() {
    unsigned long currentTime = millis();

    // Calculate time between this pulse and the last pulse
    if (lastPulseTime != 0) { // Skip the first pulse
        pulseInterval = currentTime - lastPulseTime;
        rpm = (pulseInterval > 0) ? (60000 / pulseInterval) : 0; // Calculate RPM
    }

    lastPulseTime = currentTime; // Update the time of the last pulse
}

/*void IRAM_ATTR onPulse() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Increment pulse count
    pulse_count++;

    // Check for a full rotation
    if (pulse_count >= total_teeth) {
        pulse_count = 0;  // Reset pulse count
    }

    // Give semaphore to notify main task
    xSemaphoreGiveFromISR(pulse_semaphore, &xHigherPriorityTaskWoken);

    // Perform a context switch if necessary
    if (xHigherPriorityTaskWoken == pdTRUE) {
    	vPortYield();
    }
}*/

// Interrupção gerada na transição do sinal do sensor de Hall

/*void IRAM_ATTR onPulse( void )
{

	//Serial.print( "interrpcao\r\n" );
	//lapCount++; // Increase lap count on each pulse
	//pulseInterval = long(millis()) - lastPulse;
	//lastPulse = millis();
	pulseCount++; // Increase pulse count on each pulse

}*/
//------------------------------------------------------------------------------
void loop()
{
  vTaskDelete( NULL );
}
