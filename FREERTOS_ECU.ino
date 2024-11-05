#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

/* The tasks to be created. */
void calculateRPM( void *pvParameters );

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


void setup( void )
{
  // Set loopTask max priority before deletion
  vTaskPrioritySet(NULL, configMAX_PRIORITIES-1);

  // Init USART and set Baud-rate to 115200
  Serial.begin(115200);
    /* Before a semaphore is used it must be explicitly created.  In this example
  a binary semaphore is created. */
    vSemaphoreCreateBinary( xBinarySemaphore );


   pinMode(interruptPin, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(interruptPin), onPulse, FALLING);



  /* Check the semaphore was created successfully. */
  if( xBinarySemaphore != NULL )
  {
    /* Create the 'handler' task.  This is the task that will be synchronized
    with the interrupt.  The handler task is created with a high priority to
    ensure it runs immediately after the interrupt exits.  In this case a
    priority of 3 is chosen. */
    xTaskCreatePinnedToCore( calculateRPM, "Calculate RPM", 1024, NULL, 3, NULL, 1);

  }

}

// Task to calculate RPM
void calculateRPM(void *pvParameters) {
    while (true) {
        // Calculate RPM every second
        unsigned long currentTime = millis();
        unsigned long interval = currentTime - lastTime;

        if (interval >= 200) { // Calculate every xms (x second)



            rpm = ((pulseCount * 60) / teethNum) / interval; // RPM calculation
            pulseCount = 0; // Reset pulse count
            lastTime = currentTime;

            /*Serial.print("RPM: ");
            Serial.println(rpm); // Output RPM*/

            /*Serial.print("Voltas de Cambota: ");
            Serial.println(lapCount); // Output RPM*/
        }

        vTaskDelay( 200 / portTICK_PERIOD_MS); // Delay for task scheduling
    }
}

// Interrupção gerada na transição do sinal do sensor de Hall
void IRAM_ATTR onPulse( void )
{

	Serial.print( "interrpcao\r\n" );
	//lapCount++; // Increase lap count on each pulse
	pulseCount++; // Increase pulse count on each pulse

	/*if (pulseCount > 34) {
		lapCount++;
		pulseCount = 0;
	}*/
}
//------------------------------------------------------------------------------
void loop()
{
  vTaskDelete( NULL );
}
