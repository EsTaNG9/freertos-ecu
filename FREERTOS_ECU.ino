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


void setup( void )
{
	// Set loopTask max priority before deletion
	vTaskPrioritySet(NULL, configMAX_PRIORITIES-1);

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
    xTaskCreatePinnedToCore( vInitDisplay, "Display Boot", 1024, NULL, 1, NULL, 1);

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
            Serial.println(maxrpm); // Output RPM

            /*Serial.print("Voltas de Cambota: ");
            Serial.println(lapCount); // Output RPM*/
        }

        vTaskDelay( 200 / portTICK_PERIOD_MS); // Delay for task scheduling
    }
}

// Interrupção gerada na transição do sinal do sensor de Hall
void IRAM_ATTR onPulse( void )
{

	//Serial.print( "interrpcao\r\n" );
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
