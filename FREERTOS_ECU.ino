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
void vInitDisplay(void *pvParameters);

/* The service routine for the interrupt.  This is the interrupt that the task
 will be synchronized with. */
void IRAM_ATTR onPulse(void);

/* The tasks to be created. */
//void calculateRPM(void *pvParameters);
void vInitDisplay(void *pvParameters);

// pin to generate interrupts
const uint8_t interruptPin = 4;

//Configurar roda fonica
uint16_t numRealDentes = 35;

volatile unsigned long tempoAtual = 0;
volatile unsigned long falhaAtual = 0;
volatile unsigned long tempoUltimoDente = 0;
volatile unsigned long tempoFiltro = 0;
volatile unsigned long tempoPenultimoDente = 0;
volatile unsigned long falhaObjetivo = 0;
volatile uint16_t contadorDentes = 0;
volatile unsigned long tempoPrimeiroDenteMenosUm = 0;
volatile unsigned long tempoPrimeiroDente = 0;
bool sincronizacao = false;
volatile uint32_t contadorrevolucoes = 0;

void setup(void) {
	// Set loopTask max priority before deletion
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	// Init USART and set Baud-rate to 115200
	Serial.begin(115200);

	//Interrupção Hall Sensor
	pinMode(interruptPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(interruptPin), onPulse, FALLING);

	xTaskCreatePinnedToCore(vInitDisplay, "Display Boot", 2048, NULL, 1,
	NULL, 1);

}

void vInitDisplay(void *pvParameters) {
	// Inicializar o tft
	tft.begin();
	// Colocar fundo preto
	tft.fillScreen(ILI9341_BLACK);
	// Definir orientação da escrita
	tft.setRotation(0);

	for (;;) {
		tft.setCursor(0, 10);
		tft.setTextColor(ILI9341_WHITE);
		tft.setTextSize(2);
		tft.println(" Microcontroladores ");
		tft.println(" Sistemas Embebidos ");
		tft.println("     2016/2017      ");

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

// Interrupção gerada na transição do sinal do sensor de Hall
void IRAM_ATTR onPulse(void) {

	tempoAtual = esp_timer_get_time();
	falhaAtual = tempoAtual - tempoUltimoDente;

	if (falhaAtual < tempoFiltro) {
		return; //Debounce
	}

	//Add queue aberturaAtual

	falhaObjetivo = 1.5 * (tempoUltimoDente - tempoPenultimoDente);

	if (falhaAtual < falhaObjetivo || contadorDentes > numRealDentes) {
		contadorDentes = 1;
		tempoPrimeiroDenteMenosUm = tempoPrimeiroDente;
		tempoPrimeiroDente = tempoAtual;
		sincronizacao = true;
		contadorrevolucoes++;
	} else {
		tempoFiltro = falhaAtual * 0.25; // 25% de filtro
	}

	tempoPenultimoDente = tempoUltimoDente;
	tempoUltimoDente = tempoAtual;
}

//------------------------------------------------------------------------------
void loop() {
	vTaskDelete( NULL);
}
