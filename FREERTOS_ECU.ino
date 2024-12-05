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

/* The service routine for the interrupt.  This is the interrupt that the task
 will be synchronized with. */
void IRAM_ATTR onPulse(void);

/* The tasks to be created. */
//void calculateRPM(void *pvParameters);
void vInitDisplay(void *pvParameters);
void vGetLowRPM(void *pvParameters);
//void vGetCrankAngle(void *pvParameters);

//Handles
QueueHandle_t xRPM;
QueueHandle_t xContadorDentes;
QueueHandle_t xtempoUltimoDente;

// pin to generate interrupts
const uint8_t interruptPin = 4;

//Configurar roda fonica
uint16_t numRealDentes = 35;
uint16_t anguloPorDente = 360 / 36;
uint16_t desvioPrimeiroDenteTDC = 180; //Desvio real do TDC 1ºcil do primeiro dente da roda

volatile unsigned long tempoAtual = 0;
volatile unsigned long falhaAtual = 0;
unsigned long tempoUltimoDente = 0; // n pode ser volatile pq quero mandar valores para a queeue
volatile unsigned long tempoFiltro = 0;
volatile unsigned long tempoPenultimoDente = 0;
volatile unsigned long falhaObjetivo = 0;
uint16_t contadorDentes = 0; // n pode ser volatile pq quero mandar valores para a queeue
volatile unsigned long tempoPrimeiroDenteMenosUm = 0;
volatile unsigned long tempoPrimeiroDente = 0;
unsigned long tempoPorGrau = 0; // n pode ser volatile pq quero mandar valores para a queeue
bool sincronizacao = false;
volatile uint32_t contadorrevolucoes = 0;
uint64_t tempoEntreDentes = 0;
unsigned long tempoDecorrido = 0;

void setup(void) {
	// Set loopTask max priority before deletion
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	// Init USART and set Baud-rate to 115200
	Serial.begin(115200);

	//Interrupção Hall Sensor
	pinMode(interruptPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(interruptPin), onPulse, RISING);

	xRPM = xQueueCreate(5, sizeof(uint16_t));
	xContadorDentes = xQueueCreate(5, sizeof(uint8_t));
	xtempoUltimoDente = xQueueCreate(5, sizeof(long));

	xTaskCreatePinnedToCore(vInitDisplay, "Display Boot", 2048, NULL, 1, NULL,
			1);
	xTaskCreatePinnedToCore(vGetLowRPM, "Get low rpm", 2048, NULL, 1, NULL, 1);
	//xTaskCreatePinnedToCore(vGetCrankAngle, "Get crank angle", 2048, NULL, 1, NULL, 1);

}

/*void vGetCrankAngle(void *pvParameters) {
 unsigned long temp_tempoUltimoDente;
 int temp_contadorDentes;

 // Prioridade máxima
 xQueuePeek(xContadorDentes, &temp_contadorDentes,
 portMAX_DELAY);
 xQueuePeek(xtempoUltimoDente, &temp_tempoUltimoDente,
 portMAX_DELAY);
 //Prioridade normal

 int anguloCambota = ((temp_contadorDentes - 1) * anguloPorDente)
 + desvioPrimeiroDenteTDC;

 //Estimar o num de graus desde o ultimo dente
 tempoDecorrido = micros() - temp_tempoUltimoDente;
 tempoPorGrau = (temp_tempoUltimoDente / anguloPorDente); //Se n funcionar faço apartir da rpm
 anguloCambota = anguloCambota + ldiv(tempoDecorrido, tempoPorGrau).quot;

 if (anguloCambota > 360) {
 anguloCambota = anguloCambota - 360;
 }
 }*/

void vGetLowRPM(void *pvParameters) {
	while (true) {
		//Maxima prioridade ou desativar interrupcoes
		//taskDISABLE_INTERRUPTS();
		//tempoEntreDentes = (tempoUltimoDente - tempoPenultimoDente);
		tempoEntreDentes = (tempoUltimoDente - tempoPenultimoDente);
		uint64_t rpm_temporary =
				(((60000000 / (tempoEntreDentes * 36)) + 1) * 2); //+1 pq falha, n sei pk x2??
		//taskENABLE_INTERRUPTS();
		//Prioridade normal
		xQueueSendToFront(xRPM, &rpm_temporary, 0);

		Serial.println(rpm_temporary);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

// Interrupção gerada na transição do sinal do sensor de Hall
void IRAM_ATTR onPulse(void) {

	//Serial.println(falhaAtual);
	BaseType_t xHigherPriorityTaskWoken;

	tempoAtual = micros();
	falhaAtual = tempoAtual - tempoUltimoDente;

	if (falhaAtual < tempoFiltro) {
		//return; //Debounce
		//Serial.println("Debounce");
	} else {

		//Serial.print("Tempo Falha: ");
		//Serial.println(falhaAtual);
		//Add para guardar, queue aberturaAtual

		falhaObjetivo = 1.5 * (tempoUltimoDente - tempoPenultimoDente);

		if (falhaAtual > falhaObjetivo || contadorDentes > numRealDentes) {
			contadorDentes = 1;
			tempoPrimeiroDenteMenosUm = tempoPrimeiroDente;
			tempoPrimeiroDente = tempoAtual;
			sincronizacao = true;
			contadorrevolucoes++;
		} else {
			tempoFiltro = falhaAtual * 0.25; // 25% de filtro
			contadorDentes++;
			//Serial.print("Dente: ");
			//Serial.println(contadorDentes);
			xQueueSendToFrontFromISR(xContadorDentes, &contadorDentes,
					&xHigherPriorityTaskWoken);
		}

		tempoPenultimoDente = tempoUltimoDente;
		tempoUltimoDente = tempoAtual;
		//xQueueSendToFrontFromISR(xtempoUltimoDente, &tempoUltimoDente, &xHigherPriorityTaskWoken);

		//Serial.println(contadorrevolucoes);
	}

//Serial.println("int");
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

//------------------------------------------------------------------------------
void loop() {
	vTaskDelete( NULL);
}
