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
#include <XPT2046_Touchscreen.h>

// Imagens em bitmap
#include "engauto.h" //logo automovel
#include "logo_map.h" //logo map
#include "logo_temp_motor.h" //logo temp_motor
#include "logo_avanco.h"
#include "logo_temp_gases_adm.h" //logo logo_humidade
#include "logo_tps.h" //logo logo_tps
#include "logo_rpm.h" //logo logo_rpm

// Definir os terminais do LCD
#define TFT_CS   05
#define TFT_DC   26
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18
#define TFT_RST -1 // ligar ao 3V3
#define T_CS     22  // Pino do touch

// Pinos do controlador Touch
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 27  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 14    // T_CS

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

//Constantes Global
const uint8_t DEBUG = 0; // DEBUG
#define ADCres 12 //Definicao  de ADC

//Variaveis Globais
bool ESTADODISPLAY = false; // display on off

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,
TFT_RST, TFT_MISO);

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

/* Definicoes de prototipos das tarefas */
void vIniciarDisplay(void *pvParameters);
//void vApagarValoresDisplay(void *pvParameters);
//void vEscreverValoresDisplay( void * pvParameters);
void vLerTempMotor(void *pvparameters);
void vTouchDetection(void *pvparameters);

//estrutura
typedef struct {
	bool proxima;
	bool antes;
	int pagina_atual;
} pagina_t;

//interrupts
//void IRAM_ATTR touchDetection(void);

/*Handler de tarefas*/
TaskHandle_t xResetValoresDisplayHandle;

//Queue para transferencia de temperatu/
QueueHandle_t tempmotorQueue;
QueueHandle_t xPagina;

/* Protoripoa de semaforos para sincronizacao de interrupcoees*/
SemaphoreHandle_t limparValores;
SemaphoreHandle_t xPaginaMutex;

// setup
void setup() {
	//ADC
	analogReadResolution(ADCres);
	//Inputs
	/*pinMode(DHTPIN, INPUT_PULLUP);
	 pinMode(LM35PIN, INPUT_PULLUP);*/

	//SPI.usingInterrupt(digitalPinToInterrupt(XPT2046_IRQ))
	pinMode(XPT2046_CS, OUTPUT);
	//attachInterrupt(digitalPinToInterrupt(XPT2046_IRQ), touchDetection, FALLING);

	// Criar queues
	tempmotorQueue = xQueueCreate(3, sizeof(float));
	xPagina = xQueueCreate(1, sizeof(pagina_t));

	xPaginaMutex = xSemaphoreCreateMutex();

	//iniciar a queue com os valores iniciais
	pagina_t initPagina = { false, false, 0 };
	xQueueSend(xPagina, &initPagina, portMAX_DELAY);

	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	// Criar tarefas
	xTaskCreatePinnedToCore(vIniciarDisplay, "DISPLAY", 8192, NULL, 1, NULL, 1);
	//xTaskCreatePinnedToCore(vApagarValoresDisplay,"Apagar os valores do dispplay", 8192, NULL, 2,	&xResetValoresDisplayHandle, 0);
	xTaskCreatePinnedToCore(vLerTempMotor, "Temperatura Motor", 8192, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(vTouchDetection, "detectar touch", 8192, NULL, 3, NULL, 1);

	Serial.begin(115200);
}
void vTouchDetection(void *pvParameters) {
	pagina_t updateValores;

	while (1) {
		TickType_t xLastWakeTime = xTaskGetTickCount();
		//xQueuePeek(xPagina, &updateValores.pagina_atual, portMAX_DELAY);

		if (touchscreen.tirqTouched() && touchscreen.touched() && xSemaphoreTake(xPaginaMutex, portMAX_DELAY) == pdTRUE && xQueuePeek(xPagina, &updateValores, 0) == pdPASS) {
			int x, y, z;
			// Get Touchscreen points
			TS_Point p = touchscreen.getPoint();
			// Calibrate Touchscreen points with map function to the correct width and height
			x = map(p.x, 200, 3700, 1, SCREEN_WIDTH); //COORDENADA X
			y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT); //COORDENADA Y
			z = p.z; //PRESSAO

			if (x < 160) {
				Serial.print("NEXTTTT");
				updateValores.proxima = true;
				updateValores.antes = false; //redundancia

			} else {
				Serial.print("PREVIOUS");
				updateValores.antes = true;
				updateValores.proxima = false; //redundancia
			}

			xQueueOverwrite(xPagina, &updateValores);
			//xQueueOverwrite(xPagina, &updateValores.antes);
			//xQueueOverwrite(xPagina, &updateValores.pagina_atual);

			/*Serial.print("X = ");
			 Serial.print(x);
			 Serial.print(" | Y = ");
			 Serial.print(y);
			 Serial.print(" | Pressure = ");
			 Serial.print(z);
			 Serial.println();*/
			xSemaphoreGive(xPaginaMutex);

		}
		//Serial.println("  Tarefa: Update TOUCH");
		vTaskDelayUntil(&xLastWakeTime, (150 / portTICK_PERIOD_MS)); //DEBOUCE MUITO RUDIMENTAR :(
	}
}

// Task para desenhar o logotipo de EAU
void vIniciarDisplay(void *pvParameters) {
	float temp_motor = 0, map = 0, tps = 0, avanco = 0;
	uint16_t rpm = 65530;
	char buffer[10];
	bool inicializacao = true;

	pagina_t getValores;
	getValores.proxima = false;
	getValores.antes = false;
	getValores.pagina_atual = 0;
	xQueueOverwrite(xPagina, &getValores);

	TickType_t xLastWakeTime = xTaskGetTickCount();

	touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
	touchscreen.begin(touchscreenSPI);
	touchscreen.setRotation(3);

	tft.begin();
	tft.setRotation(3);

	Serial.println(ESTADODISPLAY);
	Serial.println("  Tarefa: Iniciar e escrever imagens no display");

	for (;;) {
		Serial.println("	start");
		Serial.println(getValores.pagina_atual);
		if (xSemaphoreTake(xPaginaMutex, portMAX_DELAY) == pdTRUE && xQueuePeek(xPagina, &getValores, portMAX_DELAY) == pdPASS) {
			Serial.println(getValores.pagina_atual);

			//if()

			//Primeira inicialização do "sistema"
			/*if (inicializacao == true) {
			 inicializacao = false;
			 getValores.proxima = false;
			 getValores.antes = false;
			 getValores.pagina_atual = 0; //garantir pagina inicial
			 xQueueOverwrite(xPagina, &getValores);
			 Serial.println("init");
			 }*/

			//atualizar pagina
			if (getValores.proxima == true) {
				getValores.proxima = false;
				getValores.antes = false;
				Serial.println(getValores.pagina_atual);
				getValores.pagina_atual = ((getValores.pagina_atual) + 1);
				if (getValores.pagina_atual > 2) {
					getValores.pagina_atual = 0;
				}
				inicializacao = true;
				//xQueueOverwrite(xPagina, &getValores.proxima);
				xQueueOverwrite(xPagina, &getValores);
				Serial.println("proxima");
				Serial.println(getValores.pagina_atual);
			}

			if (getValores.antes == true) {
				getValores.antes = false;
				getValores.proxima = false;
				Serial.println(getValores.pagina_atual);
				if (getValores.pagina_atual <= 0) {
					getValores.pagina_atual = 2;
				} else {
					getValores.pagina_atual--;
				}
				inicializacao = true;
				//xQueueOverwrite(xPagina, &getValores.antes);
				xQueueOverwrite(xPagina, (void* )&getValores);
				Serial.println("antes");
				Serial.println(getValores.pagina_atual);
			}
			Serial.println(getValores.pagina_atual);
			xQueueOverwrite(xPagina, (void* )&getValores);
			Serial.println(getValores.pagina_atual);
			xSemaphoreGive(xPaginaMutex);

			//Updates de valores, apenas vamos atualizar valores na pagina principal
			if (inicializacao == false && getValores.pagina_atual == 0) {

				xQueueReceive(tempmotorQueue, &temp_motor, 0);

				// Atualiza o valor de `tps`
				tps += 0.01; // Incrementa `tps` em passos de 0.01
				if (tps >= 100.0)
					tps = 0.0; // Reinicia para 0 quando atinge 100

				//Serial.println(temp_motor);
				//Serial.println(receber_dados_dht11.temp_exterior);
				//Serial.println(receber_dados_dht11.humidade);

				Serial.println("Dados escrever display: ");
				//	Serial.println(temp_motor);
				//	Serial.println(temp_peneu);
				//Serial.println("8");
				//Serial.println(tps);
				//Serial.println(rpm);

				tft.setCursor(160, 150);
				tft.setTextSize(1);
				tft.print(temp_motor);
				tft.print((char) 167);
				tft.print("C");

				tft.setCursor(60, 50);
				tft.print(map);
				tft.print((char) 167);
				tft.print("C");

				tft.setCursor(110, 50);
				tft.print((char) 167);
				tft.print("C");

				tft.setCursor(16, 110);
				tft.print("%");

				dtostrf(tps, 5, 2, buffer);
				tft.setCursor(130, 230);
				tft.print(tps);
				tft.print("%");

				sprintf(buffer, "%5d", rpm);
				rpm++;
				tft.setCursor(250, 230);
				tft.print(buffer);

				Serial.println("  Tarefa: Update valores");

			}

			// Desenhos estacionários
			if (inicializacao == true && getValores.pagina_atual == 1) {
				//Desenhar tabela VE
				tft.fillScreen(ILI9341_BLACK);
				tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
				tft.setCursor(160, 150);
				tft.setTextSize(2);
				tft.print("DESENHAR TABELA VE");
				inicializacao = false;
			}

			if (inicializacao == true && getValores.pagina_atual == 2) {
				//Desenhar tabela IGN
				tft.fillScreen(ILI9341_BLACK);
				tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
				tft.setCursor(160, 150);
				tft.setTextSize(2);
				tft.print("DESENHAR TABELA IGN");
				inicializacao = false;
			}

			if (inicializacao == true && getValores.pagina_atual == 0) {
				//Desenhar pagina principal

				tft.fillScreen(ILI9341_BLACK);
				tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

				//  LogoEAU
				int altura = 34, largura = 320;   // Definicao da dimensao da imagem
				int linha, coluna, inicio_x = 0; // Definicao de ints temporarios
				// Ler a linha do bitmap
				for (linha = 0; linha < altura; linha++) {
					// Ler coluna do bitmap
					for (coluna = 0; coluna < largura; coluna++) {
						tft.drawPixel(coluna, linha, pgm_read_word(engauto + inicio_x)); // Desenhar o pixel no sitio correto
						inicio_x++;
					}
				}

				// RPM
				altura = 40;
				largura = 78;   // Definicao da dimensao da imagem
				inicio_x = 0;  // Definicao de ints temporarios
				// Ler a linha do bitmap
				for (linha = 0; linha < altura; linha++) {
					// Ler coluna do bitmap
					for (coluna = 0; coluna < largura; coluna++) {
						tft.drawPixel(coluna + 230, linha + 180, pgm_read_word(logo_rpm + inicio_x)); // Desenhar o pixel no sitio correto
						inicio_x++;
					} // end pixel
				}

				// TPS
				altura = 40;
				largura = 78;   // Definicao da dimensao da imagem
				inicio_x = 0;  // Definicao de ints temporarios
				// Ler a linha do bitmap
				for (linha = 0; linha < altura; linha++) {
					// Ler coluna do bitmap
					for (coluna = 0; coluna < largura; coluna++) {
						tft.drawPixel(coluna + 110, linha + 180, pgm_read_word(logo_tps + inicio_x)); // Desenhar o pixel no sitio correto
						inicio_x++;
					} // end pixel
				}

				// Temp Gases de Admissão
				altura = 45;
				largura = 78;   // Definicao da dimensao da imagem
				inicio_x = 0;  // Definicao de ints temporarios
				// Ler a linha do bitmap
				for (linha = 0; linha < altura; linha++) {
					// Ler coluna do bitmap
					for (coluna = 0; coluna < largura; coluna++) {
						tft.drawPixel(coluna + 20, linha + 80, pgm_read_word(logo_temp_gases_adm + inicio_x)); // Desenhar o pixel no sitio correto
						inicio_x++;
					} // end pixel
				}

				// Avanço
				altura = 60;
				largura = 60;   // Definicao da dimensao da imagem
				inicio_x = 0;  // Definicao de ints temporarios
				// Ler a linha do bitmap
				for (linha = 0; linha < altura; linha++) {
					// Ler coluna do bitmap
					for (coluna = 0; coluna < largura; coluna++) {
						tft.drawPixel(coluna + 20, linha + 170, pgm_read_word(logo_avanco + inicio_x)); // Desenhar o pixel no sitio correto
						inicio_x++;
					} // end pixel
				}

				// MAP
				altura = 78;
				largura = 78;   // Definicao da dimensao da imagem
				inicio_x = 0;  // Definicao de ints temporarios
				// Ler a linha do bitmap
				for (linha = 0; linha < altura; linha++) {
					// Ler coluna do bitmap
					for (coluna = 0; coluna < largura; coluna++) {
						tft.drawPixel(coluna + 230, linha + 60, pgm_read_word(logo_map + inicio_x)); // Desenhar o pixel no sitio correto
						inicio_x++;
					} // end pixel
				}

				// Temp. Motor
				altura = 78;    // Definicao da dimensao da imagem
				largura = 78;   // Definicao da dimensao da imagem
				inicio_x = 0;   // Definicao de ints temporarios
				// Ler a linha do bitmap
				for (linha = 0; linha < altura; linha++) {
					// Ler coluna do bitmap
					for (coluna = 0; coluna < largura; coluna++) {
						tft.drawPixel(coluna + 140, linha + 60, pgm_read_word(logo_temp_motor + inicio_x)); // Desenhar o pixel no sitio correto
						inicio_x++;
					} // end pixel
				}

				inicializacao = false;
			}

			//xQueueOverwrite(xPagina, &getValores);

			//Serial.println(getValores.antes);
			//Serial.println(getValores.proxima);
			Serial.println(getValores.pagina_atual);
			Serial.println("	reset");

			vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
		}
	}
}

// task para apagar valores no display
/*void vApagarValoresDisplay(void *pvParameters) {
 TickType_t xLastWakeTime = xTaskGetTickCount();

 for (;;) {
 tft.fillRect(10, 50, 42, 7, ILI9341_BLACK);
 tft.fillRect(60, 50, 42, 7, ILI9341_BLACK);
 tft.fillRect(110, 50, 42, 7, ILI9341_BLACK);
 tft.fillRect(16, 110, 18, 7, ILI9341_BLACK);
 tft.fillRect(68, 110, 18, 7, ILI9341_BLACK);
 tft.fillRect(118, 110, 18, 7, ILI9341_BLACK);

 Serial.println("  Tarefa: vApagarValoresDisplay");
 vTaskDelayUntil(&xLastWakeTime, (2000 / portTICK_PERIOD_MS));
 }
 }/*/

void vLerTempMotor(void *pvparameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float tempmotor = 0, valorBruto = 0, ddP = 0;
	for (;;) {
		//sensors.requestTemperatures();
		//float tempmotor = sensors.getTempCByIndex(0);
		//valorBruto = analogRead(LM35PIN);
		valorBruto = 1024;
		//Serial.println(valorBruto);
		ddP = (valorBruto / 2048.0) * 3300;		// Convesão para milivolts (ddp)
		//Serial.println(ddP);
		tempmotor = ddP * 0.1;
		xQueueSendToBack(tempmotorQueue, &tempmotor, 0);

		if (DEBUG == 1) {
			Serial.print("Temp Motor: ");
			Serial.print(tempmotor);
			Serial.println("ºC");
		}

		Serial.println("  Tarefa: vLerTempMotor");
		vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
	}
}

// loop
void loop() {
	vTaskDelete(NULL);  //matar a tarefa arduino
}
