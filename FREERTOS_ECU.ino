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
#define T_CS 21
#define T_IRQ 4



//Constantes Global
const uint8_t DEBUG = 1; // DEBUG
#define ADCres 12 //Definicao  de ADC

//Variaveis Globais
bool ESTADODISPLAY = false; // display on off

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,
TFT_RST, TFT_MISO);
XPT2046_Touchscreen ts(T_CS, T_IRQ);

/* Definicoes de prototipos das tarefas */
void vIniciarDisplay(void *pvParameters);
//void vApagarValoresDisplay(void *pvParameters);
//void vEscreverValoresDisplay( void * pvParameters);
void vLerTempMotor(void *pvparameters);

/*Handler de tarefas*/
TaskHandle_t xResetValoresDisplayHandle;

//Queue para transferencia de temperatu/
QueueHandle_t tempmotorQueue;

/* Protoripoa de semaforos para sincronizacao de interrupcoees*/
SemaphoreHandle_t limparValores;

// setup
void setup() {
	//ADC
	analogReadResolution(ADCres);
	//Inputs
	/*pinMode(DHTPIN, INPUT_PULLUP);
	 pinMode(LM35PIN, INPUT_PULLUP);*/

	// Criar queues
	tempmotorQueue = xQueueCreate(3, sizeof(float));

	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	// Criar tarefas
	xTaskCreatePinnedToCore(vIniciarDisplay, "DISPLAY", 8192, NULL, 1, NULL, 1);
	//xTaskCreatePinnedToCore(vApagarValoresDisplay,"Apagar os valores do dispplay", 8192, NULL, 2,	&xResetValoresDisplayHandle, 0);
	xTaskCreatePinnedToCore(vLerTempMotor, "Temperatura Motor", 8192, NULL, 3, NULL, 1);

	Serial.begin(115200);
}

// Task para desenhar o logotipo de EAU
void vIniciarDisplay(void *pvParameters) {
	float temp_motor = 0, map = 0, tps = 0, avanco = 0;
	uint16_t rpm = 65530;
	char buffer[10];

	TickType_t xLastWakeTime = xTaskGetTickCount();

	tft.begin();
	tft.setRotation(3);
	tft.fillScreen(ILI9341_BLACK);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
	if (DEBUG == 1)
		Serial.println("Ligar Display");

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
			tft.drawPixel(coluna + 230, linha + 180,
					pgm_read_word(logo_rpm + inicio_x)); // Desenhar o pixel no sitio correto
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
			tft.drawPixel(coluna + 110, linha + 180,
					pgm_read_word(logo_tps + inicio_x)); // Desenhar o pixel no sitio correto
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
			tft.drawPixel(coluna + 20, linha + 80,
					pgm_read_word(logo_temp_gases_adm + inicio_x)); // Desenhar o pixel no sitio correto
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
			tft.drawPixel(coluna + 20, linha + 170,
					pgm_read_word(logo_avanco + inicio_x)); // Desenhar o pixel no sitio correto
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
			tft.drawPixel(coluna + 230, linha + 60,
					pgm_read_word(logo_map + inicio_x)); // Desenhar o pixel no sitio correto
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
			tft.drawPixel(coluna + 140, linha + 60,
					pgm_read_word(logo_temp_motor + inicio_x)); // Desenhar o pixel no sitio correto
			inicio_x++;
		} // end pixel
	}

	Serial.println(ESTADODISPLAY);
	Serial.println("  Tarefa: Iniciar e escrever imagens no display");

	for (;;) {
		xQueueReceive(tempmotorQueue, &temp_motor, 0);


		  // Atualiza o valor de `tps`
		        tps += 0.01; // Incrementa `tps` em passos de 0.01
		        if (tps >= 100.0) tps = 0.0; // Reinicia para 0 quando atinge 100

		//Serial.println(temp_motor);
		//Serial.println(receber_dados_dht11.temp_exterior);
		//Serial.println(receber_dados_dht11.humidade);

		Serial.println("Dados escrever display: ");
	//	Serial.println(temp_motor);
	//	Serial.println(temp_peneu);
		Serial.println("8");
		Serial.println(tps);
		Serial.println(rpm);

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

		sprintf(buffer, "%5d",rpm);
		rpm++;
		tft.setCursor(250, 230);
		tft.print(buffer);

		Serial.println("  Tarefa: Update valores");
		vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
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
	//vTaskDelete(NULL);  //matar a tarefa arduino
	 // Verificar se há toque
	if (ts.touched()) { // Verifica se o display foi tocado
	    TS_Point p = ts.getPoint(); // Lê o ponto tocado
	    Serial.print("X: ");
	    Serial.print(p.x);
	    Serial.print(" Y: ");
	    Serial.println(p.y);
	}
}
