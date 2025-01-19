#include <math.h>
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"
#include "esp_err.h"
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

//Mapas de igniçao e injecao
#include "tables.h"

//Caso não haja sinal em 1000ms, ent o motor foi abaixo
#define TEMPO_MAXIMO_MOTOR_MORRER 1000

//Config do ADC
#define ADC_RESOLUTION 6
#define VREF_PLUS  5
#define VREF_MINUS  0.0

//Calibração dos sensores
#define MAP_5V  100 //kpa
#define MAP_0V 10 //kpa

#define TPS_5V  100 //%
#define TPS_0V 0 //%

#define CLT_5V  120 //ºC
#define CLT_0V -10 //ºC

#define IAT_5V  80 //ºC
#define IAT_0V 5 //ºC

//Sistema de debug
#define DEBUG "OFF"
#define DEBUG_LEVEL 2

// Definir os terminais do LCD
#define TFT_CS   05
#define TFT_DC   26
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18
#define TFT_RST -1 // ligar ao 3V3

// Pinos do controlador Touch
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 27  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 14    // T_CS

//Dimensão do display
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

//Configuração da injeção
#define INJ1_FREQ 1000 //Se o injector deadtime for 1ms@12V
#define INJ1_TIMER LEDC_TIMER_3//Selecionar o timer a ser usado
#define INJ1_RESO LEDC_TIMER_13_BIT//Resolução do timer utilizado
#define INJ1_MODE LEDC_LOW_SPEED_MODE
#define INJ1_CHANNEL LEDC_CHANNEL_0
#define INJ1_MAX_DUTY ((1 << 13) - 1)// Max duty (8191 for 13 bits)

// Criar um objeto tft com indicação dos terminais CS e DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,
TFT_RST, TFT_MISO);

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

/* The service routine for the interrupt.  This is the interrupt that the task
 will be synchronized with. */
void IRAM_ATTR onPulse(void);
void coilDischargeTimer_callback(void *arg);
//void IRAM_ATTR coilDischargeTimer(void);

/* The tasks to be created. */
void vDisplay(void *pvParameters);
void vBrain(void *pvParameters);

//Declarar funcoes
int getCrankAngle(int, int);
int getLowRPM(long, long, int);
void prntIGN(void);
float interpolation(int, int, int);
float getADC(int);
//void setup_timers(void);

//estruturas
typedef struct {
	bool proxima;
	bool antes;
	int pagina_atual;
} pagina_t;

typedef struct {
	bool sincronizacao;
	bool descargaBobine;
	bool bombaCombustivel;
	int RPM;
	uint16_t atualRPS;
	uint16_t ContadorDentes;
	uint16_t contadorrevolucoes;
	uint16_t avancoIGN;
	long tempoUltimoDente;
	long tempoPenultimoDente;
	long tempoFiltro;
	long tempoPrimeiroDenteMenosUm;
	long tempoPrimeiroDente;
	long tempoPorGrau;
	long falhaAtual;
	float MAP;
	float TPS;
	float CLT;
	float IAT;
} ecu_info_t;

//Handles
QueueHandle_t xECU;
QueueHandle_t xPagina;

/* Protoripoa de semaforos para sincronizacao de interrupcoees*/
SemaphoreHandle_t xPaginaMutex;
SemaphoreHandle_t xECUMutex;

// Timers
esp_timer_handle_t coilTimer; //Definir a handle globalmente para a poder chamar na interrupcao

// Estruturas

// pin to generate interrupts
const uint8_t interruptPin = 4;
//const uint8_t bombaCombustivelPin = 28;
const uint8_t MAPSensorPin = 35;
const uint8_t bobine1Pin = 32;
const uint8_t injetor1Pin = 33;

//Configurar roda fonica
uint16_t numRealDentes = 35;
uint16_t anguloPorDente = 360 / 36;
uint16_t desvioPrimeiroDenteTDC = 180; //Desvio real do TDC 1ºcil do primeiro dente da roda

volatile unsigned long tempoAtual_Brain = 1; // Para n dar erro no startup

void setup(void) {
	// Set loopTask max priority before deletion
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	// Init USART and set Baud-rate to 115200
	Serial.begin(115200);

	//setup adc
	analogReadResolution(ADC_RESOLUTION);

	//Interrupção Hall Sensor
	pinMode(interruptPin, INPUT);
	pinMode(bobine1Pin, OUTPUT);
	pinMode(injetor1Pin, OUTPUT);
	pinMode(MAPSensorPin, INPUT_PULLUP);
	pinMode(XPT2046_CS, OUTPUT);
	attachInterrupt(digitalPinToInterrupt(interruptPin), onPulse, RISING);

	//Tive q definir primeiro o pwm, pq dps da erro, presumidamente, o esp tentava acessar dois timers iguais
	ledc_timer_config_t inj1_timer = { .speed_mode = INJ1_MODE, .duty_resolution = INJ1_RESO, .timer_num = INJ1_TIMER, .freq_hz = INJ1_FREQ, .clk_cfg = LEDC_AUTO_CLK };
	ESP_ERROR_CHECK(ledc_timer_config(&inj1_timer));

	// Configure the LEDC channel
	ledc_channel_config_t inj1_channel = { .gpio_num = injetor1Pin, .speed_mode = INJ1_MODE, .channel = INJ1_CHANNEL, .intr_type = LEDC_INTR_DISABLE, .timer_sel = INJ1_TIMER, .duty = 0, .hpoint = 0 };
	ESP_ERROR_CHECK(ledc_channel_config(&inj1_channel));

	//timers
	//coilTimer = timerBegin(1000000);
	//timerAttachInterrupt(coilTimer, &coilDischargeTimer);
	//esp_timer_create_args_t coilTimer_args; //= { .callback = &coilDischargeTimer_callback, .arg = NULL, .name = "coilTimer" };
	//coilTimer_args.arg = NULL, coilTimer_args.callback = &coilDischargeTimer_callback, coilTimer_args.name = "coilTimer";
	//ESP_ERROR_CHECK(esp_timer_create(&coilTimer_args, &coilTimer));
	//ESP_ERROR_CHECK(esp_timer_start_periodic(coilTimer, 1000000)); // 5 seconds (5,000,000 µs)

	//queues
	xECU = xQueueCreate(1, sizeof(ecu_info_t));
	xPagina = xQueueCreate(1, sizeof(pagina_t));

	//Semafro Mutex
	xPaginaMutex = xSemaphoreCreateMutex();
	xECUMutex = xSemaphoreCreateMutex();

	//Assegurar valores iniciais nas queues
	pagina_t initPagina = { false, false, 0 };
	xQueueSend(xPagina, &initPagina, portMAX_DELAY);

	ecu_info_t initECU = { false, LOW, LOW, 0, 0, 35, 0, 10, 1, 0, 0, 0, 0, 0, 0, 100, 0, 90, 90 };
	xQueueSend(xECU, &initECU, portMAX_DELAY);

	// Criar tarefas
	xTaskCreatePinnedToCore(vDisplay, "vDisplay", 4096, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(vBrain, "vBrain", 4096, NULL, 1, NULL, 1);
	//xTaskCreatePinnedToCore(vTouchDetection, "detectar touch", 8192, NULL, 3, NULL, 1);

	if (DEBUG == "ON" && DEBUG_LEVEL <= 3) {
		Serial.println("Setup acabou de correr");
	}

}

void vBrain(void *pvParameters) {
	pagina_t updateValores;
	ecu_info_t getECU;
	portBASE_TYPE xStatus;
	unsigned long temp_tempoUltimoDente = 0, tempoUltimoDente = 1;
	int temp_atualRPM = 0, atualRPM = 0;
	volatile unsigned long loopAtual_Brain = 1; // Para n dar erro no startup
	unsigned long tempoUltimoloop_Brain = 0; // n pode ser volatile pq quero mandar valores para a queeue
	float temp_voltage = 0;

	while (true) {
		//loopAtual_Brain = tempoAtual_Brain - tempoUltimoloop_Brain;
		loopAtual_Brain = tempoAtual_Brain;
		tempoAtual_Brain = micros();
		//Serial.println("	gay");

		//if (xSemaphoreTake(xECUMutex, portMAX_DELAY) == pdTRUE && xQueuePeek(xECU, &getECU, portMAX_DELAY) == pdPASS) {
		//xSemaphoreGive(xECUMutex);
		if (xQueuePeek(xECU, &getECU, (TickType_t) 250) == pdPASS) {
			//tempoUltimoDente = temp_tempoUltimoDente;
		}
		//xQueuePeek(xECU, &getECU, portMAX_DELAY);
		//Serial.println(getECU.RPM);
		/*if (xQueuePeek(xtempoUltimoDente, &tempoUltimoDente, (TickType_t) 25) == pdPASS) {
		 //tempoUltimoDente = temp_tempoUltimoDente;
		 }*/

		/*uint64_t tempoEntreDentes = (getECU.tempoUltimoDente - getECU.tempoPenultimoDente);
		 if (tempoEntreDentes == 0) {
		 } else {

		 uint64_t rpm_temporary = (((60000000 / (tempoEntreDentes * 36)) + 1) * 2); //+1 pq falha, n sei pk x2??
		 Serial.print("RPM: ");
		 Serial.println(rpm_temporary);
		 }*/

		unsigned long tempoParaUltimoDente = (loopAtual_Brain - getECU.tempoUltimoDente);

		//Serial.println("Brain: RUNNING");

		if ((tempoParaUltimoDente < TEMPO_MAXIMO_MOTOR_MORRER) || (tempoParaUltimoDente > loopAtual_Brain)) {
			int ultimaRPM = atualRPM;
			//atualRPM = getECU.RPM;

			atualRPM = getLowRPM(getECU.tempoUltimoDente, getECU.tempoPenultimoDente, getECU.RPM);

			/*Serial.print("RPM: ");
			 Serial.println(atualRPM);*/

			if (getECU.bombaCombustivel == false) {
				//digitalWrite(bombaCombustivel, HIGH);
				getECU.bombaCombustivel = true;
			} //Check if the fuel pump is on and turn it on if it isn't.

		} else {
			atualRPM = 0;
			getECU.contadorrevolucoes = 0;
			//bombaCombustivel = false;
			getECU.sincronizacao = false;
			getECU.atualRPS = 0;
			//Serial.println("Brain: ENGINE DEAD :(");
		}

		if (getECU.sincronizacao && (atualRPM > 0)) {

			// Podemos configurar a ignição

		}

		//Updatar o valor MAP
		temp_voltage = getADC(MAPSensorPin);
		float temp_MAP = temp_voltage * (MAP_0V - MAP_5V) / (VREF_MINUS - VREF_PLUS);
		getECU.MAP = temp_MAP;

		temp_voltage = getADC(MAPSensorPin);
		float temp_TPS = temp_voltage * (TPS_0V - TPS_5V) / (VREF_MINUS - VREF_PLUS);
		getECU.TPS = temp_TPS;

		temp_voltage = getADC(MAPSensorPin);
		float temp_IAT = temp_voltage * (IAT_0V - IAT_5V) / (VREF_MINUS - VREF_PLUS);
		getECU.IAT = temp_IAT;

		temp_voltage = getADC(MAPSensorPin);
		float temp_CLT = temp_voltage * (CLT_0V - CLT_5V) / (VREF_MINUS - VREF_PLUS);
		getECU.CLT = temp_CLT;

		getECU.RPM = atualRPM;
		//Atualizar a Queue principal
		if (xQueueOverwrite(xECU, &getECU) == pdPASS) {
			//tempoUltimoDente = temp_tempoUltimoDente;
		}

		//Código de detecção touch
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
			xSemaphoreGive(xPaginaMutex);

		}
		//}

		if ( DEBUG == "ON" && DEBUG_LEVEL <= 2) {
			Serial.print("Correu a task: ");
			Serial.println(pcTaskGetTaskName(NULL));
		}
		vTaskDelay(25 / portTICK_PERIOD_MS);
	}
}

/* Nota: n estou a conseguir implementar as queeue peeks sem crashar cpu*/
int getCrankAngle(int tempoPorGrau, int contadorDentes) {
	unsigned long temp_tempoUltimoDente;
	unsigned long tempoUltimoDente = 1; // n pode ser volatile pq quero mandar valores para a queeue
	//int temp_contadorDentes;

// Prioridade máxima

	//xQueuePeek(xContadorDentes, &temp_contadorDentes, (TickType_t) 10); //portMAX_DELAY
	//xQueuePeek(xtempoUltimoDente, &temp_tempoUltimoDente, (TickType_t) 10);
	//Prioridade normal

	//int anguloCambota = ((temp_contadorDentes - 1) * anguloPorDente) + desvioPrimeiroDenteTDC;
	int anguloCambota = ((contadorDentes - 1) * anguloPorDente) + desvioPrimeiroDenteTDC;

	//Estimar o num de graus desde o ultimo dente
	//long tempoDecorridoDesdeUltimoDente = micros() - temp_tempoUltimoDente;
	long tempoDecorridoDesdeUltimoDente = micros() - tempoUltimoDente;
	//tempoPorGrau = (temp_tempoUltimoDente / anguloPorDente); //Se n funcionar faço apartir da rpm
	tempoPorGrau = (tempoUltimoDente / anguloPorDente); //Se n funcionar faço apartir da rpm
	anguloCambota = anguloCambota + ldiv(tempoDecorridoDesdeUltimoDente, tempoPorGrau).quot;

	if (anguloCambota > 360) {
		anguloCambota = anguloCambota - 360;
	}

	return anguloCambota;
}

int getLowRPM(long temp_tempoUltimoDente, long temp_tempoPenultimoDente, int temp_RPMatual) {

	//unsigned long temp_tempoUltimoDente = 1, temp_tempoPenultimoDente = 0;

	int tempoEntreDentes = (temp_tempoUltimoDente - temp_tempoPenultimoDente);
	if (tempoEntreDentes == 0) {
		return temp_RPMatual;
	} else {

		int rpm_temporary = (((60000000 / (tempoEntreDentes * 36)) + 1) * 2); //+1 pq falha, n sei pk x2??
		return rpm_temporary;
	}
	//Serial.println("	gay");
}

void coilDischargeTimer_callback(void *arg) {

	digitalWrite(bobine1Pin, HIGH);

	//prntIGN();
}

// Interrupção gerada na transição do sinal do sensor de Hall
void IRAM_ATTR onPulse(void) {
	//Serial.println("	gay");
	ecu_info_t updateECU;
	//unsigned long tempoUltimoDente = 1, tempoPenultimoDente = 0, tempoAtual = 0, VALOR_QUEUE = 0, falhaAtual = 0, falhaObjetivo = 0, tempoPrimeiroDente = 0, tempoFiltro = 0, tempoPrimeiroDenteMenosUm = 0; // n pode ser volatile pq quero mandar valores para a queeue
	unsigned long falhaAtual = 0, falhaObjetivo = 0, tempoFiltro = 0, tempoAtual = 0; // n pode ser volatile pq quero mandar valores para a queeue
	BaseType_t xHigherPriorityTaskWoken;
	//uint16_t contadorDentes = 35, VALOR_QUEUE_INT = 0;

	if (xQueueReceiveFromISR(xECU, &updateECU, &xHigherPriorityTaskWoken) == pdPASS) {

		xQueueSendFromISR(xECU, &updateECU, &xHigherPriorityTaskWoken);
	}

	tempoAtual = micros();
	falhaAtual = tempoAtual - updateECU.tempoUltimoDente;

	if (falhaAtual < updateECU.tempoFiltro) {
		//Serial.println("Debounce");
	} else {

		//Serial.print("Tempo Falha: ");
		//Serial.println(falhaAtual);
		//Add para guardar, queue aberturaAtual

		falhaObjetivo = 1.5 * (updateECU.tempoUltimoDente - updateECU.tempoPenultimoDente);

		if (falhaAtual > falhaObjetivo || updateECU.ContadorDentes > numRealDentes) {

			updateECU.ContadorDentes = 1;
			updateECU.tempoPrimeiroDenteMenosUm = updateECU.tempoPrimeiroDente;
			updateECU.tempoPrimeiroDente = tempoAtual;
			updateECU.sincronizacao = true;
			updateECU.contadorrevolucoes++;
			updateECU.descargaBobine = LOW;

			//setup injecao  -- DA ERRO QND TENTO INICIALIZAR O DUTY CYCLE
			/*float duty = (interpolation(50, 2500, 1) / 0.012); // y = 0,012x (X0; Y0),(X8191;Y100)
			 ESP_ERROR_CHECK(ledc_set_duty(INJ1_MODE, INJ1_CHANNEL, (int)duty));
			 ESP_ERROR_CHECK(ledc_update_duty(INJ1_MODE, INJ1_CHANNEL));*/

			//digitalWrite(bobine1Pin, updateECU.descargaBobine);
			digitalWrite(bobine1Pin, LOW);
			//xQueueOverwriteFromISR(xtempoPrimeiroDenteMenosUm, &tempoPrimeiroDenteMenosUm, &xHigherPriorityTaskWoken);
		} else {
			updateECU.tempoFiltro = falhaAtual * 0.25; // 25% de filtro
			updateECU.ContadorDentes++;
			/*Serial.print("counter 1.5: ");
			 Serial.println(contadorDentes);*/
			//xQueueOverwriteFromISR(xtempoFiltro, &tempoFiltro, &xHigherPriorityTaskWoken);
			//xQueueSendToFrontFromISR(xContadorDentes, &contadorDentes, &xHigherPriorityTaskWoken);
		}

		/*if (xQueueOverwriteFromISR(xContadorDentes, &contadorDentes, &xHigherPriorityTaskWoken) == pdPASS) {
		 //printf("Value %d sent to the queue\n", contadorDentes);
		 }*/

		if (updateECU.ContadorDentes == 3) {

			/*//Buscar o valor da ignição
			 float valorignicao = 7;
			 float tempoPorGrau = (tempoUltimoDente / anguloPorDente);
			 float anguloDisparo = (desvioPrimeiroDenteTDC - valorignicao) - getCrankAngle(tempoPorGrau, contadorDentes);
			 //timerAlarm(coilTimer, (anguloDisparo * tempoPorGrau), false, 0);
			 esp_timer_start_once(coilTimer, long(anguloDisparo * tempoPorGrau));
			 Serial.println("TIMER HAS BEEN TRIGGERED!!!!!");*/
		}

		updateECU.tempoPenultimoDente = updateECU.tempoUltimoDente;
		updateECU.tempoUltimoDente = tempoAtual;

		xQueueOverwriteFromISR(xECU, &updateECU, &xHigherPriorityTaskWoken);
	}
}

void vDisplay(void *pvParameters) {
	float temp_motor = 0, map = 0, tps = 0, avanco = 0;
	uint16_t rpm = 65530;
	char buffer[10];
	bool inicializacao = true;
	ecu_info_t getECU;
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

	for (;;) {

		if (xSemaphoreTake(xPaginaMutex, portMAX_DELAY) == pdTRUE && xQueuePeek(xPagina, &getValores, portMAX_DELAY) == pdPASS) {
			//atualizar pagina
			if (getValores.proxima == true) {
				getValores.proxima = false;
				getValores.antes = false;
				getValores.pagina_atual = ((getValores.pagina_atual) + 1);
				if (getValores.pagina_atual > 2) {
					getValores.pagina_atual = 0;
				}
				inicializacao = true;
				//xQueueOverwrite(xPagina, &getValores.proxima);
				xQueueOverwrite(xPagina, &getValores);
				if ( DEBUG == "ON" && DEBUG_LEVEL <= 3) {
					Serial.println("proxima");
					Serial.println(getValores.pagina_atual);
				}
			}

			if (getValores.antes == true) {
				getValores.antes = false;
				getValores.proxima = false;
				if (getValores.pagina_atual <= 0) {
					getValores.pagina_atual = 2;
				} else {
					getValores.pagina_atual--;
				}
				inicializacao = true;
				//xQueueOverwrite(xPagina, &getValores.antes);
				xQueueOverwrite(xPagina, (void* )&getValores);
				if ( DEBUG == "ON" && DEBUG_LEVEL <= 3) {
					Serial.println("antes");
					Serial.println(getValores.pagina_atual);
				}
			}
			xQueueOverwrite(xPagina, (void* )&getValores);
			xSemaphoreGive(xPaginaMutex);

			//Updates de valores, apenas vamos atualizar valores na pagina principal
			if (inicializacao == false && getValores.pagina_atual == 0) {
				tft.setTextSize(2);

				if (xQueuePeek(xECU, &getECU, portMAX_DELAY) == pdPASS) {
					//tempoUltimoDente = temp_tempoUltimoDente;

					dtostrf(getECU.IAT, 3, 0, buffer);
					tft.setCursor(35, 50);
					tft.setTextSize(1);
					tft.print(buffer);
					tft.print((char) 167);
					tft.print("C");

					dtostrf(getECU.CLT, 3, 0, buffer);
					tft.setCursor(145, 50);
					tft.print(buffer);
					tft.print((char) 167);
					tft.print("C");

					dtostrf(getECU.MAP, 3, 0, buffer);
					tft.setCursor(250, 50);
					tft.print(buffer);
					tft.print(" kPa");

					dtostrf(getECU.avancoIGN, 2, 0, buffer);
					tft.setCursor(40, 220);
					tft.print(buffer);
					tft.print((char) 167);

					dtostrf(getECU.TPS, 3, 0, buffer);
					tft.setCursor(145, 220);
					//tft.print(getECU.TPS);
					tft.print(buffer);
					tft.print("%");

					//sprintf() usa demasiados recursos, dtostrf() é mais eficiente a converter floats to strings
					sprintf(buffer, "%5.0d", getECU.RPM); //avancoIGN
					tft.setCursor(250, 220);
					tft.print(buffer);

					//Serial.println(getECU.RPM);
				}

				if ( DEBUG == "ON" && DEBUG_LEVEL <= 3) {
					Serial.println("  Tarefa: Update valores");
				}
			}

			// Desenhos estacionários
			if (inicializacao == true && getValores.pagina_atual == 1) {
				//Desenhar tabela VE
				inicializacao = false;

				uint16_t textColor = ILI9341_WHITE;
				uint16_t lineColor = ILI9341_WHITE;
				uint16_t bgColor = ILI9341_BLACK;

				// Limpa a tela
				tft.fillScreen(bgColor);

				// Define o número de linhas e colunas
				int numLin = 13;
				int numCol = 17;

				// Define o tamanho da tabela
				int startX = 5;
				int startY = 5;
				int cellWidth = 18;
				int cellHeight = 18;

				// Define o tamanho do texto
				tft.setTextSize(1);

				// Desenha as linhas horizontais
				for (int i = 0; i <= numLin; i++) {
					tft.drawLine(startX, startY + i * cellHeight, startX + numCol * cellWidth, startY + i * cellHeight, lineColor);
				}

				// Desenha as linhas verticais
				for (int i = 0; i <= numCol; i++) {
					tft.drawLine(startX + i * cellWidth, startY, startX + i * cellWidth, startY + numLin * cellHeight, lineColor);
				}

				// Preenche a tabela com texto
				for (int row = 0; row < numLin; row++) {
					for (int col = 0; col < numCol; col++) {
						//String cellText = String(row + 1) + "," + String(col + 1);
						String cellText;
						if (row == 12 && col > 0) {
							cellText = String(PRE[col - 1]);
							tft.setTextColor(ILI9341_RED, bgColor);
						} else if (col == 0) {
							if (row == 12) {
								cellText = "F";
								tft.setTextColor(ILI9341_GREEN, bgColor);
							} else {
								cellText = String(RPM[row]);
								tft.setTextColor(ILI9341_CYAN, bgColor);
							}
						} else {
							cellText = String(VE[row][col]);
							tft.setTextColor(textColor, bgColor);
						}
						int16_t textWidth = cellText.length() * 6; // Cada caractere tem ~6px de largura em setTextSize(1)
						int16_t textHeight = 8; // A altura do texto em setTextSize(1) é ~8px

						// Calcula as coordenadas para centralizar o texto
						int textX = startX + col * cellWidth + (cellWidth - textWidth) / 2;
						int textY = startY + row * cellHeight + (cellHeight - textHeight) / 2;

						tft.setCursor(textX, textY);
						//tft.setTextColor(textColor, bgColor);
						tft.print(cellText);
					}
				}
			}

			if (inicializacao == true && getValores.pagina_atual == 2) {

				//Desenhar tabela IGN

				uint16_t textColor = ILI9341_WHITE;
				uint16_t lineColor = ILI9341_WHITE;
				uint16_t bgColor = ILI9341_BLACK;

				// Limpa a tela
				tft.fillScreen(bgColor);

				// Define o número de linhas e colunas
				int numRows = 13;
				int numCols = 17;

				// Define o tamanho da tabela
				int startX = 5;
				int startY = 5;
				int cellWidth = 18;
				int cellHeight = 18;

				// Define o tamanho do texto
				tft.setTextSize(1);

				// Desenha as linhas horizontais
				for (int i = 0; i <= numRows; i++) {
					tft.drawLine(startX, startY + i * cellHeight, startX + numCols * cellWidth, startY + i * cellHeight, lineColor);
				}

				// Desenha as linhas verticais
				for (int i = 0; i <= numCols; i++) {
					tft.drawLine(startX + i * cellWidth, startY, startX + i * cellWidth, startY + numRows * cellHeight, lineColor);
				}

				// Preenche a tabela com texto
				for (int row = 0; row < numRows; row++) {
					for (int col = 0; col < numCols; col++) {
						String cellText;
						if (row == 12 && col > 0) {
							cellText = String(PRE[col - 1]);
							tft.setTextColor(ILI9341_RED, bgColor);
						} else if (col == 0) {
							if (row == 12) {
								cellText = "I";
								tft.setTextColor(ILI9341_GREEN, bgColor);
							} else {
								cellText = String(RPM[row]);
								tft.setTextColor(ILI9341_CYAN, bgColor);
							}
						} else {
							cellText = String(IGN[row][col]);
							tft.setTextColor(textColor, bgColor);
						}
						int16_t textWidth = cellText.length() * 6; // Cada caractere tem ~6px de largura em setTextSize(1)
						int16_t textHeight = 8; // A altura do texto em setTextSize(1) é ~8px

						// Calcula as coordenadas para centralizar o texto
						int textX = startX + col * cellWidth + (cellWidth - textWidth) / 2;
						int textY = startY + row * cellHeight + (cellHeight - textHeight) / 2;

						tft.setCursor(textX, textY);
						//tft.setTextColor(textColor, bgColor);
						tft.print(cellText);
					}
				}
				/*	//Desenhar tabela IGN
				 */
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
						tft.drawPixel(coluna + 230, linha + 170, pgm_read_word(logo_rpm + inicio_x)); // Desenhar o pixel no sitio correto
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
						tft.drawPixel(coluna + 110, linha + 170, pgm_read_word(logo_tps + inicio_x)); // Desenhar o pixel no sitio correto
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
						tft.drawPixel(coluna + 20, linha + 160, pgm_read_word(logo_avanco + inicio_x)); // Desenhar o pixel no sitio correto
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

			vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
		}
	}
}

float interpolation(int hpa, int revs, int type) {

	// SE TYPE == 1, VE
	// SE TYPE == 0, IGN

	// vTaskDelay(3000/portTICK_PERIOD_MS);
	// ESP_LOGI(SPFS, "Initializing Interpoltion\n");
	// printf("Pressure: %d RPMS: %d\n",hpa,revs);
	int i = 0, j = 0;

	float x, y;
	int xl, xh, yl, yh;

	if (type == 0) {
		do {
			i++;
		} while (hpa >= PRE[i] && PRE[i] != 1050);
		xl = PRE[i - 1];
		xh = PRE[i];

		do {
			j++;
		} while (revs >= RPM[j] && RPM[j] != 8000);
		yl = RPM[j - 1];
		yh = RPM[j];

		// printf("\n%.2f   %.2f\n",IGN[j-1][i-1], IGN[j-1][i]);
		// printf("%.2f   %.2f\n",IGN[j][i-1], IGN[j][i]);

		x = (100 * (hpa - PRE[i - 1]) / (xh - xl)) * (100 * (IGN[j - 1][i] - IGN[j - 1][i - 1]));

		y = (100 * (revs - RPM[j - 1]) / (yh - yl)) * (100 * (IGN[j][i - 1] - IGN[j - 1][i - 1]));

		float dist = 0;
		if (IGN[j][i] >= IGN[j - 1][i - 1]) {
			dist = (sqrt(pow(x, 2) + pow(y, 2)) / 10000) + IGN[j - 1][i - 1];
		} else {
			dist = IGN[j - 1][i - 1] - (sqrt(pow(x, 2) + pow(y, 2)) / 10000);
		};

		/* VALOR FINAL DE AVANCO, MANDAR PARA QUEUE, OU REQUISITAR FUNCAO DIRETAMENTE*/
		return dist;
	}
	if (type == 1) {
		do {
			i++;
		} while (hpa >= PRE[i] && PRE[i] != 1050);
		xl = PRE[i - 1];
		xh = PRE[i];

		do {
			j++;
		} while (revs >= RPM[j] && RPM[j] != 8000);
		yl = RPM[j - 1];
		yh = RPM[j];

		// printf("\n%.2f   %.2f\n",IGN[j-1][i-1], IGN[j-1][i]);
		// printf("%.2f   %.2f\n",IGN[j][i-1], IGN[j][i]);

		x = (100 * (hpa - PRE[i - 1]) / (xh - xl)) * (100 * (VE[j - 1][i] - VE[j - 1][i - 1]));

		y = (100 * (revs - RPM[j - 1]) / (yh - yl)) * (100 * (VE[j][i - 1] - VE[j - 1][i - 1]));

		float dist = 0;
		if (VE[j][i] >= VE[j - 1][i - 1]) {
			dist = (sqrt(pow(x, 2) + pow(y, 2)) / 10000) + VE[j - 1][i - 1];
		} else {
			dist = VE[j - 1][i - 1] - (sqrt(pow(x, 2) + pow(y, 2)) / 10000);
		};

		/* VALOR FINAL DE AVANCO, MANDAR PARAS QUEUE, OU REQUISITAR FUNCAO DIRETAMENTE*/
		//VE_Value = dist;
		// printf("\ndist= %.4f\n",dist);
		// float VolEff = VE[j-1][i-1]+ sqrt(  power( x-VE[j-1][i-1] ) + power(y-VE[j-1][i-1])   );
		// VE_Value=VolEff;
		// printf("\n");
		//printf("this ran\n");
		return dist;
	}
	return 0; // só para o compiler não guinchar
}

void prntIGN(void) {

	//Print VE Table
	int i, j;

	printf("Pressao:\n");
	for (i = 0; i < 16; i++) {
		printf("%d ", PRE[i]);
	}
	printf("\n");

	printf("RPM:\n");
	for (i = 0; i < 12; i++) {
		printf("%d ", RPM[i]);
	}
	printf("\n");

	printf("Avanco:\n");

	j = 0;
	while (j < 12) {
		for (i = 0; i < 16; i++) {
			printf("%d ", IGN[j][i]);

		}
		printf("\n");
		j++;
	}

	printf("\n");
	return;
}

float getADC(int Pino) {
	int temp_valorAnalogico;
	float temp_ddpAnalogica;
	//float temp_ADCValue;

	temp_valorAnalogico = analogRead(Pino);
	temp_ddpAnalogica = temp_valorAnalogico * (VREF_PLUS - VREF_MINUS) / (pow(2.0, (float) ADC_RESOLUTION)) + VREF_MINUS;

	if ( DEBUG == "ON" && DEBUG_LEVEL <= 2) {
		Serial.println("Correu a função getADC() ");
	}

	return temp_ddpAnalogica;
}

//------------------------------------------------------------------------------
void loop() {
	vTaskDelete(NULL);
}
