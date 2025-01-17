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

#include "tables.h"

#define TEMPO_MAXIMO_MOTOR_MORRER 1000
#define ADC_RESOLUTION 6
#define VREF_PLUS  5
#define VREF_MINUS  0.0

#define MAP_5V  100 //kpa
#define MAP_0V 10 //kpa

#define TPS_5V  100 //%
#define TPS_0V 0 //%

#define CLT_5V  120 //ºC
#define CLT_0V -10 //ºC

#define IAT_5V  120 //ºC
#define IAT_0V -10 //ºC

#define DEBUG "OFF"
#define DEBUG_LEVEL 2

// Definir os terminais do LCD
#define TFT_CS   05
#define TFT_DC   26
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18
#define TFT_RST -1 // ligar ao 3V3

#define INJ1_FREQ 1000 //Se o injector deadtime for 1ms@12V
#define INJ1_TIMER LEDC_TIMER_3//Selecionar o timer a ser usado
#define INJ1_RESO LEDC_TIMER_13_BIT//Resolução do timer utilizado
#define INJ1_MODE LEDC_LOW_SPEED_MODE
#define INJ1_CHANNEL LEDC_CHANNEL_0
#define INJ1_MAX_DUTY ((1 << 13) - 1)// Max duty (8191 for 13 bits)

// Criar um objeto tft com indicação dos terminais CS e DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,
TFT_RST, TFT_MISO);

/* The service routine for the interrupt.  This is the interrupt that the task
 will be synchronized with. */
void IRAM_ATTR onPulse(void);
void coilDischargeTimer_callback(void *arg);
//void IRAM_ATTR coilDischargeTimer(void);

/* The tasks to be created. */
void vInitDisplay(void *pvParameters);
void vBrain(void *pvParameters);

//Declarar funcoes
int getCrankAngle(int, int);
void getLowRPM(void);
void prntIGN(void);
float interpolation(int, int, int);
float getADC(int);
//void setup_timers(void);

//Handles
QueueHandle_t xRPM;
QueueHandle_t xContadorDentes;
QueueHandle_t xtempoUltimoDente;
QueueHandle_t xtempoPenultimoDente;
QueueHandle_t xMAP;
QueueHandle_t xtempoPrimeiroDente;
QueueHandle_t xtempoFiltro;
QueueHandle_t xtempoPrimeiroDenteMenosUm;

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

//volatile unsigned long tempoAtual = 0;
//QueueHandle_t xtempoAtual;

volatile unsigned long tempoAtual_Brain = 1; // Para n dar erro no startup
unsigned long tempoPorGrau = 0; // n pode ser volatile pq quero mandar valores para a queeue
bool sincronizacao = false;
volatile bool descargaBobine = LOW;
volatile uint32_t contadorrevolucoes = 0;
uint64_t atualRPS = 0;
bool bombaCombustivel = false;
//unsigned long tempoDecorrido = 0;

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
	esp_timer_create_args_t coilTimer_args; //= { .callback = &coilDischargeTimer_callback, .arg = NULL, .name = "coilTimer" };
	coilTimer_args.arg = NULL, coilTimer_args.callback = &coilDischargeTimer_callback, coilTimer_args.name = "coilTimer";
	ESP_ERROR_CHECK(esp_timer_create(&coilTimer_args, &coilTimer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(coilTimer, 1000000)); // 5 seconds (5,000,000 µs)

	//queues
	xRPM = xQueueCreate(1, sizeof(uint16_t));
	xContadorDentes = xQueueCreate(1, sizeof(uint16_t));
	xtempoUltimoDente = xQueueCreate(1, sizeof(long));
	xtempoPenultimoDente = xQueueCreate(1, sizeof(long));
	xtempoFiltro = xQueueCreate(1, sizeof(long));
	xtempoPrimeiroDenteMenosUm = xQueueCreate(1, sizeof(long));
	xMAP = xQueueCreate(1, sizeof(float));
	xtempoPrimeiroDente = xQueueCreate(1, sizeof(float));

	//xtempoAtual = xQueueCreate(1, sizeof(float));

	xTaskCreatePinnedToCore(vInitDisplay, "vInitDisplay", 2048, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(vBrain, "vBrain", 4096, NULL, 1, NULL, 1);
	//xTaskCreatePinnedToCore(vGetMAP, "vGetMAP", 512, NULL, 1, NULL, 1);

	if (DEBUG == "ON" && DEBUG_LEVEL <= 3) {
		Serial.println("Setup acabou de correr");
	}

}

/*void setup_timers(void) {

 //timers
 //coilTimer = timerBegin(1000000);
 //timerAttachInterrupt(coilTimer, &coilDischargeTimer);
 esp_timer_create_args_t coilTimer_args; //= { .callback = &coilDischargeTimer_callback, .arg = NULL, .name = "coilTimer" };
 coilTimer_args.arg = NULL, coilTimer_args.callback = &coilDischargeTimer_callback, coilTimer_args.name = "coilTimer";
 ESP_ERROR_CHECK(esp_timer_create(&coilTimer_args, &coilTimer));
 ESP_ERROR_CHECK(esp_timer_start_periodic(coilTimer, 1000000)); // 5 seconds (5,000,000 µs)
 // Configure the LEDC timer

 ledc_timer_config_t inj1_timer = { .speed_mode = INJ1_MODE, .duty_resolution = INJ1_RESO, .timer_num = INJ1_TIMER, .freq_hz = INJ1_FREQ, .clk_cfg = LEDC_AUTO_CLK };
 ESP_ERROR_CHECK(ledc_timer_config(&inj1_timer));

 // Configure the LEDC channel
 ledc_channel_config_t inj1_channel = { .gpio_num = injetor1Pin, .speed_mode = INJ1_MODE, .channel = INJ1_CHANNEL, .intr_type = LEDC_INTR_DISABLE, .timer_sel = INJ1_TIMER, .duty = 0, .hpoint = 0 };
 ESP_ERROR_CHECK(ledc_channel_config(&inj1_channel));

 printf("Timers setup complete\n");
 }*/

void vBrain(void *pvParameters) {
	portBASE_TYPE xStatus;
	unsigned long temp_tempoUltimoDente = 0, tempoUltimoDente = 1;
	uint64_t temp_atualRPM = 0, atualRPM = 0;
	volatile unsigned long loopAtual_Brain = 1; // Para n dar erro no startup
	unsigned long tempoUltimoloop_Brain = 0; // n pode ser volatile pq quero mandar valores para a queeue
	float temp_voltage = 0;

	while (true) {
		//loopAtual_Brain = tempoAtual_Brain - tempoUltimoloop_Brain;
		loopAtual_Brain = tempoAtual_Brain;
		tempoAtual_Brain = micros();

		//xQueuePeek(xtempoUltimoDente, &temp_tempoUltimoDente, 0);
		//Serial.println(temp_tempoUltimoDente);
		//unsigned long tempoParaUltimoDente = (loopAtual_Brain - temp_tempoUltimoDente);
		if (xQueuePeek(xtempoUltimoDente, &tempoUltimoDente, (TickType_t) 25) == pdPASS) {
			//tempoUltimoDente = temp_tempoUltimoDente;
		}
		unsigned long tempoParaUltimoDente = (loopAtual_Brain - tempoUltimoDente);

		//Serial.println("Brain: RUNNING");

		if ((tempoParaUltimoDente < TEMPO_MAXIMO_MOTOR_MORRER) || (tempoParaUltimoDente > loopAtual_Brain)) {
			int ultimaRPM = atualRPM;
			getLowRPM();
			if (xQueuePeek(xRPM, &temp_atualRPM, (TickType_t) 25) == pdPASS) { // Para calcular RPS
				atualRPM = temp_atualRPM;
			}
			//xQueuePeek(xRPM, &atualRPM, (TickType_t) 10);
			if (bombaCombustivel == false) {
				//digitalWrite(bombaCombustivel, HIGH);
				bombaCombustivel = true;
			} //Check if the fuel pump is on and turn it on if it isn't.
			  //atualRPS = ldiv(1000000, (loopAtual_Brain - tempoUltimoloop_Brain)).quot * (atualRPM - ultimaRPM); //This is the RPM per second that the engine has accelerated/decelleratedin the last loop
			  //Serial.println("Brain: ENGINE RUNNING");
		} else {
			atualRPM = 0;
			contadorrevolucoes = 0;
			//bombaCombustivel = false;
			sincronizacao = false;
			atualRPS = 0;
			//Serial.println("Brain: ENGINE DEAD :(");
		}

		if (sincronizacao && (atualRPM > 0)) {

			// Podemos configurar a ignição

		}

		//Updatar o valor MAP
		temp_voltage = getADC(MAPSensorPin);
		float temp_MAP = temp_voltage * (MAP_0V - MAP_5V) / (VREF_MINUS - VREF_PLUS);
		//xQueueSendToFront(xMAP, &temp_MAP, portMAX_DELAY);
		//Serial.println((int) temp_MAP);

		//int anguloCambota = getCrankAngle(tempoPorGrau);

		/*Serial.println(sincronizacao ? "true" : "false");

		 Serial.print("atualRPS: ");
		 Serial.println(atualRPS);

		 Serial.print("anguloCambota: ");
		 Serial.println(anguloCambota);

		 Serial.print("contadorrevolucoes: ");
		 Serial.println(contadorrevolucoes);*/

		if ( DEBUG == "ON" && DEBUG_LEVEL <= 2) {
			Serial.print("Correu a task: ");
			Serial.println(pcTaskGetTaskName(NULL));
		}
		vTaskDelay(5 / portTICK_PERIOD_MS);
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

void getLowRPM(void) {
	unsigned long temp_tempoUltimoDente = 1, temp_tempoPenultimoDente = 0;
	//Maxima prioridade ou desativar interrupcoes
	//taskDISABLE_INTERRUPTS();
	//tempoEntreDentes = (tempoUltimoDente - tempoPenultimoDente);
	//if (xQueuePeek(xtempoUltimoDente, &temp_tempoUltimoDente, (TickType_t) 250) == pdPASS && xQueuePeek(xtempoPenultimoDente, &temp_tempoPenultimoDente, (TickType_t) 250) == pdPASS) { // Para calcular RPS
	xQueuePeek(xtempoUltimoDente, &temp_tempoUltimoDente, (TickType_t) 25);
	xQueuePeek(xtempoPenultimoDente, &temp_tempoPenultimoDente, (TickType_t) 25);
	uint64_t tempoEntreDentes = (temp_tempoUltimoDente - temp_tempoPenultimoDente);
	if (tempoEntreDentes == 0) {
		/*Serial.println("tempoEntreDentes: 0");
		 Serial.print("temp_tempoUltimoDente: ");
		 Serial.println(temp_tempoUltimoDente);
		 Serial.print("temp_tempoPenultimoDente: ");
		 Serial.println(temp_tempoPenultimoDente);*/
	} else {
		uint64_t rpm_temporary = (((60000000 / (tempoEntreDentes * 36)) + 1) * 2); //+1 pq falha, n sei pk x2??
		if (xQueueOverwrite(xRPM, &rpm_temporary) == pdPASS) {
		}
	}

}

//uint64_t tempoEntreDentes = (tempoUltimoDente - tempoPenultimoDente);
//uint64_t rpm_temporary = (((60000000 / (tempoEntreDentes * 36)) + 1) * 2); //+1 pq falha, n sei pk x2??
//taskENABLE_INTERRUPTS();
//Prioridade normal

/*if (xQueueOverwrite(xRPM, &rpm_temporary) == pdPASS) {
 //printf("Value %d sent to the queue\n", rpm_temporary);
 }*/
//xQueueSendToFront(xRPM, &rpm_temporary, 0);
//Serial.println(rpm_temporary);
//}
/*void IRAM_ATTR coilDischargeTimer(void) {
 /*portENTER_CRITICAL_ISR(&coilTimerMux);

 descargaBobine = HIGH;
 digitalWrite(bobine1Pin, descargaBobine);

 portEXIT_CRITICAL_ISR(&coilTimerMux);*/
//}
void coilDischargeTimer_callback(void *arg) {
	uint64_t temp_atualRPM = 0;
	float duty = 0;

	if (xQueuePeek(xRPM, &temp_atualRPM, (TickType_t) 250) == pdPASS) {
		//atualRPM = temp_atualRPM;
	}

	digitalWrite(bobine1Pin, HIGH);
	//xQueuePeek(xRPM, &atualRPM, (TickType_t) 10);
	duty = (interpolation(50, 2500, 1) / 0.012); // y = 0,012x (X0; Y0),(X8191;Y100)
	ESP_ERROR_CHECK(ledc_set_duty(INJ1_MODE, INJ1_CHANNEL, (int)duty));
	ESP_ERROR_CHECK(ledc_update_duty(INJ1_MODE, INJ1_CHANNEL));

	if ( DEBUG == "ON" && DEBUG_LEVEL <= 2) {
		printf("Duty Cycle VE: %d\n", (int) duty);
		Serial.print("Número total de dentes: ");
		Serial.println(numRealDentes);
		Serial.print("RPM: ");
		Serial.println(temp_atualRPM);
		Serial.print("advance: ");
		Serial.println(interpolation(50, 2500, 0));
	}

	//prntIGN();

}

// Interrupção gerada na transição do sinal do sensor de Hall
void IRAM_ATTR onPulse(void) {
	unsigned long tempoUltimoDente = 1, tempoPenultimoDente = 0, tempoAtual = 0, VALOR_QUEUE = 0, falhaAtual = 0, falhaObjetivo = 0, tempoPrimeiroDente = 0, tempoFiltro = 0, tempoPrimeiroDenteMenosUm = 0; // n pode ser volatile pq quero mandar valores para a queeue
	BaseType_t xHigherPriorityTaskWoken;
	uint16_t contadorDentes = 35, VALOR_QUEUE_INT = 0;

	if (xQueueReceiveFromISR(xtempoUltimoDente, &VALOR_QUEUE, &xHigherPriorityTaskWoken) == pdPASS) {
		tempoUltimoDente = VALOR_QUEUE;
		// Push it back to the queue
		xQueueSendFromISR(xtempoUltimoDente, &tempoUltimoDente, &xHigherPriorityTaskWoken);
	}
	if (xQueueReceiveFromISR(xtempoPenultimoDente, &VALOR_QUEUE, &xHigherPriorityTaskWoken) == pdPASS) {
		tempoPenultimoDente = VALOR_QUEUE;
		// Push it back to the queue
		xQueueSendFromISR(xtempoPenultimoDente, &tempoPenultimoDente, &xHigherPriorityTaskWoken);
	}
	if (xQueueReceiveFromISR(xtempoFiltro, &VALOR_QUEUE, &xHigherPriorityTaskWoken) == pdPASS) {
		tempoFiltro = VALOR_QUEUE;
		// Push it back to the queue
		xQueueSendFromISR(xtempoFiltro, &tempoFiltro, &xHigherPriorityTaskWoken);
	}

	if (xQueueReceiveFromISR(xContadorDentes, &VALOR_QUEUE_INT, &xHigherPriorityTaskWoken) == pdPASS) {
		contadorDentes = VALOR_QUEUE_INT;
		// Push it back to the queue
		xQueueSendFromISR(xContadorDentes, &contadorDentes, &xHigherPriorityTaskWoken);
	}
	tempoAtual = micros();
	falhaAtual = tempoAtual - tempoUltimoDente;

	if (falhaAtual < tempoFiltro) {
		//Serial.println("Debounce");
	} else {

		//Serial.print("Tempo Falha: ");
		//Serial.println(falhaAtual);
		//Add para guardar, queue aberturaAtual

		falhaObjetivo = 1.5 * (tempoUltimoDente - tempoPenultimoDente);

		if (falhaAtual > falhaObjetivo || contadorDentes > numRealDentes) {
			if (xQueueReceiveFromISR(xtempoPrimeiroDente, &VALOR_QUEUE, &xHigherPriorityTaskWoken) == pdPASS) {
				tempoPrimeiroDente = VALOR_QUEUE;
				// Push it back to the queue
				xQueueSendFromISR(xtempoPrimeiroDente, &tempoPrimeiroDente, &xHigherPriorityTaskWoken);
			}
			contadorDentes = 1;
			tempoPrimeiroDenteMenosUm = tempoPrimeiroDente;
			tempoPrimeiroDente = tempoAtual;
			sincronizacao = true;
			contadorrevolucoes++;
			descargaBobine = LOW;
			digitalWrite(bobine1Pin, descargaBobine);
			xQueueOverwriteFromISR(xtempoPrimeiroDenteMenosUm, &tempoPrimeiroDenteMenosUm, &xHigherPriorityTaskWoken);
		} else {
			tempoFiltro = falhaAtual * 0.25; // 25% de filtro
			contadorDentes++;
			/*Serial.print("counter 1.5: ");
			 Serial.println(contadorDentes);*/
			xQueueOverwriteFromISR(xtempoFiltro, &tempoFiltro, &xHigherPriorityTaskWoken);
			//xQueueSendToFrontFromISR(xContadorDentes, &contadorDentes, &xHigherPriorityTaskWoken);
		}

		if (xQueueOverwriteFromISR(xContadorDentes, &contadorDentes, &xHigherPriorityTaskWoken) == pdPASS) {
			//printf("Value %d sent to the queue\n", contadorDentes);
		}

		if (contadorDentes == 3) {
			/*//Buscar o valor da ignição
			 float valorignicao = 7;
			 float tempoPorGrau = (tempoUltimoDente / anguloPorDente);
			 float anguloDisparo = (desvioPrimeiroDenteTDC - valorignicao) - getCrankAngle(tempoPorGrau, contadorDentes);
			 //timerAlarm(coilTimer, (anguloDisparo * tempoPorGrau), false, 0);
			 esp_timer_start_once(coilTimer, long(anguloDisparo * tempoPorGrau));
			 Serial.println("TIMER HAS BEEN TRIGGERED!!!!!");*/
		}

		tempoPenultimoDente = tempoUltimoDente;
		tempoUltimoDente = tempoAtual;
		if (xQueueOverwriteFromISR(xtempoUltimoDente, &tempoUltimoDente, &xHigherPriorityTaskWoken) == pdPASS && xQueueOverwriteFromISR(xtempoPenultimoDente, &tempoPenultimoDente, &xHigherPriorityTaskWoken) == pdPASS) {
			//printf("Value %d sent to the queue\n", tempoUltimoDente);
		}
		//xQueueOverwriteFromISR(xtempoUltimoDente, &tempoUltimoDente, &xHigherPriorityTaskWoken);
		/*Serial.print("counter 3: ");
		 Serial.println(contadorDentes);*/
	}
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

		if ( DEBUG == "ON" && DEBUG_LEVEL <= 2) {
			Serial.print("Correu a task: ");
			Serial.println(pcTaskGetTaskName(NULL));
		}
		vTaskDelay(500 / portTICK_PERIOD_MS);
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
	//float voltage = (temp_valorAnalogico / 4095.0) * 5.0;

	//Serial.println(temp_valorAnalogico);
	//Serial.println(voltage);
	//Serial.println(voltage);

	// temp_ADCValue = 45.2042 * temp_ddpAnalogica;

	//xQueueSendToFront(xMAP, &temp_MAPValue, 0);

	if ( DEBUG == "ON" && DEBUG_LEVEL <= 2) {
		Serial.println("Correu a função getADC() ");
	}

	return temp_ddpAnalogica;
}

//------------------------------------------------------------------------------
void loop() {
	vTaskDelete(NULL);
}
