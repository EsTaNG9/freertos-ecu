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

#define TEMPO_MAXIMO_MOTOR_MORRER 1000
#define ADC_RESOLUTION 6
#define VREF_PLUS  5
#define VREF_MINUS  0.0

#define DEBUG "OFF"
#define DEBUG_LEVEL 2

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
void coilDischargeTimer_callback(void *arg);
//void IRAM_ATTR coilDischargeTimer(void);

/* The tasks to be created. */
//void calculateRPM(void *pvParameters);
void vInitDisplay(void *pvParameters);
void vBrain(void *pvParameters);
void vGetMAP(void *pvParameters);

//Declarar funcoes
int getCrankAngle(int);
void getLowRPM(void);

//Handles
QueueHandle_t xRPM;
QueueHandle_t xContadorDentes;
QueueHandle_t xtempoUltimoDente;
QueueHandle_t xMAP;

// Timers
/*hw_timer_t *coilTimer = NULL;
 portMUX_TYPE coilTimerMux = portMUX_INITIALIZER_UNLOCKED;*/

// Estruturas
//struct table3D fuelTable; //16x16 fuel map
//struct table3D ignitionTable; //16x16 ignition map
// pin to generate interrupts
const uint8_t interruptPin = 4;
//const uint8_t bombaCombustivelPin = 28;
const uint8_t MAPSensorPin = 36;
const uint8_t bobine1Pin = 32;
const uint8_t injetor1Pin = 33;

//Configurar roda fonica
uint16_t numRealDentes = 35;
uint16_t anguloPorDente = 360 / 36;
uint16_t desvioPrimeiroDenteTDC = 180; //Desvio real do TDC 1ºcil do primeiro dente da roda

volatile unsigned long tempoAtual = 0;
volatile unsigned long tempoAtual_Brain = 1; // Para n dar erro no startup
volatile unsigned long falhaAtual = 0;
volatile unsigned long loopAtual_Brain = 1; // Para n dar erro no startup
unsigned long tempoUltimoDente = 1; // n pode ser volatile pq quero mandar valores para a queeue
unsigned long tempoUltimoloop_Brain = 0; // n pode ser volatile pq quero mandar valores para a queeue
volatile unsigned long tempoFiltro = 0;
volatile unsigned long tempoPenultimoDente = 0;
volatile unsigned long falhaObjetivo = 0;
uint16_t contadorDentes = 0; // n pode ser volatile pq quero mandar valores para a queeue
volatile unsigned long tempoPrimeiroDenteMenosUm = 0;
volatile unsigned long tempoPrimeiroDente = 0;
unsigned long tempoPorGrau = 0; // n pode ser volatile pq quero mandar valores para a queeue
bool sincronizacao = false;
volatile bool descargaBobine = LOW;
volatile uint32_t contadorrevolucoes = 0;
uint64_t tempoEntreDentes = 1;
uint64_t atualRPS = 0;
uint64_t atualRPM = 0;
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
	pinMode(MAPSensorPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(interruptPin), onPulse, RISING);

	//timers
	//coilTimer = timerBegin(1000000);
	//timerAttachInterrupt(coilTimer, &coilDischargeTimer);
	esp_timer_handle_t coilTimer;
	esp_timer_create_args_t coilTimer_args = { .callback = &coilDischargeTimer_callback, .arg = NULL, .name = "coilTimer" };
	ESP_ERROR_CHECK(esp_timer_create(&coilTimer_args, &coilTimer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(coilTimer, 1000000)); // 5 seconds (5,000,000 µs)

	//queues
	xRPM = xQueueCreate(5, sizeof(uint16_t));
	xContadorDentes = xQueueCreate(5, sizeof(uint8_t));
	xtempoUltimoDente = xQueueCreate(5, sizeof(long));
	xMAP = xQueueCreate(5, sizeof(float));

	xTaskCreatePinnedToCore(vInitDisplay, "vInitDisplay", 2048, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(vBrain, "vBrain", 4096, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(vGetMAP, "vGetMAP", 512, NULL, 1, NULL, 1);

	if (DEBUG == "ON" && DEBUG_LEVEL <= 3) {
		Serial.println("Setup acabou de correr");
	}

}

void vBrain(void *pvParameters) {
	while (true) {
		unsigned long temp_tempoUltimoDente;

		//loopAtual_Brain = tempoAtual_Brain - tempoUltimoloop_Brain;
		loopAtual_Brain = tempoAtual_Brain;
		tempoAtual_Brain = micros();

		//xQueuePeek(xtempoUltimoDente, &temp_tempoUltimoDente, 0);
		//Serial.println(temp_tempoUltimoDente);
		//unsigned long tempoParaUltimoDente = (loopAtual_Brain - temp_tempoUltimoDente);
		unsigned long tempoParaUltimoDente = (loopAtual_Brain - tempoUltimoDente);

		//Serial.println(tempoParaUltimoDente);

		if ((tempoParaUltimoDente < TEMPO_MAXIMO_MOTOR_MORRER) || (tempoParaUltimoDente > loopAtual_Brain)) {
			int ultimaRPM = atualRPM;
			getLowRPM();
			xQueuePeek(xRPM, &atualRPM, (TickType_t) 10); // Para calcular RPS
			if (bombaCombustivel == false) {
				//digitalWrite(bombaCombustivel, HIGH);
				bombaCombustivel = true;
			} //Check if the fuel pump is on and turn it on if it isn't.
			atualRPS = ldiv(1000000, (loopAtual_Brain - tempoUltimoloop_Brain)).quot * (atualRPM - ultimaRPM); //This is the RPM per second that the engine has accelerated/decelleratedin the last loop
		} else {
			atualRPM = 0;
			contadorrevolucoes = 0;
			//bombaCombustivel = false;
			sincronizacao = false;
			atualRPS = 0;
		}

		if (sincronizacao && (atualRPM > 0)) {

			// Podemos configurar a ignição

		}

		//Serial.println(atualRPM);

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
int getCrankAngle(int tempoPorGrau) {
	unsigned long temp_tempoUltimoDente;
	int temp_contadorDentes;

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

	//Maxima prioridade ou desativar interrupcoes
	//taskDISABLE_INTERRUPTS();
	//tempoEntreDentes = (tempoUltimoDente - tempoPenultimoDente);
	tempoEntreDentes = (tempoUltimoDente - tempoPenultimoDente);
	uint64_t rpm_temporary = (((60000000 / (tempoEntreDentes * 36)) + 1) * 2); //+1 pq falha, n sei pk x2??
	//taskENABLE_INTERRUPTS();
	//Prioridade normal
	xQueueSendToFront(xRPM, &rpm_temporary, 0);

	//Serial.println(rpm_temporary);
}

/*void IRAM_ATTR coilDischargeTimer(void) {
 /*portENTER_CRITICAL_ISR(&coilTimerMux);

 descargaBobine = HIGH;
 digitalWrite(bobine1Pin, descargaBobine);

 portEXIT_CRITICAL_ISR(&coilTimerMux);*/
//}
void coilDischargeTimer_callback(void *arg) {

	xQueuePeek(xRPM, &atualRPM, (TickType_t) 10);

	Serial.print("Número total de dentes: ");
	Serial.println(numRealDentes);
	Serial.print("RPM: ");
	Serial.println(atualRPM);
}

// Interrupção gerada na transição do sinal do sensor de Hall
void IRAM_ATTR onPulse(void) {

	BaseType_t xHigherPriorityTaskWoken;

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
			contadorDentes = 1;
			tempoPrimeiroDenteMenosUm = tempoPrimeiroDente;
			tempoPrimeiroDente = tempoAtual;
			sincronizacao = true;
			contadorrevolucoes++;
			descargaBobine = LOW;
			digitalWrite(bobine1Pin, descargaBobine);
		} else {
			tempoFiltro = falhaAtual * 0.25; // 25% de filtro
			contadorDentes++;
			//Serial.print("Dente: ");
			//Serial.println(contadorDentes);
			xQueueSendToFrontFromISR(xContadorDentes, &contadorDentes, &xHigherPriorityTaskWoken);
		}

		if (contadorDentes == 5) {
			//Buscar o valor da ignição
			/*float valorignicao = 7;
			 float tempoPorGrau = (tempoUltimoDente / anguloPorDente);
			 float anguloDisparo = (desvioPrimeiroDenteTDC - valorignicao) - getCrankAngle(tempoPorGrau);
			 timerAlarm(coilTimer, (anguloDisparo * tempoPorGrau), false, 0);*/

		}

		tempoPenultimoDente = tempoUltimoDente;
		tempoUltimoDente = tempoAtual;
		xQueueSendToFrontFromISR(xtempoUltimoDente, &tempoUltimoDente, &xHigherPriorityTaskWoken);

		//Serial.println(tempoUltimoDente);
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

void vGetMAP(void *pvParameters) {
	while (true) {
		/*uint8_t temp_valorAnalogico;
		 uint8_t temp_ddpAnalogica;
		 uint8_t temp_MAPValue;

		 temp_valorAnalogico = analogRead(MAPSensorPin);
		 temp_ddpAnalogica = temp_valorAnalogico * (VREF_PLUS - VREF_MINUS) / (pow(2.0, (float) ADC_RESOLUTION)) + VREF_MINUS;

		 temp_MAPValue = 45.2042 * temp_ddpAnalogica;

		 xQueueSendToFront(xMAP, &temp_MAPValue, 0);

		 if ( DEBUG == "ON" && DEBUG_LEVEL <= 2) {
		 Serial.print("Correu a task: ");
		 Serial.println(pcTaskGetTaskName(NULL));
		 }*/
		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}

//------------------------------------------------------------------------------
void loop() {
	vTaskDelete( NULL);
}
