#include <stdio.h>
#include "Arduino.h"
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include "DHT.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

// Imagens em bitmap
#include "engauto.h" //logo automovel
#include "logo_temp_motor.h" //logo temp_motor
#include "logo_temp_pneu.h" //logo temp_pneu
#include "logo_temp_exterior.h" //logo temp_exterior
#include "logo_humidade.h" //logo logo_humidade
#include "logo_tps.h" //logo logo_tps
#include "logo_rpm.h" //logo logo_rpm

//Constantes Global
const uint8_t DEBUG = 1; // DEBUG
#define DHTTYPE DHT11
#define ADCres 12 //Definicao  de ADC

//Variaveis Globais
bool ESTADODISPLAY = false; // diaply on off

//Definir os pinos de input
#define TFT_DC 12
#define TFT_CS 13
#define TFT_MOSI 14
#define TFT_CLK 27
#define TFT_RST 0
#define TFT_MISO 0
#define DHTPIN 5
#define LM35PIN 12

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST);
DHT dht(DHTPIN, DHTTYPE);

// Definicao de estruturas
typedef struct {
  int humidade;
  float temp_exterior;
} temphumidade_para_display;


/* Definicoes de prototipos das tarefas */
void vIniciarDisplay( void * pvParameters);
void vApagarValoresDisplay( void * pvParameters);
void vEscreverValoresDisplay( void * pvParameters);
void vLerTempHumidade( void * pvParameters);
void vLerTempMotor (void * pvparameters);

/*Handler de tarefas*/
TaskHandle_t xResetValoresDisplayHandle;

//Queue para transferencia de temperatu/
QueueHandle_t temphumidadeQueue;
QueueHandle_t tempmotorQueue;

/* Protoripoa de semaforos para sincronizacao de interrupcoees*/
SemaphoreHandle_t limparValores;


// setup
void setup()
{
  //ADC
  analogReadResolution(ADCres);
  dht.begin();

  //Inputs
  pinMode(DHTPIN, INPUT_PULLUP);
  pinMode(LM35PIN, INPUT_PULLUP);

  // Criar queues
  temphumidadeQueue = xQueueCreate(3, sizeof(temphumidade_para_display));
  tempmotorQueue = xQueueCreate(3, sizeof(float));

  vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

  // Criar tarefas
  xTaskCreatePinnedToCore(vIniciarDisplay, "Iniciar e escrever imagens no display", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vApagarValoresDisplay, "Apagar os valores do dispplay", 1024, NULL, 2, &xResetValoresDisplayHandle, 0);
  xTaskCreatePinnedToCore(vLerTempHumidade, "Temperatura e Humidade", 1024, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(vLerTempMotor, "Temperatura Motor", 1024, NULL, 3, NULL, 0);

  Serial.begin(115200);
}

// Task para desenhar o logotipo de EAU
void vIniciarDisplay(void *pvParameters)
{
  temphumidade_para_display receber_dados_dht11;
  float temp_motor = 0, temp_peneu = 0, tps = 0, rpm = 0;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    if (ESTADODISPLAY == false)
    {
      tft.initR(INITR_BLACKTAB);
      tft.setRotation(3);
      tft.fillScreen(ST77XX_BLACK);
      if (DEBUG == 1) {
        Serial.println("Ligar Display");
      }

      //  LogoEAU
      int altura = 17, largura = 160;   // Definicao da dimensao da imagem
      int linha, coluna, inicio_x = 0; // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna, linha, pgm_read_word(engauto + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        }
      }

      // RPM
      altura = 20;
      largura = 38;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 110, linha + 80, pgm_read_word(logo_rpm + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }

      // TPS
      altura = 20;
      largura = 31;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 62, linha + 80, pgm_read_word(logo_tps + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }

      // Humidade
      altura = 20;
      largura = 15;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 19, linha + 80, pgm_read_word(logo_humidade + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }

      // Temp. Exterior
      altura = 20;
      largura = 16;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 120, linha + 25, pgm_read_word(logo_temp_exterior + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }

      // Temp. Pneu
      altura = 20;
      largura = 16;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 70, linha + 25, pgm_read_word(logo_temp_pneu + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }

      // Temp. Motor
      altura = 20;    // Definicao da dimensao da imagem
      largura = 23;   // Definicao da dimensao da imagem
      inicio_x = 0;   // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 15, linha + 25, pgm_read_word(logo_temp_motor + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }

      ESTADODISPLAY = true;
      Serial.println(ESTADODISPLAY);
      Serial.println("  Tarefa: Iniciar e escrever imagens no display");
      vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));

    } else {

      xQueueReceive(temphumidadeQueue, &receber_dados_dht11, 0);
      xQueueReceive(tempmotorQueue, &temp_motor, 0);
      //Serial.println(temp_motor);
      //Serial.println(receber_dados_dht11.temp_exterior);
      //Serial.println(receber_dados_dht11.humidade);

      Serial.println("Dados escrever display: ");
      Serial.println(temp_motor);
      Serial.println(temp_peneu);
      Serial.println(receber_dados_dht11.temp_exterior);
      Serial.println(receber_dados_dht11.humidade);
      Serial.println(tps);
      Serial.println(rpm);


      tft.setCursor(10, 50);
      tft.setTextSize(1);
      tft.setTextColor(ST7735_WHITE);
      tft.print(temp_motor);
      tft.print((char)167);
      tft.print("C");
      tft.setCursor(60, 50);
      tft.print(temp_peneu);
      tft.print((char)167);
      tft.print("C");
      tft.setCursor(110, 50);
      tft.print(receber_dados_dht11.temp_exterior);
      tft.print((char)167);
      tft.print("C");
      tft.setCursor(16, 110);
      tft.print(receber_dados_dht11.humidade);
      tft.print("%");
      tft.setCursor(68, 110);
      tft.print(tps);
      tft.print("%");
      tft.setCursor(118, 110);
      tft.print(rpm);
    }

    Serial.println("  Tarefa: Update valores");
    vTaskDelayUntil(&xLastWakeTime, (5000 / portTICK_PERIOD_MS));
  }
}

// task para apagar valores no display
void vApagarValoresDisplay(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    tft.fillRect(10, 50, 42, 7, ST7735_BLACK);
    tft.fillRect(60, 50, 42, 7, ST7735_BLACK);
    tft.fillRect(110, 50, 42, 7, ST7735_BLACK);
    tft.fillRect(16, 110, 18, 7, ST7735_BLACK);
    tft.fillRect(68, 110, 18, 7, ST7735_BLACK);
    tft.fillRect(118, 110, 18, 7, ST7735_BLACK);

    Serial.println("  Tarefa: vApagarValoresDisplay");
    vTaskDelayUntil(&xLastWakeTime, (2000 / portTICK_PERIOD_MS));
  }
}

// task para ler o sensor dht11
void vLerTempHumidade (void*pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  temphumidade_para_display enviar_dados_dht11;

  for (;;)
  {
    enviar_dados_dht11.humidade = dht.readHumidity();
    enviar_dados_dht11.temp_exterior = dht.readTemperature();

    if (isnan(enviar_dados_dht11.humidade) || isnan(enviar_dados_dht11.temp_exterior))
    {
      Serial.println("Erro: Nao foi possivel ler o sensor DHT11!");
    } else {
      //xQueueSendToBack(temphumidadeQueue, &enviar_dados_dht11, 0);
      xQueueSend(temphumidadeQueue, (void *) &enviar_dados_dht11, 0);
      if (DEBUG == 1) {
        Serial.print("Humidity: ");
        Serial.print(enviar_dados_dht11.humidade);
        Serial.print("%  Temperature: ");
        Serial.print(enviar_dados_dht11.temp_exterior);
        Serial.println("°C ");
      }
    }

    Serial.println("  Tarefa: vLerTempHumidade");
    vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
  }
}

void vLerTempMotor (void * pvparameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float tempmotor = 0, valorBruto = 0, ddP = 0;
  for (;;)
  {
    //sensors.requestTemperatures();
    //float tempmotor = sensors.getTempCByIndex(0);
    valorBruto = analogRead(LM35PIN);
    //Serial.println(valorBruto);
    ddP = (valorBruto / 2048.0) * 3300; // Convesão para milivolts (ddp)
    //Serial.println(ddP);
    tempmotor = ddP * 0.1;
    xQueueSendToBack(tempmotorQueue, &tempmotor, 0);

    if (DEBUG == 1)
    {
      Serial.print("Temp Motor: ");
      Serial.print(tempmotor);
      Serial.println("ºC");
    }

    Serial.println("  Tarefa: vLerTempMotor");
    vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
  }
}

// loop
void loop()
{
  vTaskDelete(NULL);  //matar a tarefa arduino
}
