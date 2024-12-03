#include <TFT_eSPI.h>
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

// Tarefas e funções
void calculateRPM(void *pvParameters);
void IRAM_ATTR onPulse(void);

// Declaração do semáforo
SemaphoreHandle_t xBinarySemaphore;

// Configuração de pino e variáveis de cálculo
const uint8_t interruptPin = 4;
const uint8_t teethNum = 35;
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
unsigned int rpm = 0;
unsigned int maxrpm = 0;

// Configuração para o medidor de ponteiro
const int minRPM = 0;
const int maxRPM = 8000;  // Máximo do medidor de RPM
const int centerX = 150;  // Centro do medidor (ajuste conforme seu display)
const int centerY = 110;
const int radius = 90;

void setup() {
  // Define a prioridade máxima para a tarefa loop antes de deletá-la
  vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

  // Inicializa o serial
  Serial.begin(115200);

  // Criação do semáforo binário
  vSemaphoreCreateBinary(xBinarySemaphore);

  // Configurações do pino de interrupção
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onPulse, FALLING);

  // Configuração do display TFT
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  // Verifica se o semáforo foi criado com sucesso
  if (xBinarySemaphore != NULL) {
    xTaskCreatePinnedToCore(calculateRPM, "Calculate RPM", 1024, NULL, 3, NULL, 1);
  }
}

// Tarefa para calcular o RPM
void calculateRPM(void *pvParameters) {
  while (true) {
    // Calcula o RPM a cada 200 ms
    unsigned long currentTime = millis();
    unsigned long interval = currentTime - lastTime;

    if (interval >= 200) {  // Calcula a cada 200 ms (0,2 segundo)
      rpm = ((pulseCount * 60 * 1000) / teethNum) / interval; // Cálculo do RPM
      pulseCount = 0; // Reseta o contador de pulsos
      lastTime = currentTime;

      // Atualiza o valor máximo de RPM se necessário
      if (maxrpm < rpm) {
        maxrpm = rpm;
      }

      // Exibe no monitor serial
      Serial.print("RPM: ");
      Serial.println(rpm);

      Serial.print("Max RPM: ");
      Serial.println(maxrpm);

      // Atualiza o display com o valor de RPM
      displayRPM(rpm);

      vTaskDelay(100 / portTICK_PERIOD_MS);  // Aguarda antes de atualizar novamente
    }
  }
}

void displayRPM(int rpmValue) {
  tft.fillScreen(TFT_BLACK);  // Limpa a tela

  // Desenha o círculo do medidor
  tft.drawCircle(centerX, centerY, radius, TFT_WHITE);
  tft.drawCircle(centerX, centerY, radius + 5, TFT_WHITE);  // Borda extra para o medidor

  // Calcula o ângulo do ponteiro baseado no valor de RPM
  float angle = map(rpmValue, minRPM, maxRPM, -150, 150);
  float radianAngle = angle * 0.0174533;  // Converte para radianos

  // Coordenadas para a ponta do ponteiro
  int x = centerX + radius * cos(radianAngle);
  int y = centerY - radius * sin(radianAngle);

  // Desenha o ponteiro
  tft.drawLine(centerX, centerY, x, y, TFT_RED);

  // Exibe o título "RPM" no meio da tela (acima do ponteiro)
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);  // Ajuste o tamanho do texto conforme necessário
  tft.setCursor(centerX - 30, centerY - 50);  // Ajuste a posição conforme necessário
  tft.println("RPM");

  // Exibe o valor atual de RPM abaixo do título
  tft.setTextSize(3);  // Tamanho do texto maior para o valor do RPM
  tft.setCursor(centerX - 50, centerY + 20);  // Ajuste a posição conforme necessário
  tft.printf("%d", rpmValue);  // Mostra o valor de RPM
}


// Interrupção para contar pulsos
void IRAM_ATTR onPulse(void) {
  pulseCount++; // Incrementa o contador de pulsos a cada pulso detectado
}

void loop() {
  // Deleta a tarefa loop padrão para usar o FreeRTOS
  vTaskDelete(NULL);
}
