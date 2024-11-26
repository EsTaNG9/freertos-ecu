#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <TFT_eSPI.h>  // Biblioteca para o display TFT

TFT_eSPI tft = TFT_eSPI();  // Instância do display TFT

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
const int centerX = 120;  // Centro do medidor (ajuste conforme seu display)
const int centerY = 160;
const int radius = 100;

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
  tft.init();
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

      vTaskDelay(200 / portTICK_PERIOD_MS);  // Aguarda antes de atualizar novamente
    }
  }
}

// Função para desenhar o medidor de RPM no display TFT
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

  // Exibe o valor de RPM em texto
  tft.setCursor(60, 250);
  tft.printf("RPM: %d", rpmValue);
}

// Interrupção para contar pulsos
void IRAM_ATTR onPulse(void) {
  pulseCount++; // Incrementa o contador de pulsos a cada pulso detectado
}

void loop() {
  // Deleta a tarefa loop padrão para usar o FreeRTOS
  vTaskDelete(NULL);
}
