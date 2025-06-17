/* 
 * Laboratorio 0 - Pisca LED
 * Disciplina: Projeto de Sistemas Embarcados
 *
 * Objetivo: Controlar um LED usando um botao no ESP32-C3.
 * O LED executa a seguinte sequencia toda vez que o botao e pressionado e solto:
 * 1. Acende por 1 segundo
 * 2. Apaga por 1 segundo
 * 3. Acende por 0,25 segundos
 * 4. Volta ao estado inicial
 */

 #include "freertos/FreeRTOS.h"   // Sistema operacional em tempo real
 #include "freertos/task.h"       // Funcoes de tarefas e delays
 #include "driver/gpio.h"         // Controle dos pinos digitais
 #include "esp_log.h"             // Mensagens de log no terminal
 #include "driver/uart.h"         // Comunicacao serial via UART
 
 // ---------- DEFINICOES DE PINOS E CONSTANTES ----------
 
 #define BUTTON_PIN    10       // Pino do botao (GPIO10)
 #define LED_PIN       20       // Pino do LED (GPIO20)
 #define DEBOUNCE_MS   100      // Tempo de debounce para o botao (em ms)
 
 #define UART_PORT     UART_NUM_0 // UART padrao para logs
 #define BUF_SIZE      1024       // Tamanho do buffer da UART
 
 // ---------- ESTADOS DA MAQUINA DE ESTADOS ----------
 
 typedef enum {
     STATE_INIT = 0,              // Garante que o botao esta solto ao iniciar
     STATE_WAIT_BUTTON_PRESS,     // Espera o botao ser pressionado
     STATE_DEBOUNCE_PRESS,        // Espera debounce apos pressao
     STATE_WAIT_BUTTON_RELEASE,   // Espera o botao ser solto
     STATE_DEBOUNCE_RELEASE,      // Espera debounce apos soltura
     STATE_LED_ON,                // Liga o LED por 1s
     STATE_LED_OFF,               // Apaga o LED por 1s
     STATE_LED_ON_SHORT,          // Liga o LED por 0,25s
     STATE_BLINK_COMPLETE         // Fim da sequencia
 } system_state_t;
 
 // ---------- FUNCAO DE INICIALIZACAO DA UART ----------
 
 void init_serial() {
     uart_config_t uart_config = {
         .baud_rate = 115200,
         .data_bits = UART_DATA_8_BITS,
         .parity    = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
     };
     uart_param_config(UART_PORT, &uart_config);
     uart_driver_install(UART_PORT, BUF_SIZE, 0, 0, NULL, 0);
 }
 
 // ---------- FUNCAO PARA OBTER TEMPO EM MILISSEGUNDOS ----------
 
 uint32_t get_time_ms() {
     return pdTICKS_TO_MS(xTaskGetTickCount());
 }
 
 // ---------- FUNCAO PRINCIPAL ----------
 
 void app_main() {
     init_serial(); // Inicializa UART para log
     ESP_LOGI("MAIN", "Sistema iniciado. Aguardando botao...");
 
     // ---------- CONFIGURACAO DOS GPIOs ----------
 
     gpio_config_t io_conf = {0}; // Estrutura de configuracao zerada
 
     // LED como saida
     io_conf.pin_bit_mask = (1ULL << LED_PIN);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     gpio_config(&io_conf);
     gpio_set_level(LED_PIN, 0); // LED apagado

     // Botao como entrada com pull-up
     io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
     io_conf.mode = GPIO_MODE_INPUT;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
     gpio_config(&io_conf);

     // ---------- VARIAVEIS DE CONTROLE ----------
 
     system_state_t state = STATE_INIT;
     uint32_t timer_start = 0;
 
     // ---------- LOOP PRINCIPAL ----------
 
     while (1) {
         switch (state) {
             case STATE_INIT:
                 gpio_set_level(LED_PIN, 0); // LED apagado
                 if (gpio_get_level(BUTTON_PIN) == 1) {
                     state = STATE_WAIT_BUTTON_PRESS;
                 }
                 break;
 
             case STATE_WAIT_BUTTON_PRESS:
                 if (gpio_get_level(BUTTON_PIN) == 0) {
                     timer_start = get_time_ms();
                     state = STATE_DEBOUNCE_PRESS;
                     ESP_LOGI("FSM", "Botao pressionado, verificando debounce...");
                 }
                 break;
 
             case STATE_DEBOUNCE_PRESS:
                 if (get_time_ms() - timer_start > DEBOUNCE_MS) {
                     if (gpio_get_level(BUTTON_PIN) == 0) {
                         state = STATE_WAIT_BUTTON_RELEASE;
                         ESP_LOGI("FSM", "Pressionamento confirmado.");
                     } else {
                         state = STATE_WAIT_BUTTON_PRESS;
                         ESP_LOGI("FSM", "Falso acionamento.");
                     }
                 }
                 break;
 
             case STATE_WAIT_BUTTON_RELEASE:
                 if (gpio_get_level(BUTTON_PIN) == 1) {
                     timer_start = get_time_ms();
                     state = STATE_DEBOUNCE_RELEASE;
                     ESP_LOGI("FSM", "Botao solto, verificando debounce...");
                 }
                 break;
 
             case STATE_DEBOUNCE_RELEASE:
                 if (get_time_ms() - timer_start > DEBOUNCE_MS) {
                     if (gpio_get_level(BUTTON_PIN) == 1) {
                         state = STATE_LED_ON;
                         timer_start = get_time_ms();
                         ESP_LOGI("FSM", "Soltura confirmada. Acendendo LED...");
                     } else {
                         state = STATE_WAIT_BUTTON_RELEASE;
                         ESP_LOGI("FSM", "Falsa soltura.");
                     }
                 }
                 break;
 
             case STATE_LED_ON:
                 gpio_set_level(LED_PIN, 1); // Liga o LED
                 if (get_time_ms() - timer_start > 1000) {
                     state = STATE_LED_OFF;
                     timer_start = get_time_ms();
                     ESP_LOGI("FSM", "LED ligado por 1s. Apagando...");
                 }
                 break;
 
             case STATE_LED_OFF:
                 gpio_set_level(LED_PIN, 0); // Apaga o LED
                 if (get_time_ms() - timer_start > 1000) {
                     state = STATE_LED_ON_SHORT;
                     timer_start = get_time_ms();
                     ESP_LOGI("FSM", "LED apagado por 1s. Ligando por 0,25s...");
                 }
                 break;
 
             case STATE_LED_ON_SHORT:
                 gpio_set_level(LED_PIN, 1); // Liga LED por 0,25s
                 if (get_time_ms() - timer_start > 250) {
                     gpio_set_level(LED_PIN, 0); // Apaga LED
                     state = STATE_BLINK_COMPLETE;
                     ESP_LOGI("FSM", "Fim da sequencia.");
                 }
                 break;
 
             case STATE_BLINK_COMPLETE:
                 state = STATE_INIT; // Reinicia a maquina
                 break;
         }
 
         vTaskDelay(10 / portTICK_PERIOD_MS); // Espera 10ms entre iteracoes
     }
 }
 