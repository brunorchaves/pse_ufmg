#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/timer.h"
#include <inttypes.h>

// ---------- DEFINICOES DE PINOS E CONSTANTES ----------
#define BUTTON_PIN    10       // Pino do botao (GPIO10)
#define LED_PIN       20       // Pino do LED (GPIO20)
#define DEBOUNCE_MS    100        // Tempo de debounce para o botão (em ms)
#define TIMER_DIVIDER  80         // Divisor do clock (80 MHz / 80 = 1 MHz = 1 us)

// ---------- VARIAVEIS GLOBAIS ----------
volatile uint64_t button_press_time = 0;
volatile uint64_t button_release_time = 0;
volatile bool button_pressed = false;
volatile bool button_released = false;
volatile bool timer_timeout = false;

// ---------- ESTADOS DA MAQUINA DE ESTADOS ----------
typedef enum {
    STATE_INIT = 0,
    STATE_WAIT_BUTTON_PRESS,
    STATE_DEBOUNCE_PRESS,
    STATE_WAIT_BUTTON_RELEASE,
    STATE_DEBOUNCE_RELEASE,
    STATE_CALCULATE_BLINKS,
    STATE_BLINK_SEQUENCE,
    STATE_FINAL_BLINKS,
    STATE_CYCLE_COMPLETE
} system_state_t;

// ---------- PROTOTIPOS DE FUNCAO ----------
void init_serial(void);
void init_gpio(void);
void init_timer(void);
void IRAM_ATTR timer_isr(void *arg);
uint32_t get_pressed_duration(void);
void blink_led(uint32_t times, uint32_t duration);

// ---------- INICIALIZACAO DA UART ----------
void init_serial() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
}

// ---------- INICIALIZACAO DOS GPIOs ----------
void init_gpio() {
    gpio_config_t io_conf = {0};

    // LED como saída
    io_conf.pin_bit_mask = (1ULL << LED_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);

    // Botão como entrada com pull-up
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

// ---------- INICIALIZACAO DO TIMER ----------
void init_timer() {
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS, // Não usamos alarme, apenas contagem
        .auto_reload = TIMER_AUTORELOAD_DIS,
        .intr_type = TIMER_INTR_LEVEL
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
}

// ---------- FUNCAO PARA OBTER DURACAO DO PRESSIONAMENTO ----------
uint32_t get_pressed_duration() {
    return (uint32_t)((button_release_time - button_press_time) / 1000000); // Retorna em segundos
}

// ---------- FUNCAO PARA PISCAR O LED ----------
void blink_led(uint32_t times, uint32_t duration_ms) {
    for (uint32_t i = 0; i < times; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(duration_ms / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(duration_ms / portTICK_PERIOD_MS);
    }
}

// ---------- FUNCAO PRINCIPAL ----------
void app_main() {
    init_serial();
    init_gpio();
    init_timer();
    ESP_LOGI("MAIN", "Sistema iniciado. Aguardando botao...");

    system_state_t state = STATE_INIT;
    uint32_t debounce_start = 0;
    uint32_t press_duration = 0;
    uint32_t blink_count = 0;

    while (1) {
        switch (state) {
            case STATE_INIT:
                gpio_set_level(LED_PIN, 0);
                if (gpio_get_level(BUTTON_PIN) == 1) {
                    state = STATE_WAIT_BUTTON_PRESS;
                }
                break;

            case STATE_WAIT_BUTTON_PRESS:
                if (gpio_get_level(BUTTON_PIN) == 0) {
                    debounce_start = xTaskGetTickCount();
                    state = STATE_DEBOUNCE_PRESS;
                    ESP_LOGI("FSM", "Botao pressionado, verificando debounce...");
                }
                break;

            case STATE_DEBOUNCE_PRESS:
                if ((xTaskGetTickCount() - debounce_start) * portTICK_PERIOD_MS > DEBOUNCE_MS) {
                    if (gpio_get_level(BUTTON_PIN) == 0) {
                        // Inicia temporizador para medir duração do pressionamento
                        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
                        timer_start(TIMER_GROUP_0, TIMER_0);
                        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &button_press_time);
                        state = STATE_WAIT_BUTTON_RELEASE;
                        ESP_LOGI("FSM", "Pressionamento confirmado. Medindo tempo...");
                    } else {
                        state = STATE_WAIT_BUTTON_PRESS;
                        ESP_LOGI("FSM", "Falso acionamento.");
                    }
                }
                break;

            case STATE_WAIT_BUTTON_RELEASE:
                if (gpio_get_level(BUTTON_PIN) == 1) {
                    debounce_start = xTaskGetTickCount();
                    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &button_release_time);
                    timer_pause(TIMER_GROUP_0, TIMER_0);
                    state = STATE_DEBOUNCE_RELEASE;
                    ESP_LOGI("FSM", "Botao solto, verificando debounce...");
                }
                break;

            case STATE_DEBOUNCE_RELEASE:
                if ((xTaskGetTickCount() - debounce_start) * portTICK_PERIOD_MS > DEBOUNCE_MS) {
                    if (gpio_get_level(BUTTON_PIN) == 1) {
                        press_duration = get_pressed_duration();
                        ESP_LOGI("FSM", "Tempo de pressionamento: %" PRIu32 " segundos", press_duration);
                        state = STATE_CALCULATE_BLINKS;
                    } else {
                        state = STATE_WAIT_BUTTON_RELEASE;
                        ESP_LOGI("FSM", "Falsa soltura.");
                    }
                }
                break;

            case STATE_CALCULATE_BLINKS:
                if (press_duration < 1) {
                    blink_count = 1;
                } else if (press_duration < 2) {
                    blink_count = 2;
                } else {
                    blink_count = 3;
                }
                state = STATE_BLINK_SEQUENCE;
                break;

            case STATE_BLINK_SEQUENCE:
                blink_led(blink_count, 500); // Pisca LED por 500ms
                state = STATE_FINAL_BLINKS;
                break;

            case STATE_FINAL_BLINKS:
                blink_led(2, 250); // Pisca rápido 2 vezes (250ms)
                state = STATE_CYCLE_COMPLETE;
                break;

            case STATE_CYCLE_COMPLETE:
                state = STATE_INIT;
                break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}