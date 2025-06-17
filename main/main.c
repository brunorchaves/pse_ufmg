/******************************************************************************
 * Laboratório 2 – Pisca LED com Temporizador e Duas ISRs
 * Placa-alvo : Seeed XIAO ESP32-C3
 *
 * Botão : D6 → GPIO6  (ativo-LOW, pull-up interno)
 * LED   : GPIO7 (LED azul on-board) — mude LED_PIN p/ 10 se usar LED externo.
 *
 * Regras
 * ──────
 * • “Press” só é aceito se LOW durar ≥ 100 ms  (DEBOUNCE_MS).
 * • “Release” idem para HIGH.
 * • Durante as piscadas o botão é ignorado (interrupção desabilitada).
 * • Logs via printf mostram cada transição.
 ******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include <stdio.h>
#include <inttypes.h>

/* -------------------- Pinos / constantes -------------------- */
#define BUTTON_PIN   10          /* D6 */
#define LED_PIN      20          /* LED on-board azul — troque p/ 10 se externo */
#define DEBOUNCE_MS  100        /* 100 ms */

static gptimer_handle_t gptimer;
static volatile uint32_t ms_counter = 0;  /* tick de 1 ms */

/* Flags de borda geradas pela ISR do botão */
static volatile bool press_irq   = false;
static volatile bool release_irq = false;

/* -------------------- Máquina de estados -------------------- */
typedef enum {
    STATE_INIT = 0,
    STATE_WAIT_BUTTON_PRESS,
    STATE_DEBOUNCE_PRESS,
    STATE_WAIT_BUTTON_RELEASE,
    STATE_DEBOUNCE_RELEASE,
    STATE_CALC_BLINKS,
    STATE_BLINK,
    STATE_END_BLINKS
} fsm_state_t;

/* -------------------- Protótipos -------------------- */
static void init_serial(void);
static void init_gpio(void);
static void init_timer(void);
static bool  IRAM_ATTR timer_isr(gptimer_handle_t,
                                 const gptimer_alarm_event_data_t*, void*);
static void  IRAM_ATTR button_isr(void*);
static inline uint32_t get_time_ms(void);
static void blink_led(uint32_t times, uint32_t on_ms);

/* -------------------- Inicialização -------------------- */
static void init_serial(void)
{
    const uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &cfg);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);

    setvbuf(stdout, NULL, _IONBF, 0);   /* printf sem buffer */
}

static void init_gpio(void)
{
    /* LED */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    /* Botão */
    io.pin_bit_mask = 1ULL << BUTTON_PIN;
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.intr_type    = GPIO_INTR_ANYEDGE;
    gpio_config(&io);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);
}

static bool IRAM_ATTR timer_isr(gptimer_handle_t t,
                                const gptimer_alarm_event_data_t *e,
                                void *arg)
{
    ms_counter++;               /* 1 ms */
    return true;                /* auto-reload */
}

static void init_timer(void)
{
    const gptimer_config_t cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000   /* 1 MHz → 1 µs */
    };
    gptimer_new_timer(&cfg, &gptimer);

    const gptimer_event_callbacks_t cbs = { .on_alarm = timer_isr };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    const gptimer_alarm_config_t alarm_cfg = {
        .alarm_count  = 1000,             /* 1000 µs = 1 ms */
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };
    gptimer_set_alarm_action(gptimer, &alarm_cfg);
    gptimer_enable(gptimer);
    gptimer_start(gptimer);
}

/* -------------------- ISR do botão -------------------- */
static void IRAM_ATTR button_isr(void *arg)
{
    const int level = gpio_get_level(BUTTON_PIN);  /* 0 = LOW */

    if (level == 0)
        press_irq = true;         /* borda de descida */
    else
        release_irq = true;       /* borda de subida  */
}

/* -------------------- Helpers -------------------- */
static inline uint32_t get_time_ms(void) { return ms_counter; }

static void blink_led(uint32_t times, uint32_t on_ms)
{
    for (uint32_t i = 0; i < times; ++i) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        taskYIELD();              /* evita reset de watchdog */
    }
}

/* -------------------- app_main -------------------- */
void app_main(void)
{
    init_serial();
    init_gpio();
    init_timer();

    printf("Sistema iniciado.\n");

    fsm_state_t state = STATE_INIT;
    uint32_t timer_start = 0;     /* genérico p/ medir intervalos */
    uint32_t t_press_ms  = 0;
    uint32_t t_release_ms = 0;
    uint32_t dt_ms       = 0;
    uint32_t blks        = 0;

    while (true) {

        switch (state) {

        /* ---------- INIT ---------- */
        case STATE_INIT:
            gpio_set_level(LED_PIN, 0);
            if (gpio_get_level(BUTTON_PIN) == 1)   /* botão solto */
                state = STATE_WAIT_BUTTON_PRESS;
            break;

        /* ---------- Wait PRESS ---------- */
        case STATE_WAIT_BUTTON_PRESS:
            if (press_irq) {
                press_irq   = false;
                timer_start = get_time_ms();
                state       = STATE_DEBOUNCE_PRESS;
                printf("FSM: Botão pressionado — verificando debounce...\n");
            }
            break;

        /* ---------- Debounce PRESS ---------- */
        case STATE_DEBOUNCE_PRESS:
            if (get_time_ms() - timer_start >= DEBOUNCE_MS) {
                if (gpio_get_level(BUTTON_PIN) == 0) {
                    t_press_ms = timer_start;          /* início oficial */
                    printf("FSM: Pressionamento confirmado @ %" PRIu32 " ms\n",
                           t_press_ms);
                    state = STATE_WAIT_BUTTON_RELEASE;
                } else {
                    printf("FSM: Falso acionamento.\n");
                    state = STATE_WAIT_BUTTON_PRESS;
                }
            }
            break;

        /* ---------- Wait RELEASE ---------- */
        case STATE_WAIT_BUTTON_RELEASE:
            if (release_irq) {
                release_irq = false;
                timer_start = get_time_ms();
                state       = STATE_DEBOUNCE_RELEASE;
                printf("FSM: Botão solto — verificando debounce...\n");
            }
            break;

        /* ---------- Debounce RELEASE ---------- */
        case STATE_DEBOUNCE_RELEASE:
            if (get_time_ms() - timer_start >= DEBOUNCE_MS) {
                if (gpio_get_level(BUTTON_PIN) == 1) {
                    t_release_ms = timer_start;
                    dt_ms        = t_release_ms - t_press_ms;
                    printf("FSM: Soltura confirmada @ %" PRIu32
                           " ms (T = %" PRIu32 " ms)\n",
                           t_release_ms, dt_ms);
                    gpio_intr_disable(BUTTON_PIN);   /* trava eventos */
                    state = STATE_CALC_BLINKS;
                } else {
                    printf("FSM: Falsa soltura.\n");
                    state = STATE_WAIT_BUTTON_RELEASE;
                }
            }
            break;

        /* ---------- Converte T → piscadas ---------- */
        case STATE_CALC_BLINKS:
            if      (dt_ms < 1000) blks = 1;
            else if (dt_ms < 2000) blks = 2;
            else                   blks = 3;

            printf("FSM: T = %ld ms\n", dt_ms);
            state = STATE_BLINK;
            break;

        /* ---------- Piscadas principais ---------- */
        case STATE_BLINK:
            blink_led(blks, 500);        /* 0,5 s ON / 0,5 s OFF */
            state = STATE_END_BLINKS;
            break;

        /* ---------- Piscadas finais ---------- */
        case STATE_END_BLINKS:
            blink_led(2, 250);
            printf("FSM: Ciclo completo.\n-----------------------------\n");

            /* Limpa variáveis e libera botão */
            press_irq = release_irq = false;
            gpio_intr_enable(BUTTON_PIN);
            state = STATE_WAIT_BUTTON_PRESS;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));   /* passo da MEF */
    }
}
