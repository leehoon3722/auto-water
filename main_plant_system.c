/* ========================================================================
 *  STM32F401 (Nucleo-F401RE)  Automatic Plant Watering System  - full code
 *  Replace the WHOLE generated Core/Src/main.c with this file.
 *
 *  Wiring (Nucleo label -> part)
 *    A0  (PA0)  : soil moisture sensor AOUT        (sensor VCC = 3V3)
 *    D7  (PA8)  : DHT22 / DHT11 DATA               (sensor VCC = 3V3)
 *    A3  (PB0)  : relay module IN                  (relay  VCC = 5V)
 *    D15 (PB8)  : OLED D0 (SCL)                    (OLED   VCC = 3V3)
 *    D14 (PB9)  : OLED D1 (SDA)
 *    OLED RES -> 3V3, OLED DC -> GND, OLED CS -> GND
 *    Pump : external 5V + -> relay COM, relay NO -> pump +, pump - -> external 5V -
 *
 *  CubeMX peripherals needed: ADC1 (IN0), TIM3 (Internal Clock), USART2 (Async)
 *  The OLED uses software I2C on PB8/PB9, so I2C1 in CubeMX is NOT needed
 *  (if it is already enabled, that is fine, it is simply not used).
 *
 *  OLED screen
 *      Soil : 68%
 *      Temp : 24.5C
 *      Humi : 52%           [water drop icon while the pump is ON]
 *      Pump : OFF
 *
 *  Green LED LD2 blinks once per second = program is running.
 *  Serial log: ST-Link virtual COM port, 115200 8N1.
 * ======================================================================== */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================ USER SETTINGS ============================ */
#define DEMO_MODE           0        /* 1: fixed sample values 68% / 24.5C / 52%, no sensors */
#define DEMO_PUMP_ON        0        /* DEMO_MODE only: 1 = show "Pump : ON" + water drop    */

#define DHT_TYPE            22       /* 22 = DHT22,  11 = DHT11                               */

#define SOIL_ADC_DRY        3000     /* ADC raw value with the sensor in dry air              */
#define SOIL_ADC_WET        1300     /* ADC raw value with the sensor in water                */
#define SOIL_ON_PCT         30       /* pump ON  when soil moisture <  30 %                   */
#define SOIL_OFF_PCT        45       /* pump OFF when soil moisture >= 45 %                   */

#define PUMP_MAX_ON_MS      5000UL   /* safety: longest continuous pump run                   */
#define PUMP_SOAK_MS        30000UL  /* wait after watering before the next check             */

#define RELAY_ACTIVE_LOW    1        /* 1: relay turns ON with LOW input (most modules)       */

#define DROP_ALWAYS         0        /* 1: always show the water drop icon                    */

#define I2C_DELAY_US        5        /* OLED bus speed. 5 = about 80 kHz.
                                        If the screen shows noise, try 10 or 20 (slower)     */

#define LOOP_PERIOD_MS      1000UL   /* sensor read + screen refresh period                   */
#define DHT_PERIOD_MS       2000UL   /* DHT22 needs at least 2 s between reads                */
/* ====================================================================== */

/* ---- pins ---- */
#define DHT_PORT        GPIOA
#define DHT_PIN         GPIO_PIN_8
#define RELAY_PORT      GPIOB
#define RELAY_PIN       GPIO_PIN_0
#define OLED_PORT       GPIOB
#define OLED_SCL_PIN    GPIO_PIN_8
#define OLED_SDA_PIN    GPIO_PIN_9
#define LED_PORT        GPIOA
#define LED_PIN         GPIO_PIN_5      /* LD2 green */

/* ---- peripheral handles ---- */
ADC_HandleTypeDef  hadc1;
TIM_HandleTypeDef  htim3;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* ======================================================================
 *  printf -> USART2
 * ====================================================================== */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}

/* ======================================================================
 *  microsecond delay (TIM3 free running @ 1 MHz)
 * ====================================================================== */
static void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    while (__HAL_TIM_GET_COUNTER(&htim3) < us) { }
}

/* ======================================================================
 *  Software I2C on PB8 (SCL) / PB9 (SDA), open-drain + pull-up
 *  every byte is checked for ACK, so a bad transfer is detected and resent
 * ====================================================================== */
#define SCL_H()     (OLED_PORT->BSRR = OLED_SCL_PIN)
#define SCL_L()     (OLED_PORT->BSRR = (uint32_t)OLED_SCL_PIN << 16)
#define SDA_H()     (OLED_PORT->BSRR = OLED_SDA_PIN)
#define SDA_L()     (OLED_PORT->BSRR = (uint32_t)OLED_SDA_PIN << 16)
#define SDA_IN()    ((OLED_PORT->IDR & OLED_SDA_PIN) != 0U)
#define I2C_WAIT()  delay_us(I2C_DELAY_US)

static void si2c_start(void)
{
    SDA_H(); SCL_H(); I2C_WAIT();
    SDA_L(); I2C_WAIT();
    SCL_L(); I2C_WAIT();
}

static void si2c_stop(void)
{
    SDA_L(); I2C_WAIT();
    SCL_H(); I2C_WAIT();
    SDA_H(); I2C_WAIT();
}

/* returns 1 if the device answered with ACK */
static uint8_t si2c_write(uint8_t b)
{
    for (int i = 0; i < 8; i++) {
        if (b & 0x80) SDA_H(); else SDA_L();
        b <<= 1;
        I2C_WAIT();
        SCL_H(); I2C_WAIT();
        SCL_L();
    }
    SDA_H(); I2C_WAIT();            /* release SDA for ACK */
    SCL_H(); I2C_WAIT();
    uint8_t ack = SDA_IN() ? 0 : 1;
    SCL_L(); I2C_WAIT();
    return ack;
}

/* unlock a stuck bus: 9 clocks, then STOP */
static void si2c_recover(void)
{
    SDA_H();
    for (int i = 0; i < 9; i++) {
        SCL_L(); I2C_WAIT();
        SCL_H(); I2C_WAIT();
    }
    SCL_L(); I2C_WAIT();
    si2c_stop();
}

/* ======================================================================
 *  SSD1306 128x64 OLED driver + 5x7 font
 * ====================================================================== */
#define OLED_W      128
#define OLED_H      64
#define CHUNK       16                  /* bytes per data packet */

static uint8_t  oled_addr     = 0;      /* 7-bit address, 0 = not found */
static uint8_t  oled_ready    = 0;
static uint16_t oled_err      = 0;      /* errors during the current frame */
static uint8_t  oled_buf[OLED_W * OLED_H / 8];

/* ASCII 32..126, 5 columns per glyph, bit0 = top row */
static const uint8_t font5x7[] = {
    0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00, 0x14,0x7F,0x14,0x7F,0x14,
    0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62, 0x36,0x49,0x56,0x20,0x50, 0x00,0x08,0x07,0x03,0x00,
    0x00,0x1C,0x22,0x41,0x00, 0x00,0x41,0x22,0x1C,0x00, 0x2A,0x1C,0x7F,0x1C,0x2A, 0x08,0x08,0x3E,0x08,0x08,
    0x00,0x80,0x70,0x30,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x00,0x60,0x60,0x00, 0x20,0x10,0x08,0x04,0x02,
    0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00, 0x72,0x49,0x49,0x49,0x46, 0x21,0x41,0x49,0x4D,0x33,
    0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x31, 0x41,0x21,0x11,0x09,0x07,
    0x36,0x49,0x49,0x49,0x36, 0x46,0x49,0x49,0x29,0x1E, 0x00,0x00,0x14,0x00,0x00, 0x00,0x40,0x34,0x00,0x00,
    0x00,0x08,0x14,0x22,0x41, 0x14,0x14,0x14,0x14,0x14, 0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x59,0x09,0x06,
    0x3E,0x41,0x5D,0x59,0x4E, 0x7C,0x12,0x11,0x12,0x7C, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
    0x7F,0x41,0x41,0x41,0x3E, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01, 0x3E,0x41,0x41,0x51,0x73,
    0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41,
    0x7F,0x40,0x40,0x40,0x40, 0x7F,0x02,0x1C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
    0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46, 0x26,0x49,0x49,0x49,0x32,
    0x03,0x01,0x7F,0x01,0x03, 0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F, 0x3F,0x40,0x38,0x40,0x3F,
    0x63,0x14,0x08,0x14,0x63, 0x03,0x04,0x78,0x04,0x03, 0x61,0x59,0x49,0x4D,0x43, 0x00,0x7F,0x41,0x41,0x00,
    0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04, 0x40,0x40,0x40,0x40,0x40,
    0x00,0x03,0x07,0x08,0x00, 0x20,0x54,0x54,0x78,0x40, 0x7F,0x28,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x28,
    0x38,0x44,0x44,0x28,0x7F, 0x38,0x54,0x54,0x54,0x18, 0x00,0x08,0x7E,0x09,0x02, 0x18,0xA4,0xA4,0x9C,0x78,
    0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x40,0x3D,0x00, 0x7F,0x10,0x28,0x44,0x00,
    0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x78,0x04,0x78, 0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38,
    0xFC,0x18,0x24,0x24,0x18, 0x18,0x24,0x24,0x18,0xFC, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x24,
    0x04,0x04,0x3F,0x44,0x24, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C, 0x3C,0x40,0x30,0x40,0x3C,
    0x44,0x28,0x10,0x28,0x44, 0x4C,0x90,0x90,0x90,0x7C, 0x44,0x64,0x54,0x4C,0x44, 0x00,0x08,0x36,0x41,0x00,
    0x00,0x00,0x77,0x00,0x00, 0x00,0x41,0x36,0x08,0x00, 0x02,0x01,0x02,0x04,0x02
};

/* one I2C transaction: address, control byte, data.
 * ctrl 0x00 = command bytes follow, ctrl 0x40 = display data follows */
static uint8_t oled_tx(uint8_t ctrl, const uint8_t *data, uint16_t len)
{
    uint8_t ok;
    si2c_start();
    ok = si2c_write((uint8_t)(oled_addr << 1)) && si2c_write(ctrl);
    for (uint16_t i = 0; ok && i < len; i++) {
        ok = si2c_write(data[i]);
    }
    si2c_stop();
    return ok;
}

static uint8_t oled_probe(uint8_t addr7)
{
    si2c_start();
    uint8_t ack = si2c_write((uint8_t)(addr7 << 1));
    si2c_stop();
    return ack;
}

static void oled_cmds(const uint8_t *cmds, uint16_t len)
{
    for (int t = 0; t < 3; t++) {
        if (oled_tx(0x00, cmds, len)) return;
        oled_err++;
        si2c_recover();
    }
}

/* position + data are always resent together, so data can never shift */
static void oled_send_chunk(uint8_t page, uint8_t col)
{
    uint8_t pos[3] = { (uint8_t)(0xB0 + page),            /* page        */
                       (uint8_t)(col & 0x0F),             /* column low  */
                       (uint8_t)(0x10 | (col >> 4)) };    /* column high */
    for (int t = 0; t < 5; t++) {
        if (oled_tx(0x00, pos, 3) &&
            oled_tx(0x40, &oled_buf[OLED_W * page + col], CHUNK)) {
            return;
        }
        oled_err++;
        si2c_recover();
    }
}

static void oled_update(void)
{
    for (uint8_t p = 0; p < 8; p++) {
        for (uint8_t c = 0; c < OLED_W; c += CHUNK) {
            oled_send_chunk(p, c);
        }
    }
}

static void oled_clear(void)
{
    memset(oled_buf, 0, sizeof(oled_buf));
}

/* resend the key settings every frame: fixes a setting broken by a glitch */
static void oled_restore(void)
{
    static const uint8_t seq[] = { 0x20, 0x02, 0x40, 0xD3, 0x00, 0xA1, 0xC8,
                                   0x81, 0xCF, 0xA4, 0xA6, 0xAF };
    oled_cmds(seq, sizeof(seq));
}

static void oled_init(void)
{
    static const uint8_t seq[] = {
        0xAE,             /* display off              */
        0xD5, 0x80,       /* clock divide             */
        0xA8, 0x3F,       /* multiplex 64             */
        0xD3, 0x00,       /* display offset 0         */
        0x40,             /* start line 0             */
        0x8D, 0x14,       /* charge pump on           */
        0x20, 0x02,       /* page addressing mode     */
        0xA1,             /* segment remap            */
        0xC8,             /* COM scan direction       */
        0xDA, 0x12,       /* COM pins                 */
        0x81, 0xCF,       /* contrast                 */
        0xD9, 0xF1,       /* pre-charge               */
        0xDB, 0x40,       /* VCOMH                    */
        0xA4,             /* show RAM content         */
        0xA6              /* normal (not inverted)    */
    };
    oled_cmds(seq, sizeof(seq));
    oled_clear();
    oled_update();        /* wipe old RAM content while the panel is still off */
    static const uint8_t on = 0xAF;
    oled_cmds(&on, 1);
}

/* find the OLED (0x3C or 0x3D) and initialise it */
static void oled_start(void)
{
    si2c_recover();
    if (oled_probe(0x3C))      oled_addr = 0x3C;
    else if (oled_probe(0x3D)) oled_addr = 0x3D;
    else                       oled_addr = 0;

    if (oled_addr) {
        oled_err = 0;
        oled_init();
        oled_ready = 1;
        printf("OLED found at 0x%02X\r\n", oled_addr);
    } else {
        oled_ready = 0;
        printf("OLED not found - check D15/D14, VCC, GND, RES wiring\r\n");
    }
}

/* ---------- drawing ---------- */
static void oled_pixel(int x, int y)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    oled_buf[x + (y / 8) * OLED_W] |= (uint8_t)(1 << (y % 8));
}

static void oled_char(int x, int y, char c, int scale)
{
    if (c < 32 || c > 126) c = '?';
    const uint8_t *g = &font5x7[(c - 32) * 5];
    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if ((g[col] >> row) & 1) {
                for (int sx = 0; sx < scale; sx++)
                    for (int sy = 0; sy < scale; sy++)
                        oled_pixel(x + col * scale + sx, y + row * scale + sy);
            }
        }
    }
}

static void oled_string(int x, int y, const char *s, int scale)
{
    while (*s) {
        oled_char(x, y, *s++, scale);
        x += 6 * scale;
    }
}

/* water drop icon, 25 px wide x 32 px tall. cx = center, y = top tip */
static void draw_drop(int cx, int y)
{
    const int r  = 12;
    const int cy = y + 20;
    for (int dy = 0; dy < 32; dy++) {
        int hw;
        if (dy <= 12) {
            hw = (dy * 3) / 4;                 /* pointed top */
        } else {
            int d = (y + dy) - cy;             /* round bottom */
            int t = r * r - d * d;
            hw = 0;
            while ((hw + 1) * (hw + 1) <= t) hw++;
        }
        for (int x = cx - hw; x <= cx + hw; x++) oled_pixel(x, y + dy);
    }
}

/* one row: label at x=0, colon at x=48, value at x=62, size 2 (16 px rows) */
static void draw_row(int row, const char *label, const char *value)
{
    int y = 1 + row * 16;
    oled_string(0,  y, label, 2);
    oled_string(48, y, ":",   2);
    oled_string(62, y, value, 2);
}

static void screen_splash(void)
{
    oled_clear();
    oled_string(4,  8,  "Auto Plant", 2);
    oled_string(16, 28, "Watering",   2);
    oled_string(37, 52, "STM32F401",  1);
    oled_update();
}

static void screen_main(uint8_t soil, int16_t t10, int16_t h10, uint8_t dht_ok, uint8_t pump)
{
    char v[16];

    oled_clear();

    snprintf(v, sizeof(v), "%u%%", (unsigned)soil);
    draw_row(0, "Soil", v);

    if (dht_ok) snprintf(v, sizeof(v), "%s%d.%dC", (t10 < 0) ? "-" : "", abs(t10) / 10, abs(t10) % 10);
    else        snprintf(v, sizeof(v), "--");
    draw_row(1, "Temp", v);

    if (dht_ok) snprintf(v, sizeof(v), "%d%%", h10 / 10);
    else        snprintf(v, sizeof(v), "--");
    draw_row(2, "Humi", v);

    draw_row(3, "Pump", pump ? "ON" : "OFF");

    if (pump || DROP_ALWAYS) {
        draw_drop(114, 32);                   /* lower right corner */
    }

    oled_update();
}

/* ======================================================================
 *  Relay / pump
 * ====================================================================== */
typedef enum { PUMP_IDLE = 0, PUMP_RUNNING, PUMP_SOAKING } PumpState_t;

static PumpState_t pump_state = PUMP_IDLE;
static uint32_t    pump_t0    = 0;
static uint8_t     pump_on    = 0;

static void pump_set(uint8_t on)
{
    pump_on = on;
#if RELAY_ACTIVE_LOW
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

static void pump_control(uint8_t soil, uint32_t now)
{
    switch (pump_state) {
    case PUMP_IDLE:
        if (soil < SOIL_ON_PCT) {
            pump_set(1);
            pump_t0 = now;
            pump_state = PUMP_RUNNING;
        }
        break;

    case PUMP_RUNNING:
        if (soil >= SOIL_OFF_PCT || (now - pump_t0) >= PUMP_MAX_ON_MS) {
            pump_set(0);
            pump_t0 = now;
            pump_state = PUMP_SOAKING;
        }
        break;

    case PUMP_SOAKING:
        if ((now - pump_t0) >= PUMP_SOAK_MS) {
            pump_state = PUMP_IDLE;
        }
        break;
    }
}

/* ======================================================================
 *  Soil moisture sensor (ADC1 IN0 = PA0)
 * ====================================================================== */
static uint16_t soil_read_raw(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            sum += HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }
    return (uint16_t)(sum / 16);
}

static uint8_t soil_to_percent(uint16_t raw)
{
    if (raw >= SOIL_ADC_DRY) return 0;
    if (raw <= SOIL_ADC_WET) return 100;
    return (uint8_t)(((uint32_t)(SOIL_ADC_DRY - raw) * 100U) / (SOIL_ADC_DRY - SOIL_ADC_WET));
}

/* ======================================================================
 *  DHT22 / DHT11 (PA8, open-drain + pull-up)
 * ====================================================================== */
#define DHT_LOW()   HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET)
#define DHT_HIGH()  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET)
#define DHT_READ()  (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET)

static uint8_t dht_wait_level(uint8_t level, uint16_t timeout_us)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    while ((DHT_READ() ? 1 : 0) != level) {
        if (__HAL_TIM_GET_COUNTER(&htim3) > timeout_us) return 0;
    }
    return 1;
}

/* returns 1 on success, values in 0.1 units (245 = 24.5) */
static uint8_t dht_read(int16_t *temp_x10, int16_t *hum_x10)
{
    uint8_t d[5] = {0};
    uint8_t ok;

    DHT_LOW();                       /* start signal */
    HAL_Delay(20);

    __disable_irq();
    DHT_HIGH();
    delay_us(30);

    ok = dht_wait_level(0, 100) && dht_wait_level(1, 100) && dht_wait_level(0, 100);

    for (int i = 0; ok && i < 40; i++) {
        if (!dht_wait_level(1, 100)) { ok = 0; break; }
        delay_us(40);                /* '0' ~27 us high, '1' ~70 us high */
        d[i / 8] <<= 1;
        if (DHT_READ()) d[i / 8] |= 1;
        if (!dht_wait_level(0, 100)) { ok = 0; break; }
    }
    __enable_irq();
    DHT_HIGH();

    if (!ok) return 0;
    if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return 0;   /* checksum */

#if (DHT_TYPE == 22)
    *hum_x10 = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
    int16_t t = (int16_t)((((uint16_t)d[2] & 0x7F) << 8) | d[3]);
    *temp_x10 = (d[2] & 0x80) ? (int16_t)-t : t;
#else
    *hum_x10  = (int16_t)(d[0] * 10 + d[1]);
    *temp_x10 = (int16_t)(d[2] * 10 + (d[3] & 0x7F));
    if (d[3] & 0x80) *temp_x10 = (int16_t)-*temp_x10;
#endif
    return 1;
}

/* ======================================================================
 *  main
 * ====================================================================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    HAL_TIM_Base_Start(&htim3);     /* 1 MHz counter for delay_us */

    pump_set(0);                    /* pump OFF at start */
    DHT_HIGH();

    printf("\r\n== Auto Plant Watering System ==\r\n");

    HAL_Delay(100);                 /* OLED power-up time */
    oled_start();
    if (oled_ready) screen_splash();
    HAL_Delay(1500);                /* DHT needs ~1 s after power-up */

    uint32_t t_loop   = 0;
    uint32_t t_dht    = 0;
    int16_t  temp_x10 = 0;
    int16_t  hum_x10  = 0;
    uint8_t  dht_ok   = 0;
    uint16_t raw      = 0;
    uint8_t  soil     = 0;
    uint8_t  pump_shown;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - t_loop) < LOOP_PERIOD_MS) continue;
        t_loop = now;

#if DEMO_MODE
        soil = 68; temp_x10 = 245; hum_x10 = 520; dht_ok = 1;
        pump_set(0);
        pump_shown = DEMO_PUMP_ON;
#else
        /* 1) temperature / humidity, every 2 s */
        if ((now - t_dht) >= DHT_PERIOD_MS) {
            t_dht  = now;
            dht_ok = dht_read(&temp_x10, &hum_x10);
        }

        /* 2) soil moisture */
        raw  = soil_read_raw();
        soil = soil_to_percent(raw);

        /* 3) pump control (keeps working even without the OLED) */
        pump_control(soil, now);
        pump_shown = pump_on;
#endif

        /* 4) OLED: retry if missing, re-init if it stopped answering */
        if (!oled_ready) {
            oled_start();
        }
        if (oled_ready) {
            oled_err = 0;
            oled_restore();
            screen_main(soil, temp_x10, hum_x10, dht_ok, pump_shown);
            if (oled_err > 20) {
                printf("OLED errors: %u -> re-init\r\n", (unsigned)oled_err);
                oled_ready = 0;
            } else if (oled_err) {
                printf("OLED errors fixed by retry: %u\r\n", (unsigned)oled_err);
            }
        }

        /* 5) heartbeat LED + serial log */
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

        printf("raw=%u soil=%u%% temp=%s%d.%dC hum=%d.%d%% dht=%s pump=%s oled=%s\r\n",
               raw, soil,
               (temp_x10 < 0) ? "-" : "", abs(temp_x10) / 10, abs(temp_x10) % 10,
               hum_x10 / 10, hum_x10 % 10,
               dht_ok ? "OK" : "ERR",
               pump_shown ? "ON" : "OFF",
               oled_ready ? "OK" : "NOT FOUND");
    }
}

/* ======================================================================
 *  Clock: HSE 8 MHz (from ST-Link, bypass) -> PLL -> SYSCLK 84 MHz
 *  APB1 = 42 MHz (timer clock 84 MHz), APB2 = 84 MHz
 * ====================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 4;
    RCC_OscInitStruct.PLL.PLLN       = 168;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/* ======================================================================
 *  Peripheral init
 * ====================================================================== */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 83;            /* 84 MHz / 84 = 1 MHz */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* initial levels: relay OFF, DHT released, I2C lines released, LED off */
#if RELAY_ACTIVE_LOW
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
#endif
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_PORT, OLED_SCL_PIN | OLED_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    /* PB0 : relay (push-pull) */
    GPIO_InitStruct.Pin   = RELAY_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RELAY_PORT, &GPIO_InitStruct);

    /* PA5 : LD2 green LED */
    GPIO_InitStruct.Pin   = LED_PIN;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* PA8 : DHT data (open-drain + pull-up) */
    GPIO_InitStruct.Pin   = DHT_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);

    /* PB8 / PB9 : OLED software I2C (open-drain + pull-up) */
    GPIO_InitStruct.Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_PORT, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
