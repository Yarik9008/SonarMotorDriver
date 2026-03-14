/* Test_TMC2208 — USB CDC + TMC2209 Stepper Motor Control.
 *
 * Команды: i(init) m<N>(move) s(stop) e(on) d(off) st(status) v(ver) c(cs x10) p/u(mode) h(help)
 */

#include "board.h"
#include "usb_cdc.h"
#include "tmc2209.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define CMD_BUF_SIZE  64
#define RSP_BUF_SIZE  160
#define CS_INTERVAL_MS 100U

static char cmd[CMD_BUF_SIZE];
static char rsp[RSP_BUF_SIZE];
static uint8_t continuous_cs = 0;

/* Регистры TMC2209 для телеметрии */
#define TMC_GSTAT     0x01U
#define TMC_IFCNT     0x02U
#define TMC_IOIN      0x06U
#define TMC_TSTEP     0x12U
#define TMC_SG_RESULT 0x41U
#define TMC_MSCNT     0x6AU
#define TMC_MSCURACT  0x6BU
#define TMC_CHOPCONF  0x6CU
#define TMC_DRV_STATUS 0x6FU
#define TMC_PWM_SCALE 0x71U

static void SystemClock_Config(void);

static void tx(const char *s)
{
    USB_CDC_Transmit((const uint8_t *)s, (uint16_t)strlen(s));
}

static void tmc_debug_print(const char *s)
{
    tx(s);
}

static void output_telemetry(void)
{
    uint32_t v;
    if (!TMC2209_ReadReg(TMC_GSTAT, &v)) v = 0;
    snprintf(rsp, sizeof(rsp), "GSTAT=0x%02lX reset=%lu drv_err=%lu uv_cp=%lu\r\n",
             v & 0xFF, (v >> 0) & 1, (v >> 1) & 1, (v >> 2) & 1);
    tx(rsp);

    if (TMC2209_ReadReg(TMC_IFCNT, &v)) {
        snprintf(rsp, sizeof(rsp), "IFCNT=%lu\r\n", v & 0xFF);
        tx(rsp);
    } else tx("IFCNT=?\r\n");

    if (TMC2209_ReadReg(TMC_IOIN, &v)) {
        snprintf(rsp, sizeof(rsp), "IOIN=0x%08lX ENN=%lu MS1=%lu MS2=%lu DIAG=%lu STEP=%lu DIR=%lu VERSION=0x%02lX\r\n",
                 v, (v >> 0) & 1, (v >> 2) & 1, (v >> 3) & 1, (v >> 4) & 1, (v >> 7) & 1, (v >> 9) & 1, (v >> 24) & 0xFF);
        tx(rsp);
    } else tx("IOIN=?\r\n");

    if (TMC2209_ReadReg(TMC_TSTEP, &v)) {
        snprintf(rsp, sizeof(rsp), "TSTEP=%lu\r\n", v & 0xFFFFF);
        tx(rsp);
    } else tx("TSTEP=?\r\n");

    if (TMC2209_ReadReg(TMC_SG_RESULT, &v)) {
        snprintf(rsp, sizeof(rsp), "SG_RESULT=%lu\r\n", v & 0x3FF);
        tx(rsp);
    } else tx("SG_RESULT=?\r\n");

    if (TMC2209_ReadReg(TMC_MSCNT, &v)) {
        snprintf(rsp, sizeof(rsp), "MSCNT=%lu\r\n", v & 0x3FF);
        tx(rsp);
    } else tx("MSCNT=?\r\n");

    if (TMC2209_ReadReg(TMC_MSCURACT, &v)) {
        int32_t cur_a = (int32_t)(v & 0x1FF);
        if (cur_a & 0x100) cur_a -= 512;
        int32_t cur_b = (int32_t)((v >> 16) & 0x1FF);
        if (cur_b & 0x100) cur_b -= 512;
        snprintf(rsp, sizeof(rsp), "MSCURACT CUR_A=%ld CUR_B=%ld\r\n", (long)cur_a, (long)cur_b);
        tx(rsp);
    } else tx("MSCURACT=?\r\n");

    if (TMC2209_ReadReg(TMC_CHOPCONF, &v)) {
        snprintf(rsp, sizeof(rsp), "CHOPCONF=0x%08lX MRES=%lu\r\n", v, (v >> 24) & 0x0F);
        tx(rsp);
    } else tx("CHOPCONF=?\r\n");

    if (TMC2209_ReadReg(TMC_DRV_STATUS, &v)) {
        snprintf(rsp, sizeof(rsp), "DRV_STATUS=0x%08lX stst=%lu stealth=%lu cs_actual=%lu s2ga=%lu s2gb=%lu ola=%lu olb=%lu ot=%lu otpw=%lu\r\n",
                 v, (v >> 31) & 1, (v >> 30) & 1, (v >> 16) & 0x1F, (v >> 2) & 1, (v >> 3) & 1, (v >> 6) & 1, (v >> 7) & 1, (v >> 1) & 1, (v >> 0) & 1);
        tx(rsp);
    } else tx("DRV_STATUS=?\r\n");

    if (TMC2209_ReadReg(TMC_PWM_SCALE, &v)) {
        int32_t auto_off = (int32_t)((v >> 16) & 0x1FF);
        if (auto_off & 0x100) auto_off -= 512;
        snprintf(rsp, sizeof(rsp), "PWM_SCALE sum=%lu auto=%ld\r\n", v & 0xFF, (long)auto_off);
        tx(rsp);
    } else tx("PWM_SCALE=?\r\n");
}

static void process_cmd(void)
{
    if (!USB_CDC_IsConnected())
        return;

    uint16_t n = USB_CDC_ReadLine(cmd, CMD_BUF_SIZE);
    if (n == 0)
        return;
    cmd[n] = '\0';

    const char *mode_str = (TMC2209_GetMode() == TMC_MODE_UART) ? "UA" : "SD";

    if (cmd[0] == 'h' && n == 1) {
        tx("i  init\r\n"
           "m<N> move (UART:VACTUAL / SD:steps/s)\r\n"
           "s  stop\r\n"
           "e  enable\r\n"
           "d  disable\r\n"
           "p  STEP/DIR mode\r\n"
           "u  UART mode\r\n"
           "st status\r\n"
           "t  telemetry (all regs)\r\n"
           "v  version\r\n"
           "c  current x10 (cs loop, s=stop)\r\n"
           "h  help\r\n");
    }
    else if (cmd[0] == 'i' && n == 1) {
        TMC2209_InitStart();
        tx("init\r\n");
    }
    else if (cmd[0] == 'm') {
        int32_t val = (int32_t)atoi(cmd + 1);
        TMC2209_Move(val);
        snprintf(rsp, sizeof(rsp), "%s=%ld\r\n", mode_str, (long)val);
        tx(rsp);
    }
    else if (cmd[0] == 's' && n == 1) {
        continuous_cs = 0;
        TMC2209_Stop();
        tx("stop\r\n");
    }
    else if (cmd[0] == 's' && cmd[1] == 't') {
        uint32_t st = TMC2209_ReadDrvStatus();
        snprintf(rsp, sizeof(rsp), "DRV=0x%08lX [%s]\r\n", (unsigned long)st, mode_str);
        tx(rsp);
    }
    else if (cmd[0] == 'e' && n == 1) {
        TMC2209_SetEnable(1);
        tx("on\r\n");
    }
    else if (cmd[0] == 'd' && n == 1) {
        TMC2209_SetEnable(0);
        tx("off\r\n");
    }
    else if (cmd[0] == 'p' && n == 1) {
        TMC2209_SetMode(TMC_MODE_STEP_DIR);
        tx("mode:SD\r\n");
    }
    else if (cmd[0] == 'u' && n == 1) {
        TMC2209_SetMode(TMC_MODE_UART);
        tx("mode:UA\r\n");
    }
    else if (cmd[0] == 't' && n == 1) {
        output_telemetry();
    }
    else if (cmd[0] == 'v' && n == 1) {
        uint8_t v = TMC2209_ReadVersion();
        snprintf(rsp, sizeof(rsp), "IC=0x%02X%s\r\n", v, (v == 0x21) ? " OK" : " ??");
        tx(rsp);
    }
    else if (cmd[0] == 'c' && n == 1) {
        continuous_cs = 1;
        tx("cs loop (s=stop)\r\n");
    }
    else {
        tx("h\r\n");
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 4;
    osc.PLL.PLLN       = 168;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 7;
    osc.PLL.PLLR       = 2;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);

    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    pclk.Clk48ClockSelection  = RCC_CLK48CLKSOURCE_PLLQ;
    HAL_RCCEx_PeriphCLKConfig(&pclk);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    USB_CDC_Init();
    TMC2209_SetDebugPrint(tmc_debug_print);
    TMC2209_InitStart();

    uint8_t greeted = 0, reported = 0;
    uint32_t last_cs_tick = 0;

    while (1) {
        USB_CDC_Task();
        TMC2209_Status st = TMC2209_Poll();

        if (USB_CDC_IsConnected()) {
            if (continuous_cs) {
                uint32_t now = HAL_GetTick();
                if ((uint32_t)(now - last_cs_tick) >= CS_INTERVAL_MS) {
                    last_cs_tick = now;
                    uint8_t cs[10];
                    for (int i = 0; i < 10; i++) {
                        uint32_t drv = TMC2209_ReadDrvStatus();
                        cs[i] = (uint8_t)((drv >> 16) & 0x1FU);
                    }
                    snprintf(rsp, sizeof(rsp), "cs:%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                             cs[0], cs[1], cs[2], cs[3], cs[4], cs[5], cs[6], cs[7], cs[8], cs[9]);
                    tx(rsp);
                }
            }
            if (!greeted) {
                tx("TMC2209 ready. h for help\r\n");
                greeted = 1;
            }
            if (!reported && st == TMC_DONE) {
#if TMC2209_USE_UART_MODE
                TMC2209_SetMode(TMC_MODE_UART);
#endif
                tx("init ok (128 mstep, 1A)\r\n");
                output_telemetry();
                reported = 1;
            } else if (!reported && st == TMC_ERROR) {
                tx("init FAIL\r\n");
                reported = 1;
            }
            process_cmd();
        } else {
            greeted = reported = 0;
            continuous_cs = 0;
        }
    }
}
