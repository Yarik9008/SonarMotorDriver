/* Test_TMC2208 — USB CDC + TMC2209 Stepper Motor Control.
 *
 * Команды: i(init) m<N>(move) s(stop) e(on) d(off) st(status) v(ver) p/u(mode) ?(help)
 */

#include "board.h"
#include "usb_cdc.h"
#include "tmc2209.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define CMD_BUF_SIZE  64
#define RSP_BUF_SIZE  96

static char cmd[CMD_BUF_SIZE];
static char rsp[RSP_BUF_SIZE];

static void SystemClock_Config(void);

static void tx(const char *s)
{
    USB_CDC_Transmit((const uint8_t *)s, (uint16_t)strlen(s));
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

    if (cmd[0] == '?' || cmd[0] == 'h') {
        tx("i  init\r\n"
           "m<N> move (UART:VACTUAL / SD:steps/s)\r\n"
           "s  stop\r\n"
           "e  enable\r\n"
           "d  disable\r\n"
           "p  STEP/DIR mode\r\n"
           "u  UART mode\r\n"
           "st status\r\n"
           "v  version\r\n"
           "?  help\r\n");
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
    else if (cmd[0] == 'v' && n == 1) {
        uint8_t v = TMC2209_ReadVersion();
        snprintf(rsp, sizeof(rsp), "IC=0x%02X%s\r\n", v, (v == 0x21) ? " OK" : " ??");
        tx(rsp);
    }
    else {
        tx("?\r\n");
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
    TMC2209_InitStart();

    uint8_t greeted = 0, reported = 0;

    while (1) {
        USB_CDC_Task();
        TMC2209_Status st = TMC2209_Poll();

        if (USB_CDC_IsConnected()) {
            if (!greeted) {
                tx("TMC2209 ready. ? for help\r\n");
                greeted = 1;
            }
            if (!reported && st == TMC_DONE) {
                tx("init ok\r\n");
                reported = 1;
            } else if (!reported && st == TMC_ERROR) {
                tx("init FAIL\r\n");
                reported = 1;
            }
            process_cmd();
        } else {
            greeted = reported = 0;
        }
    }
}
