/* main.cpp - STM32 NEMA17 + TMC2209 Flow Diagram Implementation
 * 
 * Hardware connections (STM32 Blue Pill / Nucleo):
 *   STEP  → PA0  (Timer PWM or GPIO toggle)
 *   DIR   → PA1  (GPIO Output)
 *   EN    → PA2  (GPIO Output, active LOW on TMC2209)
 *   UART  → PA9/PA10 (USART1 TX/RX for TMC2209 UART config - optional)
 *
 * "Target Captured" signal → PA3 (GPIO Input, set HIGH by external source)
 */

#include "main.h"

// ─── Pin Definitions ─────────────────────────────────────────────────────────
#define STEP_PIN       GPIO_PIN_0
#define STEP_PORT      GPIOA

#define DIR_PIN        GPIO_PIN_1
#define DIR_PORT       GPIOA

#define EN_PIN         GPIO_PIN_2
#define EN_PORT        GPIOA

#define TARGET_PIN     GPIO_PIN_3   // "Target Captured" input
#define TARGET_PORT    GPIOA

// ─── Motor Configuration ──────────────────────────────────────────────────────
#define STEPS_PER_REV       200     // NEMA17 = 200 full steps/rev
#define MICROSTEPS          16      // TMC2209 microstepping (set via MS1/MS2 or UART)
#define STEPS_PER_DEG       ((STEPS_PER_REV * MICROSTEPS) / 360.0f) 

#define TARGET_ANGLE        45.0f   // deg
#define HOLD_TIME_MS        30000   // 30 secs
#define STEP_DELAY_US       500     // microseconds between steps (controls speed)
                                    // Lower = faster. At 500us = ~360 steps/sec

// ─── State Machine ────────────────────────────────────────────────────────────
typedef enum {
    STATE_WAIT,
    STATE_ANGLE_TO_45,
    STATE_HOLD_45,
    STATE_RETURN_TO_0,
    STATE_END
} SystemState;

// ─── Globals ──────────────────────────────────────────────────────────────────
static SystemState currentState = STATE_WAIT;
static uint32_t    holdStartTick = 0;

// ─── Function Prototypes ──────────────────────────────────────────────────────
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void Motor_Enable(void);
void Motor_Disable(void);
void Motor_SetDirection(GPIO_PinState dir);
void Motor_Step(void);
void Motor_MoveSteps(int32_t steps);
void Motor_MoveToAngle(float angleDeg);

bool IsTargetCaptured(void);

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(void)
{
    HAL_Init(); #hardware abstraction layer
    SystemClock_Config();
    MX_GPIO_Init();

    // Enable the TMC2209 driver (active LOW)
    Motor_Enable();

    // ── Flow Diagram State Machine ──────────────────────────────────────────
    while (1)
    {
        switch (currentState)
        {
           // WAIT — poll until targetCaptured == true
            case STATE_WAIT:
                if (IsTargetCaptured())
                {
                    currentState = STATE_ANGLE_TO_45;
                }
                // else: keep waiting (loop back)
                HAL_Delay(10);  // small poll interval
                break;

          //  ANGLE FROM 0° → 45°
            case STATE_ANGLE_TO_45:
                Motor_MoveToAngle(TARGET_ANGLE);    // move forward 45°
                holdStartTick = HAL_GetTick();
                currentState = STATE_HOLD_45;
                break;

            //HOLD AT 45° FOR 30 SECONDS  
            case STATE_HOLD_45:
                if ((HAL_GetTick() - holdStartTick) >= HOLD_TIME_MS)
                {
                    currentState = STATE_RETURN_TO_0;
                }
                // Motor stays enabled and holding position (TMC2209 hold current)
                break;

            // MOVE OUTPUT ANGLE BACK TO 0° 
            case STATE_RETURN_TO_0:
                Motor_MoveToAngle(-TARGET_ANGLE);   // reverse 45° back to 0°
                currentState = STATE_END;
                break;

            // END — disable motor, halt 
            case STATE_END:
                Motor_Disable();
                // Optionally: reset and go back to WAIT for next trigger
                // currentState = STATE_WAIT;
                while (1) { __NOP(); }  // Halt
                break;

            default:
                currentState = STATE_WAIT;
                break;
        }
    }
}

// ─── Motor Driver Functions ────────────────

void Motor_Enable(void)
{
    // TMC2209 EN is active LOW
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_RESET);
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_SET);
}

void Motor_SetDirection(GPIO_PinState dir)
{
    // GPIO_PIN_SET   = clockwise (forward)
    // GPIO_PIN_RESET = counter-clockwise (reverse)
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir);
    HAL_Delay(1);   // direction setup time (TMC2209 needs ~20ns but 1ms is safe)
}

void Motor_Step(void)
{
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
    // Use DWT cycle counter for microsecond delay if available,
    // otherwise use a simple busy-wait loop:
    uint32_t start = DWT->CYCCNT;
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 1000000) * STEP_DELAY_US;
    while ((DWT->CYCCNT - start) < delay) { __NOP(); }

    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);

    start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < delay) { __NOP(); }
}

void Motor_MoveSteps(int32_t steps)
{
    if (steps == 0) return;

    Motor_SetDirection(steps > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    int32_t absSteps = (steps > 0) ? steps : -steps;
    for (int32_t i = 0; i < absSteps; i++)
    {
        Motor_Step();
    }
}

void Motor_MoveToAngle(float angleDeg)
{
    int32_t steps = (int32_t)(angleDeg * STEPS_PER_DEG);
    Motor_MoveSteps(steps);
}

// ─── Target Captured Input ─────────

bool IsTargetCaptured(void)
{
    // Returns true when PA3 is pulled HIGH by your external trigger
    // (camera signal, sensor, button, UART flag, etc.)
    return (HAL_GPIO_ReadPin(TARGET_PORT, TARGET_PIN) == GPIO_PIN_SET);
}

// ─── GPIO Initialisation ─────────────────

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // STEP, DIR, EN → Output Push-Pull
    GPIO_InitStruct.Pin   = STEP_PIN | DIR_PIN | EN_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TARGET → Input with internal pull-down
    GPIO_InitStruct.Pin  = TARGET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(TARGET_PORT, &GPIO_InitStruct);

    // Default output states
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_PORT,  DIR_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_PORT,   EN_PIN,   GPIO_PIN_SET);   // disabled until ready

    // Enable DWT cycle counter for microsecond delays
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

// ─── System Clock (72 MHz for Blue Pill, adjust for your board) ───────────────

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;   // 8MHz * 9 = 72MHz
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}