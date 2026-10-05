/**
 * @file atlas_time.c
 * @brief Scheduler-aware delay implementation for startup and runtime driver calls.
 *
 * Major functions:
 * - AtlasTime_DelayMs(): selects HAL busy-wait timing before scheduling and task delay after.
 * - AtlasTime_StartCounter(): validates/start-once semantics for the shared counter.
 * - AtlasTime_UseMicrosecondCounter()/NowUs(): register and read the sample timebase.
 */

#include "atlas_time.h"

#include <stddef.h>

#include "main.h"

#if defined(ATLAS_USE_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#endif

/** Registered 1 MHz counter; written once during single-threaded startup. */
static TIM_HandleTypeDef *volatile atlas_time_microsecond_timer;

/**
 * @brief Delay without busy-spinning a running RTOS task.
 * @param delay_ms Delay duration; zero returns immediately.
 */
void AtlasTime_DelayMs(uint32_t delay_ms)
{
    if (delay_ms == 0U)
    {
        return;
    }

#if defined(ATLAS_USE_FREERTOS)
    /* Driver delays are task-only: blocking inside an exception could deadlock SysTick. */
    configASSERT(__get_IPSR() == 0U);
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* A relative N-tick delay may last only N-1 complete ticks. Round the
         * requested duration UP, then add a full phase-guard tick. Chunk long
         * waits to keep arithmetic and every kernel timeout below half range. */
        const uint64_t maximum_ms =
            (((uint64_t)portMAX_DELAY / 2U - 1U) * 1000U) / configTICK_RATE_HZ;
        configASSERT(maximum_ms > 0U);
        while (delay_ms != 0U)
        {
            const uint32_t chunk_ms = (delay_ms > maximum_ms) ?
                                     (uint32_t)maximum_ms : delay_ms;
            const TickType_t ticks = (TickType_t)(
                (((uint64_t)chunk_ms * configTICK_RATE_HZ + 999U) / 1000U) + 1U);
            vTaskDelay(ticks);
            delay_ms -= chunk_ms;
        }
        return;
    }
    configASSERT(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);
#endif

    /* HAL adds a tick internally; do not let that addition overflow on UINT32_MAX. */
    while (delay_ms != 0U)
    {
        const uint32_t chunk_ms = (delay_ms > 60000U) ? 60000U : delay_ms;
        HAL_Delay(chunk_ms);
        delay_ms -= chunk_ms;
    }
}

/**
 * @brief Acquire a running counter without restarting a previous user's clock.
 * @param timer Configured timer handle with a valid peripheral instance.
 * @return ATLAS_OK, ATLAS_ERROR_NULL, ATLAS_ERROR_STATE, or ATLAS_ERROR_IO.
 */
AtlasStatus AtlasTime_StartCounter(TIM_HandleTypeDef *timer)
{
    if ((timer == NULL) || (timer->Instance == NULL))
    {
        return ATLAS_ERROR_NULL;
    }
    if ((timer->State == HAL_TIM_STATE_BUSY) &&
        ((timer->Instance->CR1 & TIM_CR1_CEN) != 0U))
    {
        return ATLAS_OK; /* BNO085 and GNSS share time; neither may reset it. */
    }
    if ((timer->State != HAL_TIM_STATE_READY) ||
        ((timer->Instance->CR1 & TIM_CR1_CEN) != 0U))
    {
        return ATLAS_ERROR_STATE;
    }
    if (HAL_TIM_Base_Start(timer) != HAL_OK)
    {
        return ATLAS_ERROR_IO;
    }
    return ((timer->State == HAL_TIM_STATE_BUSY) &&
            ((timer->Instance->CR1 & TIM_CR1_CEN) != 0U)) ?
           ATLAS_OK : ATLAS_ERROR_IO;
}

/**
 * @brief Start and register the shared microsecond sample timebase.
 * @param timer Free-running 1 MHz, 32-bit counter handle.
 * @return ATLAS_OK or the counter start failure.
 */
AtlasStatus AtlasTime_UseMicrosecondCounter(TIM_HandleTypeDef *timer)
{
    const AtlasStatus status = AtlasTime_StartCounter(timer);

    atlas_time_microsecond_timer = (status == ATLAS_OK) ? timer : NULL;
    return status;
}

/**
 * @brief Report whether the sample timebase is registered.
 * @return true when AtlasTime_NowUs() is meaningful.
 */
bool AtlasTime_MicrosecondsAvailable(void)
{
    return atlas_time_microsecond_timer != NULL;
}

/**
 * @brief Read the shared microsecond counter.
 * @return Current count, or 0 without a registered timebase.
 */
uint32_t AtlasTime_NowUs(void)
{
    TIM_HandleTypeDef *const timer = atlas_time_microsecond_timer;

    return (timer == NULL) ? 0U : (uint32_t)__HAL_TIM_GET_COUNTER(timer);
}
