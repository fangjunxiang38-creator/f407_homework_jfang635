#include "threads.hpp"
#include "LED.hpp" 

/* ============ 信号量（别的线程要用，所以是全局的） ============ */

TX_SEMAPHORE imu_alive_sem;
TX_SEMAPHORE control_alive_sem;
TX_SEMAPHORE imu_init_sem;
/* ============ 线程三件套 ============ */

static TX_THREAD alive_thread;
static uint8_t   alive_stack[1024];

/* 等一个"活着"信号：先清积压，再等新的 */
static UINT wait_alive(TX_SEMAPHORE *sem, ULONG timeout)
{
    while (tx_semaphore_get(sem, TX_NO_WAIT) == TX_SUCCESS)
    {
        /* 把积压的取干净 */
    }
    return tx_semaphore_get(sem, timeout);
}

/* ============ 线程函数 ============ */

static void alive_thread_entry(ULONG thread_input)
{
    (void)thread_input;

    LED_AllOff();

    /* 等各线程初始化完成（imu 标定要 4 秒） */
    tx_thread_sleep(5000);

    /* 检查 IMU 初始化是否成功 */
    UINT init_state = tx_semaphore_get(&imu_init_sem, TX_NO_WAIT);

    while (1)
    {
        /* 初始化检查*/
        if (init_state != TX_SUCCESS)
        {
            LED_Toggle(LED_RED);
            LED_Off(LED_GREEN);
            LED_Off(LED_BLUE);
            tx_thread_sleep(500);
            continue;
        }


        tx_thread_sleep(100);          /* 每 100ms 检查一次 */

        /* 50ms 内有没有喂 → 有就是活着 */
        UINT s_imu     = wait_alive(&imu_alive_sem,     50);
        UINT s_control = wait_alive(&control_alive_sem, 50);

            if (s_imu == TX_SUCCESS && s_control == TX_SUCCESS)
        {
            /* 都活着：绿灯亮，红灯灭 */
            LED_On(LED_GREEN);
            LED_Off(LED_RED);
        }
        else
        {
            /* 有线程挂了：红灯亮，绿灯灭 */
            LED_Off(LED_GREEN);
            LED_On(LED_RED);
        }

        /* 蓝灯每轮翻转一次 —— 表示 alive 自己也还活着 */
        LED_Toggle(LED_BLUE);

    }
}

/* ============ 创建函数 ============ */

extern "C" void alive_thread_create(void)
{
    /* ★ 先创建信号量，再创建线程 ★ */
    tx_semaphore_create(&imu_alive_sem,     (CHAR *)"imu_alive",     0);
    tx_semaphore_create(&control_alive_sem, (CHAR *)"control_alive", 0);
    tx_semaphore_create(&imu_init_sem,      (CHAR *)"imu_init",      0);

    tx_thread_create(&alive_thread,
                     (CHAR *)"alive",
                     alive_thread_entry,
                     0,
                     alive_stack,
                     sizeof(alive_stack),
                     20,                    /* 优先级 20：最低，后台任务 */
                     20,
                     TX_NO_TIME_SLICE,
                     TX_AUTO_START);
}