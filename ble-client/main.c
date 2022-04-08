#include <stdio.h>
#include <string.h>

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

// Bouffalo device
#include <bl_sys.h>
#include <bl_uart.h>
#include <bl_chip.h>
#include <bl_sec.h>
#include <bl_cks.h>
#include <bl_irq.h>
#include <bl_timer.h>
#include <bl_dma.h>
#include <bl_gpio.h>
#include <bl_rtc.h>

// HAL
#include <hal_sys.h>
#include <hal_boot2.h>
#include <hal_board.h>
#include <hal_uart.h>

// VFS
#include <vfs.h>

// Flattened Device Tree
#include <fdt.h>
#include <libfdt.h>

// AliOS Things
#include <aos/kernel.h>
#include <aos/yloop.h>

#include <event_device.h>

#include "ble_lib_api.h"
#include "ble_app.h"

extern void uart_init(uint8_t uartid);

// Global heap
extern uint8_t _heap_start;
extern uint8_t _heap_size;
static HeapRegion_t xHeapRegions[] = {
    { &_heap_start,  (unsigned int) &_heap_size},
    { NULL, 0 }
};

void user_vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) __attribute__ ((weak, alias ("vApplicationStackOverflowHook")));
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /*empty*/
}

void user_vApplicationMallocFailedHook(void) __attribute__ ((weak, alias ("vApplicationMallocFailedHook")));
void vApplicationMallocFailedHook(void)
{
    printf("Memory Allocate Failed. Current left size is %d bytes\r\n",
        xPortGetFreeHeapSize()
    );
}

void user_vApplicationIdleHook(void) __attribute__ ((weak, alias ("vApplicationIdleHook")));
void vApplicationIdleHook(void)
{
    __asm volatile("wfi");
}

void user_vAssertCalled(void) __attribute__ ((weak, alias ("vAssertCalled")));
void vAssertCalled(void)
{
    volatile uint32_t ulSetTo1ToExitFunction = 0;

    taskDISABLE_INTERRUPTS();
    while (ulSetTo1ToExitFunction != 1) {
        __asm volatile( "NOP" );
    }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t
 **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    /* If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static - otherwise they will be allocated
    on the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[512];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes.
    size 512 words is For ble pds mode, otherwise stack overflow of idle task
    will happen. */
    *pulIdleTaskStackSize = 512;
}

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

static int get_dts_addr(const char *name, uint32_t *start, uint32_t *off)
{
    uint32_t addr = hal_board_get_factory_addr();
    const void *fdt = (const void *)addr;
    uint32_t offset;

    if (!name || !start || !off) {
        return -1;
    }

    offset = fdt_subnode_offset(fdt, 0, name);
    if (offset <= 0) {
       printf("%s NULL.\r\n", name);
       return -1;
    }

    *start = (uint32_t)fdt;
    *off = offset;

    return 0;
}

static void event_cb_ble_event(input_event_t *event, void *private_data)
{
    printf("%s: 0x%02x\n", __func__, event->code);
}

static void proc_hello_entry(void *pvParameters)
{
    vfs_init();
    vfs_device_init();

    { uint32_t fdt = 0;
        uint32_t offset = 0;
        if (get_dts_addr("uart", &fdt, &offset) == 0) {
            vfs_uart_init(fdt, offset);
        }
    }

    aos_loop_init();
    aos_register_event_filter(EV_BLE_TEST, event_cb_ble_event, NULL);

    uart_init(1);
    ble_controller_init(configMAX_PRIORITIES - 1);

    aos_loop_run();

    vTaskDelete(NULL);
}

void bfl_main(void)
{
    static StackType_t proc_hello_stack[1024];
    static StaticTask_t proc_hello_task;

    bl_sys_early_init();

    // Init UART using pins 16+7 (TX+RX) and baudrate of 2M
    bl_uart_init(0, 16, 7, 255, 255, 2 * 1000 * 1000);
    bl_sys_init();

    vPortDefineHeapRegions(xHeapRegions);
    printf("Heap %u@%p\r\n", (unsigned int)&_heap_size, &_heap_start);

    bl_irq_init();
    bl_sec_init();
    bl_sec_test();
    bl_dma_init();
    bl_rtc_init();
    hal_boot2_init();
    // Board config is set after system is initialized
    hal_board_cfg(0);

    printf("[OS] Starting proc_hello_entry task...\r\n");
    xTaskCreateStatic(proc_hello_entry, (char*)"event_loop", 1024, NULL, 15, proc_hello_stack, &proc_hello_task);

    printf("[OS] Starting OS Scheduler...\r\n");
    vTaskStartScheduler();
}

