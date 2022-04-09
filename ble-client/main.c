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

#include "bluetooth.h"
#include "conn.h"
#include "hci_core.h"
#include "hci_driver.h"
#include "oad_api.h"
#include "gatt.h"
#include "ble_lib_api.h"

extern void uart_init(uint8_t uartid);

// BLE defines

#define NAME_LEN        30
#define CHAR_SIZE_MAX       512
#define BT_UUID_TEST        BT_UUID_DECLARE_16(0xFFF0)
#define BT_UUID_TEST_RX     BT_UUID_DECLARE_16(0xFFF1)
#define BT_UUID_TEST_TX     BT_UUID_DECLARE_16(0xFFF2)


#define EV_BLE_TEST     0x0504
#define BLE_ADV_START   0x01
#define BLE_ADV_STOP    0x02
#define BLE_DEV_CONN    0x03
#define BLE_DEV_DISCONN 0x04
#define BLE_SCAN_START  0x05
#define BLE_SCAN_STOP   0x06

#define LED_RED_PIN         (17)
#define LED_GREEN_PIN       (14)
#define LED_BLUE_PIN        (11)


// BLE impl
static struct bt_conn *ble_bl_conn=NULL;
static bool notify_flag=false;
static void ble_bl_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t vblfue)
{
    if(vblfue == BT_GATT_CCC_NOTIFY) {
        
     printf("enable notify.\r\n");
     notify_flag=true;
  
    } else {
        printf("disable notify.\r\n");
        notify_flag=false;
    }

}
static void bl_connected(struct bt_conn *conn, uint8_t err)
{
    struct bt_le_conn_param param;
    param.interval_max=24;
    param.interval_min=24;
    param.latency=0;
    param.timeout=600;
    int update_err;
    if (err) {
        printf("Connection failed (err 0x%02x)\r\n", err);
    } else {
        printf("Connected\r\n");
        ble_bl_conn=conn;
        update_err = bt_conn_le_param_update(conn, &param);

        if (err) {
            printf("conn update failed (err %d)\r\n", err);
        } else {
            printf("conn update initiated\r\n");
        }
        aos_post_event(EV_BLE_TEST,BLE_DEV_CONN,NULL);
    }
}
static void bl_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printf("Disconnected (reason 0x%02x)\n", reason);
    aos_post_event(EV_BLE_TEST,BLE_DEV_DISCONN,NULL);
}


static int ble_blf_recv(struct bt_conn *conn,
              const struct bt_gatt_attr *attr, const void *buf,
              u16_t len, u16_t offset, u8_t flags)
{
    uint8_t *recv_buffer;
    recv_buffer=pvPortMalloc(sizeof(uint8_t)*len);
    memcpy(recv_buffer, buf, len);
    printf("ble rx=%d\n",len);
    for (size_t i = 0; i < len; i++)
    {
         printf("0x%x ",recv_buffer[i]);
    }
    vPortFree(recv_buffer);
    printf("\n");
    return 0;
}

// BLE config
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "BL_602", 6),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, "BL_602", 6),
};
static struct bt_conn_cb conn_callbacks = {
    .connected = bl_connected,
    .disconnected = bl_disconnected,
};
static struct bt_gatt_attr blattrs[]= {
    BT_GATT_PRIMARY_SERVICE(BT_UUID_TEST),

    BT_GATT_CHARACTERISTIC(BT_UUID_TEST_RX,
                            BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            NULL,
                            NULL,
                            NULL),

    BT_GATT_CCC(ble_bl_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_TEST_TX,
                            BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                            NULL,
                            ble_blf_recv,
                            NULL)
};
struct bt_gatt_service ble_bl_server = BT_GATT_SERVICE(blattrs);

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
    switch (event->code) {
        case BLE_ADV_START: {
            printf("ble adv start \r\n");
            bl_gpio_output_set(LED_BLUE_PIN,0);
            break;
        }

        case BLE_ADV_STOP: {
            printf("ble adv stop \r\n");
            bl_gpio_output_set(LED_BLUE_PIN,1);
            break;
        }

        default: {
            printf("%s: 0x%02x\r\n", __func__, event->code);
            break;
        }
    }
}

static void ble_app_init(int err)
{
    if (err != 0) {
        printf("BT FAILED started\n");
    }else{
        printf("BT SUCCESS started\n");

        bt_set_name("blf_602");
        printf("Bluetooth Advertising start\r\n");
        { int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
            if (err) {
                printf("Advertising failed to start (err %d)\r\n", err);
            } else {
                 aos_post_event(EV_BLE_TEST,BLE_ADV_START,NULL);
            }
        }
    }
   
}

static void proc_hello_entry(void *pvParameters)
{
    vfs_init();
    vfs_device_init();

    { uint32_t fdt = 0; uint32_t offset = 0;
        if (get_dts_addr("uart", &fdt, &offset) == 0) {
            vfs_uart_init(fdt, offset);
        }
    }

    aos_loop_init();
    aos_register_event_filter(EV_BLE_TEST, event_cb_ble_event, NULL);

    uart_init(1);

    // Initiblfize BLE controller
    ble_controller_init(configMAX_PRIORITIES - 1);
    // Initiblfize BLE Host stack
    hci_driver_init();
    bt_enable(ble_app_init);

    bt_conn_cb_register(&conn_callbacks);
    { int err = bt_gatt_service_register(&ble_bl_server);
        if (err==0) {
            printf("bt_gatt_service_register ok\n");
        } else {
            printf("bt_gatt_service_register err\n");
        }
    }

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

    bl_gpio_enable_output(LED_BLUE_PIN,1,0);
    bl_gpio_output_set(LED_BLUE_PIN,1);

    printf("[OS] Starting proc_hello_entry task...\r\n");
    xTaskCreateStatic(proc_hello_entry, (char*)"event_loop", 1024, NULL, 15, proc_hello_stack, &proc_hello_task);

    printf("[OS] Starting OS Scheduler...\r\n");
    vTaskStartScheduler();
}

