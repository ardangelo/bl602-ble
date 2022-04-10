#include <stdio.h>
#include <string.h>

#include <memory>
#include <array>
#include <tuple>
#include <type_traits>

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

// Bouffalo device
extern "C" {
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
} // extern "C"

// HAL
extern "C" {
#include <hal_sys.h>
#include <hal_boot2.h>
#include <hal_board.h>
#include <hal_uart.h>
} // extern "C"

// VFS
#include <vfs.h>

// Flattened Device Tree
extern "C" {
#include <fdt.h>
#include <libfdt.h>
} // extern "C"

// AliOS Things
#include <aos/kernel.h>
#include <aos/yloop.h>

#include <event_device.h>

#include "bluetooth.hpp"

// C declarations
extern "C" {

// Global heap
extern uint8_t _heap_start;
extern uint8_t _heap_size;

extern void uart_init(uint8_t uartid);

void bfl_main(void);

} // extern "C"

#define NAME_LEN        30
#define CHAR_SIZE_MAX       512

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

// BLE advertisement

namespace bt_advertisement {

static auto flags = std::array
    { bt_le_ad::general | bt_le_ad::no_bredr
};
static auto data = std::array
    { make_bt_data(bt_eir::flags, flags)
    , make_bt_data(bt_eir::name_complete, "BL_602", 6)
    , make_bt_data(bt_eir::manufacturer_data, "MANUFACTURER", 13)
};

static auto parameters = bt_le_adv::make_param(
    bt_le_adv::connectable | bt_le_adv::use_name,
    bt_gap::adv_fast_int_min_2, bt_gap::adv_fast_int_max_2);

} // namespace bt_advertisement

// BT callbacks
namespace bt_callbacks {

static struct bt_conn *ble_bl_conn = nullptr;
static bool notify_flag = false;

static void initialized(int err)
{
    if (err) {
        printf("%s callback started with error code %d\r\n", __func__, err);
        return;
    }

    printf("Bluetooth Advertising starting\r\n");

    bt_set_name("blf_602");

    if (auto const err = bt_le_adv_start(&bt_advertisement::parameters,
        bt_advertisement::data.begin(), bt_advertisement::data.size(), nullptr, 0)) {
        printf("Advertising failed to start (err %d)\r\n", err);
        return;
    }

    aos_post_event(EV_BLE_TEST, BLE_ADV_START, 0);
}

static void connected(bt_conn* conn, uint8_t err)
{
    if (err) {
        printf("%s callback started (error %d)\r\n", __func__, err);
        return;
    }

    printf("Connected\r\n");
    ble_bl_conn = conn;

    auto conn_param = bt_le_conn_param
        { .interval_min = 24
        , .interval_max = 24
        , .latency = 0
        , .timeout = 600
    };

    if (auto err = bt_conn_le_param_update(ble_bl_conn, &conn_param)) {
        printf("Failed to update connection parameters (error %d)\r\n", err);
        return;
    }

    aos_post_event(EV_BLE_TEST, BLE_DEV_CONN, 0);
}
static void disconnected(bt_conn* conn, uint8_t reason)
{
    printf("Disconnected (reason 0x%02x)\r\n", reason);
    aos_post_event(EV_BLE_TEST, BLE_DEV_DISCONN, 0);
}

static void ccc_configuration_changed(bt_gatt_attr const* attr, u16_t vblfue)
{
    if(vblfue == BT_GATT_CCC_NOTIFY) {
        printf("enable notify.\r\n");
        notify_flag = true;
    } else {
        printf("disable notify.\r\n");
        notify_flag = false;
    }
}

static int data_received(bt_conn *conn,
    bt_gatt_attr const* attr, void const* buf,
    uint16_t len, uint16_t offset, uint8_t flags)
{
    auto recv_buf = std::unique_ptr<uint8_t, decltype(&vPortFree)>
        { (uint8_t*)pvPortMalloc(sizeof(uint8_t) * len)
        , vPortFree
    };

    memcpy(recv_buf.get(), buf, len);

    for (size_t i = 0; i < len; i++) {
         printf("%02x", recv_buf.get()[i]);
    }
    printf("\r\n");
    return 0;
}

} // namespace bt_callbacks

// AOS callbacks

namespace aos_callbacks {

static void event_received(input_event_t* event, void* private_data)
{
    switch (event->code) {
        case BLE_ADV_START: {
            printf("ble adv start \r\n");
            bl_gpio_output_set(LED_BLUE_PIN, 0);
            break;
        }

        case BLE_ADV_STOP: {
            printf("ble adv stop \r\n");
            bl_gpio_output_set(LED_BLUE_PIN, 1);
            break;
        }

        default: {
            printf("%s: 0x%02x\r\n", __func__, event->code);
            break;
        }
    }
}

} // namespace aos_callbacks

namespace globals {

// BT connection callback registry
static auto bt_connection_callbacks = bt_conn_cb
    { .connected = bt_callbacks::connected
    , .disconnected = bt_callbacks::disconnected
    , .le_param_req = nullptr
    , .le_param_updated = nullptr
    , .identity_resolved = nullptr
    , .security_changed = nullptr
    , ._next = nullptr
};

// Global heap
static auto heap_regions = std::array
    { HeapRegion_t { &_heap_start,  (unsigned int) &_heap_size}
    , HeapRegion_t { nullptr, 0 }
};

} // namespace globals

// BLE GATT services
namespace gatt_services {

// Test service attribute
static auto test_service_uuid = uuid::make_16(0xfff0);
static auto test_service_attr = gatt::make_primary_service_attribute(test_service_uuid);

// Test RX characteristic attribute
static auto test_rx_uuid = uuid::make_16(0xfff1);
static auto test_rx_char = gatt::make_characteristic(test_rx_uuid,
    0x0, gatt::char_prop::notify);
static auto test_rx_char_attr = gatt::make_characteristic_attribute(nullptr, nullptr,
    test_rx_char, gatt::perm::read);

// CCC characteristic attribute
static auto ccc = gatt::make_ccc(bt_callbacks::ccc_configuration_changed, nullptr, nullptr);
static auto ccc_attr = gatt::make_ccc_attribute(ccc, gatt::perm::read | gatt::perm::write);

// Test TX characteristic attribute
static auto test_tx_uuid = uuid::make_16(0xfff2);
static auto test_tx_char = gatt::make_characteristic(test_rx_uuid,
    0x0, gatt::char_prop::write_without_resp);
static auto test_tx_char_attr = gatt::make_characteristic_attribute(nullptr, bt_callbacks::data_received,
    test_tx_char, gatt::perm::read | gatt::perm::write);

// Test service
static auto test_service_attributes = std::array
    { test_service_attr
    , test_rx_char_attr
    , ccc_attr
    , test_tx_char_attr
};
static auto test_service = bt_gatt_service
    { .attrs = test_service_attributes.begin()
    , .attr_count = test_service_attributes.size()
    , .node = {}
};

} // gatt_services

// Helpers

static auto get_dts_addr(const char *name)
{
    auto addr = hal_board_get_factory_addr();
    auto offset = fdt_subnode_offset((void const*)addr, 0, name);

    if (offset == 0) {
        printf("FDT offset lookup failed for '%s'.\r\n", name);
        return std::make_tuple(false, 0UL, 0);
    }

    return std::make_tuple(true, addr, offset);
}

static auto proc_hello_stack = std::array<StackType_t, 1024>{};
static auto proc_hello_task = StaticTask_t{};
static void proc_hello_entry(void *pvParameters)
{
    // Initialize VFS
    vfs_init();
    vfs_device_init();
    { auto [success, fdt, offset] = get_dts_addr("uart");
        if (success) {
            vfs_uart_init(fdt, offset);
        }
    }

    // Initialize AOS and register event callback
    aos_loop_init();
    aos_register_event_filter(EV_BLE_TEST, aos_callbacks::event_received, 0);

    // Initialize UART
    uart_init(1);

    // Initialize BLE
    ble_controller_init(configMAX_PRIORITIES - 1);
    hci_driver_init();

    // Register initialization and connection callbacks
    bt_enable(bt_callbacks::initialized);
    bt_conn_cb_register(&globals::bt_connection_callbacks);

    // Register test service
    if (auto err = bt_gatt_service_register(&gatt_services::test_service)) {
        printf("Registering services failed with code %d\r\n", err);
        goto cleanup_proc_hello_entry;
    }

    // Run AOS task loop
    printf("Registering services succeeded, running task loop\r\n");
    aos_loop_run();

cleanup_proc_hello_entry:
    vTaskDelete(nullptr);
}

void bfl_main(void)
{
    bl_sys_early_init();

    // Init UART using pins 16+7 (TX+RX) and baudrate of 2M
    bl_uart_init(0, 16, 7, 255, 255, 2 * 1000 * 1000);
    bl_sys_init();

    vPortDefineHeapRegions(globals::heap_regions.begin());

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
    xTaskCreateStatic(proc_hello_entry, (char*)"event_loop", proc_hello_stack.size(), nullptr, 15, proc_hello_stack.begin(), &proc_hello_task);

    printf("[OS] Starting OS Scheduler...\r\n");
    vTaskStartScheduler();
}
