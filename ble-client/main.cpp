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
#include <event_device.h>
} // extern "C"

#include "led.hpp"
#include "Bluetooth.hpp"
#include "AosLoop.hpp"

// C declarations
extern "C" {

// Global heap
extern uint8_t _heap_start;
extern uint8_t _heap_size;

extern void uart_init(uint8_t uartid);

void bfl_main(void);

} // extern "C"

// Helper classes

template <size_t StackSize>
class RtosTask
{
private: // member variables
    std::array<StackType_t, StackSize> m_stack;
    StaticTask_t m_task;

public: // interface
    RtosTask()
        : m_stack{}
        , m_task{}
    {}

    void create_static(char const* task_name, TaskFunction_t entry_fn)
    {
        auto const priority = 15;
        xTaskCreateStatic(entry_fn, task_name, m_stack.size(), nullptr, priority,
            m_stack.begin(), &m_task);
    }
};

// Helper functions

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

namespace globals {

// Global heap
static auto heap_regions = std::array
    { HeapRegion_t { &_heap_start, (unsigned int)&_heap_size}
    , HeapRegion_t { nullptr, 0 }
};

// RTOS Tasks
static auto ble_handler_task = RtosTask<1024>{};

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
static auto test_rx_attr = gatt::make_attribute(test_rx_uuid,
    bt_callbacks::data_requested, nullptr, nullptr, 0x0, gatt::perm::read);

// CCC characteristic attribute
static auto ccc = gatt::make_ccc(bt_callbacks::ccc_configuration_changed, nullptr, nullptr);
static auto ccc_attr = gatt::make_ccc_attribute(ccc, gatt::perm::read | gatt::perm::write);

// Test TX characteristic attribute
static auto test_tx_uuid = uuid::make_16(0xfff2);
static auto test_tx_char = gatt::make_characteristic(test_tx_uuid,
    0x0, gatt::char_prop::write_without_resp);
static auto test_tx_char_attr = gatt::make_characteristic_attribute(nullptr, bt_callbacks::data_received,
    test_tx_char, gatt::perm::read | gatt::perm::write);
static auto test_tx_attr = gatt::make_attribute(test_tx_uuid,
    nullptr, bt_callbacks::data_received, nullptr, 0x0, gatt::perm::read | gatt::perm::write);

// Test service
static auto test_service_attributes = std::array
    { test_service_attr
    , test_rx_char_attr
    , test_rx_attr
    , ccc_attr
    , test_tx_char_attr
    , test_tx_attr
};
static auto test_service = bt_gatt_service
    { .attrs = test_service_attributes.begin()
    , .attr_count = test_service_attributes.size()
    , .node = {}
};

} // gatt_services

static void ble_handler_entry(void *pvParameters)
{
    // Initialize VFS
    vfs_init();
    vfs_device_init();

    // Initialize AOS and register event callback
    aos_loop_init();
    aos_register_event_filter(aos::bt_tag, aos::bt_event_received, 0);

    // Initialize UART
    { auto [success, fdt, offset] = get_dts_addr("uart");
        if (success) {
            vfs_uart_init(fdt, offset);
        } else {
            printf("Failed to get DTS address for UART\r\n");
            goto cleanup_ble_handler_entry;
        }
    }
    uart_init(1);

    // Initialize BLE
    ble_controller_init(configMAX_PRIORITIES - 1);
    hci_driver_init();

    // BT connection callbacks
    { auto bt_connection_callbacks = bt_conn_cb
            { .connected = bt_callbacks::connected
            , .disconnected = bt_callbacks::disconnected
            , .le_param_req = nullptr
            , .le_param_updated = nullptr
            , .identity_resolved = nullptr
            , .security_changed = nullptr
            , ._next = nullptr
        };

        // Register initialization and connection callbacks
        bt_enable(bt_callbacks::initialized);
        bt_conn_cb_register(&bt_connection_callbacks);
    }

    // Register test service
    if (auto err = bt_gatt_service_register(&gatt_services::test_service)) {
        printf("Registering services failed with code %d\r\n", err);
        goto cleanup_ble_handler_entry;
    }

    // Run AOS task loop
    printf("Registering services succeeded, running task loop\r\n");
    aos_loop_run();

cleanup_ble_handler_entry:
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

    led::init();

    printf("[OS] Starting ble_handler_task task...\r\n");
    globals::ble_handler_task.create_static("ble_handler_task", ble_handler_entry);

    printf("[OS] Starting OS Scheduler...\r\n");
    vTaskStartScheduler();
}
