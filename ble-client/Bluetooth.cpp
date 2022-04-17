#include <memory>
#include <array>

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

#include "Bluetooth.hpp"

#include "AosLoop.hpp"

// BLE advertisement settings
namespace advertisement {

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

} // namespace advertisement

static struct bt_conn *ble_bl_conn = nullptr;
static bool notify_flag = false;

void bt_callbacks::initialized(int err)
{
    if (err) {
        printf("[BT] %s callback started with error code %d\r\n", __func__, err);
        return;
    }

    bt_commands::start_advertising();
}

void bt_callbacks::connected(bt_conn* conn, uint8_t err)
{
    if (err) {
        printf("[BT] %s callback started (error %d)\r\n", __func__, err);
        return;
    }

    printf("[BT] Connected\r\n");
    ble_bl_conn = conn;

    auto conn_param = bt_le_conn_param
        { .interval_min = 24
        , .interval_max = 24
        , .latency = 0
        , .timeout = 600
    };

    if (auto err = bt_conn_le_param_update(ble_bl_conn, &conn_param)) {
        printf("[BT] Failed to update connection parameters (error %d)\r\n", err);
        return;
    }

    aos_post_event(aos::bt_tag, (int)aos::BtEvent::Connect, 0);
}

void bt_callbacks::disconnected(bt_conn* conn, uint8_t reason)
{
    printf("[BT] Disconnected (reason 0x%02x)\r\n", reason);
    aos_post_event(aos::bt_tag, (int)aos::BtEvent::Disconnect, 0);
}

void bt_callbacks::ccc_configuration_changed(bt_gatt_attr const* attr, u16_t vblfue)
{
    if(vblfue == BT_GATT_CCC_NOTIFY) {
        printf("[BT] Enable notify.\r\n");
        notify_flag = true;
    } else {
        printf("[BT] Disable notify.\r\n");
        notify_flag = false;
    }
}

ssize_t bt_callbacks::data_requested(bt_conn* conn,
    bt_gatt_attr const* attr, void* raw_buf,
    uint16_t len, uint16_t offset)
{
    auto buf = (unsigned char*)raw_buf;

    static auto send_buf = []() {
        auto result = std::array<unsigned char, 128>{};
        for (int i = 0; i < result.size(); i++) {
            result[i] = (uint8_t)i;
        }
        return result;
    }();

    printf("[BT] Send: ");
    auto sent = ssize_t{0};
    for (size_t i = 0; i < len; i++) {
        if (offset + i >= send_buf.size()) {
            break;
        }
        buf[i] = send_buf[offset + i];
        printf("%02x", send_buf[offset + i]);
        sent++;
    }
    printf("\r\n");

    return sent;
}

ssize_t bt_callbacks::data_received(bt_conn *conn,
    bt_gatt_attr const* attr, void const* buf,
    uint16_t len, uint16_t offset, uint8_t flags)
{
    auto recv_buf = std::unique_ptr<uint8_t, decltype(&vPortFree)>
        { (uint8_t*)pvPortMalloc(sizeof(uint8_t) * len)
        , vPortFree
    };

    memcpy(recv_buf.get(), buf, len);

    printf("[BT] Receive: ");
    for (size_t i = 0; i < len; i++) {
         printf("%02x", recv_buf.get()[i]);
    }
    printf("\r\n");
    return len;
}

// BT commands

void bt_commands::start_advertising()
{
    printf("Bluetooth Advertising starting\r\n");

    bt_set_name("blf_602");

    if (auto const err = bt_le_adv_start(&advertisement::parameters,
        advertisement::data.begin(), advertisement::data.size(), nullptr, 0)) {
        printf("Advertising failed to start (error %d)\r\n", err);
        return;
    }

    aos_post_event(aos::bt_tag, (int)aos::BtEvent::Start, 0);
}

void bt_commands::stop_advertising()
{
    printf("Bluetooth Advertising stopping\r\n");

    if (auto const err = bt_le_adv_stop()) {
        printf("Advertising failed to stop (error %d)\r\n", err);
        return;
    }

    aos_post_event(aos::bt_tag, (int)aos::BtEvent::Stop, 0);
}
