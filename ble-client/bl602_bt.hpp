#pragma once

#include <array>
#include <type_traits>

// BT system
extern "C" {
#include "bluetooth.h"
#include "conn.h"
#include "gatt.h"
#include "uuid.h"
#include "hci_core.h"
#include "hci_driver.h"
#include "oad_api.h"
#include "ble_lib_api.h"
} // extern "C"

// Bluetooth defines

namespace uuid
{
constexpr auto make_16(uint16_t val)
{
    return bt_uuid_16
        { .uuid = { .type = BT_UUID_TYPE_16 }
        , .val = val
    };
}

constexpr auto gatt_primary = make_16(0x2800);
constexpr auto gatt_characteristic = make_16(0x2802);
constexpr auto gatt_ccc = make_16(0x2902);
}

namespace gatt {

namespace perm {
    static constexpr auto none = 0x0;
    static constexpr auto read = 0x1;
    static constexpr auto write = 0x2;
    static constexpr auto read_encrypt = 0x4;
    static constexpr auto write_encrypt = 0x8;
    static constexpr auto read_authen = 0x10;
    static constexpr auto write_authen = 0x20;
    static constexpr auto prepare_write = 0x40;
} // namespace perm

namespace char_prop {
    static constexpr auto broadcast = 0x1;
    static constexpr auto read = 0x2;
    static constexpr auto write_without_resp = 0x4;
    static constexpr auto write = 0x8;
    static constexpr auto notify = 0x10;
    static constexpr auto indicate = 0x20;
    static constexpr auto auth = 0x40;
    static constexpr auto ext_prop = 0x80;
} // namespace char_prop

template <typename Uuid>
static constexpr auto make_attribute(Uuid& uuid,
    decltype(bt_gatt_attr::read) read_callback,
    decltype(bt_gatt_attr::write) write_callback,
    void* user_data, uint16_t handle, uint8_t perm)
{
    return bt_gatt_attr
        { .uuid = (bt_uuid const*)&uuid
        , .read = read_callback
        , .write = write_callback
        , .user_data = user_data
        , .handle = handle
        , .perm = perm
    };
}

template <typename Uuid, typename UserData>
static constexpr auto make_attribute(Uuid& uuid,
    decltype(bt_gatt_attr::read) read_callback,
    decltype(bt_gatt_attr::write) write_callback,
    UserData &user_data, uint16_t handle, uint8_t perm)
{
    return make_attribute(uuid, read_callback, write_callback, (void*)&user_data, handle, perm);
}

template <typename Uuid>
static constexpr auto make_primary_service_attribute(Uuid& uuid)
{
    return make_attribute(
        uuid::gatt_primary, // uuid
        bt_gatt_attr_read_service, // read callback
        nullptr, // write callback
        uuid, // user data
        0x0, // handle
        gatt::perm::read); // permissions
}

template <typename Uuid>
static constexpr auto make_characteristic(Uuid& uuid, uint16_t value_handle,
    uint8_t properties)
{
    return bt_gatt_chrc
        { .uuid = (bt_uuid const*)&uuid
        , .value_handle = value_handle
        , .properties = properties
    };
}

static constexpr auto make_characteristic_attribute(
    decltype(bt_gatt_attr::read) read_callback,
    decltype(bt_gatt_attr::write) write_callback,
    bt_gatt_chrc& characteristic_attribute,
    uint8_t perm)
{
    return make_attribute(
        uuid::gatt_characteristic, // uuid
        bt_gatt_attr_read_chrc, // read callback
        nullptr, // write callback
        characteristic_attribute, // user data
        0x0, // handle
        gatt::perm::read); // permissions
}

static constexpr auto make_ccc(
    decltype(_bt_gatt_ccc::cfg_changed) changed_callback,
    decltype(_bt_gatt_ccc::cfg_write) write_callback,
    decltype(_bt_gatt_ccc::cfg_match) match_callback)
{
    return _bt_gatt_ccc
        { .cfg = {}
        , .value = 0x0
        , .cfg_changed = changed_callback
        , .cfg_write = write_callback
        , .cfg_match = match_callback
    };
}

static constexpr auto make_ccc_attribute(_bt_gatt_ccc& ccc, uint8_t perm)
{
    return make_attribute(
        uuid::gatt_ccc, // uuid
        bt_gatt_attr_read_ccc, // read callback
        bt_gatt_attr_write_ccc, // write callback
        ccc, // user data
        0x0, // handle
        perm); // permissions
}

} // namespace gatt

// components/network/ble/blestack/src/include/bluetooth/gap.h
namespace bt_eir {

static constexpr auto flags                = 0x01;
static constexpr auto uuid16_some          = 0x02;
static constexpr auto uuid16_all           = 0x03;
static constexpr auto uuid32_some          = 0x04;
static constexpr auto uuid32_all           = 0x05;
static constexpr auto uuid128_some         = 0x06;
static constexpr auto uuid128_all          = 0x07;
static constexpr auto name_shortened       = 0x08;
static constexpr auto name_complete        = 0x09;
static constexpr auto tx_power             = 0x0a;
static constexpr auto sm_tk_value          = 0x10;
static constexpr auto sm_oob_flags         = 0x11;
static constexpr auto solicit16            = 0x14;
static constexpr auto solicit128           = 0x15;
static constexpr auto svc_data16           = 0x16;
static constexpr auto gap_appearance       = 0x19;
static constexpr auto le_bt_device_address = 0x1b;
static constexpr auto le_role              = 0x1c;
static constexpr auto solicit32            = 0x1f;
static constexpr auto svc_data32           = 0x20;
static constexpr auto svc_data128          = 0x21;
static constexpr auto le_sc_confirm_value  = 0x22;
static constexpr auto le_sc_random_value   = 0x23;
static constexpr auto uri                  = 0x24;
static constexpr auto mesh_prov            = 0x29;
static constexpr auto mesh_message         = 0x2a;
static constexpr auto mesh_beacon          = 0x2b;

static constexpr auto manufacturer_data    = 0xff;

} // namespace bt_eir

namespace bt_le_ad {

static constexpr auto limited  = 0x1;
static constexpr auto general  = 0x2;
static constexpr auto no_bredr = 0x4;

} // namespace bt_le_ad

template <typename T, size_t N>
static constexpr auto make_bt_data(uint8_t eir_type, std::array<T, N> const& data)
{
    static_assert(sizeof(T) * N < std::numeric_limits<uint8_t>::max());
    return bt_data
        { .type = eir_type
        , .data_len = sizeof(T) * N
        , .data = (uint8_t const*)data.begin()
    };
}

static constexpr auto make_bt_data(uint8_t eir_type, char const* str, uint8_t len)
{
    return bt_data
        { .type = eir_type
        , .data_len = len
        , .data = (uint8_t const*)str
    };
}

// components/network/ble/blestack/src/include/bluetooth/gap.h
namespace bt_gap {
static constexpr auto scan_fast_interval= 0x0060;
static constexpr auto scan_fast_window = 0x0030;
static constexpr auto scan_fast_interval_1= 0x0800;
static constexpr auto scan_fast_window_1 = 0x0012;
static constexpr auto scan_fast_interval_2 = 0x1000;
static constexpr auto scan_fast_window_2 = 0x0012;

static constexpr auto adv_fast_int_min_1 = 0x0030;
static constexpr auto adv_fast_int_max_1 = 0x0060;
static constexpr auto adv_fast_int_min_2 = 0x00a0;
static constexpr auto adv_fast_int_max_2 = 0x00f0;

static constexpr auto adv_slow_int_min = 0x0640;
static constexpr auto adv_slow_int_max = 0x0780;
static constexpr auto init_conn_int_min = 0x0018;
static constexpr auto init_conn_int_max = 0x0028;
} // namespace bt_gap

// components/network/ble/blestack/src/include/bluetooth/bluetooth.h
namespace bt_le_adv {
static constexpr auto none = 0x0;
static constexpr auto connectable = 0x1;
static constexpr auto one_time = 0x2;
static constexpr auto use_identity = 0x4;
static constexpr auto use_name = 0x8;
static constexpr auto dir_mode_low_duty = 0x10;
static constexpr auto dir_addr_rpa = 0x20;
static constexpr auto filter_scan_req = 0x40;
static constexpr auto filter_conn = 0x80;

static constexpr auto make_param(uint8_t options,
    uint16_t interval_min, uint16_t interval_max)
{
    return bt_le_adv_param
        { .id = 0x0
        , .options = options
        , .interval_min = interval_min
        , .interval_max = interval_max
    };
}

} // namespace ble_adv
