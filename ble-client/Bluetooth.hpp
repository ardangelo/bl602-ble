#pragma once

#include "bl602_bt.hpp"

// Interface

namespace bt_callbacks {

void initialized(int err);

void connected(bt_conn* conn, uint8_t err);

void disconnected(bt_conn* conn, uint8_t reason);

void ccc_configuration_changed(bt_gatt_attr const* attr, u16_t vblfue);

ssize_t data_requested(bt_conn* conn,
    bt_gatt_attr const* attr, void* buf,
    uint16_t len, uint16_t offset);

ssize_t data_received(bt_conn* conn,
    bt_gatt_attr const* attr, void const* buf,
    uint16_t len, uint16_t offset, uint8_t flags);

} // namespace bt_callbacks

namespace bt_commands {

void start_advertising();

void stop_advertising();

} // namespace bt_commands
