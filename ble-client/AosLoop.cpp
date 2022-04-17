#include <stdio.h>

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

#include "led.hpp"
#include "Bluetooth.hpp"

#include "AosLoop.hpp"

void aos::bt_event_received(input_event_t* event, void* private_data)
{
    switch ((BtEvent)event->code) {
        case BtEvent::Start: {
            printf("[AOS] BLE Adv. start\r\n");
            led::off();
            led::blue_on();
            break;
        }

        case BtEvent::Stop: {
            printf("[AOS] BLE Adv. stop\r\n");
            led::off();
            break;
        }

        case BtEvent::Connect: {
            printf("[AOS] BLE Connected\r\n");
            led::off();
            led::green_on();
            break;
        }

        case BtEvent::Disconnect: {
            printf("[AOS] BLE Disconnected\r\n");
            led::off();

            vTaskDelay(5000);
            bt_commands::start_advertising();
            break;
        }

        default: {
            printf("%s: 0x%02x\r\n", __func__, event->code);
            break;
        }
    }
}
