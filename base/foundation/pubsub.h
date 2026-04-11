#ifndef PUBSUB_H
#define PUBSUB_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    // Scheduler bands
    SCHEDULER_1HZ,
    SCHEDULER_10HZ,
    SCHEDULER_25HZ,
    SCHEDULER_100HZ,
    SCHEDULER_1KHZ,

    // Sensor events
    SENSOR_BUS_VOLTAGE,

    // Motor control
    FOC_RELEASE,
    MOTOR_THROTTLE,

    // UART / CAN
    UART_RAW_RECEIVED,
    UART_TX_COMPLETE,
    UART_RAW_SEND,
    CAN_MESSAGE_RECEIVED,
    CAN_MESSAGE_SEND,

    // ADC
    ADC_INJECTED_COMPLETE,
    ADC_REGULAR_COMPLETE,

    // System
    NOTIFY_LOG_CLASS,
    SEND_LOG,

    TOPIC_NULL
} topic_t;

typedef void (*subscriber_callback_t)(uint8_t *data, size_t size);

void publish(topic_t topic, uint8_t *data, size_t size);
void subscribe(topic_t topic, subscriber_callback_t callback);

#endif // PUBSUB_H
