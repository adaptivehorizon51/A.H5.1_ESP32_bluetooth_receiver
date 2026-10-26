#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Debounced GPIO polling cadence is configured in config.h.
bool InputManager_Begin(QueueHandle_t eventQueue);
