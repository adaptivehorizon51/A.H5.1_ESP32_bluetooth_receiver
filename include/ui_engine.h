#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Starts the Horizon OS v4 card UI on Core 0. Dashboard rendering is capped
// by kUiFramePeriodMs (about 30 FPS in the optimized configuration), uses
// per-segment redraw, and keeps the volume overlay non-blocking.
bool UiEngine_Begin(QueueHandle_t eventQueue);
