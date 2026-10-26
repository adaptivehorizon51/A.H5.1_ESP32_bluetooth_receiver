#include "input_manager.h"
#include "config.h"
#include <freertos/task.h>

namespace {
struct ButtonState {
  uint8_t pin;
  InputEvent event;
  bool isVolume;
  bool rawPressed;
  bool stablePressed;
  bool center;
  bool longSent;
  uint32_t rawChangedAt;
  uint32_t pressedAt;
  uint32_t lastRepeatAt;
};
static QueueHandle_t s_eventQueue = nullptr;
static ButtonState s_buttons[] = {
    {Horizon::kPinUp, InputEvent::Up, false, false, false, false, false, 0, 0, 0},
    {Horizon::kPinDown, InputEvent::Down, false, false, false, false, false, 0, 0, 0},
    {Horizon::kPinLeft, InputEvent::Left, false, false, false, false, false, 0, 0, 0},
    {Horizon::kPinRight, InputEvent::Right, false, false, false, false, false, 0, 0, 0},
    {Horizon::kPinCenter, InputEvent::Center, false, false, false, true, false, 0, 0, 0},
    {Horizon::kPinVolumeUp, InputEvent::VolumeUp, true, false, false, false, false, 0, 0, 0},
    {Horizon::kPinVolumeDown, InputEvent::VolumeDown, true, false, false, false, false, 0, 0, 0}};

void emit(InputEvent event) {
  if (s_eventQueue != nullptr) xQueueSend(s_eventQueue, &event, 0);
}

void inputTask(void*) {
  const uint32_t initial = millis();
  for (auto& button : s_buttons) {
    const bool pressed = digitalRead(button.pin) == LOW;
    button.rawPressed = pressed;
    button.stablePressed = pressed;
    button.rawChangedAt = initial;
    button.pressedAt = initial;
    button.lastRepeatAt = initial;
  }

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    const uint32_t now = millis();
    for (auto& button : s_buttons) {
      const bool rawPressed = digitalRead(button.pin) == LOW;
      if (rawPressed != button.rawPressed) {
        button.rawPressed = rawPressed;
        button.rawChangedAt = now;
      }
      if (button.rawPressed != button.stablePressed &&
          (now - button.rawChangedAt) >= Horizon::kButtonDebounceMs) {
        button.stablePressed = button.rawPressed;
        if (button.stablePressed) {
          button.pressedAt = now;
          button.lastRepeatAt = now;
          button.longSent = false;
          if (!button.center) emit(button.event);
        } else if (button.center && !button.longSent) {
          emit(InputEvent::Center);
        }
      }
      if (button.center && button.stablePressed && !button.longSent &&
          (now - button.pressedAt) >= Horizon::kCenterLongPressMs) {
        button.longSent = true;
        emit(InputEvent::CenterLong);
      }
      if (button.isVolume && button.stablePressed && (now - button.pressedAt) >= 350 &&
          (now - button.lastRepeatAt) >= 140) {
        button.lastRepeatAt = now;
        emit(button.event);
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(Horizon::kInputPollIntervalMs));
  }
}
}  // namespace

bool InputManager_Begin(QueueHandle_t eventQueue) {
  if (eventQueue == nullptr) return false;
  s_eventQueue = eventQueue;
  for (const auto& button : s_buttons) pinMode(button.pin, INPUT_PULLUP);
  return xTaskCreatePinnedToCore(inputTask, "InputTask", Horizon::kInputTaskStack,
                                 nullptr, 3, nullptr, 0) == pdPASS;
}
