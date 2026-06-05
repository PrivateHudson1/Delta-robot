// firmware/main/safety_watchdog.cpp
#include "safety_watchdog.h"

// Конструктор
SafetyWatchdog::SafetyWatchdog() {
    _last_event = EVENT_NONE;
    _emergency_active = false;
    _estop_pin = -1;
    _buzzer_pin = -1;
    _buzzer_enabled = false;
    _alarm_start = 0;
    
    // Устанавливаем безопасные лимиты по умолчанию
    for (int i = 0; i < 3; i++) {
        _limits[i].max_current_ma = 2500;     // 2.5A
        _limits[i].max_temperature_c = 85;    // 85°C
        _limits[i].max_position_error = 5.0;  // 5 градусов
    }
}

// Инициализация
void SafetyWatchdog::begin(int estop_pin) {
    _estop_pin = estop_pin;
    pinMode(_estop_pin, INPUT_PULLUP);
    
    // Сбрасываем состояние при запуске
    _emergency_active = false;
    _last_event = EVENT_NONE;
}

// Настройка лимитов мотора
void SafetyWatchdog::set_motor_limits(int motor_id, int max_current, int max_temp, float max_error) {
    if (motor_id >= 0 && motor_id < 3) {
        _limits[motor_id].max_current_ma = max_current;
        _limits[motor_id].max_temperature_c = max_temp;
        _limits[motor_id].max_position_error = max_error;
    }
}

// Проверка состояния мотора
SafetyWatchdog::SafetyEvent SafetyWatchdog::check_motor(int motor_id, int current_ma, 
                                                         int temp_c, float pos_error) {
    if (_emergency_active) return _last_event;
    
    // Проверка тока
    if (current_ma > _limits[motor_id].max_current_ma) {
        trigger_emergency_stop(EVENT_OVERCURRENT);
        return EVENT_OVERCURRENT;
    }
    
    // Проверка температуры
    if (temp_c > _limits[motor_id].max_temperature_c) {
        trigger_emergency_stop(EVENT_OVERTEMP);
        return EVENT_OVERTEMP;
    }
    
    // Проверка ошибки позиции
    if (fabs(pos_error) > _limits[motor_id].max_position_error) {
        trigger_emergency_stop(EVENT_POSITION_ERROR);
        return EVENT_POSITION_ERROR;
    }
    
    return EVENT_NONE;
}

// Проверка аварийной кнопки
SafetyWatchdog::SafetyEvent SafetyWatchdog::check_emergency_stop() {
    if (_estop_pin == -1) return EVENT_NONE;
    
    // Кнопка замыкает на GND (LOW при нажатии)
    if (digitalRead(_estop_pin) == LOW && !_emergency_active) {
        trigger_emergency_stop(EVENT_EMERGENCY_STOP);
        return EVENT_EMERGENCY_STOP;
    }
    
    return EVENT_NONE;
}

// Проверка связи с ПК
SafetyWatchdog::SafetyEvent SafetyWatchdog::check_communication(unsigned long last_heartbeat_ms, 
                                                                  unsigned long timeout_ms) {
    if (_emergency_active) return _last_event;
    
    unsigned long now = millis();
    if (now - last_heartbeat_ms > timeout_ms) {
        trigger_emergency_stop(EVENT_COMMUNICATION_LOSS);
        return EVENT_COMMUNICATION_LOSS;
    }
    
    return EVENT_NONE;
}

// Активация аварийной остановки
void SafetyWatchdog::trigger_emergency_stop(SafetyEvent event) {
    if (_emergency_active) return;  // Уже в аварии
    
    _emergency_active = true;
    _last_event = event;
    
    // Включаем звуковой сигнал
    sound_alarm(true);
    
    // Выводим сообщение в Serial
    Serial.print("EMERGENCY STOP! Event: ");
    Serial.println(get_event_description(event));
}

// Сброс аварийного состояния
void SafetyWatchdog::reset() {
    // Проверяем, что аварийная кнопка не нажата
    if (digitalRead(_estop_pin) == LOW) {
        Serial.println("Cannot reset: E-stop still pressed");
        return;
    }
    
    _emergency_active = false;
    _last_event = EVENT_NONE;
    sound_alarm(false);
    Serial.println("Safety system reset - ready to operate");
}

// Получение описания события
String SafetyWatchdog::get_event_description(SafetyEvent event) {
    switch (event) {
        case EVENT_NONE:
            return "No error";
        case EVENT_OVERCURRENT:
            return "Motor overcurrent detected";
        case EVENT_OVERTEMP:
            return "Motor overtemperature detected";
        case EVENT_POSITION_ERROR:
            return "Excessive position error";
        case EVENT_EMERGENCY_STOP:
            return "Emergency stop button pressed";
        case EVENT_LIMIT_SWITCH:
            return "Limit switch triggered";
        case EVENT_COMMUNICATION_LOSS:
            return "PC communication lost";
        case EVENT_POWER_FAULT:
            return "Power supply fault";
        default:
            return "Unknown error";
    }
}

// Установка пина зуммера
void SafetyWatchdog::set_buzzer_pin(int pin) {
    _buzzer_pin = pin;
    pinMode(_buzzer_pin, OUTPUT);
    digitalWrite(_buzzer_pin, LOW);
}

// Управление звуковым сигналом
void SafetyWatchdog::sound_alarm(bool enable) {
    if (_buzzer_pin == -1) return;
    
    _buzzer_enabled = enable;
    if (enable) {
        _alarm_start = millis();
        digitalWrite(_buzzer_pin, HIGH);
    } else {
        digitalWrite(_buzzer_pin, LOW);
    }
}