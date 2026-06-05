// firmware/headers/safety_watchdog.h
#ifndef SAFETY_WATCHDOG_H
#define SAFETY_WATCHDOG_H

#include <Arduino.h>

class SafetyWatchdog {
public:
    // Типы аварийных событий
    enum SafetyEvent {
        EVENT_NONE = 0,
        EVENT_OVERCURRENT = 1,      // Превышение тока мотора
        EVENT_OVERTEMP = 2,          // Перегрев мотора
        EVENT_POSITION_ERROR = 3,    // Ошибка позиционирования
        EVENT_EMERGENCY_STOP = 4,    // Аварийная кнопка
        EVENT_LIMIT_SWITCH = 5,      // Сработал концевой выключатель
        EVENT_COMMUNICATION_LOSS = 6, // Потеря связи с ПК
        EVENT_POWER_FAULT = 7        // Сбой питания
    };
    
    // Лимиты безопасности для одного мотора
    struct MotorLimits {
        int max_current_ma;      // Максимальный ток (мА)
        int max_temperature_c;    // Максимальная температура (°C)
        float max_position_error; // Максимальная ошибка позиции (градусы)
    };
    
public:
    // Конструктор
    SafetyWatchdog();
    
    // Инициализация с пином аварийной кнопки
    void begin(int estop_pin);
    
    // Настройка лимитов для каждого мотора
    void set_motor_limits(int motor_id, int max_current, int max_temp, float max_error);
    
    // Проверка состояния мотора (вызывать в цикле)
    SafetyEvent check_motor(int motor_id, int current_ma, int temp_c, float pos_error);
    
    // Проверка аппаратной аварийной кнопки
    SafetyEvent check_emergency_stop();
    
    // Проверка связи с ПК (heartbeat)
    SafetyEvent check_communication(unsigned long last_heartbeat_ms, unsigned long timeout_ms);
    
    // Активация аварийной остановки
    void trigger_emergency_stop(SafetyEvent event);
    
    // Сброс аварийного состояния (после устранения причины)
    void reset();
    
    // Проверка, активна ли аварийная остановка
    bool is_emergency_stop_active() const { return _emergency_active; }
    
    // Получение последнего события
    SafetyEvent get_last_event() const { return _last_event; }
    
    // Получение текстового описания события
    String get_event_description(SafetyEvent event);
    
    // Включение/выключение звукового сигнала
    void set_buzzer_pin(int pin);
    void sound_alarm(bool enable);
    
private:
    MotorLimits _limits[3];     // Лимиты для 3 моторов
    SafetyEvent _last_event;     // Последнее событие
    bool _emergency_active;      // Флаг аварии
    int _estop_pin;              // Пин аварийной кнопки
    int _buzzer_pin;             // Пин зуммера
    bool _buzzer_enabled;        // Включен ли зуммер
    unsigned long _alarm_start;  // Время начала сигнала
};

#endif