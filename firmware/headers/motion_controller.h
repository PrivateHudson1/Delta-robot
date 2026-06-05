// firmware/headers/motion_controller.h
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>

class MotionController {
public:
    // Типы профилей движения
    enum ProfileType {
        PROFILE_TRAPEZOIDAL,  // Трапецеидальный профиль (классический)
        PROFILE_S_CURVE       // S-образный профиль (плавный, рекомендуется)
    };
    
    // Параметры движения
    struct MotionParams {
        float max_velocity;     // Максимальная скорость (мм/с)
        float max_acceleration; // Максимальное ускорение (мм/с²)
        float max_jerk;         // Максимальный рывок (мм/с³) - для S-кривой
        ProfileType type;       // Тип профиля
    };
    
    // Состояние движения
    struct MotionState {
        bool is_moving;         // Движется ли сейчас
        bool is_paused;         // На паузе ли
        float start_pos[3];     // Начальная позиция (X,Y,Z)
        float target_pos[3];    // Целевая позиция
        float current_pos[3];   // Текущая позиция
        float start_time;       // Время старта (секунды)
        float duration;         // Длительность движения
        float progress;         // Прогресс (0.0 до 1.0)
        MotionParams params;    // Параметры движения
    };
    
public:
    // Конструктор
    MotionController();
    
    // Инициализация с параметрами по умолчанию
    void begin(float max_vel = 500.0, float max_acc = 2000.0);
    
    // Планирование движения в целевую точку
    bool move_to(float target_x, float target_y, float target_z, 
                 MotionParams* params = nullptr);
    
    // Планирование относительного движения
    bool move_rel(float delta_x, float delta_y, float delta_z,
                  MotionParams* params = nullptr);
    
    // Обновление текущей позиции (вызывать в цикле управления)
    // Возвращает true, если движение завершено
    bool update(float delta_time_seconds);
    
    // Немедленная остановка движения
    void stop();
    
    // Пауза/возобновление движения
    void pause();
    void resume();
    
    // Геттеры
    bool is_moving() const { return _state.is_moving; }
    bool is_paused() const { return _state.is_paused; }
    float get_progress() const { return _state.progress; }
    void get_current_pos(float &x, float &y, float &z);
    void get_target_pos(float &x, float &y, float &z);
    
    // Расчет расстояния между двумя точками
    static float distance(float x1, float y1, float z1, 
                         float x2, float y2, float z2);
    
    // Установка параметров по умолчанию
    void set_default_params(float max_vel, float max_acc);
    
private:
    MotionState _state;
    MotionParams _default_params;
    
    // Реализации профилей траекторий
    float _compute_trapezoidal(float t, float total_time, float distance);
    float _compute_s_curve(float t, float total_time, float distance);
    
    // Расчет времени движения
    float _calculate_time(float distance, const MotionParams &params);
    
    // Линейная интерполяция позиции
    void _interpolate_position(float t, float result[3]);
};

#endif