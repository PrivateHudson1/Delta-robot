// firmware/main/motion_controller.cpp
#include "motion_controller.h"
#include <math.h>

// #define PI 3.14159265358979323846

// Конструктор
MotionController::MotionController() {
    _state.is_moving = false;
    _state.is_paused = false;
    _state.progress = 0.0;
    
    for (int i = 0; i < 3; i++) {
        _state.start_pos[i] = 0;
        _state.target_pos[i] = 0;
        _state.current_pos[i] = 0;
    }
}

// Инициализация
void MotionController::begin(float max_vel, float max_acc) {
    _default_params.max_velocity = max_vel;
    _default_params.max_acceleration = max_acc;
    _default_params.max_jerk = max_acc * 10;  // Типичное ограничение рывка
    _default_params.type = PROFILE_S_CURVE;   // S-образный профиль для плавности
}

// Планирование движения к цели
bool MotionController::move_to(float target_x, float target_y, float target_z, 
                                MotionParams* params) {
    // Останавливаем текущее движение
    if (_state.is_moving) stop();
    
    // Устанавливаем целевую позицию
    _state.target_pos[0] = target_x;
    _state.target_pos[1] = target_y;
    _state.target_pos[2] = target_z;
    
    // Сохраняем начальную позицию
    for (int i = 0; i < 3; i++) {
        _state.start_pos[i] = _state.current_pos[i];
    }
    
    // Рассчитываем расстояние
    float dist = distance(_state.start_pos[0], _state.start_pos[1], _state.start_pos[2],
                          target_x, target_y, target_z);
    
    // Если уже на месте
    if (dist < 0.01) {
        _state.is_moving = false;
        return true;
    }
    
    // Выбираем параметры движения
    if (params != nullptr) {
        _state.params = *params;
    } else {
        _state.params = _default_params;
    }
    
    // Рассчитываем время движения
    _state.duration = _calculate_time(dist, _state.params);
    _state.start_time = micros() / 1000000.0;
    _state.progress = 0.0;
    _state.is_moving = true;
    _state.is_paused = false;
    
    return true;
}

// Относительное движение
bool MotionController::move_rel(float delta_x, float delta_y, float delta_z,
                                 MotionParams* params) {
    return move_to(_state.current_pos[0] + delta_x,
                   _state.current_pos[1] + delta_y,
                   _state.current_pos[2] + delta_z,
                   params);
}

// Обновление движения (вызывать в цикле управления)
bool MotionController::update(float delta_time_seconds) {
    if (!_state.is_moving || _state.is_paused) return false;
    
    float current_time = micros() / 1000000.0;
    float elapsed = current_time - _state.start_time;
    
    // Проверяем, завершилось ли движение
    if (elapsed >= _state.duration) {
        // Движение закончено
        for (int i = 0; i < 3; i++) {
            _state.current_pos[i] = _state.target_pos[i];
        }
        _state.is_moving = false;
        _state.progress = 1.0;
        return true;
    }
    
    // Нормализованное время (0 до 1)
    float t = elapsed / _state.duration;
    _state.progress = t;
    
    // Рассчитываем расстояние
    float dist = distance(_state.start_pos[0], _state.start_pos[1], _state.start_pos[2],
                          _state.target_pos[0], _state.target_pos[1], _state.target_pos[2]);
    
    // Вычисляем пройденное расстояние в зависимости от профиля
    float s;
    switch (_state.params.type) {
        case PROFILE_TRAPEZOIDAL:
            s = _compute_trapezoidal(t, _state.duration, dist) / dist;
            break;
        case PROFILE_S_CURVE:
        default:
            s = _compute_s_curve(t, _state.duration, dist) / dist;
            break;
    }
    
    // Линейная интерполяция позиции
    _interpolate_position(s, _state.current_pos);
    
    return false;
}

// Трапецеидальный профиль скорости
float MotionController::_compute_trapezoidal(float t, float total_time, float distance) {
    float v_max = _state.params.max_velocity;
    float a = _state.params.max_acceleration;
    
    // Время разгона
    float t_a = v_max / a;
    float d_a = 0.5 * a * t_a * t_a;
    
    if (2 * d_a >= distance) {
        // Треугольный профиль (не достигаем макс скорости)
        t_a = sqrt(distance / a);
        float t_norm = t / total_time;
        if (t_norm <= 0.5) {
            return 0.5 * a * pow(t, 2);
        } else {
            float t_rem = total_time - t;
            return distance - 0.5 * a * pow(t_rem, 2);
        }
    } else {
        // Полный трапецеидальный профиль
        float d_c = distance - 2 * d_a;
        float t_c = d_c / v_max;
        
        if (t <= t_a) {
            // Разгон
            return 0.5 * a * pow(t, 2);
        } else if (t <= t_a + t_c) {
            // Постоянная скорость
            return d_a + v_max * (t - t_a);
        } else {
            // Торможение
            float t_dec = t - (t_a + t_c);
            return distance - 0.5 * a * pow(t_dec, 2);
        }
    }
}

// S-образный профиль (плавный)
float MotionController::_compute_s_curve(float t, float total_time, float distance) {
    // Кубический полином: 3t² - 2t³
    float u = t / total_time;
    if (u < 0) return 0;
    if (u > 1) return distance;
    
    // Ограничиваем для численной стабильности
    u = fmin(fmax(u, 0.0), 1.0);
    
    // S-образная кривая с нулевыми скоростями в начале и конце
    float s = u * u * (3.0 - 2.0 * u);
    return distance * s;
}

// Расчет времени движения
float MotionController::_calculate_time(float distance, const MotionParams &params) {
    if (distance <= 0) return 0;
    
    float t_acc = params.max_velocity / params.max_acceleration;
    float dist_acc = 0.5 * params.max_acceleration * t_acc * t_acc;
    
    if (2 * dist_acc >= distance) {
        // Треугольный профиль
        return 2 * sqrt(distance / params.max_acceleration);
    } else {
        // Трапецеидальный профиль
        float dist_const = distance - 2 * dist_acc;
        float t_const = dist_const / params.max_velocity;
        return 2 * t_acc + t_const;
    }
}

// Линейная интерполяция
void MotionController::_interpolate_position(float t, float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = _state.start_pos[i] + t * (_state.target_pos[i] - _state.start_pos[i]);
    }
}

// Остановка движения
void MotionController::stop() {
    _state.is_moving = false;
    _state.is_paused = false;
}

// Пауза
void MotionController::pause() {
    if (_state.is_moving) {
        _state.is_paused = true;
    }
}

// Возобновление
void MotionController::resume() {
    if (_state.is_moving && _state.is_paused) {
        // Пересчитываем время старта для сохранения траектории
        _state.start_time = micros() / 1000000.0 - (_state.progress * _state.duration);
        _state.is_paused = false;
    }
}

// Получение текущей позиции
void MotionController::get_current_pos(float &x, float &y, float &z) {
    x = _state.current_pos[0];
    y = _state.current_pos[1];
    z = _state.current_pos[2];
}

// Получение целевой позиции
void MotionController::get_target_pos(float &x, float &y, float &z) {
    x = _state.target_pos[0];
    y = _state.target_pos[1];
    z = _state.target_pos[2];
}

// Расчет расстояния
float MotionController::distance(float x1, float y1, float z1, 
                                  float x2, float y2, float z2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Установка параметров по умолчанию
void MotionController::set_default_params(float max_vel, float max_acc) {
    _default_params.max_velocity = max_vel;
    _default_params.max_acceleration = max_acc;
    _default_params.max_jerk = max_acc * 10;
    _default_params.type = PROFILE_S_CURVE;
}