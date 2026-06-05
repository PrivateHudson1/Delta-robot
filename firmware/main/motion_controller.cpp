// firmware/motion_controller.cpp
// Реализация контроллера движения
#include "motion_controller.h"
#include <math.h>

MotionController::MotionController() {
    // Инициализация состояния движения
    state.is_moving = false;
    state.is_paused = false;
    state.progress = 0.0;
    last_update_us = 0;
}

void MotionController::begin(float max_vel, float max_acc) {
    // Настройка параметров по умолчанию для промышленного использования
    default_params.max_velocity = max_vel;
    default_params.max_acceleration = max_acc;
    default_params.max_jerk = max_acc * 10;  // Типичное ограничение рывка
    default_params.type = PROFILE_S_CURVE;   // Плавное движение 
    
    // Обнуление позиций
    for(int i = 0; i < 3; i++) {
        state.start_pos[i] = 0;
        state.target_pos[i] = 0;
        state.current_pos[i] = 0;
    }
}

bool MotionController::move_to(float target_x, float target_y, float target_z, 
                                MotionParams* params) {
    // Останавливаем текущее движение, если оно есть
    if(state.is_moving) stop();
    
    // Устанавливаем целевую позицию
    state.target_pos[0] = target_x;
    state.target_pos[1] = target_y;
    state.target_pos[2] = target_z;
    
    // Сохраняем начальную позицию
    for(int i = 0; i < 3; i++) {
        state.start_pos[i] = state.current_pos[i];
    }
    
    // Рассчитываем расстояние до цели
    float dist = distance(state.current_pos[0], state.current_pos[1], state.current_pos[2],
                          target_x, target_y, target_z);
    
    // Если уже на месте - выходим
    if(dist < 0.001) {
        state.is_moving = false;
        return true;
    }
    
    // Устанавливаем параметры движения
    if(params != nullptr) {
        state.params = *params;
    } else {
        state.params = default_params;
    }
    
    // Рассчитываем длительность движения
    state.duration = calculate_time(dist, state.params);
    state.start_time = micros() / 1000000.0;
    state.progress = 0.0;
    state.is_moving = true;
    state.is_paused = false;
    
    return true;
}

bool MotionController::move_rel(float delta_x, float delta_y, float delta_z,
                                 MotionParams* params) {
    // Относительное движение через абсолютные координаты
    return move_to(state.current_pos[0] + delta_x,
                   state.current_pos[1] + delta_y,
                   state.current_pos[2] + delta_z,
                   params);
}

bool MotionController::update(float delta_time_seconds) {
    // Если не движемся или на паузе - ничего не делаем
    if(!state.is_moving || state.is_paused) return false;
    
    float current_time = micros() / 1000000.0;
    float elapsed = current_time - state.start_time;
    
    // Проверяем, завершилось ли движение
    if(elapsed >= state.duration) {
        // Движение закончено - устанавливаем точную целевую позицию
        for(int i = 0; i < 3; i++) {
            state.current_pos[i] = state.target_pos[i];
        }
        state.is_moving = false;
        state.progress = 1.0;
        return true;  // Цель достигнута
    }
    
    // Нормализованное время (0 до 1)
    float t = elapsed / state.duration;
    state.progress = t;
    
    // Вычисляем пройденное расстояние в зависимости от профиля
    float s;  // Масштабированное расстояние (0 до 1)
    float distance = distance(state.start_pos[0], state.start_pos[1], state.start_pos[2],
                              state.target_pos[0], state.target_pos[1], state.target_pos[2]);
    
    switch(state.params.type) {
        case PROFILE_TRAPEZOIDAL:
            s = compute_trapezoidal(t, state.duration, distance) / distance;
            break;
        case PROFILE_S_CURVE:
            s = compute_s_curve(t, state.duration, distance) / distance;
            break;
        case PROFILE_CYCLOIDAL:
            s = compute_cycloidal(t, state.duration, distance) / distance;
            break;
        default:
            s = t;  // Линейная интерполяция (запасной вариант)
    }
    
    // Линейная интерполяция между начальной и целевой точкой
    for(int i = 0; i < 3; i++) {
        state.current_pos[i] = state.start_pos[i] + s * (state.target_pos[i] - state.start_pos[i]);
    }
    
    return false;  // Движение еще продолжается
}

float MotionController::compute_trapezoidal(float t, float total_time, float distance) {
    // Трапецеидальный профиль скорости
    float v_max = state.params.max_velocity;
    float a = state.params.max_acceleration;
    
    // Время разгона и торможения
    float t_a = v_max / a;
    float d_a = 0.5 * a * t_a * t_a;  // Путь при разгоне
    
    if(2 * d_a >= distance) {
        // Треугольный профиль (не достигаем максимальной скорости)
        t_a = sqrt(distance / a);
        float t_norm = t / total_time;
        if(t_norm <= 0.5) {
            return 0.5 * a * pow(t, 2);
        } else {
            float t_rem = total_time - t;
            return distance - 0.5 * a * pow(t_rem, 2);
        }
    } else {
        // Полный трапецеидальный профиль
        float d_c = distance - 2 * d_a;  // Путь с постоянной скоростью
        float t_c = d_c / v_max;         // Время движения с постоянной скоростью
        
        if(t <= t_a) {
            // Фаза разгона
            return 0.5 * a * pow(t, 2);
        } else if(t <= t_a + t_c) {
            // Фаза постоянной скорости
            return d_a + v_max * (t - t_a);
        } else {
            // Фаза торможения
            float t_dec = t - (t_a + t_c);
            return distance - 0.5 * a * pow(t_dec, 2);
        }
    }
}

float MotionController::compute_s_curve(float t, float total_time, float distance) {
    // Плавная S-образная кривая с помощью кубического полинома: 3t² - 2t³
    float u = t / total_time;
    if(u < 0) return 0;
    if(u > 1) return distance;
    
    // Ограничиваем для предотвращения численных ошибок
    u = fmin(fmax(u, 0.0), 1.0);
    
    // Формула S-кривой
    float s = u * u * (3.0 - 2.0 * u);
    return distance * s;
}

float MotionController::compute_cycloidal(float t, float total_time, float distance) {
    // Циклоидальный профиль для минимальных вибраций
    float u = t / total_time;
    if(u < 0) return 0;
    if(u > 1) return distance;
    
    // Циклоидальный профиль
    float s = u - sin(2.0 * PI * u) / (2.0 * PI);
    return distance * s;
}

float MotionController::calculate_time(float distance, const MotionParams& params) {
    // Расчет времени движения в зависимости от профиля
    if(distance <= 0) return 0;
    
    switch(params.type) {
        case PROFILE_TRAPEZOIDAL: {
            float t_acc = params.max_velocity / params.max_acceleration;
            float dist_acc = 0.5 * params.max_acceleration * t_acc * t_acc;
            
            if(2 * dist_acc >= distance) {
                // Треугольный профиль (не успеваем разогнаться до максимума)
                return 2 * sqrt(distance / params.max_acceleration);
            } else {
                // Полный трапецеидальный профиль
                float dist_const = distance - 2 * dist_acc;
                float t_const = dist_const / params.max_velocity;
                return 2 * t_acc + t_const;
            }
        }
        
        case PROFILE_S_CURVE:
        case PROFILE_CYCLOIDAL: {
            // S-образный и циклоидальный профили на 15% медленнее для той же пиковой скорости
            float t_trapezoidal = calculate_time(distance, params);
            return t_trapezoidal * 1.15;
        }
        
        default:
            return distance / params.max_velocity;
    }
}

void MotionController::stop() {
    // Немедленная остановка
    state.is_moving = false;
    state.is_paused = false;
}

void MotionController::pause() {
    // Постановка движения на паузу
    if(state.is_moving) {
        state.is_paused = true;
    }
}

void MotionController::resume() {
    // Возобновление движения с паузы
    if(state.is_moving && state.is_paused) {
        // Пересчитываем время старта для сохранения траектории
        state.start_time = micros() / 1000000.0 - (state.progress * state.duration);
        state.is_paused = false;
    }
}

void MotionController::get_current_pos(float& x, float& y, float& z) {
    // Получение текущей позиции
    x = state.current_pos[0];
    y = state.current_pos[1];
    z = state.current_pos[2];
}

void MotionController::get_target_pos(float& x, float& y, float& z) {
    // Получение целевой позиции
    x = state.target_pos[0];
    y = state.target_pos[1];
    z = state.target_pos[2];
}

float MotionController::distance(float x1, float y1, float z1, 
                                  float x2, float y2, float z2) {
    // Евклидово расстояние между двумя точками в 3D
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

void MotionController::set_default_params(float max_vel, float max_acc) {
    // Установка параметров движения по умолчанию
    default_params.max_velocity = max_vel;
    default_params.max_acceleration = max_acc;
    default_params.max_jerk = max_acc * 10;
    default_params.type = PROFILE_S_CURVE;
}