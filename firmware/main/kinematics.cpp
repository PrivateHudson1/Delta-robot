// firmware/main/kinematics.cpp
#include "kinematics.h"
#include <math.h>

// Константы
#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)
#define EPSILON 0.001

// Конструктор
KinematicsEngine::KinematicsEngine() {
    _base_radius = 0;
    _platform_radius = 0;
    _upper_leg = 0;
    _lower_leg = 0;
    
    for (int i = 0; i < 3; i++) {
        _current_angles[i] = 0;
        _current_position[i] = 0;
    }
}

// Инициализация параметров робота
void KinematicsEngine::init(float base_radius, float platform_radius, 
                             float upper_leg_length, float lower_leg_length) {
    _base_radius = base_radius;
    _platform_radius = platform_radius;
    _upper_leg = upper_leg_length;
    _lower_leg = lower_leg_length;
}

// Обратная кинематика
bool KinematicsEngine::inverse_kinematics(float point[3], float angles[3]) {
    // Координаты в системе робота (Z вниз)
    float x = point[0];
    float y = point[1];
    float z = -point[2];  // Инверсия для удобства
    
    // Углы расположения плеч: 0°, 120°, 240°
    float theta_p[3] = {0, 120.0 * DEG_TO_RAD, 240.0 * DEG_TO_RAD};
    
    // Решение для каждого из трёх рычагов
    for (int i = 0; i < 3; i++) {
        // Координаты точки крепления на подвижной платформе
        float x_i = x - _platform_radius * cos(theta_p[i]);
        float y_i = y - _platform_radius * sin(theta_p[i]);
        float z_i = z;
        
        // Расстояние от привода до точки на платформе
        float R = sqrt(x_i * x_i + y_i * y_i + z_i * z_i);
        
        // Проверка достижимости
        if (R > (_upper_leg + _lower_leg) || R < fabs(_upper_leg - _lower_leg)) {
            return false;  // Точка вне рабочей зоны
        }
        
        // Угол возвышения
        float alpha = atan2(z_i, sqrt(x_i * x_i + y_i * y_i));
        
        // Угол по теореме косинусов
        float cos_gamma = (_upper_leg * _upper_leg + R * R - _lower_leg * _lower_leg) 
                          / (2 * _upper_leg * R);
        
        // Ограничиваем значение в пределах [-1, 1] из-за погрешностей
        if (cos_gamma > 1.0) cos_gamma = 1.0;
        if (cos_gamma < -1.0) cos_gamma = -1.0;
        
        float gamma = acos(cos_gamma);
        
        // Угол для сервопривода
        angles[i] = (alpha + gamma) * RAD_TO_DEG;
        
        // Проверка на валидность
        if (isnan(angles[i]) || isinf(angles[i])) {
            return false;
        }
    }
    
    // Сохраняем результат
    for (int i = 0; i < 3; i++) {
        _current_angles[i] = angles[i];
        _current_position[i] = point[i];
    }
    
    return true;
}

// Прямая кинематика (численное решение)
bool KinematicsEngine::forward_kinematics(float angles[3], float point[3]) {
    // Углы расположения плеч
    float theta_p[3] = {0, 120.0 * DEG_TO_RAD, 240.0 * DEG_TO_RAD};
    
    // Преобразуем углы в радианы
    float theta[3];
    for (int i = 0; i < 3; i++) {
        theta[i] = angles[i] * DEG_TO_RAD;
    }
    
    // Координаты верхних точек рычагов (точки B)
    float B[3][3];
    for (int i = 0; i < 3; i++) {
        // Точка крепления на основании
        float Ax = _base_radius * cos(theta_p[i]);
        float Ay = _base_radius * sin(theta_p[i]);
        float Az = 0;
        
        // Вектор верхнего рычага
        float Lx = _upper_leg * cos(theta[i]) * cos(theta_p[i]);
        float Ly = _upper_leg * cos(theta[i]) * sin(theta_p[i]);
        float Lz = _upper_leg * sin(theta[i]);
        
        B[i][0] = Ax + Lx;
        B[i][1] = Ay + Ly;
        B[i][2] = Az + Lz;
    }
    
    // Начальное приближение (центр треугольника)
    point[0] = (B[0][0] + B[1][0] + B[2][0]) / 3.0;
    point[1] = (B[0][1] + B[1][1] + B[2][1]) / 3.0;
    point[2] = (B[0][2] + B[1][2] + B[2][2]) / 3.0;
    
    // Итерационное уточнение методом градиентного спуска
    float step = 0.1;
    float error = 1.0;
    int iterations = 0;
    int max_iterations = 100;
    
    while (error > EPSILON && iterations < max_iterations) {
        error = 0;
        float grad_x = 0, grad_y = 0, grad_z = 0;
        
        for (int i = 0; i < 3; i++) {
            // Координаты точки на платформе для текущего рычага
            float Px = point[0] + _platform_radius * cos(theta_p[i]);
            float Py = point[1] + _platform_radius * sin(theta_p[i]);
            float Pz = point[2];
            
            // Вектор от верхнего шарнира до точки на платформе
            float dx = Px - B[i][0];
            float dy = Py - B[i][1];
            float dz = Pz - B[i][2];
            
            float dist = sqrt(dx*dx + dy*dy + dz*dz);
            float err = dist - _lower_leg;
            error += err * err;
            
            if (dist > EPSILON) {
                grad_x += 2 * err * dx / dist;
                grad_y += 2 * err * dy / dist;
                grad_z += 2 * err * dz / dist;
            }
        }
        
        // Обновляем позицию
        point[0] -= step * grad_x;
        point[1] -= step * grad_y;
        point[2] -= step * grad_z;
        
        iterations++;
    }
    
    // Инвертируем Z обратно
    point[2] = -point[2];
    
    // Сохраняем результат
    for (int i = 0; i < 3; i++) {
        _current_angles[i] = angles[i];
        _current_position[i] = point[i];
    }
    
    return error < EPSILON;
}

// Проверка достижимости точки
bool KinematicsEngine::is_reachable(float x, float y, float z) {
    float test_angles[3];
    float point[3] = {x, y, z};
    return inverse_kinematics(point, test_angles);
}

// Получение текущей позиции
void KinematicsEngine::get_current_position(float &x, float &y, float &z) {
    x = _current_position[0];
    y = _current_position[1];
    z = _current_position[2];
}

// Получение текущих углов
void KinematicsEngine::get_current_angles(float angles[3]) {
    for (int i = 0; i < 3; i++) {
        angles[i] = _current_angles[i];
    }
}

// Нормализация угла в диапазон [-180, 180]
float KinematicsEngine::normalize_angle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}