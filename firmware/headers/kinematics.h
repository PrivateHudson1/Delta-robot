// firmware/headers/kinematics.h
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

class KinematicsEngine {
public:
    // Конструктор
    KinematicsEngine();
    
    // Инициализация параметров робота
    void init(float base_radius, float platform_radius, 
              float upper_leg_length, float lower_leg_length);
    
    // Обратная кинематика: координаты (x,y,z) -> углы сервоприводов (degrees)
    // point[0] = x, point[1] = y, point[2] = z (мм)
    // angles[0-2] = углы для моторов 1,2,3 (градусы)
    bool inverse_kinematics(float point[3], float angles[3]);
    
    // Прямая кинематика: углы сервоприводов -> координаты (x,y,z)
    bool forward_kinematics(float angles[3], float point[3]);
    
    // Проверка, находится ли точка в рабочей зоне
    bool is_reachable(float x, float y, float z);
    
    // Получение текущих параметров
    void get_current_position(float &x, float &y, float &z);
    void get_current_angles(float angles[3]);
    
private:
    // Параметры робота (мм)
    float _base_radius;      // Радиус основания
    float _platform_radius;  // Радиус подвижной платформы
    float _upper_leg;        // Длина верхнего рычага
    float _lower_leg;        // Длина нижней штанги
    
    // Текущее состояние
    float _current_angles[3];
    float _current_position[3];
    
    // Вспомогательные методы
    float normalize_angle(float angle);
    float solve_angle_for_arm(float x, float y, float z, float theta_base);
};

#endif