// firmware/kinematics.cpp
#include <math.h>
#include "kinematics.h"

// Параметры робота (мм)
const float r_base = 300.0;    // Радиус основания (шагающая окружность сервоприводов)
const float r_plat = 100.0;    // Радиус подвижной платформы
const float upper_leg = 450.0; // Длина верхнего плеча (ведущего)
const float lower_leg = 800.0; // Длина нижнего штанг (параллелограмм)

bool KinematicsEngine::inverse_kinematics(float point[3], float angles[3]) {
  // Координаты в системе коорд инат робота (Z вниз)
  float x = point[0];
  float y = point[1];
  float z = -point[2]; // Инверсия, если Z направлена вниз
  
  // Углы расположения плеч: 0°, 120°, 240°
  float theta_p[3] = {0, 120.0 * M_PI / 180.0, 240.0 * M_PI / 180.0};
  
  for (int i = 0; i < 3; i++) {
    // Координаты точки крепления верхнего рычага к подвижной платформе
    float x_i = x - r_plat * cos(theta_p[i]);
    float y_i = y - r_plat * sin(theta_p[i]);
    float z_i = z; // Платформа горизонтальна
    
    // Расстояние от привода до точки на платформе
    float R = sqrt(x_i*x_i + y_i*y_i + z_i*z_i);
    float alpha = asin(z_i / R); // Угол возвышения
    float gamma = acos((upper_leg*upper_leg + R*R - lower_leg*lower_leg) / (2*upper_leg*R));
    
    // Угол для текущего сервопривода
    angles[i] = (alpha + gamma) * 180.0 / M_PI;
    
    // Проверка сингулярности (выход за пределы рабочей зоны)
    if (isnan(angles[i])) return false;
  }
  return true;
}