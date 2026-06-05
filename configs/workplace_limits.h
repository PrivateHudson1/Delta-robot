// configs/workplace_limits.h
#ifndef WORKPLACE_LIMITS_H
#define WORKPLACE_LIMITS_H

// Границы рабочей зоны (мм)
const float X_MIN = -800.0;
const float X_MAX = 800.0;
const float Y_MIN = -800.0;
const float Y_MAX = 800.0;
const float Z_MIN = -800.0;
const float Z_MAX = -200.0;

// Зоны безопасности
const float SAFETY_HEIGHT = -200.0;  // Безопасная высота для перемещений

#endif