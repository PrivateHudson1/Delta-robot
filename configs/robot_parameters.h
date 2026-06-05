// configs/robot_parameters.h
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

// Геометрические параметры (мм)
const float ROBOT_BASE_RADIUS = 300.0;
const float ROBOT_PLATFORM_RADIUS = 100.0;
const float ROBOT_UPPER_ARM = 450.0;
const float ROBOT_LOWER_ARM = 800.0;

// Параметры движения
const float MAX_VELOCITY = 500.0;
const float MAX_ACCELERATION = 2000.0;

// Рабочая зона
const float WORKSPACE_Z_MIN = -800.0;
const float WORKSPACE_Z_MAX = -200.0;
const float WORKSPACE_RADIUS_MAX = 1000.0;

#endif