// configs/dynamixel_config.h
#ifndef DYNAMIXEL_CONFIG_H
#define DYNAMIXEL_CONFIG_H

// ID моторов
const int MOTOR1_ID = 1;
const int MOTOR2_ID = 2;
const int MOTOR3_ID = 3;

// Параметры Dynamixel
const int DXL_BAUDRATE = 3000000;
const int PROTOCOL_VERSION = 2;

// PID коэффициенты
const int P_GAIN = 800;
const int I_GAIN = 0;
const int D_GAIN = 4000;

// Лимиты безопасности
const int MAX_CURRENT_MA = 2500;
const int MAX_TEMPERATURE_C = 85;
const float MAX_POSITION_ERROR = 3.0;

#endif