// configs/pins_config.h
#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

#include <Arduino.h>

// Dynamixel шина для OpenCM 904
// Serial3 - это аппаратный UART на пинах RX=PB11, TX=PB10
#define DXL_SERIAL Serial3

// Пин направления для RS-485 трансивера
// На OpenCM 904 это пин PC12 (GPIO_PIN_12 на порту C)
// Используем номер пина в стиле Arduino (если известен) или GPIO константу
#define DXL_DIR_PIN PB1   // Альтернативный пин, если PC12 не работает

// Промышленные входы/выходы
#define ESTOP_PIN      PA0  // Аварийная кнопка
#define BUZZER_PIN     PA1  // Зуммер
#define VACUUM_PIN     PA2  // Вакуумная присоска

// ID моторов
#define MOTOR1_ID 1
#define MOTOR2_ID 2
#define MOTOR3_ID 3

#endif