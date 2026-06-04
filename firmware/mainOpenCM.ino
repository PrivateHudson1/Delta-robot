// firmware/industrial_delta_robot.ino
#include <Dynamixel2Arduino.h>
#include "kinematics.h"
#include "motion_controller.h"

// Настройки шины Dynamixel
#define DXL_SERIAL   Serial3          // Порт OpenCM 485 EXP [citation:2][citation:4]
#define DXL_DIR_PIN  22               // Пин направления RS-485 [citation:4]
#define BAUDRATE     3000000          // 3 Мбит/с для промышленных задач

// Хардварные объекты
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
KinematicsEngine kinematics;          // Экземпляр кинематики
MotionPlanner planner;                // Экземпляр планировщика траекторий

// Массивы данных
float target_coordinates[3] = {0, 0, -600}; // X, Y, Z (мм)
float current_angles[3] = {0, 0, 0};        // Текущие углы сервоприводов
float target_angles[3] = {0, 0, 0};

void setup() {
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(2.0);     // Protocol 2.0 для Dynamixel X / PRO
  
  // Инициализация 3 сервоприводов
  for (int id = 1; id <= 3; id++) {
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_EXTENDED_POSITION); // Режим позиции
    // Настройка демпфирования для 5 кг груза [citation:1]
    dxl.writeControlTableItem(P_GAIN, id, 800);     // Увеличенный P-коэффициент
    dxl.writeControlTableItem(I_GAIN, id, 0);       
    dxl.writeControlTableItem(D_GAIN, id, 4000);
    dxl.torqueOn(id);
  }

  // Расчет стартовых углов из начальных координат (X=0, Y=0, Z=-600)
  kinematics.inverse_kinematics(target_coordinates, target_angles);
  planner.initialize(target_angles);
}

void loop() {
  // 1. Чтение реального положения от энкодеров
  for (int i = 0; i < 3; i++) {
    current_angles[i] = dxl.getPresentPosition(i+1, UNIT_DEGREE);
  }

  // 2. Обновление траектории (если требуется движение)
  if (planner.is_moving()) {
    planner.update_trajectory(target_angles, DT); // DT = дельта времени цикла
    
    // 3. Запись целевых позиций через SyncWrite (синхронный протокол) [citation:3]
    for (int i = 0; i < 3; i++) {
      dxl.setGoalPosition(i+1, target_angles[i], UNIT_DEGREE);
    }
  }

  // 4. Safety Watchdog: мониторинг тока и ошибок
  check_safety_limits();

  delay(5); // Промышленный цикл управления = ~200 Гц
}

void check_safety_limits() {
  for (int i = 0; i < 3; i++) {
    int current = dxl.getPresentCurrent(i+1);
    if (abs(current) > 1500) { // Превышение тока (авария)
      emergency_stop();
    }
  }
}