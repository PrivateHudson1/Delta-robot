// firmware/main/mainOpenCM.ino
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "kinematics.h"
#include "motion_controller.h"
#include "safety_watchdog.h"
#include "robot_parameters.h"
#include "pins_config.h"

// Константы управления
#define CONTROL_FREQ 200              // Частота управления (Гц)
#define DT (1.0 / CONTROL_FREQ)       // Дельта времени (секунды)

// Глобальные объекты
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
KinematicsEngine kinematics;
MotionController planner;
SafetyWatchdog safety;

// Переменные состояния
float target_position[3] = {0, 0, -600};
float current_angles[3] = {0, 0, 0};
float target_angles[3] = {0, 0, 0};
float current_position[3] = {0, 0, -600};

// Прототипы функций
void emergency_stop();

void setup() {
    Serial.begin(115200);
    
    // Инициализация Dynamixel
    dxl.begin(3000000);
    dxl.setPortProtocolVersion(2.0);
    
    // Настройка моторов
    for (int id = 1; id <= 3; id++) {
        dxl.torqueOff(id);
        dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
        
        // Используем правильные имена констант
        dxl.writeControlTableItem(ControlTableItem::P_GAIN, id, 800);
        dxl.writeControlTableItem(ControlTableItem::I_GAIN, id, 0);
        dxl.writeControlTableItem(ControlTableItem::D_GAIN, id, 4000);
        
        dxl.torqueOn(id);
    }
    
    // Инициализация кинематики
    kinematics.init(ROBOT_BASE_RADIUS, ROBOT_PLATFORM_RADIUS, 
                    ROBOT_UPPER_ARM, ROBOT_LOWER_ARM);
    
    // Инициализация планировщика (исправлено: begin вместо initialize)
    planner.begin(MAX_VELOCITY, MAX_ACCELERATION);
    
    // Инициализация безопасности
    safety.begin(ESTOP_PIN);
    safety.set_buzzer_pin(BUZZER_PIN);
    
    // Расчёт начальных углов из позиции
    kinematics.inverse_kinematics(target_position, target_angles);
    
    // Устанавливаем начальную позицию
    for (int i = 0; i < 3; i++) {
        dxl.setGoalPosition(i+1, target_angles[i], UNIT_DEGREE);
    }
    
    Serial.println("Delta Robot Ready!");
}

void loop() {
    static unsigned long last_time = 0;
    unsigned long now = millis();
    float delta = (now - last_time) / 1000.0;
    
    if (delta >= DT) {
        last_time = now;
        
        // 1. Чтение текущих углов с моторов
        for (int i = 0; i < 3; i++) {
            current_angles[i] = dxl.getPresentPosition(i+1, UNIT_DEGREE);
        }
        
        // 2. Прямая кинематика: углы -> позиция
        kinematics.forward_kinematics(current_angles, current_position);
        
        // 3. Обновление текущей позиции в планировщике
        // Вместо update_trajectory, используем update и получаем целевую позицию
        if (planner.is_moving()) {
            // Получаем текущую целевую позицию от планировщика
            float new_target[3];
            planner.get_target_pos(new_target[0], new_target[1], new_target[2]);
            
            // Обновляем целевые углы через обратную кинематику
            if (kinematics.inverse_kinematics(new_target, target_angles)) {
                for (int i = 0; i < 3; i++) {
                    dxl.setGoalPosition(i+1, target_angles[i], UNIT_DEGREE);
                }
            }
            
            // Обновляем состояние планировщика
            planner.update(delta);
        }
        
        // 4. Проверка безопасности
        safety.check_emergency_stop();
        
        // 5. Проверка лимитов (ток, температура)
        for (int i = 0; i < 3; i++) {
    int current_ma = dxl.getPresentCurrent(i+1);
    
    // Читаем температуру напрямую из Control Table по адресу 43 (Present Temperature)
    // Адрес 43 взят из официальной документации ROBOTIS [citation:1]
    int temp_c = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, i+1);
    
    float pos_error = fabs(target_angles[i] - current_angles[i]);
    
    SafetyWatchdog::SafetyEvent event = safety.check_motor(i, current_ma, temp_c, pos_error);
    if (event != SafetyWatchdog::EVENT_NONE) {
        emergency_stop();
    }
}
    }
}

// Функция аварийной остановки
void emergency_stop() {
    for (int id = 1; id <= 3; id++) {
        dxl.torqueOff(id);
    }
    planner.stop();
    safety.sound_alarm(true);
    Serial.println("EMERGENCY STOP ACTIVATED!");
    
    // Бесконечный цикл аварийного ожидания
    while (true) {
        delay(100);
        if (digitalRead(ESTOP_PIN) == HIGH) {
            // Кнопка отпущена, можно перезагрузить
            Serial.println("Reset emergency stop. Please reset the board.");
            delay(1000);
        }
    }
}