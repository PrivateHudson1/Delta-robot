// firmware/main/mainOpenCM.ino
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "kinematics.h"
#include "motion_controller.h"
#include "safety_watchdog.h"
#include "communication_protocol.h"
#include "robot_parameters.h"
#include "pins_config.h"

// Константы управления
#define CONTROL_FREQ 200              // Частота управления (Гц)
#define DT (1.0 / CONTROL_FREQ)       // Дельта времени (секунды)
#define HEARTBEAT_TIMEOUT 2000        // Таймаут связи с ПК (мс)

// Глобальные объекты
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
KinematicsEngine kinematics;
MotionController planner;
SafetyWatchdog safety;
CommunicationProtocol comm;

// Переменные состояния
float current_position[3] = {0, 0, -600};
float target_position[3] = {0, 0, -600};
float current_angles[3] = {0, 0, 0};
float target_angles[3] = {0, 0, 0};

bool emergency_active = false;
bool vacuum_active = false;
unsigned long last_heartbeat = 0;

// Прототипы функций
void emergency_stop();
void process_commands();
void update_vacuum();
void update_motors();
void update_safety();

void setup() {
    Serial.begin(115200);
    
    // Инициализация Dynamixel
    dxl.begin(3000000);
    dxl.setPortProtocolVersion(2.0);
    
    // Настройка моторов
    for (int id = 1; id <= 3; id++) {
        dxl.torqueOff(id);
        dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
        
        dxl.writeControlTableItem(ControlTableItem::P_GAIN, id, 800);
        dxl.writeControlTableItem(ControlTableItem::I_GAIN, id, 0);
        dxl.writeControlTableItem(ControlTableItem::D_GAIN, id, 4000);
        
        dxl.torqueOn(id);
    }
    
    // Инициализация компонентов
    kinematics.init(ROBOT_BASE_RADIUS, ROBOT_PLATFORM_RADIUS, 
                    ROBOT_UPPER_ARM, ROBOT_LOWER_ARM);
    planner.begin(MAX_VELOCITY, MAX_ACCELERATION);
    safety.begin(ESTOP_PIN);
    safety.set_buzzer_pin(BUZZER_PIN);
    comm.begin(Serial, 115200);
    
    // Начальная позиция
    target_position[0] = 0;
    target_position[1] = 0;
    target_position[2] = -600;
    
    kinematics.inverse_kinematics(target_position, target_angles);
    for (int i = 0; i < 3; i++) {
        dxl.setGoalPosition(i+1, target_angles[i], UNIT_DEGREE);
    }
    
    pinMode(VACUUM_PIN, OUTPUT);
    digitalWrite(VACUUM_PIN, LOW);
    
    comm.send_status(true, false, "Ready");
    comm.send_position(current_position[0], current_position[1], current_position[2]);
    
    Serial.println("Delta Robot Ready - Waiting for commands");
}

void loop() {
    static unsigned long last_time = 0;
    unsigned long now = millis();
    float delta = (now - last_time) / 1000.0;
    
    if (delta >= DT) {
        last_time = now;
        
        // Обновление состояния
        update_motors();
        update_safety();
        update_vacuum();
        
        // Движение
        if (planner.is_moving() && !emergency_active) {
            planner.update(delta);
            planner.get_target_pos(target_position[0], target_position[1], target_position[2]);
            
            if (kinematics.inverse_kinematics(target_position, target_angles)) {
                for (int i = 0; i < 3; i++) {
                    dxl.setGoalPosition(i+1, target_angles[i], UNIT_DEGREE);
                }
            }
        }
    }
    
    // Обработка команд (не блокирует управление)
    process_commands();
}

void update_motors() {
    for (int i = 0; i < 3; i++) {
        current_angles[i] = dxl.getPresentPosition(i+1, UNIT_DEGREE);
    }
    kinematics.forward_kinematics(current_angles, current_position);
}

void update_safety() {
    // Проверка аварийной кнопки
    safety.check_emergency_stop();
    
    // Проверка связи с ПК
    safety.check_communication(last_heartbeat, HEARTBEAT_TIMEOUT);
    
    // Проверка моторов
    for (int i = 0; i < 3; i++) {
        int current_ma = dxl.getPresentCurrent(i+1);
        int temp_c = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, i+1);
        float pos_error = fabs(target_angles[i] - current_angles[i]);
        
        SafetyWatchdog::SafetyEvent event = safety.check_motor(i, current_ma, temp_c, pos_error);
        if (event != SafetyWatchdog::EVENT_NONE) {
            emergency_stop();
        }
    }
    
    if (safety.is_emergency_stop_active() && !emergency_active) {
        emergency_stop();
    }
}

void update_vacuum() {
    static unsigned long vacuum_timer = 0;
    
    if (vacuum_active) {
        digitalWrite(VACUUM_PIN, HIGH);
        vacuum_timer = millis();
    } else {
        // Автоматическое отключение через 5 секунд для безопасности
        if (millis() - vacuum_timer > 5000) {
            digitalWrite(VACUUM_PIN, LOW);
        }
    }
}

void process_commands() {
    comm.update();
    
    if (comm.command_available()) {
        last_heartbeat = millis();
        CommunicationProtocol::Command cmd = comm.get_command();
        
        switch (cmd.type) {
            case CommunicationProtocol::CMD_MOVE_ABS:
                if (cmd.param_count >= 3 && !emergency_active) {
                    float x = cmd.params[0];
                    float y = cmd.params[1];
                    float z = cmd.params[2];
                    
                    if (kinematics.is_reachable(x, y, z)) {
                        planner.move_to(x, y, z);
                        comm.send_ack(cmd.type, true);
                        comm.send_position(x, y, z);
                    } else {
                        comm.send_error("Target unreachable");
                        comm.send_ack(cmd.type, false);
                    }
                }
                break;
                
            case CommunicationProtocol::CMD_MOVE_REL:
                if (cmd.param_count >= 3 && !emergency_active) {
                    planner.move_rel(cmd.params[0], cmd.params[1], cmd.params[2]);
                    comm.send_ack(cmd.type, true);
                }
                break;
                
            case CommunicationProtocol::CMD_STOP:
                planner.stop();
                emergency_stop();
                comm.send_ack(cmd.type, true);
                break;
                
            case CommunicationProtocol::CMD_GET_POS:
                comm.send_position(current_position[0], current_position[1], current_position[2]);
                break;
                
            case CommunicationProtocol::CMD_VACUUM_ON:
                vacuum_active = true;
                comm.send_ack(cmd.type, true);
                break;
                
            case CommunicationProtocol::CMD_VACUUM_OFF:
                vacuum_active = false;
                digitalWrite(VACUUM_PIN, LOW);
                comm.send_ack(cmd.type, true);
                break;
                
            case CommunicationProtocol::CMD_HOME:
                if (!emergency_active) {
                    planner.move_to(0, 0, -600);
                    comm.send_ack(cmd.type, true);
                }
                break;
                
            case CommunicationProtocol::CMD_RESET:
                if (!safety.is_emergency_stop_active()) {
                    emergency_active = false;
                    safety.reset();
                    planner.stop();
                    comm.send_ack(cmd.type, true);
                    comm.send_status(true, false, "Reset complete");
                } else {
                    comm.send_error("Cannot reset: E-stop active");
                }
                break;
                
            case CommunicationProtocol::CMD_SET_SPEED:
                if (cmd.param_count >= 2) {
                    planner.set_default_params(cmd.params[0], cmd.params[1]);
                    comm.send_ack(cmd.type, true);
                }
                break;
                
            case CommunicationProtocol::CMD_STATUS:
                comm.send_status(!planner.is_moving(), emergency_active, 
                                 safety.get_event_description(safety.get_last_event()).c_str());
                break;
                
            default:
                comm.send_error("Unknown command");
                break;
        }
    }
}

void emergency_stop() {
    emergency_active = true;
    
    for (int id = 1; id <= 3; id++) {
        dxl.torqueOff(id);
    }
    
    planner.stop();
    vacuum_active = false;
    digitalWrite(VACUUM_PIN, LOW);
    
    comm.send_status(false, true, "Emergency stop activated");
    comm.send_error("EMERGENCY STOP");
    
    Serial.println("EMERGENCY STOP ACTIVATED!");
}