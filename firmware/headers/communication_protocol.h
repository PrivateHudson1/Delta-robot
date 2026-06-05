// firmware/headers/communication_protocol.h
#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#include <Arduino.h>

class CommunicationProtocol {
public:
    // Типы команд
    enum CommandType {
        CMD_NONE = 0,
        CMD_MOVE_ABS,      // MOV X Y Z - абсолютное движение
        CMD_MOVE_REL,      // MOVR DX DY DZ - относительное движение
        CMD_STOP,          // STOP - аварийная остановка
        CMD_GET_POS,       // POS? - запрос текущей позиции
        CMD_VACUUM_ON,     // VAC ON - включить присоску
        CMD_VACUUM_OFF,    // VAC OFF - выключить присоску
        CMD_STATUS,        // STATUS? - запрос статуса
        CMD_HOME,          // HOME - уйти в домашнюю позицию
        CMD_RESET,         // RESET - сброс ошибок
        CMD_SET_SPEED,     // SPEED V A - установка скорости и ускорения
        CMD_SET_PID        // SETPID ID P I D - установка PID
    };
    
    struct Command {
        CommandType type;
        float params[6];
        int param_count;
    };
    
public:
    CommunicationProtocol();
    void begin(HardwareSerial& serial, unsigned long baud);
    void update();
    bool command_available();
    Command get_command();
    void send_position(float x, float y, float z);
    void send_status(bool ready, bool estop, const char* error);
    void send_ack(CommandType cmd, bool success);
    void send_error(const char* message);
    
private:
    HardwareSerial* _serial;
    String _buffer;
    Command _current_command;
    bool _command_ready;
    
    Command _parse_command(String input);
    CommandType _identify_command(String cmd_name);
    void _parse_params(String params_str, float* params, int& count);
    void _send_string(const char* str);
};

#endif