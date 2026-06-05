// firmware/main/communication_protocol.cpp
#include "communication_protocol.h"

CommunicationProtocol::CommunicationProtocol() {
    _serial = nullptr;
    _buffer = "";
    _command_ready = false;
    _current_command.type = CMD_NONE;
    _current_command.param_count = 0;
}

void CommunicationProtocol::begin(HardwareSerial& serial, unsigned long baud) {
    _serial = &serial;
    _serial->begin(baud);
}

void CommunicationProtocol::update() {
    while (_serial && _serial->available()) {
        char c = _serial->read();
        
        if (c == '\n' || c == '\r') {
            if (_buffer.length() > 0) {
                _current_command = _parse_command(_buffer);
                _command_ready = true;
                _buffer = "";
            }
        } else {
            _buffer += c;
            // Предотвращаем переполнение буфера
            if (_buffer.length() > 64) {
                _buffer = "";
            }
        }
    }
}

bool CommunicationProtocol::command_available() {
    return _command_ready;
}

CommunicationProtocol::Command CommunicationProtocol::get_command() {
    _command_ready = false;
    return _current_command;
}

void CommunicationProtocol::send_position(float x, float y, float z) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "POS:%.2f,%.2f,%.2f\n", x, y, z);
    _send_string(buffer);
}

void CommunicationProtocol::send_status(bool ready, bool estop, const char* error) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "STATUS:%s,%s,%s\n", 
             ready ? "READY" : "BUSY",
             estop ? "ESTOP" : "OK",
             error);
    _send_string(buffer);
}

void CommunicationProtocol::send_ack(CommandType cmd, bool success) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "ACK:%d,%s\n", (int)cmd, success ? "OK" : "FAIL");
    _send_string(buffer);
}

void CommunicationProtocol::send_error(const char* message) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "ERROR:%s\n", message);
    _send_string(buffer);
}

CommunicationProtocol::Command CommunicationProtocol::_parse_command(String input) {
    Command cmd;
    cmd.type = CMD_NONE;
    cmd.param_count = 0;
    
    input.trim();
    input.toUpperCase();
    
    if (input.length() == 0) return cmd;
    
    int space_index = input.indexOf(' ');
    String cmd_name;
    
    if (space_index > 0) {
        cmd_name = input.substring(0, space_index);
        String params_str = input.substring(space_index + 1);
        _parse_params(params_str, cmd.params, cmd.param_count);
    } else {
        cmd_name = input;
    }
    
    cmd.type = _identify_command(cmd_name);
    return cmd;
}

CommunicationProtocol::CommandType CommunicationProtocol::_identify_command(String cmd_name) {
    if (cmd_name == "MOV") return CMD_MOVE_ABS;
    if (cmd_name == "MOVR") return CMD_MOVE_REL;
    if (cmd_name == "STOP") return CMD_STOP;
    if (cmd_name == "POS?" || cmd_name == "GETPOS") return CMD_GET_POS;
    if (cmd_name == "VACON" || cmd_name == "VAC ON") return CMD_VACUUM_ON;
    if (cmd_name == "VACOFF" || cmd_name == "VAC OFF") return CMD_VACUUM_OFF;
    if (cmd_name == "STATUS?" || cmd_name == "STATUS") return CMD_STATUS;
    if (cmd_name == "HOME") return CMD_HOME;
    if (cmd_name == "RESET") return CMD_RESET;
    if (cmd_name == "SPEED") return CMD_SET_SPEED;
    if (cmd_name == "SETPID") return CMD_SET_PID;
    return CMD_NONE;
}

void CommunicationProtocol::_parse_params(String params_str, float* params, int& count) {
    count = 0;
    int start = 0;
    int end = params_str.indexOf(' ');
    
    while (end > 0 && count < 6) {
        String param = params_str.substring(start, end);
        params[count++] = param.toFloat();
        start = end + 1;
        end = params_str.indexOf(' ', start);
    }
    
    if (start < params_str.length() && count < 6) {
        String last_param = params_str.substring(start);
        params[count++] = last_param.toFloat();
    }
}

void CommunicationProtocol::_send_string(const char* str) {
    if (_serial) {
        _serial->print(str);
    }
}