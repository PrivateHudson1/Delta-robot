// firmware/main/communication_protocol.cpp
#include "communication_protocol.h"
#include <math.h>

// Таблица CRC16 для Modbus (первые 16 значений для экономии места)
static const uint16_t crc_table[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440
};

// ==================== КОНСТРУКТОР ====================
CommunicationProtocol::CommunicationProtocol() {
    _serial = nullptr;
    _mode = PROTOCOL_MODE_TEXT;
    _command_ready = false;
    _packet_ready = false;
    _bin_idx = 0;
    _in_packet = false;
    _telemetry_enabled = false;
    _last_telemetry = 0;
    _text_buffer = "";
}

// ==================== ИНИЦИАЛИЗАЦИЯ ====================
void CommunicationProtocol::begin(HardwareSerial& serial, uint32_t baudrate, uint8_t address) {
    _serial = &serial;
    _address = address;
    _serial->begin(baudrate);
}

// ==================== УСТАНОВКА РЕЖИМА ====================
void CommunicationProtocol::setMode(uint8_t mode) {
    _mode = mode;
    _text_buffer = "";
    _bin_idx = 0;
    _in_packet = false;
    _packet_ready = false;
    
    if (_mode == PROTOCOL_MODE_BINARY) {
        send_response(0x04, RESP_OK);  // ACK for SET_MODE
    } else {
        _serial->println("PROTOCOL:MODE=TEXT");
    }
}

// ==================== ГЛАВНЫЙ ЦИКЛ ОБНОВЛЕНИЯ ====================
void CommunicationProtocol::update() {
    if (!_serial) return;
    
    while (_serial->available()) {
        uint8_t byte = _serial->read();
        
        if (_mode == PROTOCOL_MODE_TEXT) {
            // --- Тестовый режим ---
            char c = (char)byte;
            if (c == '\n' || c == '\r') {
                if (_text_buffer.length() > 0) {
                    _current_command = _parse_command(_text_buffer);
                    _command_ready = true;
                    _text_buffer = "";
                }
            } else {
                _text_buffer += c;
                if (_text_buffer.length() > 64) _text_buffer = "";
            }
        } else {
            // --- Бинарный режим ---
            if (!_in_packet && byte == PKT_START) {
                _in_packet = true;
                _bin_idx = 0;
                _bin_buffer[_bin_idx++] = byte;
            } else if (_in_packet) {
                _bin_buffer[_bin_idx++] = byte;
                
                // Проверяем, достигли ли конца пакета
                if (_bin_idx >= 4 && _bin_idx >= _bin_buffer[1] + 4) {
                    Packet pkt;
                    if (_parse_packet(_bin_buffer, _bin_idx, &pkt)) {
                        if (pkt.addr == _address || pkt.addr == 0xFF) {
                            _current_command.type = (CommandType)pkt.cmd;
                            _current_command.param_count = pkt.data_len / sizeof(float);
                            if (_current_command.param_count > 6) _current_command.param_count = 6;
                            memcpy(_current_command.params, pkt.data, pkt.data_len);
                            _command_ready = true;
                        }
                    }
                    _in_packet = false;
                }
                
                if (_bin_idx >= PKT_MAX_LEN) {
                    _in_packet = false;
                }
            }
        }
    }
}

// ==================== CRC16 ВЫЧИСЛЕНИЕ ====================
uint16_t CommunicationProtocol::_crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc_table[(crc ^ data[i]) & 0x0F];
        crc = (crc >> 8) ^ crc_table[(crc ^ (data[i] >> 4)) & 0x0F];
    }
    return crc;
}

// ==================== ПАРСИНГ БИНАРНОГО ПАКЕТА ====================
bool CommunicationProtocol::_parse_packet(uint8_t* buffer, uint8_t len, Packet* pkt) {
    if (len < 6) return false;
    if (buffer[0] != PKT_START) return false;
    if (buffer[len-1] != PKT_END) return false;
    
    pkt->len = buffer[1];
    if (pkt->len != len - 3) return false;
    
    pkt->addr = buffer[2];
    pkt->cmd = buffer[3];
    
    pkt->data_len = pkt->len - 4;
    if (pkt->data_len > PKT_MAX_DATA) pkt->data_len = PKT_MAX_DATA;
    
    for (uint8_t i = 0; i < pkt->data_len; i++) {
        pkt->data[i] = buffer[4 + i];
    }
    
    uint16_t received_crc = buffer[len - 3] | (buffer[len - 2] << 8);
    uint16_t calculated_crc = _crc16(buffer + 2, pkt->len);
    
    return received_crc == calculated_crc;
}

// ==================== ОТПРАВКА БИНАРНОГО ОТВЕТА ====================
void CommunicationProtocol::send_response(uint8_t cmd, uint8_t code, uint8_t* data, uint8_t data_len) {
    if (!_serial) return;
    if (_mode != PROTOCOL_MODE_BINARY) return;
    
    uint8_t buffer[PKT_MAX_LEN];
    uint8_t idx = 0;
    
    buffer[idx++] = PKT_START;
    
    uint8_t len = 2 + data_len;  // addr(1) + cmd(1) + data + code(1)
    buffer[idx++] = len;
    buffer[idx++] = _address;
    buffer[idx++] = cmd;
    buffer[idx++] = code;
    
    for (uint8_t i = 0; i < data_len; i++) {
        buffer[idx++] = data[i];
    }
    
    uint16_t crc = _crc16(buffer + 2, len);
    buffer[idx++] = crc & 0xFF;
    buffer[idx++] = (crc >> 8) & 0xFF;
    buffer[idx++] = PKT_END;
    
    _serial->write(buffer, idx);
}

// ==================== ТЕЛЕМЕТРИЯ ====================
void CommunicationProtocol::send_telemetry(float x, float y, float z, float speed, 
                                            uint16_t current[3], uint8_t temp[3]) {
    if (!_serial) return;
    if (_mode != PROTOCOL_MODE_BINARY) return;
    
    uint8_t data[4 + 4 + 4 + 4 + 6 + 3];
    uint8_t idx = 0;
    
    memcpy(data + idx, &x, 4); idx += 4;
    memcpy(data + idx, &y, 4); idx += 4;
    memcpy(data + idx, &z, 4); idx += 4;
    memcpy(data + idx, &speed, 4); idx += 4;
    
    for (int i = 0; i < 3; i++) {
        data[idx++] = current[i] & 0xFF;
        data[idx++] = (current[i] >> 8) & 0xFF;
    }
    
    for (int i = 0; i < 3; i++) {
        data[idx++] = temp[i];
    }
    
    send_response(0x60, RESP_OK, data, idx);
}

// ==================== ТЕКСТОВЫЕ МЕТОДЫ ====================
bool CommunicationProtocol::command_available() {
    return _command_ready;
}

Command CommunicationProtocol::get_command() {
    _command_ready = false;
    return _current_command;
}

void CommunicationProtocol::send_position(float x, float y, float z) {
    if (_mode != PROTOCOL_MODE_TEXT) return;
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "POS:%.2f,%.2f,%.2f\n", x, y, z);
    _serial->print(buffer);
}

void CommunicationProtocol::send_status(bool ready, bool estop, const char* error) {
    if (_mode != PROTOCOL_MODE_TEXT) return;
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "STATUS:%s,%s,%s\n", 
             ready ? "READY" : "BUSY",
             estop ? "ESTOP" : "OK",
             error);
    _serial->print(buffer);
}

void CommunicationProtocol::send_ack(CommandType cmd, bool success) {
    if (_mode != PROTOCOL_MODE_TEXT) return;
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "ACK:%d,%s\n", (int)cmd, success ? "OK" : "FAIL");
    _serial->print(buffer);
}

void CommunicationProtocol::send_error(const char* message) {
    if (_mode != PROTOCOL_MODE_TEXT) return;
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "ERROR:%s\n", message);
    _serial->print(buffer);
}

// ==================== ПАРСИНГ ТЕКСТОВЫХ КОМАНД ====================
Command CommunicationProtocol::_parse_command(String input) {
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

CommandType CommunicationProtocol::_identify_command(String cmd_name) {
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
    if (cmd_name == "PROTOBIN") return CMD_PROTO_BIN;
    if (cmd_name == "PROTOTEXT") return CMD_PROTO_TEXT;
    if (cmd_name == "SAVE") return CMD_SAVE_PARAMS;
    if (cmd_name == "LOAD") return CMD_LOAD_PARAMS;
    if (cmd_name == "FACTORY") return CMD_FACTORY_RESET;
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