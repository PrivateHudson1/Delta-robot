// firmware/headers/communication_protocol.h
#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#include <Arduino.h>
#include <stdint.h>

// Режимы протокола
#define PROTOCOL_MODE_TEXT   0
#define PROTOCOL_MODE_BINARY 1

// Бинарный протокол - формат пакета
// [START][LEN][ADDR][CMD][DATA...][CRC_H][CRC_L][END]
#define PKT_START 0xAA
#define PKT_END   0x55
#define PKT_MAX_DATA 32
#define PKT_MAX_LEN (1 + 1 + 1 + 1 + PKT_MAX_DATA + 2 + 1)

// Типы команд для текстового режима
enum CommandType {
    CMD_NONE = 0,
    CMD_MOVE_ABS,      // MOV X Y Z
    CMD_MOVE_REL,      // MOVR DX DY DZ
    CMD_STOP,          // STOP
    CMD_GET_POS,       // POS?
    CMD_VACUUM_ON,     // VAC ON
    CMD_VACUUM_OFF,    // VAC OFF
    CMD_STATUS,        // STATUS?
    CMD_HOME,          // HOME
    CMD_RESET,         // RESET
    CMD_SET_SPEED,     // SPEED V A
    CMD_SET_PID,       // SETPID ID P I D
    CMD_PROTO_BIN,     // PROTOBIN - переключение в бинарный режим
    CMD_PROTO_TEXT,    // PROTOTEXT - переключение в текстовый режим
    CMD_SAVE_PARAMS,   // SAVE
    CMD_LOAD_PARAMS,   // LOAD
    CMD_FACTORY_RESET  // FACTORY
};

// Команды бинарного протокола
enum BinaryCommand : uint8_t {
    // Системные команды
    BIN_CMD_PING        = 0x01,
    BIN_CMD_RESET       = 0x02,
    BIN_CMD_GET_STATUS  = 0x03,
    BIN_CMD_SET_MODE    = 0x04,
    
    // Движение
    BIN_CMD_MOVE_ABS    = 0x10,
    BIN_CMD_MOVE_REL    = 0x11,
    BIN_CMD_STOP        = 0x12,
    BIN_CMD_GET_POS     = 0x13,
    BIN_CMD_HOME        = 0x14,
    
    // Вакуум
    BIN_CMD_VAC_ON      = 0x20,
    BIN_CMD_VAC_OFF     = 0x21,
    BIN_CMD_GET_VAC     = 0x22,
    
    // Настройки
    BIN_CMD_SET_SPEED   = 0x30,
    BIN_CMD_GET_SPEED   = 0x31,
    BIN_CMD_SET_PID     = 0x32,
    BIN_CMD_GET_PID     = 0x33,
    
    // Конфигурация
    BIN_CMD_SAVE_PARAMS = 0x40,
    BIN_CMD_LOAD_PARAMS = 0x41,
    BIN_CMD_FACTORY_RESET = 0x42,
    
    // Телеметрия
    BIN_CMD_START_TELEMETRY = 0x50,
    BIN_CMD_STOP_TELEMETRY  = 0x51
};

// Структура команды для текстового режима
struct Command {
    CommandType type;
    float params[6];
    int param_count;
};

// Коды ответов бинарного протокола
enum ResponseCode : uint8_t {
    RESP_OK         = 0x00,
    RESP_ERROR      = 0x01,
    RESP_INVALID_CMD = 0x02,
    RESP_INVALID_CRC = 0x03,
    RESP_BUSY       = 0x04,
    RESP_TIMEOUT    = 0x05,
    RESP_UNREACHABLE = 0x06
};

// Структура пакета бинарного протокола
struct Packet {
    uint8_t len;
    uint8_t addr;
    uint8_t cmd;
    uint8_t data[PKT_MAX_DATA];
    uint8_t data_len;
    uint16_t crc;
};

class CommunicationProtocol {
public:
    CommunicationProtocol();
    void begin(HardwareSerial& serial, uint32_t baudrate = 115200, uint8_t address = 1);
    void setMode(uint8_t mode);  // 0=текст, 1=бинарный
    uint8_t getMode() { return _mode; }
    
    void update();  // Вызывать в главном цикле
    
    // Для текстового режима
    bool command_available();
    Command get_command();
    
    // Для бинарного режима
    bool packet_available();
    Packet get_packet();
    
    // Отправка ответов (бинарный режим)
    void send_response(uint8_t cmd, uint8_t code, uint8_t* data = nullptr, uint8_t data_len = 0);
    void send_telemetry(float x, float y, float z, float speed, uint16_t current[3], uint8_t temp[3]);
    
    // Отправка ответов (текстовый режим)
    void send_position(float x, float y, float z);
    void send_status(bool ready, bool estop, const char* error);
    void send_ack(CommandType cmd, bool success);
    void send_error(const char* message);
    
private:
    HardwareSerial* _serial;
    uint8_t _address;
    uint8_t _mode;
    
    // Текстовый режим
    String _text_buffer;
    Command _current_command;
    bool _command_ready;
    
    // Бинарный режим
    uint8_t _bin_buffer[PKT_MAX_LEN];
    uint8_t _bin_idx;
    bool _in_packet;
    Packet _current_packet;
    bool _packet_ready;
    
    // CRC16 (Modbus-style)
    uint16_t _crc16(const uint8_t* data, uint8_t len);
    
    // Парсинг бинарного пакета
    bool _parse_packet(uint8_t* buffer, uint8_t len, Packet* pkt);
    
    // Отправка сырых байтов
    void _send_bytes(const uint8_t* data, uint8_t len);
    
    // Телеметрия
    unsigned long _last_telemetry;
    bool _telemetry_enabled;
    
    // Вспомогательные методы для текстового режима
    Command _parse_command(String input);
    CommandType _identify_command(String cmd_name);
    void _parse_params(String params_str, float* params, int& count);
};

#endif