"""
Industrial Delta Robot Controller
PC Interface for OpenCM 904 Delta Robot
"""

import serial
import serial.tools.list_ports
import threading
import time
import sys
import re
from typing import Tuple, Optional, Callable

class DeltaRobotController:
    """Основной контроллер для управления дельта-роботом"""
    
    def __init__(self, port: str = None, baudrate: int = 115200):
        self.serial = None
        self.baudrate = baudrate
        self.port = port
        self.connected = False
        self.running = False
        self.read_thread = None
        self.callbacks = []
        
        # Текущее состояние робота
        self.current_position = (0.0, 0.0, 0.0)
        self.is_moving = False
        self.emergency_stop = False
        self.last_error = ""
        
    def connect(self, port: str = None) -> bool:
        """Подключение к роботу через Serial порт"""
        if port:
            self.port = port
            
        if not self.port:
            ports = self._find_robot_port()
            if not ports:
                print("Robot not found. Check USB connection.")
                return False
            self.port = ports[0]
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.5
            )
            self.connected = True
            self.running = True
            
            # Запуск потока чтения
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            print(f"Connected to robot on {self.port}")
            return True
            
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Отключение от робота"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False
        print("🔌 Disconnected from robot")
    
    def _find_robot_port(self) -> list:
        """Поиск порта с подключенным роботом"""
        available_ports = []
        for port in serial.tools.list_ports.comports():
            # OpenCM 904 обычно определяется как STM32 или USB Serial
            if "STM32" in port.description or "USB Serial" in port.description:
                available_ports.append(port.device)
            # На Windows также проверяем COM порты
            elif "COM" in port.device:
                available_ports.append(port.device)
        return available_ports
    
    def _read_loop(self):
        """Фоновый поток для чтения данных от робота"""
        buffer = ""
        while self.running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting).decode('ascii', errors='ignore')
                    buffer += data
                    
                    # Разбор строк
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._parse_response(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Read error: {e}")
                break
    
    def _parse_response(self, response: str):
        """Парсинг ответов от робота"""
        # POS:x,y,z
        if response.startswith("POS:"):
            parts = response[4:].split(',')
            if len(parts) >= 3:
                self.current_position = (
                    float(parts[0]),
                    float(parts[1]),
                    float(parts[2])
                )
                self._notify_callbacks('position', self.current_position)
        
        # STATUS:READY/BUSY,ESTOP/OK,error
        elif response.startswith("STATUS:"):
            parts = response[7:].split(',')
            if len(parts) >= 2:
                self.is_moving = (parts[0] == "BUSY")
                self.emergency_stop = (parts[1] == "ESTOP")
                if len(parts) >= 3:
                    self.last_error = parts[2]
                self._notify_callbacks('status', (self.is_moving, self.emergency_stop, self.last_error))
        
        # ACK:cmd,OK/FAIL
        elif response.startswith("ACK:"):
            parts = response[4:].split(',')
            if len(parts) >= 2:
                self._notify_callbacks('ack', (int(parts[0]), parts[1] == "OK"))
        
        # ERROR:message
        elif response.startswith("ERROR:"):
            error_msg = response[6:]
            self.last_error = error_msg
            self._notify_callbacks('error', error_msg)
            print(f"⚠️ Robot error: {error_msg}")
        
        # Другие сообщения
        else:
            self._notify_callbacks('log', response)
    
    def send_command(self, command: str) -> bool:
        """Отправка команды роботу"""
        if not self.connected or not self.serial:
            return False
        
        try:
            self.serial.write((command + "\n").encode())
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def move_to(self, x: float, y: float, z: float) -> bool:
        """Абсолютное движение в точку (мм)"""
        return self.send_command(f"MOV {x:.1f} {y:.1f} {z:.1f}")
    
    def move_rel(self, dx: float, dy: float, dz: float) -> bool:
        """Относительное движение"""
        return self.send_command(f"MOVR {dx:.1f} {dy:.1f} {dz:.1f}")
    
    def stop(self) -> bool:
        """Экстренная остановка"""
        return self.send_command("STOP")
    
    def get_position(self) -> bool:
        """Запрос текущей позиции"""
        return self.send_command("POS?")
    
    def vacuum_on(self) -> bool:
        """Включение присоски"""
        return self.send_command("VAC ON")
    
    def vacuum_off(self) -> bool:
        """Выключение присоски"""
        return self.send_command("VAC OFF")
    
    def home(self) -> bool:
        """Возврат в домашнюю позицию"""
        return self.send_command("HOME")
    
    def reset(self) -> bool:
        """Сброс аварийного состояния"""
        return self.send_command("RESET")
    
    def set_speed(self, velocity: float, acceleration: float) -> bool:
        """Установка скорости (мм/с) и ускорения (мм/с²)"""
        return self.send_command(f"SPEED {velocity:.1f} {acceleration:.1f}")
    
    def get_status(self) -> bool:
        """Запрос статуса"""
        return self.send_command("STATUS?")
    
    def on(self, event: str, callback: Callable):
        """Подписка на события"""
        self.callbacks.append((event, callback))
    
    def _notify_callbacks(self, event: str, data):
        """Уведомление подписчиков"""
        for evt, cb in self.callbacks:
            if evt == event:
                try:
                    cb(data)
                except Exception as e:
                    print(f"Callback error: {e}")


class PickAndPlaceProgram:
    """Пример программы для pick-and-place операций"""
    
    def __init__(self, robot: DeltaRobotController):
        self.robot = robot
        self.pick_position = (150.0, -150.0, -50.0)
        self.place_position = (-150.0, 150.0, -50.0)
        self.safe_height = -200.0
    
    def execute_pick_and_place(self):
        """Выполнение цикла взять-положить"""
        print("Starting Pick-and-Place cycle...")
        
        # 1. Движение к точке захвата на безопасной высоте
        print(f"Moving to pick position (safe height)...")
        self.robot.move_to(self.pick_position[0], self.pick_position[1], self.safe_height)
        time.sleep(2)
        
        # 2. Опускание к объекту
        print(f"Descending to pick...")
        self.robot.move_to(self.pick_position[0], self.pick_position[1], self.pick_position[2])
        time.sleep(1.5)
        
        # 3. Включение вакуума
        print(f"Vacuum ON")
        self.robot.vacuum_on()
        time.sleep(0.5)
        
        # 4. Подъём с объектом
        print(f"Ascending with object...")
        self.robot.move_to(self.pick_position[0], self.pick_position[1], self.safe_height)
        time.sleep(1.5)
        
        # 5. Перемещение к точке отпускания
        print(f"Moving to place position...")
        self.robot.move_to(self.place_position[0], self.place_position[1], self.safe_height)
        time.sleep(2)
        
        # 6. Опускание к месту
        print(f"Descending to place...")
        self.robot.move_to(self.place_position[0], self.place_position[1], self.place_position[2])
        time.sleep(1.5)
        
        # 7. Выключение вакуума
        print(f"Vacuum OFF")
        self.robot.vacuum_off()
        time.sleep(0.5)
        
        # 8. Подъём
        print(f"Ascending...")
        self.robot.move_to(self.place_position[0], self.place_position[1], self.safe_height)
        time.sleep(1.5)
        
        print("Pick-and-Place cycle complete!")


def main():
    """Интерактивная консоль для управления роботом"""
    print("=" * 60)
    print("Industrial Delta Robot Controller")
    print("=" * 60)
    
    robot = DeltaRobotController()
    
    # Подключение
    if not robot.connect():
        print("\nPlease connect the robot via USB and try again.")
        print("Available ports:")
        for port in serial.tools.list_ports.comports():
            print(f"  {port.device}: {port.description}")
        return
    
    def on_position(pos):
        print(f"Position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) mm")
    
    def on_status(status):
        is_moving, estop, error = status
        status_str = "MOVING" if is_moving else "IDLE"
        estop_str = "ESTOP" if estop else "OK"
        print(f"Status: {status_str} | E-Stop: {estop_str} | {error}")
    
    def on_error(error):
        print(f"{error}")
    
    robot.on('position', on_position)
    robot.on('status', on_status)
    robot.on('error', on_error)
    
    print("\nConnected! Type 'help' for commands.\n")
    
    commands = {
        'move': lambda args: robot.move_to(float(args[0]), float(args[1]), float(args[2])),
        'movr': lambda args: robot.move_rel(float(args[0]), float(args[1]), float(args[2])),
        'pos': lambda args: robot.get_position(),
        'stop': lambda args: robot.stop(),
        'vac_on': lambda args: robot.vacuum_on(),
        'vac_off': lambda args: robot.vacuum_off(),
        'home': lambda args: robot.home(),
        'reset': lambda args: robot.reset(),
        'speed': lambda args: robot.set_speed(float(args[0]), float(args[1])) if len(args) >= 2 else None,
        'status': lambda args: robot.get_status(),
        'pick': lambda args: PickAndPlaceProgram(robot).execute_pick_and_place(),
        'quit': lambda args: None,
        'help': lambda args: _show_help()
    }
    
    def _show_help():
        print("\n" + "=" * 50)
        print("Available commands:")
        print("  move X Y Z     - Absolute move to coordinates (mm)")
        print("  movr DX DY DZ  - Relative move from current position")
        print("  pos            - Request current position")
        print("  stop           - Emergency stop")
        print("  vac_on         - Turn vacuum ON")
        print("  vac_off        - Turn vacuum OFF")
        print("  home           - Return to home position")
        print("  reset          - Reset emergency stop")
        print("  speed V A      - Set velocity (mm/s) and acceleration (mm/s²)")
        print("  status         - Request robot status")
        print("  pick           - Run pick-and-place demo cycle")
        print("  help           - Show this help")
        print("  quit           - Exit program")
        print("=" * 50 + "\n")
    
    _show_help()
    
    try:
        while True:
            cmd_input = input("> ").strip().lower()
            if not cmd_input:
                continue
            
            parts = cmd_input.split()
            cmd = parts[0]
            args = parts[1:] if len(parts) > 1 else []
            
            if cmd == 'quit':
                break
            
            if cmd in commands:
                try:
                    commands[cmd](args)
                except Exception as e:
                    print(f"Command error: {e}")
            else:
                print(f"Unknown command: {cmd}. Type 'help' for available commands.")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        robot.disconnect()
        print("Goodbye!")


if __name__ == "__main__":
    main()