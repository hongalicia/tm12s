# uart.py
import serial
import threading
import struct
import time

class UART:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, on_data=None):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
        self.running = True
        self.last_values = None
        self.on_data = on_data  # <--- 新增：資料回呼

        self.thread = threading.Thread(target=self.read_uart, daemon=True)
        self.thread.start()
        print(f"[UARTThread] 已連線到 {self.port} (baud={self.baudrate})")

    def read_uart(self):
        while self.running:
            if self.ser.in_waiting >= 24:               
                raw = self.ser.read(24)                  
                values = [round(v, 2) for v in struct.unpack("<6f", raw)]
                self.last_values = values
                print(f"[UARTThread] 接收到資料: {values}")
                if self.on_data is not None:
                    try:
                        self.on_data(values)
                    except Exception as e:
                        print(f"[UARTThread] on_data callback error: {e}")
            time.sleep(0.005)

    def close(self):
        self.running = False
        self.ser.close()
        print("[UARTThread] 已關閉 UART")
