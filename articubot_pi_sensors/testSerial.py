import serial
import time

uart = serial.Serial(
    port="/dev/ttyAMA3",
    baudrate=9600,
    timeout=1
)

print("UART started on GPIO 8 (TX) / GPIO 7 (RX)")

while True:
    uart.write(b"Hello from Raspberry Pi 5 UART\n")
    time.sleep(1)
