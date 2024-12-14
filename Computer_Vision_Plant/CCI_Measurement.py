import serial
import time

def read_serial_output(port, baudrate=9600):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize
        print("Reading serial output:")
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').rstrip()
            if line:
                print(line)
    except serial.SerialException as e:
        print(e)
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        ser.close()

def main():
    # Configuration variables
    port = "COM4"         # Update this to your actual COM port
    baudrate = 105200      # Ensure this matches your Arduino sketch

    read_serial_output(port, baudrate)

if __name__ == "__main__":
    main()
