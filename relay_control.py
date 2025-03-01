from PySide6.QtCore import QThread, Signal, QMutex, QMutexLocker
import serial
import struct
import time

class RelayControl:
    def __init__(self, port='/dev/tty.usbserial-130', baudrate=115200, address=0x01):
        self.port = port
        self.baudrate = baudrate
        self.address = address  # Device address
        self.ser = None

    def open_connection(self):
        """Open the serial connection to the relay."""
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

    def close_connection(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def calculate_checksum(self, data):
        """Calculate the checksum of the command data."""
        checksum = sum(data[:12]) & 0xFF  # Sum first 12 bytes and take the lower 8 bits
        return checksum

    def create_command(self, cmd, data_bytes):
        """Create a command frame to control or read the relay."""
        command = [0x48, 0x3A, self.address, cmd] + data_bytes
        checksum = self.calculate_checksum(command)
        command.append(checksum)
        command += [0x45, 0x44]  # Trailer as per the protocol
        return bytes(command)

    def control_relay(self, channels, states):
        """Control specific channels on the relay."""
        data_bytes = [0x00] * 8
        for channel, state in zip(channels, states):
            byte_index = (channel - 1) // 2
            bit_position = (channel - 1) % 2
            if state == 1:
                data_bytes[byte_index] |= (0x01 << (4 * bit_position))
            else:
                data_bytes[byte_index] &= ~(0x01 << (4 * bit_position))

        cmd = 0x57  # Write command
        command = self.create_command(cmd, data_bytes)
        self.ser.reset_input_buffer()
        self.ser.write(command)
        
        # Introduce a short delay before reading to avoid conflicts
        time.sleep(0.05)  # 100ms delay to allow the relay to process the command

        response = self.ser.read(15)
        return response

    def read_relay_state(self):
        """Read the state of all relay channels."""
        cmd = 0x53  # Read command
        command = self.create_command(cmd, [0x00] * 8)
        self.ser.reset_input_buffer()
        self.ser.write(command)

        response = self.ser.read(15)
        channel_states = []
        for i in range(8):
            byte = response[4 + i]
            channel_states.append(byte & 0x01)  # Odd channel
            channel_states.append((byte & 0x10) >> 4)  # Even channel
        return channel_states


class RelayControlThread(QThread):
    relay_state_updated = Signal(list)  # Signal to update the relay states in GUI
    relay_control_response = Signal(bytes)  # Signal to return relay control response

    def __init__(self, relay_control, parent=None):
        super().__init__(parent)
        self.relay_control = relay_control
        self.running = True
        self.mutex = QMutex()  # QMutex to ensure reading and writing don't run concurrently

    def run(self):
        """Continuously monitor relay states in the background."""
        while self.running:
            with QMutexLocker(self.mutex):  # Ensure safe access to the critical section
                try:
                    states = self.relay_control.read_relay_state()
                    self.relay_state_updated.emit(states)
                except Exception as e:
                    print(f"Error reading relay states: {e}")
                self.msleep(50) # add a delay after reading the relay states
            self.msleep(950)  # Poll every second

    def control_relay(self, channels, states):
        """Send a command to control the relay and emit the response."""
        with QMutexLocker(self.mutex):  # Ensure safe access to the critical section
            try:
                response = self.relay_control.control_relay(channels, states)
                self.relay_control_response.emit(response)
            except Exception as e:
                print(f"Error controlling relay: {e}")

    def stop(self):
        """Stop the thread."""
        self.running = False
        self.wait()