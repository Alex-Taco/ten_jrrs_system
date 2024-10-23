from scservo_sdk import *  # Import SCServo SDK library
from PySide6.QtCore import QThread, Signal, QMutex, QMutexLocker, QTimer

class ServoThread(QThread):
    position_updated = Signal(int, int, int, int, int)  # Signal to update the GUI (servo_id, pos, speed)
    write_position_signal = Signal(int, int)  # Signal to request a position write
    disable_torque_signal = Signal(int)  # Signal to request torque disable
    io_open_signal = Signal(list)  # Signal to request  open servo in IO mode
    io_close_signal = Signal(int, int)  # Signal to request  close servo in IO mode
    io_position_load_update = Signal(dict) # Signal to send servo position to IO thread

    def __init__(self, servos, parent=None):
        super().__init__(parent)
        self.servos = servos  # Dictionary of ServoControl instances
        self.servos_positions_loads = {}
        self.mutex = QMutex()  # QMutex to ensure reading and writing don't run concurrently
        self.running = True
        self.write_position_signal.connect(self.write_position)  # Connect signal to slot
        self.disable_torque_signal.connect(self.disable_torque)  # Connect signal to slot
        self.io_open_signal.connect(self.io_open)  # Connect signal to slot
        # self.io_close_signal.connect(self.io_close)  # Connect signal to slot

    def run(self):
        """Main loop for servo control."""
        while self.running:
            for scs_id, servo in self.servos.items():
                self.msleep(150)  # Wait 200 ms between each servo read to give time to the device to process the command
                # Use QMutexLocker to ensure safe access to the critical section
                with QMutexLocker(self.mutex):  
                    try:
                        # Safely read the servo's position and speed
                        pos, speed, load, volt, temp = servo.read_all()
                        # Store the servo's position
                        self.servos_positions_loads[scs_id] = [pos, load]
                        # Emit signal to IO thread
                        self.io_position_load_update.emit(self.servos_positions_loads)
                        # Emit signal to update GUI
                        self.position_updated.emit(scs_id, pos, speed, temp, load)
                    except Exception as e:
                        print(f"Error reading data from servo {scs_id}: {e}")
                    self.msleep(50) # add a delay after reading the servo data
            self.msleep(200)  # Wait 100 ms between each iteration

    def stop(self):
        """Stop the thread."""
        self.running = False
        self.wait()  # Wait for the thread to finish

    def write_position(self, servo_id, position):
        """Slot to handle writing servo position."""
        # Use QMutexLocker to ensure safe access to the critical section
        with QMutexLocker(self.mutex):
            try:
                self.servos[servo_id].write_position(position)
                # Schedule torque disable 5 seconds later
                QTimer.singleShot(12000, lambda: self.disable_torque_signal.emit(servo_id))
            except Exception as e:
                print(f"Error writing position to servo {servo_id}: {e}")

    def disable_torque(self, servo_id):
        """Slot to handle disabling torque."""
        # Use QMutexLocker to ensure safe access to the critical section
        with QMutexLocker(self.mutex):
            try:
                self.servos[servo_id].write_torque_disable()
            except Exception as e:
                print(f"Error disabling torque on servo {servo_id}: {e}")

    def io_open(self, servo_ids):
        """Slot to handle opening servo in IO mode."""
        # Move target servos to position 2030
        for servo_id in servo_ids:
            with QMutexLocker(self.mutex):
                try:
                    self.servos[servo_id].write_position(2030)
                except Exception as e:
                    print(f"Error opening servo {servo_id} in IO mode: {e}")
        
        # Check the servo position after 6 seconds and resend command if necessary
        QTimer.singleShot(6000, lambda: self.check_servo_position(servo_ids))

    def check_servo_position(self, servo_ids):
        """Check if servo position is greater than 2100 after 6 seconds, resend command if not."""
        for servo_id in servo_ids:
            with QMutexLocker(self.mutex):  # Lock to ensure thread-safety
                id = servo_id  # Explicitly assign to id, but this is not necessary with the lambda fix below
                try:
                    # Check the position from self.servos_positions_loads
                    if id in self.servos_positions_loads:
                        pos = self.servos_positions_loads[id][0]  # Position is stored in the first position
                        if pos > 2100:
                            print(f"Servo {id} position is {pos}, resending command...")
                            # Resend the position command if position is greater than 2100
                            self.servos[id].write_position(2030)
                            # Check again after 6 seconds (capture id explicitly)
                            QTimer.singleShot(6000, lambda id=id: self.check_servo_position([id]))
                        else:
                            print(f"Servo {id} position is {pos}, no need to resend.")
                            # Disable torque and check load after 2 seconds
                            self.servos[id].write_torque_disable()
                            # Schedule load check after 2 seconds (capture id explicitly)
                            QTimer.singleShot(2000, lambda id=id: self.check_load_after_disable(id))
                    else:
                        print(f"Position data for Servo {id} not available.")
                except Exception as e:
                    print(f"Error checking servo {id} position: {e}")

    def check_load_after_disable(self, servo_id):
        """Check if the load is 0 after torque disable, resend disable torque command if necessary."""
        with QMutexLocker(self.mutex):  # Lock to ensure thread-safety
            try:
                # Check the load from self.servos_positions_loads
                if servo_id in self.servos_positions_loads:
                    load = self.servos_positions_loads[servo_id][1]  # Load is stored in the second position
                    if load != 0:
                        print(f"Servo {servo_id} load is {load}, resending torque disable command...")
                        # Resend the disable torque command if load is not 0
                        self.servos[servo_id].write_torque_disable()
                        # Recheck the load after 2 seconds (capture servo_id explicitly)
                        QTimer.singleShot(2000, lambda servo_id=servo_id: self.check_load_after_disable(servo_id))
                    else:
                        print(f"Servo {servo_id} load is 0, torque successfully disabled.")
                else:
                    print(f"Load data for Servo {servo_id} not available.")
            except Exception as e:
                print(f"Error checking load for servo {servo_id}: {e}")

class ServoControl:
    def __init__(self, scs_id, port_handler, packet_handler,
                 min_pos=2030, max_pos=3100, moving_speed=200, moving_acc=10):
        # Servo configuration
        self.SCS_ID = scs_id
        self.SCS_MINIMUM_POSITION_VALUE = min_pos
        self.SCS_MAXIMUM_POSITION_VALUE = max_pos
        self.SCS_MOVING_SPEED = moving_speed
        self.SCS_MOVING_ACC = moving_acc

        # Use the shared port and packet handlers
        self.portHandler = port_handler
        self.packetHandler = packet_handler

    def write_position(self, position):
        """Write the goal position to the servo."""
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(self.SCS_ID, position, self.SCS_MOVING_SPEED, self.SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            raise Exception(f"Communication Error: {self.packetHandler.getTxRxResult(scs_comm_result)}")
        elif scs_error != 0:
            raise Exception(f"Servo Error: {self.packetHandler.getRxPacketError(scs_error)}")

    def read_position_and_speed(self):
        """Read the current position and speed of the servo."""
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = self.packetHandler.ReadPosSpeed(self.SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            raise Exception(f"Communication Error: {self.packetHandler.getTxRxResult(scs_comm_result)}")
        elif scs_error != 0:
            raise Exception(f"Servo Error: {self.packetHandler.getRxPacketError(scs_error)}")

        return scs_present_position, scs_present_speed
    
    def write_torque_disable(self):
        """Disable torque on the servo."""
        scs_comm_result, scs_error = self.packetHandler.TorqueDisable(self.SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            raise Exception(f"Communication Error: {self.packetHandler.getTxRxResult(scs_comm_result)}")
        elif scs_error != 0:
            raise Exception(f"Servo Error: {self.packetHandler.getRxPacketError(scs_error)}")
    
    def read_all(self):
        """Read the temperature of the servo."""
        scs_present_position, scs_present_speed, scs_present_load, scs_present_volt, scs_present_temp, scs_comm_result, scs_error  = self.packetHandler.ReadPos_Spd_Load_Volt_Temp(self.SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            raise Exception(f"Communication Error: {self.packetHandler.getTxRxResult(scs_comm_result)}")
        elif scs_error != 0:
            raise Exception(f"Servo Error: {self.packetHandler.getRxPacketError(scs_error)}")
        
        return scs_present_position, scs_present_speed, scs_present_load, scs_present_volt, scs_present_temp