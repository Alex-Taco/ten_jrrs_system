import sys
import numpy as np
import pyqtgraph as pg  # Added for real-time plotting
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QSlider, QLabel, QComboBox, QGridLayout, QFrame, QCheckBox, QHBoxLayout, QSpinBox, QDoubleSpinBox)
from PySide6.QtCore import Qt, QTimer
from servo_control import ServoControl, ServoThread
from voltage_collector import VoltageCollector, VoltageCollectorThread
from leakage_sensor import LeakageSensor, LeakageSensorThread
from pressure_sensor import PressureSensor, PressureSensorThread
from relay_control import RelayControl, RelayControlThread
from pump_control import PumpControl, PumpControlThread
from power_supply import PowerSupplyControl, PowerSupplyControlThread
from scservo_sdk import *  # Import SCServo SDK library
from data_update import DataUpdateThread
import datetime

class MainGUI(QWidget):
    def __init__(self):
        super().__init__()

        # Initialize shared port and packet handlers
        try:
            self.portHandler = PortHandler('COM12')  # Replace with your COM port
            self.packetHandler = sms_sts(self.portHandler)
            if not self.portHandler.openPort():
                raise Exception("Failed to open the port")
            if not self.portHandler.setBaudRate(38400):
                raise Exception("Failed to set the baudrate")
        except Exception as e:
            print(f"Error initializing port: {str(e)}")
            sys.exit(1)  # Exit if initialization fails

        # Initialize ServoControl instances for servos with IDs from 1 to 10
        self.servos = {}
        for scs_id in range(1, 11):
            self.servos[scs_id] = ServoControl(scs_id, self.portHandler, self.packetHandler, min_pos=2030, max_pos=3100)

        self.current_servo_id = 1  # Default servo ID set to 1
        self.current_servo = self.servos[self.current_servo_id]

        # Initialize the servo thread
        self.servo_thread = ServoThread(self.servos)
        self.servo_thread.position_updated.connect(self.update_servo_info)
        self.servo_thread.start()

        # Initialize the voltage collector
        self.voltage_collector = VoltageCollector('COM5')
        self.voltage_thread = VoltageCollectorThread(self.voltage_collector)
        self.voltage_thread.voltages_updated.connect(self.update_voltages)

        # Initialize the leakage sensor and thread
        self.leakage_sensor = LeakageSensor('COM14')
        self.leakage_sensor_thread = LeakageSensorThread(self.leakage_sensor)
        self.leakage_sensor_thread.leak_status_signal.connect(self.update_leak_status)
        self.leakage_sensor_thread.start()

        # Initialize the pressure sensor and thread
        self.pressure_sensor = PressureSensor('COM15', baudrate=9600, address=1)
        self.pressure_sensor_thread = PressureSensorThread(self.pressure_sensor)
        self.pressure_sensor_thread.pressure_updated.connect(self.update_pressure)
        
        # Initialize the relay control and thread
        self.relay_control = RelayControl('COM16', baudrate=115200, address=0x01)
        self.relay_control.open_connection()
        self.relay_thread = RelayControlThread(self.relay_control)
        self.relay_thread.relay_state_updated.connect(self.update_relay_states)
        self.relay_thread.relay_control_response.connect(self.handle_relay_response)
        self.relay_thread.start()

        # Initialize the pump control and thread
        self.pump_control = PumpControl('COM13', baudrate=9600, address=1)
        self.pump_control.open_connection()
        self.pump_thread = PumpControlThread(self.pump_control)
        self.pump_thread.pressure_updated.connect(self.update_pump_pressure)
        self.pump_thread.flow_updated.connect(self.update_pump_flow)
        self.pump_thread.stroke_updated.connect(self.update_pump_stroke)
        self.pump_thread.status_updated.connect(self.update_pump_status)

        # Initialize the power supply control and thread
        self.power_supply = PowerSupplyControl('COM17', baudrate=19200)
        self.power_supply.open_connection()
        self.power_supply_thread = PowerSupplyControlThread(self.power_supply)
        self.power_supply_thread.power_state_updated.connect(self.update_ps_state)
        self.power_supply_thread.current_measured.connect(self.update_ps_current)
        self.power_supply_thread.voltage_measured.connect(self.update_ps_voltage)
        self.power_supply_thread.power_measured.connect(self.update_ps_power)
        self.power_supply_thread.start()

        # Initialize pressure history and time history
        self.time_history = np.linspace(-600, 0, 600)  # Time axis, representing the last 10 minutes
        self.pressure_history = np.zeros(600) 
        self.voltage_channels = 10  # Number of voltage channels
        self.voltage_data = np.zeros((self.voltage_channels, 600))  # Voltage data history
        self.flow_data = np.zeros(600)  # Flow rate data history
        self.ps_current = np.zeros(600)  # Power supply current history
        self.ps_voltage = np.zeros(600)  # Power supply voltage history

        # Initialize the date updating thread
        self.data_updater = DataUpdateThread(pressure_history_size=600, voltage_channels=self.voltage_channels)
        self.pressure_sensor_thread.pressure_updated.connect(self.data_updater.update_pressure)
        self.voltage_thread.voltages_updated.connect(self.data_updater.update_voltages)
        self.pump_thread.flow_updated.connect(self.data_updater.update_flow_rate)
        self.power_supply_thread.current_measured.connect(self.data_updater.update_ps_current)
        self.power_supply_thread.voltage_measured.connect(self.data_updater.update_ps_voltage)
        self.data_updater.plot_update_signal.connect(self.update_plots)

        self.data_updater.start()
        self.pressure_sensor_thread.start()
        self.voltage_thread.start()
        self.pump_thread.start()

        # Initialize the UI
        self.init_ui()

        

    def init_ui(self):
        self.setWindowTitle('Control with Voltage, Leak, Pressure, and Relay Management')
        self.setGeometry(300, 300, 1200, 800)

        # Main layout (horizontal layout to split the window into two sections)
        main_layout = QHBoxLayout()

        # Display the pump control panel on the far left
        pump_layout = QVBoxLayout()
        # Pump control panel
        pump_control_panel = QVBoxLayout()

        # Pump state, outlet pressure, and stroke display in the same line
        pump_info_layout = QHBoxLayout()
        
        self.pump_state_label = QLabel("Pump State: ---", self)
        pump_info_layout.addWidget(self.pump_state_label)
        
        self.pump_pressure_label = QLabel("Pump Outlet Pressure: --- Bar", self)
        pump_info_layout.addWidget(self.pump_pressure_label)
        
        self.pump_stroke_label = QLabel("Pump Stroke: --- %", self)
        pump_info_layout.addWidget(self.pump_stroke_label)
        
        pump_control_panel.addLayout(pump_info_layout)

        # Start pump button
        self.start_pump_button = QPushButton("Start Pump", self)
        self.start_pump_button.clicked.connect(self.pump_thread.start_pump)
        pump_control_panel.addWidget(self.start_pump_button)

        # Stop pump button
        self.stop_pump_button = QPushButton("Stop Pump", self)
        self.stop_pump_button.clicked.connect(self.pump_thread.stop_pump)
        pump_control_panel.addWidget(self.stop_pump_button)
        
        # Set pump stroke (range: 0-100) - QSpinBox and button
        stroke_layout = QHBoxLayout()
        self.stroke_spinbox = QSpinBox(self)
        self.stroke_spinbox.setRange(0, 100)
        self.stroke_spinbox.setValue(0)
        stroke_layout.addWidget(QLabel("Set Pump Stroke (%):", self))
        stroke_layout.addWidget(self.stroke_spinbox)

        self.set_stroke_button = QPushButton("Set Stroke", self)
        self.set_stroke_button.clicked.connect(self.set_pump_stroke)
        stroke_layout.addWidget(self.set_stroke_button)

        pump_control_panel.addLayout(stroke_layout)


        # Pump flow rate display
        self.pump_flow_label = QLabel("Pump Flow Rate: --- L/h", self)
        pump_control_panel.addWidget(self.pump_flow_label)

        # Pump flow rate real time plot set up
        self.pump_flow_plot_widget = pg.PlotWidget(title="Pump Flow Rate Over Time")
        self.pump_flow_plot_widget.setLabel('left', 'Flow Rate (L/h)')
        self.pump_flow_plot_widget.setLabel('bottom', 'Time (s)')
        self.pump_flow_plot_widget.setYRange(0, 400)  # Adjust the y-axis range as needed
        self.pump_flow_curve = self.pump_flow_plot_widget.plot(self.time_history, self.flow_data, pen='b')

        pump_control_panel.addWidget(self.pump_flow_plot_widget)

        # Add pump control panel to the main layout
        pump_layout.addLayout(pump_control_panel)
        
        # Label to display the pressure value
        self.pressure_label = QLabel("Pressure: --- MPa", self)
        pump_layout.addWidget(self.pressure_label)

        # Pressure plot setup
        self.pressure_plot_widget = pg.PlotWidget(title="Inlet Pressure Over Time")
        self.pressure_plot_widget.setLabel('left', 'Pressure (MPa)')
        self.pressure_plot_widget.setLabel('bottom', 'Time (s)')
        self.pressure_plot_widget.setYRange(0, 1)  # Set y-axis from 0 to 1 MPa
        self.pressure_curve = self.pressure_plot_widget.plot(self.time_history, self.pressure_history, pen='y')

        pump_layout.addWidget(self.pressure_plot_widget)
        main_layout.addLayout(pump_layout)

        # Left side layout for voltage display (both label and plot)
        display_layout = QVBoxLayout()

        # Power supply state and measured power display in the same line
        power_info_layout = QHBoxLayout()
        
        self.power_state_label = QLabel("Power Supply State: ---", self)
        power_info_layout.addWidget(self.power_state_label)
        
        self.measured_power_label = QLabel("Measured Power: --- W", self)
        power_info_layout.addWidget(self.measured_power_label)
        
        display_layout.addLayout(power_info_layout)

        # Set current control
        set_current_layout = QHBoxLayout()
        self.set_current_spinbox = QDoubleSpinBox(self)
        self.set_current_spinbox.setRange(0, 0.5)  # Adjust the range as needed
        self.set_current_spinbox.setValue(0)
        set_current_layout.addWidget(QLabel("Set Current (A):", self))
        set_current_layout.addWidget(self.set_current_spinbox)

        self.set_current_button = QPushButton("Set Current", self)
        self.set_current_button.clicked.connect(self.set_power_supply_current)
        set_current_layout.addWidget(self.set_current_button)

        display_layout.addLayout(set_current_layout)

        # Set voltage control
        set_voltage_layout = QHBoxLayout()
        self.set_voltage_spinbox = QDoubleSpinBox(self)
        self.set_voltage_spinbox.setRange(0, 50)  # Adjust the range as needed
        self.set_voltage_spinbox.setValue(0)
        set_voltage_layout.addWidget(QLabel("Set Voltage (V):", self))
        set_voltage_layout.addWidget(self.set_voltage_spinbox)

        self.set_voltage_button = QPushButton("Set Voltage", self)
        self.set_voltage_button.clicked.connect(self.set_power_supply_voltage)
        set_voltage_layout.addWidget(self.set_voltage_button)

        display_layout.addLayout(set_voltage_layout)

        # Turn on/off power supply buttons
        self.turn_on_button = QPushButton("Turn On Power Supply", self)
        self.turn_on_button.clicked.connect(self.power_supply_thread.turn_on)
        display_layout.addWidget(self.turn_on_button)

        self.turn_off_button = QPushButton("Turn Off Power Supply", self)
        self.turn_off_button.clicked.connect(self.power_supply_thread.turn_off)
        display_layout.addWidget(self.turn_off_button)

        # Measured current and voltage display in the same line
        measured_info_layout = QHBoxLayout()
        
        self.measured_current_label = QLabel("Measured Current: --- A", self)
        measured_info_layout.addWidget(self.measured_current_label)
        
        self.measured_voltage_label = QLabel("Measured Voltage: --- V", self)
        measured_info_layout.addWidget(self.measured_voltage_label)
        
        display_layout.addLayout(measured_info_layout)

        # Power supply voltage and current real-time plot setup
        self.ps_plot_widget = pg.PlotWidget(title="Power Supply Voltage and Current Over Time")
        self.ps_plot_widget.setLabel('left', 'Voltage (V)')
        self.ps_plot_widget.setLabel('bottom', 'Time (s)')
        self.ps_plot_widget.setYRange(0, 50)  # Adjust the y-axis range for voltage as needed

        # Set the left axis color to match the voltage curve (blue)
        self.ps_plot_widget.getAxis('left').setPen(pg.mkPen(color='b'))  # 'b' represents blue

        # Create a second y-axis for current
        self.ps_plot_widget.showAxis('right')
        self.ps_plot_widget.setLabel('right', 'Current (A)')
        self.ps_plot_widget.getAxis('right').setPen(pg.mkPen(color='r'))

        # Create a new ViewBox for the second y-axis (current)
        self.current_viewbox = pg.ViewBox()
        self.ps_plot_widget.scene().addItem(self.current_viewbox)
        self.ps_plot_widget.getAxis('right').linkToView(self.current_viewbox)

        # Set the range for the second y-axis (current)
        self.current_viewbox.setYRange(0, 0.5)  # Set range for current from 0 to 0.5 A

        # Create curves for voltage and current
        self.ps_voltage_curve = self.ps_plot_widget.plot(self.time_history, self.ps_voltage, pen='b', name='Voltage')

        # Create the current curve in the second ViewBox
        self.ps_current_curve = pg.PlotCurveItem(self.time_history, self.ps_current, pen=pg.mkPen(color='r'), name='Current')
        self.current_viewbox.addItem(self.ps_current_curve)  # Add current curve to the ViewBox
        self.ps_current_curve.setZValue(1)  # Ensure current curve is on top

        # Sync the x-axis of the voltage and current plots
        self.ps_plot_widget.getViewBox().sigResized.connect(self.update_views)

        # Add legend to the plot
        self.ps_plot_widget.addLegend()

        # Add plot widget to the layout
        display_layout.addWidget(self.ps_plot_widget)

        # Voltage channel checkboxes to toggle channels
        self.voltage_checkboxes = []
        voltage_checkbox_layout = QGridLayout()
        for i in range(self.voltage_channels):
            checkbox = QCheckBox(f"Channel {i+1}")
            checkbox.setChecked(True)
            checkbox.stateChanged.connect(self.toggle_voltage_curve)
            self.voltage_checkboxes.append(checkbox)
            voltage_checkbox_layout.addWidget(checkbox, i // 5, i % 5)
        display_layout.addLayout(voltage_checkbox_layout)

        # Grid layout to display the first 10 voltages - placed before the voltage curve
        self.voltage_labels = [QLabel(f"Voltage {i+1}: --- V") for i in range(10)]
        voltage_label_layout = QGridLayout()
        for i, label in enumerate(self.voltage_labels):
            voltage_label_layout.addWidget(label, i // 5, i % 5)
        display_layout.addLayout(voltage_label_layout)

        # Voltage plot setup (new) with legend
        self.voltage_plot_widget = pg.PlotWidget(title="Voltage Channels Over Time")
        self.voltage_plot_widget.setLabel('left', 'Voltage (V)')
        self.voltage_plot_widget.setLabel('bottom', 'Time (s)')

        # Add the legend to the voltage plot
        self.voltage_plot_widget.addLegend(offset=(10, 10))

        self.voltage_curves = []
        for i in range(self.voltage_channels):  # Assuming voltage_channels = 10
            curve = self.voltage_plot_widget.plot(self.time_history, self.voltage_data[i], pen=(i, self.voltage_channels), name=f"Channel {i+1}")
            self.voltage_curves.append(curve)

        # Add voltage plot under the checkboxes and labels
        display_layout.addWidget(self.voltage_plot_widget)

        # Add the pressure display (label + plot) to the center left of the main layout
        main_layout.addLayout(display_layout)

        # Right side layout for servo controls, leakage, voltage, relay controls
        control_layout = QVBoxLayout()

        # Servo controls layout
        servo_layout = QGridLayout()
        for scs_id in self.servos.keys():
            servo_control_widget = self.create_servo_control_widget(scs_id)
            servo_layout.addWidget(servo_control_widget, (scs_id - 1) // 2, (scs_id - 1) % 2)
        control_layout.addLayout(servo_layout)

        self.io_openbutton = QPushButton("Open io", self)
        self.io_openbutton.clicked.connect(self.open)
        control_layout.addWidget(self.io_openbutton)

        # Leak detection indicator
        leak_layout = QHBoxLayout()
        self.leak_label = QLabel("Leak Status: ---", self)
        self.leak_indicator = QFrame(self)
        self.leak_indicator.setFixedSize(20, 20)
        self.leak_indicator.setStyleSheet("background-color: gray; border-radius: 10px;")
        leak_layout.addWidget(self.leak_label)
        leak_layout.addWidget(self.leak_indicator)
        control_layout.addLayout(leak_layout)

        # Relay control checkboxes
        self.relay_checkboxes = [QCheckBox(f"Channel {i+1}") for i in range(16)]
        relay_layout = QGridLayout()
        for i, checkbox in enumerate(self.relay_checkboxes):
            relay_layout.addWidget(checkbox, i // 4, i % 4)
        control_layout.addLayout(relay_layout)

        # Button to apply relay control
        self.apply_button = QPushButton("Apply Relay Control", self)
        self.apply_button.clicked.connect(self.apply_relay_control)
        control_layout.addWidget(self.apply_button)

        # Clearer relay status display using indicator frames
        self.relay_status_labels = [QLabel(f"Channel {i+1}: ---") for i in range(16)]
        self.relay_status_indicators = [QFrame(self) for _ in range(16)]

        relay_status_layout = QGridLayout()
        for i, (label, indicator) in enumerate(zip(self.relay_status_labels, self.relay_status_indicators)):
            indicator.setFixedSize(20, 20)
            indicator.setStyleSheet("background-color: grey; border-radius: 10px;")
            relay_status_layout.addWidget(label, i // 4, (i % 4) * 2)
            relay_status_layout.addWidget(indicator, i // 4, (i % 4) * 2 + 1)
        control_layout.addLayout(relay_status_layout)

        # Add the control layout to the main layout (right side)
        main_layout.addLayout(control_layout)

        # Set the main layout for the window
        self.setLayout(main_layout)
    
    def open(self):
        self.servo_thread.io_open_signal.emit([1,2,3,4])

    def update_views(self):
        """Sync the second y-axis with the main plot when resizing occurs."""
        self.current_viewbox.setGeometry(self.ps_plot_widget.getViewBox().sceneBoundingRect())
        self.current_viewbox.linkedViewChanged(self.ps_plot_widget.getViewBox(), self.current_viewbox.XAxis)

    def create_servo_control_widget(self, scs_id):
        """Create a widget for controlling a single servo."""
        widget = QWidget()
        layout = QVBoxLayout()

        # Label to display the current servo position and speed
        info_label = QLabel(f"Servo {scs_id} Info: ", self)
        info_label.setObjectName(f"servo_info_{scs_id}")
        layout.addWidget(info_label)

        # Slider to control the servo position
        position_slider = QSlider(Qt.Horizontal, self)
        position_slider.setMinimum(2047)
        position_slider.setMaximum(3071)
        position_slider.setValue(2047)
        position_slider.setObjectName(f"servo_slider_{scs_id}")
        position_slider.valueChanged.connect(lambda value, sid=scs_id: self.slider_moved(sid, value))
        layout.addWidget(position_slider)

        # Switch button (Open/Close)
        switch_button = QPushButton('Open', self)
        switch_button.setCheckable(True)
        switch_button.setChecked(False)
        switch_button.setObjectName(f"servo_button_{scs_id}")
        switch_button.clicked.connect(lambda checked, sid=scs_id: self.toggle_servo_position(sid, checked))
        layout.addWidget(switch_button)

        widget.setLayout(layout)
        return widget

    def apply_relay_control(self):
        """Apply the relay control based on the checkboxes."""
        channels = []
        states = []
        for i, checkbox in enumerate(self.relay_checkboxes):
            channels.append(i + 1)  # Channels are 1-based
            states.append(1 if checkbox.isChecked() else 0)  # 1 for ON, 0 for OFF

        self.relay_thread.control_relay(channels, states)

    def update_leak_status(self, leak_detected):
        if leak_detected:
            self.leak_label.setText("Leak Status: Leak Detected")
            self.leak_indicator.setStyleSheet("background-color: red; border-radius: 10px;")
        else:
            self.leak_label.setText("Leak Status: No Leak Detected")
            self.leak_indicator.setStyleSheet("background-color: green; border-radius: 10px;")

    def update_pressure(self, pressure):
        self.pressure_label.setText(f"Inlet pressure of reactor: {pressure:.3f} MPa")

    def update_plots(self, data):
        """Update both the pressure and voltage plots."""
        pressure_history = data['pressure']
        voltage_data = data['voltages']
        flow_history = data['flow_rate']
        ps_current = data['ps_current']
        ps_voltage = data['ps_voltage']

        # Update the pressure plot
        
        self.pressure_curve.setData(self.time_history, pressure_history)

        # Update the voltage plot with selected channels
        for i, curve in enumerate(self.voltage_curves):
            if self.voltage_checkboxes[i].isChecked():
                curve.setData(self.time_history, voltage_data[i])
            else:
                curve.setData([], [])  # Hide the curve if checkbox is unchecked

        # Update the pump flow rate plot
        self.pump_flow_curve.setData(self.time_history, flow_history)
        now = datetime.datetime.now().strftime('%H:%M:%S')
        self.pump_flow_plot_widget.setLabel('bottom', f'Time (seconds) - Now: {now}')

        #update the power supply voltage and current plot
        self.ps_voltage_curve.setData(self.time_history, ps_voltage)
        self.ps_current_curve.setData(self.time_history, ps_current)

    def toggle_voltage_curve(self):
        """Update the voltage plot when channel selection changes."""
        self.update_plots({'pressure': self.pressure_history, 'voltages': self.voltage_data})

    def update_voltages(self, voltages):
        for i, voltage in enumerate(voltages[:10]):
            self.voltage_labels[i].setText(f"Voltage {i+1}: {voltage:.2f} V")

    def toggle_servo_position(self, servo_id, checked):
        position = 3071 if checked else 2047
        self.servo_thread.write_position_signal.emit(servo_id, position)

    def slider_moved(self, servo_id, position):
        self.servo_thread.write_position_signal.emit(servo_id, position)

    def update_servo_info(self, servo_id, pos, speed, temp, load):
        # Update the specific servo's info label and slider
        info_label = self.findChild(QLabel, f"servo_info_{servo_id}")
        position_slider = self.findChild(QSlider, f"servo_slider_{servo_id}")
        switch_button = self.findChild(QPushButton, f"servo_button_{servo_id}")

        if info_label and position_slider and switch_button:
            info_label.setText(f"Servo {servo_id} - Position: {pos}, Speed: {speed}, Load: {load}, Temperature: {temp} ℃")
            position_slider.blockSignals(True)
            position_slider.setValue(pos)
            position_slider.blockSignals(False)

            if pos >= 3040:
                switch_button.blockSignals(True)
                switch_button.setChecked(True)
                switch_button.setText("Close")
                switch_button.blockSignals(False)
            elif pos <= 2100:
                switch_button.blockSignals(True)
                switch_button.setChecked(False)
                switch_button.setText("Open")
                switch_button.blockSignals(False)
            else:
                switch_button.blockSignals(True)
                switch_button.setChecked(True)
                switch_button.setText("Adjusting")
                switch_button.blockSignals(False)

    def update_relay_states(self, states):
        """Update the relay states with clear status labels and indicators."""
        for i, state in enumerate(states):
            self.relay_status_labels[i].setText(f"Channel {i+1}: {'ON' if state else 'OFF'}")
            color = "green" if state else "red"
            self.relay_status_indicators[i].setStyleSheet(f"background-color: {color}; border-radius: 10px;")

    def handle_relay_response(self, response):
        print(f"Relay control response: {response}")

    def update_pump_pressure(self, pressure):
        self.pump_pressure_label.setText(f"Pump Outlet Pressure: {pressure:.3f} Bar")
    
    def update_pump_flow(self, flow):
        self.pump_flow_label.setText(f"Pump Flow Rate: {flow:.3f} L/h")

    def update_pump_stroke(self, stroke):
        self.pump_stroke_label.setText(f"Pump Stroke: {stroke:.3f} %")
    
    def update_pump_status(self, status):
        self.pump_state_label.setText(f"Pump State: {status}")

    def set_pump_stroke(self):
        stroke = self.stroke_spinbox.value()
        self.pump_thread.set_stroke(stroke)

    def update_ps_state(self, state):
        self.power_state_label.setText(f"Power Supply State: {state}")
    
    def update_ps_current(self, current):   
        self.measured_current_label.setText(f"Measured Current: {current} A")

    def update_ps_voltage(self, voltage):
        self.measured_voltage_label.setText(f"Measured Voltage: {voltage} V")

    def update_ps_power(self, power):
        self.measured_power_label.setText(f"Measured Power: {power} W")

    def set_power_supply_current(self):
        current = self.set_current_spinbox.value()
        self.power_supply_thread.set_current(current)

    def set_power_supply_voltage(self): 
        voltage = self.set_voltage_spinbox.value()
        self.power_supply_thread.set_voltage(voltage)

    def closeEvent(self, event):
        self.power_supply_thread.stop()
        self.pump_thread.stop()
        self.servo_thread.stop()
        self.voltage_thread.stop()
        self.leakage_sensor_thread.stop()
        self.pressure_sensor_thread.stop()
        self.relay_thread.stop()
        self.data_updater.stop()
        self.portHandler.closePort()
        self.pump_control.close_connection()
        self.voltage_collector.close_connection()
        self.leakage_sensor.close_connection()
        self.pressure_sensor.close_connection()
        self.relay_control.close_connection()
        self.power_supply.close_connection()
        event.accept()


# Main program
if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = MainGUI()
    gui.show()
    sys.exit(app.exec())
