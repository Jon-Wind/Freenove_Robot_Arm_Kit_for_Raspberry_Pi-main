# - Connecting to the robot arm via TCP/IP
# - Manual control of arm movements (X, Y, Z axes)
# - Image processing for automated drawing
# - Recording and playback of command sequences
# - Servo and buzzer control

import sys
import os
import threading
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from fileDialogHelper import FileDialogHelper  # Custom file dialog utility
import cv2  # OpenCV for image processing
from client import Client  # TCP/IP client for robot communication
from ui.ui_arm import Ui_Arm  # Auto-generated UI from Qt Designer
from configuration import Configuration  # Configuration window
from led import LED  # LED control window
import messageThread  # Thread management utilities
import messageParser  # Command parsing utilities
import messageRecord  # Data persistence utilities
import messageQueue  # Queue management for commands
import command  # Command definitions and constants
import numpy as np  # Numerical operations for image processing
from datetime import datetime  # Timestamp generation
import platform  # OS detection for cross-platform compatibility

# Opens up a UI Window on the Laptop when run
# Inherits from QMainWindow and the auto-generated UI class
class myClientWindow(QMainWindow, Ui_Arm):
    # PyQt signals for thread-safe communication
    ui_arm_btn_connect = QtCore.pyqtSignal(str)  # Signal for connection status updates
    threading_cmd = QtCore.pyqtSignal(str)  # Signal for sending commands to robot

    def __init__(self, parent=None):
        """
        Initialize the main window and all control systems
        Sets up the GUI, networking, image processing, and robot control
        """
        super(myClientWindow, self).__init__(parent)
        self.setupUi(self)  # Setup the UI from the designer file
        self.setFixedSize(1074, 617)  # Lock window size to prevent resizing issues
        
        # Image processing variables (For it's drawing aspect)
        self.binary_img = None  # Stores binary (black/white) processed image
        self.contour_img = None  # Stores image with detected contours
        self.hierarchy_data = None  # OpenCV contour hierarchy data
        self.raw_img = None  # Original imported image
        self.gray_img = None  # Grayscale version of image
        self.folder_path = None  # Path for file operations
        
        # Image processing parameters with default values
        self.threshold_value = 100  # Binary threshold for image conversion
        self.gauss_value = 3  # Gaussian blur radius for noise reduction
        self.sharpen_value = 5  # Sharpening filter strength
        self.img_flag = 0  # Current image processing state flag
        

        # Ui window reference
        self.configurationWindow = None  # Reference to configuration dialog
        self.ledWindow = None  # Reference to LED control dialog
        
        # Robot communication and control
        self.read_cmd_handling = None  # Thread for receiving robot responses
        self.message_handling = None  # Thread for processing received messages
        
        # Movement control parameters
        self.main_position_mode_length = 0.0  # Position mode movement distance
        self.main_clamp_mode_length = 40.0  # Clamp mode movement distance
        self.main_servo_mode_length = 50.0  # Servo mode movement distance
        
        # Core system objects
        self.client = Client()  # TCP/IP client for robot communication
        self.record = messageRecord.MessageRecord()  # Handles data persistence
        
        # Video/image setup
        self.label_Arm_Video.setScaledContents(False)  # Maintain aspect ratio for images
        
        # Input valudation
        # IP address validator using regex pattern
        ip_validator = QRegExpValidator(QRegExp('^((2[0-4]\d|25[0-5]|\d?\d|1\d{2})\.){3}(2[0-4]\d|25[0-5]|[01]?\d\d?)$'))
        self.lineEdit_Arm_IP_Address.setValidator(ip_validator)
        
        # Servo angle validator (0-180 degrees)
        servo_validator = QRegExpValidator(QRegExp('^(?:[1-9][0-9]?|1[0-7][0-9]?|180)$'))
        self.lineEdit_Arm_Servo_Angle_Value.setValidator(servo_validator)
        
        # Robot arm connection setup
        # Load and set the previously used IP address
        self.lineEdit_Arm_IP_Address.setText(self.record.read_remote_ip())
        self.client.ip = self.lineEdit_Arm_IP_Address.text()
        print("Remote IP: " + self.client.ip)
        
        # Message process system
        self.message_parser = messageParser.MessageParser()  # Parses incoming robot messages
        self.gcode_command = messageQueue.MessageQueue()  # Queue for G-code commands
        self.painter_point = messageQueue.MessageQueue()  # Queue for drawing points
        self.send_g_code_state = False  # Flag indicating if G-code is being sent
        self.cmd = command.Command()  # Command constants and builders
        self.client_busy = False  # Flag to prevent command conflicts
        
        # Robot arm position tracking
        # Load the saved home/origin position
        self.original_position_axis = self.record.read_position_point()
        self.ui_arm_show_label_axis(self.original_position_axis)  # Display current position
        self.record_last_command = None  # Stores the last command sent for recording
        self.record_area_data_queue = messageQueue.MessageQueue()  # Queue for recorded commands
        
        # Coordinate system mapping(Grid)
        self.last_axis_point = [0, 200]  # Last known robot position
        self.label_videl_size = [self.label_Arm_Video.width(), self.label_Arm_Video.height()]  # UI display size
        self.axis_map_x = [-100, 100]  # X-axis coordinate mapping range
        self.axis_map_y = [250, 150]  # Y-axis coordinate mapping range
        
        # Drawing system
        self.lastPoint = [0, 0]  # Previous drawing point
        self.currentPoint = [0, 0]  # Current drawing point
        self.isDrawing = False  # Flag indicating if currently drawing
        self.painter_line_style = 0  # Drawing line style
        self.contours_data = None  # Processed contour data for drawing
        
        # Create a white canvas for drawing operations
        self.white_image = np.zeros((self.label_videl_size[1], self.label_videl_size[0], 3), dtype=np.uint8)
        self.white_image[:, :, :] = [255, 255, 255]  # Fill with white color
        
        # Convert numpy array to Qt image format and display
        img = QImage(self.white_image.data.tobytes(), self.white_image.shape[1], 
                    self.white_image.shape[0], QImage.Format_RGB888)
        self.label_Arm_Video.setPixmap(QPixmap.fromImage(img))
        self.original_label_img = self.white_image.copy()  # Keep backup of clean canvas
    
        # Drawing mode variables
        self.line_curves_count = 1  # Total number of curves to draw
        self.current_line_curves = 0  # Current curve being drawn
        
        # Robot controll parameters
        self.angle_value = 90  # Current servo angle
        self.arm_switched_value = 10  # Movement step multiplier (1x or 10x)
        self.arm_move_value = 1.0  # Base movement increment
        self.arm_command_count = 0  # Counter for commands sent

        # Filing system
        self.filePath = FileDialogHelper()  # Helper for file open/save dialogs
        
        
        # Initialize connections
        self.connect()  # Set up all signal-slot connections

    def connect(self):
        """
        Connects Code components to the Hardware of the Robotic Arm
        Sets up all signal-slot connections for the GUI controls
        """
        # network connection controlls
        # Save IP address whenever it changes
        self.lineEdit_Arm_IP_Address.textChanged.connect(self.save_ip_address)
        # Connect/disconnect button
        self.pushButton_Arm_Connect.clicked.connect(self.btn_connect_remote_ip)
        # Internal signal for updating connection status
        self.ui_arm_btn_connect.connect(self.ui_arm_show_btn_connect_content)
        # Internal signal for sending commands safely across threads
        self.threading_cmd.connect(self.socket_send)
        
        # Basic controll buttons 
        self.pushButton_Arm_Stop_Arm.clicked.connect(self.btn_stop_arm)  # Emergency stop
        self.pushButton_Arm_Zero_Point.clicked.connect(self.btn_move_to_zero_point)  # Go to origin
        self.pushButton_Arm_Load_Relax.clicked.connect(self.btn_load_relax_arm)  # Enable/disable motors
        
        # Buzzer control (press and hold)
        self.pushButton_Arm_Buzzer.pressed.connect(lambda: self.btn_control_buzzer(self.pushButton_Arm_Buzzer))
        self.pushButton_Arm_Buzzer.released.connect(lambda: self.btn_control_buzzer(self.pushButton_Arm_Buzzer))
        
        # Position control buttons
        self.pushButton_Arm_Position.clicked.connect(self.btn_move_to_position_point)  # Go to saved position
        self.pushButton_Arm_Home_Up.clicked.connect(self.btn_move_to_above_home_point)  # Go to position + height offset
        
        # AXIS movement control (Manual Jogging)
        # X-axis movement buttons
        self.pushButton_Arm_Axis_X_Subtract.clicked.connect(lambda: self.btn_move_to_anywhere(self.pushButton_Arm_Axis_X_Subtract))
        self.pushButton_Arm_Axis_X_Add.clicked.connect(lambda: self.btn_move_to_anywhere(self.pushButton_Arm_Axis_X_Add))
        
        # Y-axis movement buttons
        self.pushButton_Arm_Axis_Y_Subtract.clicked.connect(lambda: self.btn_move_to_anywhere(self.pushButton_Arm_Axis_Y_Subtract))
        self.pushButton_Arm_Axis_Y_Add.clicked.connect(lambda: self.btn_move_to_anywhere(self.pushButton_Arm_Axis_Y_Add))
        
        # Z-axis movement buttons
        self.pushButton_Arm_Axis_Z_Subtract.clicked.connect(lambda: self.btn_move_to_anywhere(self.pushButton_Arm_Axis_Z_Subtract))
        self.pushButton_Arm_Axis_Z_Add.clicked.connect(lambda: self.btn_move_to_anywhere(self.pushButton_Arm_Axis_Z_Add))
        
        # Step size toggle (1x vs 10x movement)
        self.pushButton_Arm_Axis_Switched.clicked.connect(lambda: self.btn_move_to_anywhere(self.pushButton_Arm_Axis_Switched))
        
        # Command recording and playback system
        self.pushButton_Arm_Record_Command.clicked.connect(lambda: self.btn_arm_operation(self.pushButton_Arm_Record_Command))
        self.pushButton_Arm_Withdraw_Command.clicked.connect(lambda: self.btn_arm_operation(self.pushButton_Arm_Withdraw_Command))
        self.pushButton_Arm_Execute_Record_Command.clicked.connect(lambda: self.btn_arm_operation(self.pushButton_Arm_Execute_Record_Command))
        self.pushButton_Arm_Import_Record_Command.clicked.connect(lambda: self.btn_arm_operation(self.pushButton_Arm_Import_Record_Command))
        self.pushButton_Arm_Save_Record_Command.clicked.connect(lambda: self.btn_arm_operation(self.pushButton_Arm_Save_Record_Command))
        
        # Drawing mode selection
        self.radioButton_Arm_Img_Mode.clicked.connect(lambda: self.radioButton_arm_select_mode(self.radioButton_Arm_Img_Mode))
        self.radioButton_Arm_Line_Mode.clicked.connect(lambda: self.radioButton_arm_select_mode(self.radioButton_Arm_Line_Mode))
        self.radioButton_Arm_Curves_Mode.clicked.connect(lambda: self.radioButton_arm_select_mode(self.radioButton_Arm_Curves_Mode))
        
        # Image processing control
        self.pushButton_Arm_Import_Picture.clicked.connect(lambda: self.set_img_action(self.pushButton_Arm_Import_Picture))
        self.pushButton_Arm_Gray_Picture.clicked.connect(lambda: self.set_img_action(self.pushButton_Arm_Gray_Picture))
        self.pushButton_Arm_Binaryzation_Picture.clicked.connect(lambda: self.set_img_action(self.pushButton_Arm_Binaryzation_Picture))
        self.pushButton_Arm_Contour_Picture.clicked.connect(lambda: self.set_img_action(self.pushButton_Arm_Contour_Picture))
        self.pushButton_Arm_Clear_All.clicked.connect(lambda: self.set_img_action(self.pushButton_Arm_Clear_All))
        self.pushButton_Arm_Change_Gcode.clicked.connect(lambda: self.set_img_action(self.pushButton_Arm_Change_Gcode))
        self.pushButton_Arm_Execute_Gcode.clicked.connect(lambda: self.set_img_action(self.pushButton_Arm_Execute_Gcode))
        
        # Image processing sliders 
        # Cross-platform compatibility: Windows uses sliderReleased, others use valueChanged
        if platform.system() == "Windows":
            self.horizontalSlider_Arm_Threshold.sliderReleased.connect(lambda: self.img_slider_control(self.horizontalSlider_Arm_Threshold))
            self.horizontalSlider_Arm_Gauss.sliderReleased.connect(lambda: self.img_slider_control(self.horizontalSlider_Arm_Gauss))
            self.horizontalSlider_Arm_Sharpen.sliderReleased.connect(lambda: self.img_slider_control(self.horizontalSlider_Arm_Sharpen))
            self.horizontalSlider_Arm_Pen_Height.sliderReleased.connect(lambda: self.img_slider_control(self.horizontalSlider_Arm_Pen_Height))
        else:
            self.horizontalSlider_Arm_Threshold.valueChanged.connect(lambda: self.img_slider_control(self.horizontalSlider_Arm_Threshold))
            self.horizontalSlider_Arm_Gauss.valueChanged.connect(lambda: self.img_slider_control(self.horizontalSlider_Arm_Gauss))
            self.horizontalSlider_Arm_Sharpen.valueChanged.connect(lambda: self.img_slider_control(self.horizontalSlider_Arm_Sharpen))
            self.horizontalSlider_Arm_Pen_Height.valueChanged.connect(lambda: self.img_slider_control(self.horizontalSlider_Arm_Pen_Height))
        
        # Servo controll
        self.pushButton_Arm_Servo_Turn_On.clicked.connect(lambda: self.control_servo_angle(self.pushButton_Arm_Servo_Turn_On))
        self.pushButton_Arm_Servo_Turn_Off.clicked.connect(lambda: self.control_servo_angle(self.pushButton_Arm_Servo_Turn_Off))
    
        # Configuration windows
        self.pushButton_Arm_Parameter_UI.clicked.connect(self.configure_parameter_ui)  # Open parameter settings
        self.pushButton_Arm_Led_UI.clicked.connect(self.configure_led_ui)  # Open LED control panel

    # Utility functions
    @staticmethod
    def map(value, fromLow, fromHigh, toLow, toHigh, num):
        """
        Maps a value from one range to another (similar to Arduino map function)
        
        Args:
            value: Input value to map
            fromLow, fromHigh: Input range
            toLow, toHigh: Output range  
            num: Number of decimal places to round to
            
        Returns:
            Mapped and rounded value
        """
        return round(((toHigh - toLow) * (value - fromLow) / (fromHigh - fromLow) + toLow), num)

    @staticmethod
    def constrain(value, range_min, range_max):
        """
        Constrains a value within specified minimum and maximum bounds
        
        Args:
            value: Input value to constrain
            range_min: Minimum allowed value
            range_max: Maximum allowed value
            
        Returns:
            Constrained value
        """
        if value > range_max:
            value = range_max
        if value < range_min:
            value = range_min
        return value

    def record_command(self, cmd):
        """
        Records the last command sent to the robot for potential replay
        
        Args:
            cmd: Command string that was sent to robot
        """
        self.record_last_command = cmd
    
    # Communication functions
    def socket_send(self, cmd):
        """
        Sends commands to the robot arm via TCP/IP connection
        Includes safety checks to ensure robot is ready to receive commands
        
        Args:
            cmd: G-code or custom command string to send
        """
        if self.client.connect_flag:
            # Check if motors are enabled (robot is ready to move)
            if self.pushButton_Arm_Load_Relax.text() == "Relax Motor":  # Motors are ON
                self.client_busy = True
                self.client.send_messages(cmd + "\r\n")  # Send command with line terminator
                self.record_command(cmd)  # Store for recording feature
                self.client_busy = False
            else:
                # Allow servo commands even when motors are off
                if cmd.split(" ")[0][0] == "S":  # Servo G-code commands start with S
                    self.client_busy = True
                    self.client.send_messages(cmd + "\r\n")
                    self.record_command(cmd)
                    self.client_busy = False
                else:
                    print("Please press the \"Load Motor\" button first.")
        else:
            print("Connect the remote IP address first.")

    def ui_arm_show_label_axis(self, axis):
        """
        Updates the GUI labels showing current robot position
        
        Args:
            axis: List containing [X, Y, Z] coordinates
        """
        self.label_Arm_Axis_X_Value.setText(str(axis[0]))  # X coordinate
        self.label_Arm_Axis_Y_Value.setText(str(axis[1]))  # Y coordinate
        self.label_Arm_Axis_Z_Value.setText(str(axis[2]))  # Z coordinate

    def ui_arm_show_record_area(self):
        """
        Updates the text area showing recorded commands
        Clears the display and shows all commands in the queue
        """
        self.textEdit_Arm_Record_Area.clear()
        cmd = self.record_area_data_queue.gets()  # Get all commands from queue
        for i in range(self.record_area_data_queue.len()):
            self.textEdit_Arm_Record_Area.append(cmd[i])  # Add each command to display

    def save_ip_address(self):
        """
        Saves the IP Address of the Robotic Arm to persistent storage
        Called whenever the IP address text field changes
        """
        self.record.write_remote_ip(self.lineEdit_Arm_IP_Address.text())

    def process_message(self):
        """
        Processes the messages received from the Robotic Arm
        Runs in a separate thread to handle robot responses
        Manages G-code command queuing and execution
        """
        while True:
            # Check if connection is still active
            if not self.client.connect_flag:
                self.client.disconnect()
                print("Disconnected the remote ip.")
                self.ui_arm_btn_connect.emit("Connect")  # Update UI to show disconnected state
                break
            
            # Process incoming messages from robot
            if self.client.data_queue.empty() is not True:
                try:
                    buf = self.client.data_queue.get()  # Get message from queue
                    self.message_parser.parser(buf)  # Parse the received message
                    
                    # Handle custom action responses
                    if self.message_parser.commandArray[0] == self.cmd.CUSTOM_ACTION:
                        if self.message_parser.commandArray[1] == self.cmd.ARM_QUERY:
                            # Robot is asking for next command in sequence
                            if self.gcode_command.len() > 0:
                                self.arm_command_count = self.message_parser.intParameter[1]
                                self.send_g_code_state = True
                            elif self.gcode_command.len() == 0:
                                # No more commands to send
                                self.arm_command_count = 0
                                self.send_g_code_state = False
                                # Send completion signal to robot
                                cmd = self.cmd.CUSTOM_ACTION + str('12') + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_QUERY + str('0')
                                self.threading_cmd.emit(cmd)
                        else:
                            print(self.message_parser.inputCommandArray)
                    else:
                        print(self.message_parser.inputCommandArray)
                        self.message_parser.clearParameters()
                except:
                    pass  # Continue on parsing errors

    # Connection to the UI - enhancement function for the robot arm.
    def btn_connect_remote_ip(self):
        """
        Handles connection/disconnection to the robot arm
        Manages connection state, starts/stops communication threads
        Plays connection confirmation beep
        """
        if self.pushButton_Arm_Connect.text() == "Connect":  # Currently disconnected, try to connect
            self.client.ip = self.lineEdit_Arm_IP_Address.text()
            if self.client.connect(self.client.ip):
                print("Connected the remote ip.")
                
                # Play a short beep when connected and turn it off after 0.7 seconds
                if self.client.connect_flag:
                    self.client_busy = True
                    # Turn on the buzzer
                    cmd_on = self.cmd.CUSTOM_ACTION + str("2") + self.cmd.DECOLLATOR_CHAR + self.cmd.BUZZER_ACTION + str("700")
                    self.threading_cmd.emit(cmd_on)
                    
                    # Schedule the buzzer to turn off after 0.7 seconds
                    def stop_buzzer():
                        if self.client.connect_flag:  # Only if still connected
                            self.client_busy = True
                            cmd_off = self.cmd.CUSTOM_ACTION + str("2") + self.cmd.DECOLLATOR_CHAR + self.cmd.BUZZER_ACTION + str("0")
                            self.threading_cmd.emit(cmd_off)
                            self.client_busy = False
                    
                    # Use timer to stop buzzer automatically
                    timer = threading.Timer(0.7, stop_buzzer)
                    timer.daemon = True  # This ensures the timer won't prevent program exit
                    timer.start()
                    
                    self.client_busy = False
                
                # Start communication threads
                self.read_cmd_handling = threading.Thread(target=self.client.receive_messages)
                self.read_cmd_handling.start()
                self.message_handling = threading.Thread(target=self.process_message)
                self.message_handling.start()
                self.pushButton_Arm_Connect.setText("Disconnect")  # Update button text
            else:
                print("Failed to connect the remote ip.")
                self.pushButton_Arm_Connect.setText("Connect")
                
        elif self.pushButton_Arm_Connect.text() == "Disconnect":  # Currently connected, disconnect
            try:
                # Stop communication threads
                messageThread.stop_thread(self.read_cmd_handling)
                messageThread.stop_thread(self.message_handling)
                # Close any open configuration windows
                if self.configurationWindow != None:
                    self.configurationWindow.close()
                if self.ledWindow != None:
                    self.ledWindow.close()
            except:
                pass
            # Disconnect from robot
            self.client.disconnect()
            print("Disconnected the remote ip.")
            self.pushButton_Arm_Connect.setText("Connect")
            self.pushButton_Arm_Load_Relax.setText("Load Motor")  # Reset motor state display

    def ui_arm_show_btn_connect_content(self, content):
        """
        Changes the text of the Connect button (called via signal)
        Also closes any open configuration windows and resets motor state
        
        Args:
            content: New text for the connect button ("Connect" or "Disconnect")
        """
        if self.configurationWindow != None:
            self.configurationWindow.close()
        if self.ledWindow != None:
            self.ledWindow.close()
        self.pushButton_Arm_Load_Relax.setText("Load Motor")
        self.pushButton_Arm_Connect.setText(content)

    # Arm controll functions    
    def btn_stop_arm(self):
        """
        Emergency stop function - immediately stops the Robotic Arm
        Sends stop command and closes configuration windows
        """
        if self.client.connect_flag:
            self.client_busy = True
            # Send emergency stop command
            cmd = self.cmd.CUSTOM_ACTION + str("13") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_STOP + str("1")
            self.threading_cmd.emit(cmd)
            self.client_busy = False
            
            # Close configuration windows for safety
            if self.configurationWindow != None:
                self.configurationWindow.close()
            if self.ledWindow != None:
                self.ledWindow.close()
            self.pushButton_Arm_Load_Relax.setText("Load Motor")  # Reset motor state
        else:
            print("Connect the remote IP address first.")

    def btn_move_to_zero_point(self):
        """
        Moves the robot arm to its zero/origin position using sensor detection
        """
        cmd = self.cmd.CUSTOM_ACTION + str("10") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_SENSOR_POINT + str("1")
        self.threading_cmd.emit(cmd)

    def btn_load_relax_arm(self):
        """
        Toggles the motor enable/disable state
        When "Load Motor": Enables motors so arm can move
        When "Relax Motor": Disables motors so arm can be moved by hand
        """
        if self.client.connect_flag:
            cmd = None
            if self.pushButton_Arm_Load_Relax.text() == "Load Motor":
                # Enable motors (arm will hold position and can be commanded)
                cmd = self.cmd.CUSTOM_ACTION + str("8") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_ENABLE + str("0")
                self.pushButton_Arm_Load_Relax.setText("Relax Motor")
            elif self.pushButton_Arm_Load_Relax.text() == "Relax Motor":
                # Disable motors (arm can be moved by hand)
                cmd = self.cmd.CUSTOM_ACTION + str("8") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_ENABLE + str("1")
                self.pushButton_Arm_Load_Relax.setText("Load Motor")
            self.threading_cmd.emit(cmd)
        else:
            print("Connect the remote IP address first.")

    def btn_control_buzzer(self, index):
        """
        Controls the robot's buzzer for audio feedback
        Buzzer sounds while button is pressed, stops when released
        
        Args:
            index: The buzzer button widget
        """
        if self.client.connect_flag:
            self.client_busy = True
            cmd = None
            if index.isDown():  # Button just pressed
                # Turn on buzzer with 2000Hz frequency
                cmd = self.cmd.CUSTOM_ACTION + str("2") + self.cmd.DECOLLATOR_CHAR + self.cmd.BUZZER_ACTION + str("2000")
            elif not index.isDown():  # Button released
                # Turn off buzzer
                cmd = self.cmd.CUSTOM_ACTION + str("2") + self.cmd.DECOLLATOR_CHAR + self.cmd.BUZZER_ACTION + str("0")
            self.threading_cmd.emit(cmd)
            self.client_busy = False

    def btn_move_to_position_point(self):
        
        #Moves the robot to the saved home/origin position
         if self.client.connect_flag:
            if self.pushButton_Arm_Load_Relax.text() == "Relax Motor":  # Motors are enabled
                self.client_busy = True
                # Check if we need to run sensor calibration first
                if self.record_last_command == "S8 E0":  # Last command was motor enable
                    # Run sensor point calibration before moving
                    cmd = self.cmd.CUSTOM_ACTION + str("10") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_SENSOR_POINT + str("1")
                    self.threading_cmd.emit(cmd)
                    # Move to saved position
                    cmd = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_X_ACTION + str(self.original_position_axis[0]) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Y_ACTION + str(self.original_position_axis[1]) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Z_ACTION + str(self.original_position_axis[2])
                    self.threading_cmd.emit(cmd)
                    self.record_command(cmd)
                    self.last_axis_point = [round(float(self.original_position_axis[0]), 1),
                                            round(float(self.original_position_axis[1]), 1)]
                    self.ui_arm_show_label_axis(self.original_position_axis)
                else:
                    # Move directly to position without sensor calibration
                    cmd = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_X_ACTION + str(self.original_position_axis[0]) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Y_ACTION + str(self.original_position_axis[1]) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Z_ACTION + str(self.original_position_axis[2])
                    self.threading_cmd.emit(cmd)
                    self.record_command(cmd)
                    self.last_axis_point = [round(float(self.original_position_axis[0]), 1),
                                            round(float(self.original_position_axis[1]), 1)]
                    self.ui_arm_show_label_axis(self.original_position_axis)
                self.client_busy = False
            else:
                print("Please press the \"Load Motor\" button first.")
         else:
            print("Connect the remote IP address first.")

    def btn_move_to_above_home_point(self, enable):
        """
        Moves the robot to the saved position plus a height offset (pen-up position)
        Useful for moving to drawing start position without touching the surface
        """
        if self.client.connect_flag:
            if self.pushButton_Arm_Load_Relax.text() == "Relax Motor":
                self.client_busy = True
                # Get the pen height offset from slider
                home_up_height = self.horizontalSlider_Arm_Pen_Height.value()
                
                if self.record_last_command == "S8 E0":  # Need sensor calibration first
                    cmd = self.cmd.CUSTOM_ACTION + str("10") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_SENSOR_POINT + str("1")
                    self.threading_cmd.emit(cmd)
                    # Move to position + height offset
                    cmd = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_X_ACTION + str(self.original_position_axis[0]) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Y_ACTION + str(self.original_position_axis[1]) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Z_ACTION + str(float(self.original_position_axis[2]) + home_up_height)
                    self.threading_cmd.emit(cmd)
                    self.record_command(cmd)
                    self.last_axis_point = [round(float(self.original_position_axis[0]), 1),
                                            round(float(self.original_position_axis[1]), 1)]
                    # Update UI display with elevated Z position
                    ui_show_axis = self.original_position_axis.copy()
                    ui_show_axis[2] = float(ui_show_axis[2]) + home_up_height
                    self.ui_arm_show_label_axis(ui_show_axis)
                else:
                    # Move directly without sensor calibration
                    cmd = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_X_ACTION + str(self.original_position_axis[0]) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Y_ACTION + str(self.original_position_axis[1]) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Z_ACTION + str(float(self.original_position_axis[2]) + home_up_height)
                    self.threading_cmd.emit(cmd)
                    self.record_command(cmd)
                    self.last_axis_point = [round(float(self.original_position_axis[0]), 1),
                                            round(float(self.original_position_axis[1]), 1)]
                    ui_show_axis = self.original_position_axis.copy()
                    ui_show_axis[2] = float(ui_show_axis[2]) + home_up_height
                    self.ui_arm_show_label_axis(ui_show_axis)
                self.client_busy = False
            else:
                print("Please press the \"Load Motor\" button first.")
        else:
            print("Connect the remote IP address first.")

    def btn_move_to_anywhere(self, index):
        """
        Handles manual jogging of robot axes using directional buttons
        Supports both 1x and 10x step sizes for fine and coarse movement
        
        Args:
            index: The button widget that was clicked
        """
        # Get current position from UI labels
        int_axis = [0, 0, 0]
        int_axis[0] = float(self.label_Arm_Axis_X_Value.text())
        int_axis[1] = float(self.label_Arm_Axis_Y_Value.text())
        int_axis[2] = float(self.label_Arm_Axis_Z_Value.text())
        
        # Determine movement direction and axis based on button pressed
        if index.objectName() == "pushButton_Arm_Axis_X_Subtract":
            int_axis[0] = int_axis[0] - (self.arm_move_value * self.arm_switched_value)
        elif index.objectName() == "pushButton_Arm_Axis_X_Add":
            int_axis[0] = int_axis[0] + (self.arm_move_value * self.arm_switched_value)
        elif index.objectName() == "pushButton_Arm_Axis_Y_Subtract":
            int_axis[1] = int_axis[1] - (self.arm_move_value * self.arm_switched_value)
        elif index.objectName() == "pushButton_Arm_Axis_Y_Add":
            int_axis[1] = int_axis[1] + (self.arm_move_value * self.arm_switched_value)
        elif index.objectName() == "pushButton_Arm_Axis_Z_Subtract":
            int_axis[2] = int_axis[2] - (self.arm_move_value * self.arm_switched_value)
        elif index.objectName() == "pushButton_Arm_Axis_Z_Add":
            int_axis[2] = int_axis[2] + (self.arm_move_value * self.arm_switched_value)
        elif index.objectName() == "pushButton_Arm_Axis_Switched":
            # Toggle between 1x and 10x step size
            if self.arm_switched_value == 10:
                self.arm_switched_value = 1
                self.pushButton_Arm_Axis_Switched.setText("Step: X1")
            else:
                self.arm_switched_value = 10
                self.pushButton_Arm_Axis_Switched.setText("Step: X10")
        
        if self.client.connect_flag:
            if index.objectName() != "pushButton_Arm_Axis_Switched":  # Actual movement command
                if self.pushButton_Arm_Load_Relax.text() == "Relax Motor":
                    self.client_busy = True
                    # Build and send movement command
                    cmd = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_X_ACTION + str(round(int_axis[0], 1)) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Y_ACTION + str(round(int_axis[1], 1)) + self.cmd.DECOLLATOR_CHAR \
                        + self.cmd.AXIS_Z_ACTION + str(round(int_axis[2], 1))
                    self.threading_cmd.emit(cmd)
                    self.record_command(cmd)
                    self.last_axis_point = [round(int_axis[0], 1), round(int_axis[1], 1)]
                    self.client_busy = False
                else:
                    print("1, Please press the \"Load Motor\" button first.")
                    print("2, Please press the \"Sensor Point\" button second.")
        else:
            print("Connect the remote IP address first.")
        
        # Update position display on UI
        str_axis = [str(round(int_axis[i], 1)) for i in range(3)]
        self.ui_arm_show_label_axis(str_axis)

    # Command recording system
    def btn_arm_operation(self, index):
        """
        Handles recording, playback, and file operations for command sequences
        Allows users to record a series of movements and replay them
        
        Args:
            index: The button widget that was clicked
        """
        if index.objectName() == "pushButton_Arm_Record_Command":
            # Add the last command to the recording queue
            if self.client.connect_flag:
                if self.record_last_command is not None:
                    self.record_area_data_queue.put(self.record_last_command)
                    self.ui_arm_show_record_area()  # Update display
                else:
                    print("No instruction was detected to be logged.")
            else:
                print("Connect the remote IP address first.")
                
        elif index.objectName() == "pushButton_Arm_Withdraw_Command":
            # Remove the last recorded command (undo)
            if self.record_area_data_queue.len() > 0:
                self.record_area_data_queue.delete(self.record_area_data_queue.len() - 1)
                self.ui_arm_show_record_area()
            else:
                print("Withdraw failed. The record area is None.")
                
        elif index.objectName() == "pushButton_Arm_Execute_Record_Command":
            # Play back all recorded commands in sequence
            if self.client.connect_flag:
                self.client_busy = True
                record_area_data_queue_command = self.record_area_data_queue.gets()
                if record_area_data_queue_command is not None:
                    # Send each recorded command to the robot
                    for i in range(self.record_area_data_queue.len()):
                        self.threading_cmd.emit(record_area_data_queue_command[i])
                else:
                    print("Execute failed. The record area is None.")
                self.client_busy = False
            else:
                print("Connect the remote IP address first.")
                
        elif index.objectName() == "pushButton_Arm_Save_Record_Command":
            # Save recorded commands to a text file with timestamp
            if self.textEdit_Arm_Record_Area.toPlainText() != "":
                now = datetime.now()
                file_name = now.strftime("./Record/record_%Y%m%d_%H%M%S.txt")
                # Create file and write commands
                if os.path.exists(file_name) is True:
                    os.remove(file_name)
                fb = open(file_name, "w")
                fb.seek(0)
                fb.truncate()
                record_area_data_queue_command = self.record_area_data_queue.gets()
                for i in range(self.record_area_data_queue.len()):
                    fb.write(record_area_data_queue_command[i] + "\n")
                fb.close()
            else:
                print("No instructions need to be saved.")
                
        elif index.objectName() == "pushButton_Arm_Import_Record_Command":
            # Load previously saved command sequence from file
            if not os.path.exists("./record"):
                os.mkdir("./record")
            self.folder_path = self.filePath.getFilePath("./record/*")
            print(self.folder_path)
            if self.folder_path is not None:
                if self.folder_path.split(".")[1] == "txt":  # Verify it's a text file
                    fb = open(self.folder_path, "r")
                    self.record_area_data_queue.clear()  # Clear existing commands
                    line = fb.readline()
                    while line:  # Read each line as a command
                        txt_command = line.split("\n")[0]  # Remove newline character
                        self.record_area_data_queue.put(txt_command)
                        line = fb.readline()
                    self.ui_arm_show_record_area()  # Update display
                    fb.close()
            else:
                print("Cancel the instructions in the import file.")

    # Image processing and drawing modes
    def img_btn_slider_enable(self, enable):
        """
        Enables or disables image processing controls based on current mode
        When in image mode, all processing controls are available
        In line/curve modes, these controls are disabled to prevent conflicts
        
        Args:
            enable: Boolean flag to enable/disable controls
        """
        # Enable/disable all image processing buttons and sliders
        self.pushButton_Arm_Import_Picture.setEnabled(enable)
        self.pushButton_Arm_Gray_Picture.setEnabled(enable)
        self.pushButton_Arm_Binaryzation_Picture.setEnabled(enable)
        self.pushButton_Arm_Contour_Picture.setEnabled(enable)
        self.horizontalSlider_Arm_Threshold.setEnabled(enable)
        self.horizontalSlider_Arm_Gauss.setEnabled(enable)
        self.horizontalSlider_Arm_Sharpen.setEnabled(enable)
        self.lineEdit_Arm_Threshold_Value.setEnabled(enable)
        self.lineEdit_Arm_Gauss_Value.setEnabled(enable)
        self.lineEdit_Arm_Sharpen_Value.setEnabled(enable)
        
        # CSS styles for enabled/disabled appearance
        str_push_button = "QAbstractButton{\n" \
                          "border-style:none;\n" \
                          "border-radius:0px;\n" \
                          "padding:5px;\n" \
                          "color:#DCDCDC;\n" \
                          "background:qlineargradient(spread:pad,x1:0,y1:0,x2:0,y2:1,stop:0 #858585,stop:1 #383838);\n" \
                          "}\n" \
                          "QAbstractButton:hover{\n" \
                          "color:#000000;\n" \
                          "background-color:#008aff;\n" \
                          "}\n" \
                          "QAbstractButton:pressed{\n" \
                          "color:#DCDCDC;\n" \
                          "border-style:solid;\n" \
                          "border-width:0px 0px 0px 4px;\n" \
                          "padding:4px 4px 4px 2px;\n" \
                          "border-color:#008aff;\n" \
                          "background-color:#444444;\n" \
                          "}\n"
        
        str_line_edit = "QLineEdit{\n" \
                        "border:1px solid #242424;\n" \
                        "border-radius:3px;\n" \
                        "padding:2px;\n" \
                        "background:none;\n" \
                        "selection-background-color:#484848;\n" \
                        "selection-color:#DCDCDC;\n" \
                        "}\n" \
                        "QLineEdit:focus,QLineEdit:hover{\n" \
                        "border:1px solid #242424;\n" \
                        "}\n" \
                        "QLineEdit{\n" \
                        "border:1px solid #242424;\n" \
                        "border-radius:3px;\n" \
                        "padding:2px;\n" \
                        "background:none;\n" \
                        "selection-background-color:#484848;\n" \
                        "selection-color:#DCDCDC;\n" \
                        "}\n" \
                        "\n" \
                        "QLineEdit:focus,QLineEdit:hover{\n" \
                        "border:1px solid #242424;\n" \
                        "}\n" \
                        "QLineEdit{\n" \
                        "lineedit-password-character:9679;\n" \
                        "}\n"
        
        if not enable:
            # Apply gray disabled appearance
            self.pushButton_Arm_Import_Picture.setStyleSheet("background-color: rgb(195, 195, 195);")
            self.pushButton_Arm_Gray_Picture.setStyleSheet("background-color: rgb(195, 195, 195);")
            self.pushButton_Arm_Binaryzation_Picture.setStyleSheet("background-color: rgb(195, 195, 195);")
            self.pushButton_Arm_Contour_Picture.setStyleSheet("background-color: rgb(195, 195, 195);")
            self.lineEdit_Arm_Threshold_Value.setStyleSheet("background-color: rgb(195, 195, 195);")
            self.lineEdit_Arm_Gauss_Value.setStyleSheet("background-color: rgb(195, 195, 195);")
            self.lineEdit_Arm_Sharpen_Value.setStyleSheet("background-color: rgb(195, 195, 195);")
        else:
            # Apply normal enabled appearance
            self.pushButton_Arm_Import_Picture.setStyleSheet(str_push_button)
            self.pushButton_Arm_Gray_Picture.setStyleSheet(str_push_button)
            self.pushButton_Arm_Binaryzation_Picture.setStyleSheet(str_push_button)
            self.pushButton_Arm_Contour_Picture.setStyleSheet(str_push_button)
            self.lineEdit_Arm_Threshold_Value.setStyleSheet(str_line_edit)
            self.lineEdit_Arm_Gauss_Value.setStyleSheet(str_line_edit)
            self.lineEdit_Arm_Sharpen_Value.setStyleSheet(str_line_edit)

    def radioButton_arm_select_mode(self, index):
        """
        Handles switching between different drawing modes:
        - Image Mode: Process uploaded images for robot drawing
        - Line Mode: Draw straight lines manually
        - Curves Mode: Draw curved lines manually
        
        Args:
            index: The radio button that was selected
        """
        if index.objectName() == "radioButton_Arm_Img_Mode":
            # Switch to Image Processing Mode
            self.radioButton_Arm_Line_Mode.setChecked(False)
            self.radioButton_Arm_Curves_Mode.setChecked(False)
            self.radioButton_Arm_Img_Mode.setChecked(True)
            self.img_btn_slider_enable(True)  # Enable image processing controls
            self.painter_line_style = 0  # Set drawing style to image mode
            
            # Set display image based on current processing stage
            if self.img_flag == 0:
                self.original_label_img = self.white_image.copy()  # Blank canvas
            elif self.img_flag == 1:
                self.original_label_img = self.raw_img.copy()  # Original image
            elif self.img_flag == 2:
                self.original_label_img = self.gray_img.copy()  # Grayscale
            elif self.img_flag == 3:
                self.original_label_img = self.binary_img.copy()  # Binary (black/white)
            elif self.img_flag == 4:
                self.original_label_img = self.contour_img.copy()  # Contours
            
            # Reset drawing points
            self.lastPoint = [0, 0]
            self.currentPoint = [0, 0]
            self.updata_label_show()  # Refresh display
            
        elif index.objectName() == "radioButton_Arm_Line_Mode":
            # Switch to Manual Line Drawing Mode
            self.radioButton_Arm_Line_Mode.setChecked(True)
            self.radioButton_Arm_Curves_Mode.setChecked(False)
            self.radioButton_Arm_Img_Mode.setChecked(False)
            self.img_btn_slider_enable(False)  # Disable image processing
            self.painter_line_style = 1  # Set drawing style to lines
            
        elif index.objectName() == "radioButton_Arm_Curves_Mode":
            # Switch to Manual Curve Drawing Mode
            self.radioButton_Arm_Line_Mode.setChecked(False)
            self.radioButton_Arm_Curves_Mode.setChecked(True)
            self.radioButton_Arm_Img_Mode.setChecked(False)
            self.img_btn_slider_enable(False)  # Disable image processing
            self.painter_line_style = 2  # Set drawing style to curves

    def send_Image_command(self):
        """
        Sends G-code commands generated from image processing to the robot
        Manages command flow control to prevent buffer overflow
        Commands are sent in batches of up to 50 at a time
        """
        if not self.gcode_command.empty():
            if self.client.connect_flag:
                self.client_busy = True
                # Tell robot we're ready to send G-code sequence
                cmd = self.cmd.CUSTOM_ACTION + str("12") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_QUERY + str("1")
                self.threading_cmd.emit(cmd)
                
                # Send commands in batches to avoid overwhelming robot buffer
                while self.gcode_command.len() > 0:
                    if self.send_g_code_state:  # Robot is ready for more commands
                        if self.arm_command_count < 50:  # Robot has space in buffer
                            send_num = 50 - self.arm_command_count  # Calculate how many to send
                            if self.gcode_command.len() < send_num:
                                send_num = self.gcode_command.len()  # Don't send more than we have
                            
                            # Send the batch of commands
                            for i in range(send_num):
                                buf = self.gcode_command.get()
                                if buf is not None:
                                    self.threading_cmd.emit(buf)
                self.client_busy = False
        else:
            print("Connect the remote IP address first.")

    # Image processing
    def set_img_action(self, index):
        """
        Handles all image processing operations in sequence:
        Import -> Grayscale -> Binary -> Contours -> G-code -> Execute
        Each step processes the image further for robot drawing
        
        Args:
            index: The button widget that was clicked
        """
        if index.objectName() == "pushButton_Arm_Import_Picture":
            # STEP 1: Import and resize image to fit display area
            try:
                self.folder_path = self.filePath.getFilePath("./picture/*")
                if self.folder_path is not None:
                    print(self.folder_path)
                    white_image = self.white_image.copy()  # Start with white canvas
                    self.img_flag = 1  # Set flag to indicate raw image loaded
                    
                    # Load image using OpenCV (handles UTF-8 paths)
                    img = cv2.imdecode(np.fromfile(self.folder_path.encode('utf-8'), dtype=np.uint8), cv2.IMREAD_COLOR)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert to RGB for display
                    
                    # Get image and display dimensions
                    img_h, img_w = img.shape[:2]
                    label_h = self.label_videl_size[1]
                    label_w = self.label_videl_size[0]
                    
                    if img_h <= label_h and img_w <= label_w:
                        # Image fits - center it on white background
                        picture_center = ((label_h - img_h) // 2, (label_w - img_w) // 2)
                        white_image[picture_center[0]:picture_center[0] + img_h, 
                                   picture_center[1]:picture_center[1] + img_w] = img
                    elif img_h > label_h or img_w > label_w:
                        # Image too big - scale it down proportionally
                        scale = min((label_h-2) / img_h, (label_w-2) / img_w)
                        new_img_h = int(scale * img_h)
                        new_img_w = int(scale * img_w)
                        resized_img = cv2.resize(img, (new_img_w, new_img_h), interpolation=cv2.INTER_AREA)
                        # Center the resized image
                        picture_center = ((label_h - new_img_h) // 2, (label_w - new_img_w) // 2)
                        white_image[picture_center[0]:picture_center[0] + new_img_h,
                                   picture_center[1]:picture_center[1] + new_img_w] = resized_img
                    
                    img = white_image.copy()
                    self.raw_img = img.copy()  # Store processed image
                    # Convert to Qt format and display
                    img = QImage(img.data.tobytes(), img.shape[1], img.shape[0], QImage.Format_RGB888)
                    self.label_Arm_Video.setPixmap(QPixmap.fromImage(img))
                    self.original_label_img = self.raw_img.copy()
                else:
                    print("Folder path is None.")
            except:
                # Reset to blank canvas if import fails
                self.img_flag = 0
                print("Load picture failed.")
                self.axis_paint_function = False
                img = QImage(self.white_image.data.tobytes(), self.white_image.shape[1], 
                           self.white_image.shape[0], QImage.Format_RGB888)
                self.label_Arm_Video.setPixmap(QPixmap.fromImage(img))
                self.original_label_img = self.white_image.copy()
                
        elif index.objectName() == "pushButton_Arm_Gray_Picture":
            # STEP 2: Convert to grayscale for further processing
            try:
                if len(self.raw_img):
                    self.img_flag = 2
                    # Convert RGB to grayscale
                    self.gray_img = cv2.cvtColor(self.raw_img, cv2.COLOR_BGR2GRAY)
                    # Convert to Qt format (grayscale uses different format)
                    img = QImage(self.gray_img.data.tobytes(), self.gray_img.shape[1], 
                               self.gray_img.shape[0], QImage.Format_Indexed8)
                    self.label_Arm_Video.setPixmap(QPixmap.fromImage(img))
                    self.original_label_img = self.gray_img.copy()
            except:
                print("Show gary picture failed.")
                
        elif index.objectName() == "pushButton_Arm_Binaryzation_Picture":
            # STEP 3: Convert to binary (black/white) image with filtering
            try:
                if len(self.gray_img):
                    self.img_flag = 3
                    img = self.gray_img.copy()
                    # Apply binary threshold (converts to black/white based on threshold)
                    ret, binary = cv2.threshold(img, self.threshold_value, 255, cv2.THRESH_BINARY)
                    # Apply Gaussian blur to reduce noise
                    img = cv2.GaussianBlur(binary, (self.gauss_value, self.gauss_value), 0, 0)
                    # Apply sharpening kernel to enhance edges
                    kernel = np.array([[0, -1, 0], [-1, self.sharpen_value, -1], [0, -1, 0]], np.float32)
                    img = cv2.filter2D(img, -1, kernel=kernel)
                    
                    self.binary_img = img.copy()  # Store processed image
                    # Display the binary image
                    img = QImage(img.data.tobytes(), img.shape[1], img.shape[0], QImage.Format_Indexed8)
                    self.label_Arm_Video.setPixmap(QPixmap.fromImage(img))
                    self.original_label_img = self.binary_img.copy()
            except:
                print("Show binaryzation picture failed.")
                
        elif index.objectName() == "pushButton_Arm_Contour_Picture":
            # STEP 4: Find and display contours (outlines) for drawing
            try:
                if len(self.binary_img):
                    self.img_flag = 2
                    self.gray_img = cv2.cvtColor(self.raw_img, cv2.COLOR_BGR2GRAY)  
                    img = QImage(self.gray_img.data.tobytes(), self.gray_img.shape[1], self.gray_img.shape[0], QImage.Format_Indexed8) 
                    self.label_Arm_Video.setPixmap(QPixmap.fromImage(img))  
                    self.original_label_img = self.gray_img.copy()
                    
                    # Find contours in the binary image
                    # RETR_TREE: retrieves all contours and hierarchy
                    # CHAIN_APPROX_SIMPLE: compresses horizontal/vertical/diagonal segments
              # Handle Gray Picture conversion button press
        elif index.objectName() == "pushButton_Arm_Gray_Picture": 
            try:
                # Check if raw image data exists
                if len(self.raw_img):
                    # Set image flag to indicate grayscale mode
                    self.img_flag = 2
                    # Convert BGR color image to grayscale using OpenCV
                    self.gray_img = cv2.cvtColor(self.raw_img, cv2.COLOR_BGR2GRAY)  
                    # Convert grayscale numpy array to QImage for Qt display
                    img = QImage(self.gray_img.data.tobytes(), self.gray_img.shape[1], self.gray_img.shape[0], QImage.Format_Indexed8) 
                    # Display the grayscale image in the video label widget
                    self.label_Arm_Video.setPixmap(QPixmap.fromImage(img))  
                    # Store copy of image for drawing operations
                    self.original_label_img = self.gray_img.copy()
            except:
                print("Show gary picture failed.")

        # Handle Binary Image processing button press
        elif index.objectName() == "pushButton_Arm_Binaryzation_Picture":  
            try:
                # Check if grayscale image exists
                if len(self.gray_img):
                    # Set image flag to indicate binary mode
                    self.img_flag = 3
                    img = self.gray_img.copy() 
                    # Apply threshold to convert grayscale to binary (black/white)
                    ret, binary = cv2.threshold(img, self.threshold_value, 255, cv2.THRESH_BINARY)  
                    # Apply Gaussian blur for noise reduction
                    img = cv2.GaussianBlur(binary, (self.gauss_value, self.gauss_value), 0, 0)  
                    # Create sharpening kernel matrix
                    kernel = np.array([[0, -1, 0], [-1, self.sharpen_value, -1], [0, -1, 0]], np.float32)  
                    # Apply sharpening filter using the kernel
                    img = cv2.filter2D(img, -1, kernel=kernel) 
                    # Store processed binary image
                    self.binary_img = img.copy()  
                    # Convert to QImage for display
                    img = QImage(img.data.tobytes(), img.shape[1], img.shape[0], QImage.Format_Indexed8)  
                    # Display the binary image
                    self.label_Arm_Video.setPixmap(QPixmap.fromImage(img))
                    # Store for drawing operations
                    self.original_label_img = self.binary_img.copy()
            except:
                print("Show binaryzation picture failed.")

        # Handle Contour Detection button press
        elif index.objectName() == "pushButton_Arm_Contour_Picture":
            try:
                # Check if binary image exists
                if len(self.binary_img):
                    # Set image flag to indicate contour mode
                    self.img_flag = 4
                    img = self.binary_img.copy() 
                    # Initialize contour data storage
                    self.contours_data = None  
                    self.hierarchy_data = None 
                    # Find contours using tree hierarchy and simple approximation
                    # RETR_TREE: retrieves all contours and reconstructs full hierarchy
                    # CHAIN_APPROX_SIMPLE: compresses horizontal/vertical/diagonal segments
                    self.contours_data, self.hierarchy_data = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    # Create blank white canvas for contour drawing
                    img1 = np.zeros(shape=img.shape, dtype=np.uint8)  
                    img1 += 255  # Fill with white (255)
                    # Draw all contours in black on white background
                    cv2.drawContours(img1, self.contours_data, -1, (0, 0, 0), 1)  
                    # Store contour image
                    self.contour_img = img1.copy()  
                    # Convert to QImage and display
                    img1 = QImage(img1.data.tobytes(), img1.shape[1], img1.shape[0], QImage.Format_Indexed8)  
                    self.label_Arm_Video.setPixmap(QPixmap.fromImage(img1))
                    # Store for drawing operations
                    self.original_label_img = self.contour_img.copy()
            except:
                print("Show contour picture failed.")

        # Handle Clear All button press - resets drawing and G-code data
        elif index.objectName() == "pushButton_Arm_Clear_All":
            # Reset line drawing counters
            self.line_curves_count = 1
            self.current_line_curves = 0
            # Clear all stored data
            self.gcode_command.clear()
            self.painter_point.clear()
            self.textEdit_Arm_Gcode_Area.clear()
            
            # Restore original image based on current mode
            if self.img_flag == 0:
                self.original_label_img = self.white_image.copy()
            elif self.img_flag == 1:
                self.original_label_img = self.raw_img.copy()
            elif self.img_flag == 2:
                self.original_label_img = self.gray_img.copy()
            elif self.img_flag == 3:
                self.original_label_img = self.binary_img.copy()
            elif self.img_flag == 4:
                self.original_label_img = self.contour_img.copy()
            
            # Reset drawing points
            self.lastPoint = [0, 0]
            self.currentPoint = [0, 0]
            # Update display
            self.updata_label_show()

        # Handle G-code Generation button press
        elif index.objectName() == "pushButton_Arm_Change_Gcode":
            try:
                # Calculate Z-axis heights for pen up/down movements
                z_height = float(self.lineEdit_Arm_Pen_Height_Value.text()) + float(self.original_position_axis[2])
                z_axis = float(self.original_position_axis[2])
                
                # Check if in image processing mode
                if self.radioButton_Arm_Img_Mode.isChecked():
                    # Generate G-code from contour data
                    if len(self.contour_img):
                        self.gcode_command.clear()
                        
                        # Process each contour (skip index 0 - usually outer boundary)
                        for i in range(1, len(self.contours_data)):
                            track_last_point_command = None
                            
                            # Process each point in the contour
                            for j in range(len(self.contours_data[i])):
                                # Extract x,y coordinates from contour point
                                buf = list(self.contours_data[i][j][0])
                                
                                if j == 0:  # First point of contour
                                    # Lift pen (move to z_height)
                                    gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_X_ACTION + str(self.last_axis_point[0]) + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_Y_ACTION + str(self.last_axis_point[1]) + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_Z_ACTION + str(round(z_height, 1))
                                    self.gcode_command.put(gcode_change_command)
                                    
                                    # Map pixel coordinates to physical coordinates
                                    x = self.map(buf[0], 0, self.label_videl_size[0], self.axis_map_x[0], self.axis_map_x[1], 1)
                                    y = self.map(buf[1], 0, self.label_videl_size[1], self.axis_map_y[0], self.axis_map_y[1], 1)
                                    
                                    # Move to start position with pen up
                                    gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_Z_ACTION + str(round(z_height, 1))
                                    self.gcode_command.put(gcode_change_command)
                                    
                                    # Lower pen to drawing position
                                    gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_Z_ACTION + str(z_axis)
                                    self.gcode_command.put(gcode_change_command)
                                    
                                    # Remember first point for closing contour
                                    track_last_point_command = [x, y]
                                    self.last_axis_point = [x, y]
                                else:  
                                    # Subsequent points - draw lines with pen down
                                    x = self.map(buf[0], 0, self.label_videl_size[0], self.axis_map_x[0], self.axis_map_x[1], 1)
                                    y = self.map(buf[1], 0, self.label_videl_size[1], self.axis_map_y[0], self.axis_map_y[1], 1)
                                    
                                    # Move to next point with pen down
                                    gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                                        + self.cmd.AXIS_Z_ACTION + str(z_axis)
                                    self.gcode_command.put(gcode_change_command)
                                    self.last_axis_point = [x, y]
                            
                            # Close the contour by returning to first point
                            gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                + self.cmd.AXIS_X_ACTION + str(track_last_point_command[0]) + self.cmd.DECOLLATOR_CHAR \
                                + self.cmd.AXIS_Y_ACTION + str(track_last_point_command[1]) + self.cmd.DECOLLATOR_CHAR \
                                + self.cmd.AXIS_Z_ACTION + str(z_axis)
                            self.gcode_command.put(gcode_change_command)
                            self.last_axis_point = [track_last_point_command[0], track_last_point_command[1]]
                        
                        # Final pen lift and return to home position
                        gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_X_ACTION + str(self.last_axis_point[0]) + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_Y_ACTION + str(self.last_axis_point[1]) + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_Z_ACTION + str(round(z_height, 1))
                        self.gcode_command.put(gcode_change_command)
                        
                        # Move to home position (0, 200)
                        self.last_axis_point = [0, 200]
                        gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_X_ACTION + str(self.last_axis_point[0]) + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_Y_ACTION + str(self.last_axis_point[1]) + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_Z_ACTION + str(round(z_height, 1))
                        self.gcode_command.put(gcode_change_command)
                        
                        # Display generated G-code in text area
                        self.textEdit_Arm_Gcode_Area.clear()
                        gcode_change_command = self.gcode_command.gets()
                        for i in range(self.gcode_command.len()):
                            self.textEdit_Arm_Gcode_Area.append(gcode_change_command[i])
                    else:
                        # No contour data available
                        self.textEdit_Arm_Gcode_Area.clear()
                else:
                    # Manual drawing mode - generate G-code from drawn lines
                    self.gcode_command.clear()  
                    self.textEdit_Arm_Gcode_Area.clear()
                    
                    if self.painter_point.len() > 0:
                        self.current_line_curves = 0
                        buf_point = self.painter_point.gets()
                        
                        # Process each drawn line segment
                        for i in range(self.painter_point.len()):
                            # Check if starting new line (different curve number)
                            if self.current_line_curves != buf_point[i][2]: 
                                # Lift pen and move to start of new line
                                x = self.last_axis_point[0]
                                y = self.last_axis_point[1]
                                gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Z_ACTION + str(round(z_height, 1))
                                self.gcode_command.put(gcode_change_command)
                                
                                # Map start point coordinates
                                x = self.map(buf_point[i][0][0], 0, self.label_videl_size[0], self.axis_map_x[0], self.axis_map_x[1], 1)
                                y = self.map(buf_point[i][0][1], 0, self.label_videl_size[1], self.axis_map_y[0], self.axis_map_y[1], 1)
                                
                                # Move to start position with pen up
                                gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Z_ACTION + str(round(z_height, 1))
                                self.gcode_command.put(gcode_change_command)
                                
                                # Lower pen to start drawing
                                gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Z_ACTION + str(z_axis)
                                self.gcode_command.put(gcode_change_command)
                                
                                # Map end point and draw line
                                x = self.map(buf_point[i][1][0], 0, self.label_videl_size[0], self.axis_map_x[0], self.axis_map_x[1], 1)
                                y = self.map(buf_point[i][1][1], 0, self.label_videl_size[1], self.axis_map_y[0], self.axis_map_y[1], 1)
                                gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Z_ACTION + str(z_axis)
                                self.gcode_command.put(gcode_change_command)
                                
                                self.last_axis_point = [x, y]
                                self.current_line_curves = buf_point[i][2] 
                            else:
                                # Continue drawing same line
                                x = self.map(buf_point[i][1][0], 0, self.label_videl_size[0], self.axis_map_x[0], self.axis_map_x[1], 1)
                                y = self.map(buf_point[i][1][1], 0, self.label_videl_size[1], self.axis_map_y[0], self.axis_map_y[1], 1)
                                gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                                    + self.cmd.AXIS_Z_ACTION + str(z_axis)
                                self.gcode_command.put(gcode_change_command)
                                self.last_axis_point = [x, y]
                        
                        # Lift pen at end of drawing
                        x = self.last_axis_point[0]
                        y = self.last_axis_point[1]
                        gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_Z_ACTION + str(round(z_height, 1))
                        self.gcode_command.put(gcode_change_command)
                        
                        # Return to home position
                        self.last_axis_point = [0, 200]
                        x = self.last_axis_point[0]
                        y = self.last_axis_point[1]
                        gcode_change_command = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_X_ACTION + str(x) + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_Y_ACTION + str(y) + self.cmd.DECOLLATOR_CHAR \
                            + self.cmd.AXIS_Z_ACTION + str(round(z_height, 1))
                        self.gcode_command.put(gcode_change_command)
                        
                        # Display generated G-code
                        gcode_change_command = self.gcode_command.gets()
                        for i in range(self.gcode_command.len()):
                            self.textEdit_Arm_Gcode_Area.append(gcode_change_command[i])
            except:
                print("Change Picture to G-Code failed.")

        # Handle G-code Execution button press
        elif index.objectName() == "pushButton_Arm_Execute_Gcode":
            # Check if connected to robot
            if self.client.connect_flag:
                # Check if G-code commands exist
                if self.gcode_command.len() > 0:
                    # Check if motors are loaded (not relaxed)
                    if self.pushButton_Arm_Load_Relax.text() == "Relax Motor":
                        # Start execution in separate thread
                        execte_handle = threading.Thread(target=self.send_Image_command)
                        execte_handle.start()
                    else:
                        print("Please press the \"Load Motor\" button first.")
                else:
                    # No G-code to execute, clear display
                    self.textEdit_Arm_Gcode_Area.clear()
            else:
                print("Connect the remote IP address first.")

    def img_slider_control(self, index):
        #Handle image processing parameter slider controls"""
        
        # Threshold slider for binary conversion
        if index.objectName() == "horizontalSlider_Arm_Threshold": 
            self.threshold_value = index.value()
            self.lineEdit_Arm_Threshold_Value.setText(str(self.threshold_value))  
        
        # Gaussian blur slider (must be odd number)
        elif index.objectName() == "horizontalSlider_Arm_Gauss":
            self.gauss_value = index.value()  
            # Ensure Gaussian kernel size is odd
            if self.gauss_value % 2 == 0:  
                self.gauss_value = self.gauss_value + 1  
            self.lineEdit_Arm_Gauss_Value.setText(str(self.gauss_value)) 
            index.setValue(self.gauss_value) 
        
        # Sharpening intensity slider
        elif index.objectName() == "horizontalSlider_Arm_Sharpen":
            self.sharpen_value = index.value() 
            self.lineEdit_Arm_Sharpen_Value.setText(str(self.sharpen_value)) 
        
        # Pen height slider for Z-axis control
        elif index.objectName() == "horizontalSlider_Arm_Pen_Height":
            self.lineEdit_Arm_Pen_Height_Value.setText(str(index.value()))
        
        # Real-time image processing update when in binary or contour mode
        if self.img_flag >= 3:
            try:
                # Reprocess image with new parameters
                img = self.gray_img.copy() 
                # Apply threshold with updated value
                ret, binary = cv2.threshold(img, self.threshold_value, 255, cv2.THRESH_BINARY)
                # Apply Gaussian blur with updated kernel size
                img = cv2.GaussianBlur(binary, (self.gauss_value, self.gauss_value), 0, 0)  
                # Apply sharpening with updated intensity
                kernel = np.array([[0, -1, 0], [-1, self.sharpen_value, -1], [0, -1, 0]], np.float32) 
                img = cv2.filter2D(img, -1, kernel=kernel)  
                # Update stored binary image
                self.binary_img = img.copy() 
                self.original_label_img = img.copy()
                
                # If in contour mode, recalculate contours
                if self.img_flag == 4:
                    self.contours_data = None 
                    self.hierarchy_data = None 
                    # Find contours with updated binary image
                    self.contours_data, self.hierarchy_data = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    # Create new contour display image
                    img1 = np.zeros(shape=img.shape, dtype=np.uint8) 
                    img1 += 255  # White background
                    # Draw contours in black
                    cv2.drawContours(img1, self.contours_data, -1, (0, 0, 0), 1)
                    self.contour_img = img1.copy()  
                    self.original_label_img = img1.copy()
                    # Display updated contour image
                    img1 = QImage(img1.data.tobytes(), img1.shape[1], img1.shape[0],
                                  QImage.Format_Indexed8)  
                    self.label_Arm_Video.setPixmap(QPixmap.fromImage(img1))  
                else:
                    # Display updated binary image
                    img = QImage(img.data.tobytes(), img.shape[1], img.shape[0], QImage.Format_Indexed8)  
                    self.label_Arm_Video.setPixmap(QPixmap.fromImage(img))
            except:
                print("Show binaryzation picture failed.")

    def control_servo_angle(self, index):
        #Control servo motor angle with increment/decrement buttons
        
        # Increase servo angle by 5 degrees
        if index.objectName() == "pushButton_Arm_Servo_Turn_On":
            self.angle_value = int(self.lineEdit_Arm_Servo_Angle_Value.text()) + 5
            # Constrain angle within 0-180 degree range
            self.angle_value = self.constrain(self.angle_value, 0, 180)
            self.lineEdit_Arm_Servo_Angle_Value.setText(str(self.angle_value))
        
        # Decrease servo angle by 5 degrees
        elif index.objectName() == "pushButton_Arm_Servo_Turn_Off":
            self.angle_value = int(self.lineEdit_Arm_Servo_Angle_Value.text()) - 5
            # Constrain angle within 0-180 degree range
            self.angle_value = self.constrain(self.angle_value, 0, 180)
            self.lineEdit_Arm_Servo_Angle_Value.setText(str(self.angle_value))
        
        # Send servo command if connected
        if self.client.connect_flag:
            self.client_busy = True
            # Build servo control command string
            cmd = self.cmd.CUSTOM_ACTION + str("9") + self.cmd.DECOLLATOR_CHAR \
                + self.cmd.ARM_SERVO_INDEX + str(self.comboBox_Arm_Servo.currentIndex()) + self.cmd.DECOLLATOR_CHAR \
                + self.cmd.ARM_SERVO_ANGLE + self.lineEdit_Arm_Servo_Angle_Value.text()
            # Send command via threading mechanism
            self.threading_cmd.emit(cmd)
            # Record command for logging
            self.record_command(cmd)
            self.client_busy = False
        else:
            print("Connect the remote IP address first.")

    def updata_label_show(self):
        #Update the display label with current image and drawing overlays
        
        img = None
        # Define line start and end points
        start_point = (self.lastPoint[0], self.lastPoint[1])  
        end_point = (self.currentPoint[0], self.currentPoint[1])  
        
        # Handle different drawing modes
        if self.painter_line_style == 1:
            # Temporary line mode - only show line while drawing
            if self.isDrawing:
                img = self.original_label_img.copy()  # Copy for temporary overlay
            else:
                img = self.original_label_img  # Use original without overlay
        else:
            # Permanent drawing mode
            img = self.original_label_img
        
        # Draw line if start and end points are different
        if start_point != end_point:
            cv2.line(img, start_point, end_point, (0, 0, 0), 1, cv2.LINE_AA) 
        
        # Convert to appropriate QImage format based on image type
        if self.img_flag <= 1:
            # Color images (raw/white) use RGB888 format
            img = QImage(img.data.tobytes(), img.shape[1], img.shape[0], QImage.Format_RGB888) 
        else:
            # Grayscale/binary images use Indexed8 format
            img = QImage(img.data.tobytes(), img.shape[1], img.shape[0], QImage.Format_Indexed8)  
        
        # Display updated image
        self.label_Arm_Video.setPixmap(QPixmap.fromImage(img)) 

    def mousePressEvent(self, event):
        #Handle mouse press events for drawing functionality
        
        # Only respond to left mouse button
        if event.button() == Qt.LeftButton: 
            # Only draw if drawing mode is enabled
            if self.painter_line_style != 0:
                # Constrain mouse position to image boundaries
                x = self.constrain(event.pos().x(), 0, self.label_videl_size[0])
                y = self.constrain(event.pos().y(), 0, self.label_videl_size[1])
                # Set current point and initialize last point
                self.currentPoint = [x, y]  
                self.lastPoint = self.currentPoint.copy() 
                # Enable drawing flag
                self.isDrawing = True

    def mouseMoveEvent(self, event):
        #Handle mouse move events for continuous line drawing
        
        # Check if left button is held down during movement
        if event.buttons() and Qt.LeftButton:  
            # Only draw if drawing mode is enabled
            if self.painter_line_style != 0:
                # Constrain movement to image boundaries
                x = self.constrain(event.pos().x(), 0, self.label_videl_size[0])
                y = self.constrain(event.pos().y(), 0, self.label_videl_size[1])
                self.currentPoint = [x, y]
                # Update display with current line
                self.updata_label_show()
                
                # Store line segments for permanent drawing mode
                if self.painter_line_style == 2:
                    # Store line segment with curve identifier
                    self.painter_point.put([self.lastPoint, self.currentPoint, self.line_curves_count])
                    # Update last point for next segment
                    self.lastPoint = self.curren

def mouseMoveEvent(self, event):
    #Handle mouse move events for drawing operations.
    #this method is called continuously while the mouse is being moved.
    
    # Check if left mouse button is pressed during movement (for dragging)
    if event.buttons() and Qt.LeftButton:  
        # Only process if we're in a drawing mode (painter_line_style != 0)
        if self.painter_line_style != 0:
            # Constrain mouse coordinates to stay within the video label boundaries
            # This prevents drawing outside the designated area
            x = self.constrain(event.pos().x(), 0, self.label_videl_size[0])
            y = self.constrain(event.pos().y(), 0, self.label_videl_size[1])
            
            # Update current point with constrained coordinates
            self.currentPoint = [x, y]
            
            # Refresh the display to show the drawing progress
            self.updata_label_show()
            
            # If painter_line_style == 2, this appears to be continuous drawing mode
            # Store line segments as the mouse moves (real-time drawing)
            if self.painter_line_style == 2:
                # Add a line segment from last point to current point
                self.painter_point.put([self.lastPoint, self.currentPoint, self.line_curves_count])
                # Update lastPoint for the next line segment
                self.lastPoint = self.currentPoint.copy()

def mouseReleaseEvent(self, event):
    #andle mouse release events to finalize drawing operations.
    # Only process left mouse button releases
    if event.button() == Qt.LeftButton:  
        # Only process if we're in a drawing mode
        if self.painter_line_style != 0:
            # Constrain final coordinates to video label boundaries
            x = self.constrain(event.pos().x(), 0, self.label_videl_size[0])
            y = self.constrain(event.pos().y(), 0, self.label_videl_size[1])
            self.currentPoint = [x, y]
            
            # Check if the mouse actually moved during the click
            # If lastPoint and currentPoint are the same, it was just a click (no drag)
            if self.lastPoint[0] == self.currentPoint[0] and self.lastPoint[1] == self.currentPoint[1]:
                pass  # Do nothing for single clicks
            else:
                # Add the final line segment for drawing modes other than continuous drawing
                self.painter_point.put([self.lastPoint, self.currentPoint, self.line_curves_count])
            
            # Mark drawing as finished
            self.isDrawing = False
            
            # Update the display with the completed drawing
            self.updata_label_show()
            
            # Increment the curve counter for the next drawing operation
            self.line_curves_count = self.line_curves_count + 1

def close_parameter_ui(self, data):
    #Handle closing of the parameter configuration UI and process returned data.
    
    Args:
    data (str): Comma-separated string containing position data and status flag
    # Parse the comma-separated data string
    data = data.split(",")
    
    # Check the status flag (4th element, index 3)
    if data[3] == "1":
        # Status "1": Successfully configured, update display and enable button
        self.ui_arm_show_label_axis(data[:3])  # Show first 3 elements (x,y,z positions)
        self.original_position_axis = data[:3]  # Store original axis positions
        self.pushButton_Arm_Parameter_UI.setEnabled(True)  # Re-enable the parameter UI button
    elif data[3] == "0":
        # Status "0": Configuration cancelled or failed, but still update display
        self.ui_arm_show_label_axis(data[:3])
        self.original_position_axis = data[:3]
        # Note: Button remains disabled in this case

def configure_parameter_ui(self):
     #Open the parameter configuration window for arm control.
     #Includes validation to ensure proper connection and motor state.

    # Check if client is connected to remote system
    if self.client.connect_flag:
        # Check if motors are loaded (button text indicates current state)
        if self.pushButton_Arm_Load_Relax.text() == "Relax Motor":
            # Motors are loaded, safe to configure parameters
            # Disable the parameter button to prevent multiple windows
            self.pushButton_Arm_Parameter_UI.setEnabled(False)
            
            # Create and display the configuration window
            self.configurationWindow = Configuration()
            self.configurationWindow.setWindowModality(Qt.NonModal)  # Allow interaction with parent
            self.configurationWindow.show()
            
            # Connect signals to handle window closing and command sending
            self.configurationWindow.position_axis_channel.connect(self.close_parameter_ui)
            self.configurationWindow.send_cmd_channel.connect(self.socket_send)
        else:
            # Motors not loaded - show error message
            print("Please press the \"Load Motor\" button first.")
    else:
        # Not connected to remote system - show error message
        print("Connect the remote IP address first.")

def close_led_ui(self):
    #Handle closing of the LED control UI.
    #Simply re-enables the LED UI button for future use.
    self.pushButton_Arm_Led_UI.setEnabled(True)

def configure_led_ui(self):
    #Open the LED control window for arm lighting configuration.
    #Includes connection validation.
    
    # Check if client is connected to remote system
    if self.client.connect_flag:
        # Disable the LED button to prevent multiple windows
        self.pushButton_Arm_Led_UI.setEnabled(False)
        
        # Create and display the LED control window
        self.ledWindow = LED()
        self.ledWindow.setWindowModality(Qt.NonModal)  # Allow interaction with parent
        self.ledWindow.show()
        
        # Connect signals for window management and command sending
        self.ledWindow.signal_channel.connect(self.close_led_ui)
        self.ledWindow.send_cmd_channel.connect(self.socket_send)
    else:
        # Not connected - show error message
        print("Connect the remote IP address first.")

# Application entry point
if __name__ == '__main__':
    # Main application startup code.
    # Creates the QApplication, main window, and starts the event loop.
    
    # Create the Qt application instance
    app = QApplication(sys.argv)
    
    # Create the main client window instance
    myshow = myClientWindow()
    
    # Display the main window
    myshow.show()
    
    # Start the Qt event loop and exit when it's done
    sys.exit(app.exec_())
