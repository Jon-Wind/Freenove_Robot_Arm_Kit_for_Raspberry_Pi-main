import sys
from PyQt5.QtCore import *
import numpy as np
from PyQt5.QtGui import *
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import Qt
from command import Command
from ui.ui_led import Ui_Led


class ColorDialog(QtWidgets.QColorDialog):
    """
    Custom color dialog that only shows the color wheel and luminance picker.
    Hides all other UI elements for a cleaner, simplified color selection interface.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        # Disable native dialog to ensure consistent appearance across platforms
        self.setOptions(self.options() | QtWidgets.QColorDialog.DontUseNativeDialog)
        
        # Hide all UI elements except the color picker and luminance picker
        for children in self.findChildren(QtWidgets.QWidget):
            classname = children.metaObject().className()
            if classname not in ("QColorPicker", "QColorLuminancePicker"):
                children.hide()


class LED(QtWidgets.QWidget, Ui_Led):
    """
    Main LED control widget for managing WS2812 addressable LED strips.
    Provides interface for color selection, brightness control, mode selection,
    and LED count management.
    """
    # Signal emitted when the widget is closed
    signal_channel = QtCore.pyqtSignal(str)
    # Signal emitted to send commands to the hardware
    send_cmd_channel = QtCore.pyqtSignal(str)

    def __init__(self):
        super(LED, self).__init__()
        
        # Initialize the custom color dialog
        self.colordialog = ColorDialog()
        
        # Set up the UI from the .ui file
        self.setupUi(self)
        
        # Fix window size to prevent resizing
        self.setFixedSize(408, 275)
        
        # Set application icon
        self.setWindowIcon(QIcon('./picture/Freenove.ico'))  
        
        # Initialize command builder for hardware communication
        self.cmd = Command()
        
        # Initialize LED control variables
        self.led_mode = 0          # Current LED mode (0=Off, 1=RGB, 2=Following, etc.)
        self.led_brightness = 255  # Current brightness level (0-255)
        self.ws2812_count = 8      # Number of LEDs in the strip
        
        # Set up input validation for RGB and brightness values (0-255 only)
        lineEdit_limit_validator = QRegExpValidator(QRegExp('^?([0,1]?\d?\d|2[0-4]\d|25[0-5])$'))  # 0-255
        self.lineEdit_Led_Color_R.setValidator(lineEdit_limit_validator)
        self.lineEdit_Led_Color_G.setValidator(lineEdit_limit_validator)
        self.lineEdit_Led_Color_B.setValidator(lineEdit_limit_validator)
        self.lineEdit_Led_Brightness.setValidator(lineEdit_limit_validator)
        
        # Initialize color array from default RGB input values
        r = int(self.lineEdit_Led_Color_R.text())
        g = int(self.lineEdit_Led_Color_G.text())
        b = int(self.lineEdit_Led_Color_B.text())
        self.color = [r, g, b]
        
        # Connect all UI element signals to their respective handlers
        self.connect()
        
        # Initialize brightness from slider value and update display
        self.led_brightness = self.verticalSlider_Led_Brightness.value()
        self.rgb255_brightness_transition()

    def connect(self):
        """
        Connect all UI element signals to their respective slot functions.
        Sets up event handlers for sliders, radio buttons, text inputs, and color picker.
        """
        # Brightness slider connection
        self.verticalSlider_Led_Brightness.valueChanged.connect(self.Led_Brightnessness_Show)
        
        # Radio button connections for LED mode selection
        self.radioButton_Led_Mode_Off.clicked.connect(lambda: self.Led_Mode_Select(self.radioButton_Led_Mode_Off))
        self.radioButton_Led_Mode_RGB.clicked.connect(lambda: self.Led_Mode_Select(self.radioButton_Led_Mode_RGB))
        self.radioButton_Led_Mode_Following.clicked.connect(lambda: self.Led_Mode_Select(self.radioButton_Led_Mode_Following))
        self.radioButton_Led_Mode_Blink.clicked.connect(lambda: self.Led_Mode_Select(self.radioButton_Led_Mode_Blink))
        self.radioButton_Led_Mode_Breathing.clicked.connect(lambda: self.Led_Mode_Select(self.radioButton_Led_Mode_Breathing))
        self.radioButton_Led_Mode_Rainbow.clicked.connect(lambda: self.Led_Mode_Select(self.radioButton_Led_Mode_Rainbow))
        self.radioButton_Led_Mode_Gradient.clicked.connect(lambda: self.Led_Mode_Select(self.radioButton_Led_Mode_Gradient))
        
        # RGB text input connections for real-time color updates
        self.lineEdit_Led_Color_R.textChanged.connect(self.LineEdit_Led_Text_Change)
        self.lineEdit_Led_Color_G.textChanged.connect(self.LineEdit_Led_Text_Change)
        self.lineEdit_Led_Color_B.textChanged.connect(self.LineEdit_Led_Text_Change)
        self.lineEdit_Led_Brightness.textChanged.connect(self.LineEdit_Led_Text_Change)
        
        # Color wheel selection connection
        self.colordialog.currentColorChanged.connect(self.led_color_disk_show)
        
        # Embed the color dialog into the widget layout
        lay = QtWidgets.QVBoxLayout(self.widget_Led_Color_Disk)
        lay.addWidget(self.colordialog, alignment=Qt.AlignCenter)

    def send_cmd(self, cmd):
        """
        Emit command string to be sent to the hardware controller.
        
        Args:
            cmd (str): Formatted command string for LED control
        """
        self.send_cmd_channel.emit(cmd)

    @staticmethod
    def rgbhex_to_rgb255(rgbhex: str) -> np.array:
        """
        Convert hexadecimal color string to RGB 255 values.
        
        Args:
            rgbhex (str): Hexadecimal color string (with or without '#' prefix)
            
        Returns:
            np.array: Array containing [R, G, B] values (0-255)
        """
        # Remove '#' prefix if present
        if rgbhex[0] == '#':
            rgbhex = rgbhex[1:]
        
        # Convert hex pairs to decimal values
        r = int(rgbhex[0:2], 16)
        g = int(rgbhex[2:4], 16)
        b = int(rgbhex[4:6], 16)
        return np.array((r, g, b))

    def rgb255_brightness_transition(self):
        """
        Apply brightness scaling to the base color and update the UI display.
        Calculates the final RGB values by applying brightness as a multiplier.
        
        Returns:
            tuple: Final (r, g, b) values after brightness adjustment
        """
        # Calculate brightness-adjusted RGB values (brightness acts as a multiplier)
        r = int(self.led_brightness * self.color[0] / 255)
        g = int(self.led_brightness * self.color[1] / 255)
        b = int(self.led_brightness * self.color[2] / 255)
        
        # Update the color preview label with the calculated RGB values
        label_Led_Color_Disk_Show = 'QLabel {background-color: rgb(' + str(r) + ',' + str(g) + ',' + str(b) + ');}'
        self.label_Led_Color_Disk.setStyleSheet(str(label_Led_Color_Disk_Show))
        
        # Update the RGB input fields with the brightness-adjusted values
        self.lineEdit_Led_Color_R.setText(str(r))
        self.lineEdit_Led_Color_G.setText(str(g))
        self.lineEdit_Led_Color_B.setText(str(b))
        
        return r, g, b

    def led_color_disk_show(self, color):
        """
        Handle color selection from the color wheel widget.
        Updates the base color and sends command to hardware (for applicable modes).
        
        Args:
            color: QColor object from the color picker
        """
        # Convert the selected color to RGB array and update base color
        self.color = self.rgbhex_to_rgb255(color.name())
        
        # Apply brightness and get final RGB values
        r, g, b = self.rgb255_brightness_transition()
        
        # Build command string for hardware communication
        command = self.cmd.CUSTOM_ACTION + str("1") + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_MODE + str(self.led_mode) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_RED + str(r) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_GREEN + str(g) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_BLUE + str(b)
        
        # Only send RGB commands for modes that use color data
        # Modes 0 (Off), 5 (Rainbow), 6 (Gradient) don't need RGB values
        if (self.led_mode == 0) or (self.led_mode == 5) or (self.led_mode == 6):
            pass
        else:
            self.send_cmd(command)

    def Led_Brightnessness_Show(self):
        """
        Handle brightness slider changes.
        Updates brightness value, recalculates RGB, and sends command to hardware.
        """
        # Get current brightness from slider
        self.led_brightness = self.verticalSlider_Led_Brightness.value()
        
        # Update brightness text input to match slider
        self.lineEdit_Led_Brightness.setText(str(self.led_brightness))
        
        # Recalculate RGB values with new brightness
        r, g, b = self.rgb255_brightness_transition()
        
        # Build and send command for brightness change
        command = self.cmd.CUSTOM_ACTION + str("1") + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_MODE + str(self.led_mode) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_RED + str(r) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_GREEN + str(g) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_BLUE + str(b)
        
        # Only send commands for modes that use RGB data
        if (self.led_mode == 0) or (self.led_mode == 5) or (self.led_mode == 6):
            pass
        else:
            self.send_cmd(command)

    def Led_Mode_Select(self, parameter):
        """
        Handle LED mode selection from radio buttons.
        Updates led_mode variable, manages radio button states, and sends mode command.
        
        LED Modes:
        0 = Off
        1 = RGB (solid color)
        2 = Following (follows some input signal)
        3 = Blink
        4 = Breathing (fade in/out)
        5 = Rainbow (cycles through spectrum)
        6 = Gradient (color gradient effect)
        
        Args:
            parameter: The radio button that was clicked
        """
        # Handle Off mode
        if parameter.objectName() == 'radioButton_Led_Mode_Off':
            self.led_mode = 0
            # Uncheck all other radio buttons
            self.radioButton_Led_Mode_RGB.setChecked(False)
            self.radioButton_Led_Mode_Following.setChecked(False)
            self.radioButton_Led_Mode_Blink.setChecked(False)
            self.radioButton_Led_Mode_Breathing.setChecked(False)
            self.radioButton_Led_Mode_Rainbow.setChecked(False)
            self.radioButton_Led_Mode_Gradient.setChecked(False)
            
        # Handle RGB mode
        elif parameter.objectName() == 'radioButton_Led_Mode_RGB':
            if self.radioButton_Led_Mode_RGB.isChecked():
                self.led_mode = 1
            elif not self.radioButton_Led_Mode_RGB.isChecked():
                self.led_mode = 0
            # Uncheck all other radio buttons
            self.radioButton_Led_Mode_Off.setChecked(False)
            self.radioButton_Led_Mode_Following.setChecked(False)
            self.radioButton_Led_Mode_Blink.setChecked(False)
            self.radioButton_Led_Mode_Breathing.setChecked(False)
            self.radioButton_Led_Mode_Rainbow.setChecked(False)
            self.radioButton_Led_Mode_Gradient.setChecked(False)
            
        # Handle Following mode (LEDs follow some external signal/input)
        elif parameter.objectName() == 'radioButton_Led_Mode_Following':
            if self.radioButton_Led_Mode_Following.isChecked():
                self.led_mode = 2
            elif not self.radioButton_Led_Mode_Following.isChecked():
                self.led_mode = 0
            # Uncheck all other radio buttons
            self.radioButton_Led_Mode_Off.setChecked(False)
            self.radioButton_Led_Mode_RGB.setChecked(False)
            self.radioButton_Led_Mode_Blink.setChecked(False)
            self.radioButton_Led_Mode_Breathing.setChecked(False)
            self.radioButton_Led_Mode_Rainbow.setChecked(False)
            self.radioButton_Led_Mode_Gradient.setChecked(False)
            
        # Handle Blink mode (on/off blinking)
        elif parameter.objectName() == 'radioButton_Led_Mode_Blink':
            if self.radioButton_Led_Mode_Blink.isChecked():
                self.led_mode = 3
            elif not self.radioButton_Led_Mode_Blink.isChecked():
                self.led_mode = 0
            # Uncheck all other radio buttons
            self.radioButton_Led_Mode_Off.setChecked(False)
            self.radioButton_Led_Mode_RGB.setChecked(False)
            self.radioButton_Led_Mode_Following.setChecked(False)
            self.radioButton_Led_Mode_Breathing.setChecked(False)
            self.radioButton_Led_Mode_Rainbow.setChecked(False)
            self.radioButton_Led_Mode_Gradient.setChecked(False)
            
        # Handle Breathing mode (smooth fade in/out)
        elif parameter.objectName() == 'radioButton_Led_Mode_Breathing':
            if self.radioButton_Led_Mode_Breathing.isChecked():
                self.led_mode = 4
            elif not self.radioButton_Led_Mode_Breathing.isChecked():
                self.led_mode = 0
            # Uncheck all other radio buttons
            self.radioButton_Led_Mode_Off.setChecked(False)
            self.radioButton_Led_Mode_RGB.setChecked(False)
            self.radioButton_Led_Mode_Following.setChecked(False)
            self.radioButton_Led_Mode_Blink.setChecked(False)
            self.radioButton_Led_Mode_Rainbow.setChecked(False)
            self.radioButton_Led_Mode_Gradient.setChecked(False)
            
        # Handle Rainbow mode (cycles through color spectrum)
        elif parameter.objectName() == 'radioButton_Led_Mode_Rainbow':
            if self.radioButton_Led_Mode_Rainbow.isChecked():
                self.led_mode = 5
            elif not self.radioButton_Led_Mode_Rainbow.isChecked():
                self.led_mode = 0
            # Uncheck all other radio buttons
            self.radioButton_Led_Mode_Off.setChecked(False)
            self.radioButton_Led_Mode_RGB.setChecked(False)
            self.radioButton_Led_Mode_Following.setChecked(False)
            self.radioButton_Led_Mode_Blink.setChecked(False)
            self.radioButton_Led_Mode_Breathing.setChecked(False)
            self.radioButton_Led_Mode_Gradient.setChecked(False)
            
        # Handle Gradient mode (color gradient across LED strip)
        elif parameter.objectName() == 'radioButton_Led_Mode_Gradient':
            if self.radioButton_Led_Mode_Gradient.isChecked():
                self.led_mode = 6
            elif not self.radioButton_Led_Mode_Gradient.isChecked():
                self.led_mode = 0
            # Uncheck all other radio buttons
            self.radioButton_Led_Mode_Off.setChecked(False)
            self.radioButton_Led_Mode_RGB.setChecked(False)
            self.radioButton_Led_Mode_Following.setChecked(False)
            self.radioButton_Led_Mode_Blink.setChecked(False)
            self.radioButton_Led_Mode_Breathing.setChecked(False)
            self.radioButton_Led_Mode_Rainbow.setChecked(False)
        
        # Calculate final RGB values after mode change
        r, g, b = self.rgb255_brightness_transition()
        
        # Build command string with current mode and color values
        command = self.cmd.CUSTOM_ACTION + str("1") + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_MODE + str(self.led_mode) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_RED + str(r) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_GREEN + str(g) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_BLUE + str(b)
        
        # Send the mode change command to hardware
        self.send_cmd(command)

    def LineEdit_Led_Text_Change(self):
        """
        Handle real-time text changes in RGB and brightness input fields.
        Updates the color preview label without sending commands to hardware.
        Used for live preview while user is typing.
        """
        try:
            # Get current values from text inputs
            bright = int(self.lineEdit_Led_Brightness.text())
            r1 = int(self.lineEdit_Led_Color_R.text())
            g1 = int(self.lineEdit_Led_Color_G.text())
            b1 = int(self.lineEdit_Led_Color_B.text())
            
            # Calculate brightness-adjusted values for preview
            r2 = int(bright * r1 / 255)
            g2 = int(bright * g1 / 255)
            b2 = int(bright * b1 / 255)
            
            # Update the color preview label
            label_Led_Color_Disk_Show = 'QLabel {background-color: rgb(' + str(r2) + ',' + str(g2) + ',' + str(b2) + ');}'
            self.label_Led_Color_Disk.setStyleSheet(str(label_Led_Color_Disk_Show))
        except:
            # Silently handle invalid input during typing (empty fields, non-numeric, etc.)
            pass

    def Update_Lineedit_Color(self):
        """
        Update the internal color array from the current RGB text input values.
        Used to synchronize internal state with user input.
        """
        try:
            # Update internal color array from text inputs
            self.color[0] = int(self.lineEdit_Led_Color_R.text())
            self.color[1] = int(self.lineEdit_Led_Color_G.text())
            self.color[2] = int(self.lineEdit_Led_Color_B.text())
        except:
            # Handle invalid input gracefully
            pass

    def set_ws2812_count(self, index):
        """
        Handle LED count adjustment buttons (add/subtract).
        Updates the number of LEDs in the strip with bounds checking.
        
        Args:
            index: The button widget that was clicked (Add or Subtract)
        """
        # Determine which button was clicked and adjust count accordingly
        if index.objectName() == "pushButton_Led_Count_Subtract":
            self.ws2812_count = int(self.lineEdit_Led_Count_Value.text()) - 1
        elif index.objectName() == "pushButton_Led_Count_Add":
            self.ws2812_count = int(self.lineEdit_Led_Count_Value.text()) + 1
        
        # Enforce bounds: minimum 1 LED, maximum 255 LEDs
        if self.ws2812_count < 1:
            self.ws2812_count = 1
        elif self.ws2812_count > 255:
            self.ws2812_count = 255
        
        # Update the display with the new LED count
        self.lineEdit_Led_Count_Value.setText(str(self.ws2812_count))

    def closeEvent(self, event):
        """
        Handle widget close event.
        Emits a signal to notify parent application that LED UI is closing.
        
        Args:
            event: QCloseEvent object
        """
        # Notify parent that LED UI is closing
        self.signal_channel.emit("close led ui.")
        # Accept the close event to proceed with closing
        event.accept()


def print_cmd(cmd):
    """
    Debug function to print commands to console.
    Used for testing and development purposes.
    
    Args:
        cmd (str): Command string to print
    """
    print("cmd:" + cmd)


if __name__ == '__main__':
    """
    Main entry point for standalone execution.
    Creates the application, shows the LED control window, and starts the event loop.
    """
    # Create the Qt application
    app = QtWidgets.QApplication(sys.argv)

    # Create and configure the LED control widget
    led = LED()
    led.setWindowModality(Qt.ApplicationModal)  # Make window modal (blocks other windows)
    led.show()
    
    # Connect the command signal to debug print function for testing
    led.send_cmd_channel.connect(print_cmd)

    # Start the Qt event loop
    sys.exit(app.exec_())
