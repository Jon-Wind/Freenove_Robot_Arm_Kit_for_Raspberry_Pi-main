# Freenove Robot Arm Kit for Raspberry Pi

## Client Setup and Installation

### Prerequisites
- Windows operating system
- Python 3.x installed
- Basic knowledge of command line operations

### Installation Steps

1. **Run the Setup Script**
   - Navigate to the `Client/Arm_Software_Setup/` directory
   - Run the appropriate setup script for your operating system:
     - For Windows: Double-click `setup_window.py`
     - For Linux: Run `python3 setup_linux.py`
     - For Mac: Run `python3 setup_mac.py`

2. **Install Required Dependencies**
   - The setup script will automatically install all necessary Python packages
   - Make sure you have an active internet connection during this process

### Running the Client Application

1. **Open Command Prompt**
   - Navigate to `Client/Arm_Software_Codes/` in File Explorer
   - Click on the address bar at the top of the window
   - Type `cmd` and press Enter to open Command Prompt in this directory

2. **Start the Application**
   - In the Command Prompt, type:
     ```
     python main.py
     ```
   - Press Enter to run the client application

3. **Using the Application**
   - The robot arm control interface should now open
   - Follow the on-screen instructions to connect to your robot arm
   - Use the interface to control the arm's movements and functions

### Troubleshooting
- If you encounter any permission errors, try running Command Prompt as Administrator
- Ensure all required Python packages are installed if you see any import errors
- Make sure the robot arm is properly connected to your computer

For additional help, please refer to the documentation in the `Docs/` directory or contact Freenove support.