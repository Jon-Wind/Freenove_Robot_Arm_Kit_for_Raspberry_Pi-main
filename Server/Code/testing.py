#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Motor and Sensor Test Script
This script tests one motor and one sensor from the Freenove Robot Arm.
"""

import time
import os
import sys
from stepmotor import StepMotor
from sensor import TCRT5000

def test_motor_and_sensor():
    """Test function to run one motor and read from one sensor."""
    print("Starting motor and sensor test...")
    
    try:
        # Initialize pigpio daemon if not already running
        os.system("sudo pigpiod")
        time.sleep(1)  # Give the daemon time to start
        
        # Initialize motor and sensor
        print("Initializing motor and sensor...")
        motor = StepMotor()
        sensor = TCRT5000()
        
        # Test motor 1 (base motor)
        motor_number = 1
        direction = 1  # 0 or 1 for direction
        pulse_count = 200  # Number of steps
        pulse_frequency = 500  # Hz
        
        print(f"Running motor {motor_number} for {pulse_count} steps...")
        print("Sensor values before movement:", sensor.readTCRT5000ALL())
        
        # Run the motor
        motor.motorRun(motor_number, direction, pulse_count, pulse_frequency)
        
        print("Motor movement complete.")
        print("Sensor values after movement:", sensor.readTCRT5000ALL())
        
        # Continuous sensor reading test
        print("\nStarting continuous sensor reading (press Ctrl+C to stop):")
        try:
            while True:
                sensor_values = sensor.readTCRT5000ALL()
                print(f"Sensor values: {sensor_values}", end='\r')
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nTest stopped by user.")
    
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Cleanup
        print("Cleaning up...")
        # Disable motor drivers
        motor.setA4988Enable(0)
        print("Test complete.")

if __name__ == "__main__":
    test_motor_and_sensor()