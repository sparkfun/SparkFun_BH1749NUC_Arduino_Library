# pyRGBVisualizer RGB Control 

This example pairs a simple Serial firmata with a Python sketch to dynamically monitor the RGB values of the color sensor.

It can also set maximum RGB values to help hone in the calibrated RGB reading.

![Example RGB Sensor Evaluation Window usage](TODO)

## Arduino Setup Instructions

Connect your RGB Sensor via a Qwiic cable to an Arduino via either a Qwiic shield or using a SparkX BlackBoard.

Upload the Example6_pyRGBVisualizer.ino sketch to your Arduino and run it.

## Python Setup Instructions

1. Install Python 3 -- [https://www.python.org/download/releases/3.0/](https://www.python.org/download/releases/3.0/)
    - You may need to add the Python directory to your system's PATH.
2. Install pyserial: 
    - Open a command window and type: `pip install pyserial`

### Modifying the Python Script

1. Set your serial port on line 18 of the _pyRGBVisualizer.py_ script to e.g. "COM4" or "/dev/ttyUSB0".
2. You may also want to configure the value of `MAX_COLOR_VALUE` on line 15.

### Running the Python Script

To run the python script, open a command window in this directory and type `python pyRGBVisualizer.py` -- a GUI should open immediately.

Close the program by closing the GUI window.

## Using the pyRGBVisualizer

The pyRGB visualizer allows you to turn on/off all four on-board LEDs with the "White", "Red", "Green", and "Blue" buttons.

The maximum red, green, and blue values can be set with the slider or text boxes/set button. The "Calibrate Max Values" button can be pressed to set all 3 max values to the current readings.