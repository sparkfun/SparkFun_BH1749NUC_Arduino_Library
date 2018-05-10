from graphics import *
import serial
import tkinter as tk

RED   = 0
GREEN = 1
BLUE  = 2
IR    = 3

WHITE_LED = 0
RED_LED   = 1
GREEN_LED = 2
BLUE_LED  = 3

maxColorValues = [1, 1, 1]

ser = serial.Serial('COM4', 115200) # Open serial port
ledStates = [0, 0, 0, 0]

def whiteLED():
    if ledStates[WHITE_LED] == 0:
        ser.write('w1'.encode('utf-8'))
        ledStates[WHITE_LED] = 1
        whiteButton.configure(bg='black')
    else:
        ser.write('w0'.encode('utf-8'))
        ledStates[WHITE_LED] = 0
        whiteButton.configure(bg='white')

def redLED():
    if ledStates[RED_LED] == 0:
        ser.write('r1'.encode('utf-8'))
        ledStates[RED_LED] = 1
        redButton.configure(bg='red')
    else:
        ser.write('r0'.encode('utf-8'))
        ledStates[RED_LED] = 0
        redButton.configure(bg='white')
    
def greenLED():
    if ledStates[GREEN_LED] == 0:
        ser.write('g1'.encode('utf-8'))
        ledStates[GREEN_LED] = 1
        greenButton.configure(bg='green')
    else:
        ser.write('g0'.encode('utf-8'))
        ledStates[GREEN_LED] = 0
        greenButton.configure(bg='white')
    
def blueLED():
    if ledStates[BLUE_LED] == 0:
        ser.write('b1'.encode('utf-8'))
        ledStates[BLUE_LED] = 1
        blueButton.configure(bg='blue')
    else:
        ser.write('b0'.encode('utf-8'))
        ledStates[BLUE_LED] = 0
        blueButton.configure(bg='white')

def setRedMax(newValue):
    maxColorValues[RED] = newValue
    
def setGreenMax(newValue):
    maxColorValues[GREEN] = newValue
    
def setBlueMax(newValue):
    maxColorValues[BLUE] = newValue

root = tk.Tk()
frame = tk.Frame(root)
root.grid()
#frame.pack()

canvas = tk.Canvas(root, width=200, height=100)
canvas.create_rectangle(30, 10, 120, 80, outline="black", fill="black")

whiteButton = tk.Button(frame, text="White", command=whiteLED)
whiteButton.grid(row=1, column=1)
#whiteButton.pack(side=tk.LEFT)
redButton = tk.Button(frame, text="Red", command=redLED)
redButton.grid(row=1, column=2)
#redButton.pack(side=tk.LEFT, fill=tk.X)
greenButton = tk.Button(frame, text="Green", command=greenLED)
greenButton.grid(row=1, column=3)
#greenButton.pack(side=tk.LEFT, fill=tk.X)
blueButton = tk.Button(frame, text="Blue", command=blueLED)
blueButton.grid(row=1, column=4)
#blueButton.pack(side=tk.LEFT, fill=tk.X)
quitButton = tk.Button(frame, text="Quit", command=quit)
quitButton.grid(row=1, column=5)
#quitButton.pack()

redLabel = tk.Label(root, fg="red")
redLabel.config(text=str(0))
#redLabel.grid(row=2, column=0)
#redLabel.pack(side=tk.LEFT)
redSlider = tk.Scale(root, from_=1, to=65535, orient=tk.HORIZONTAL, command=setRedMax)
#redSlider.grid(row=2, column=1)
#redSlider.pack(side=tk.LEFT)

greenLabel = tk.Label(root, fg="green")
greenLabel.config(text=str(0))
#greenLabel.pack()
greenSlider = tk.Scale(root, from_=1, to=65535, orient=tk.HORIZONTAL, command=setGreenMax)
#greenSlider.pack(side=tk.LEFT)

blueLabel = tk.Label(root, fg="blue")
blueLabel.config(text=str(0))
#blueLabel.pack()
blueSlider = tk.Scale(root, from_=1, to=65535, orient=tk.HORIZONTAL, command=setBlueMax)
#blueSlider.pack(side=tk.LEFT)


def readRGB():
    line = ser.readline().rstrip().decode("utf-8")
    rgb = line.split(',')
    print("{0},{1},{2}".format(int(rgb[RED]), int(rgb[GREEN]), int(rgb[BLUE])))

    redLabel.config(text=rgb[RED])
    greenLabel.config(text=rgb[GREEN])
    blueLabel.config(text=rgb[BLUE])

    for color in range(3):
        rgb[color] = int(rgb[color])
        maxColorValues[color] = int(maxColorValues[color])
        if rgb[color] > int(maxColorValues[color]):
            rgb[color] = maxColorValues[color]
    rgb[RED] = rgb[RED] / maxColorValues[RED] * 255
    rgb[GREEN] = rgb[GREEN] / maxColorValues[GREEN] * 255
    rgb[BLUE] = rgb[BLUE] / maxColorValues[BLUE] * 255
    colorval = "#%02x%02x%02x" % (int(rgb[RED]), int(rgb[GREEN]), int(rgb[BLUE]))
    canvas.create_rectangle(30, 10, 120, 80, outline=colorval, fill =colorval)
    #canvas.pack()

    root.after(100, readRGB)

root.after(100, readRGB)
root.mainloop()