import tkinter as tk
import serial

''' Constants '''
RED   = 0
GREEN = 1
BLUE  = 2
IR    = 3
WHITE_LED = 0
RED_LED   = 1
GREEN_LED = 2
BLUE_LED  = 3

RGB_POLL_RATE = 100
MAX_COLOR_VALUE = 9192

''' Serial Control '''
ser = serial.Serial('COM4', 115200)

''' Globals '''
ledStates = [0, 0, 0, 0]
maxColorValues = [1, 1, 1]
latestColorValues = [0, 0, 0]

''' UI set up '''
master=tk.Tk()
master.winfo_toplevel().title("RGB Sensor Evaluation")
master.grid()
whiteButton=tk.Button()
redButton=tk.Button()
greenButton=tk.Button()
blueButton=tk.Button()
redEntry=tk.Entry(master, width=5)
greenEntry=tk.Entry(master, width=5)
blueEntry=tk.Entry(master, width=5)
redSlider=tk.Scale(master)
greenSlider=tk.Scale(master)
blueSlider=tk.Scale(master)
redText=tk.Label(master, text="0", fg="red")
greenText=tk.Label(master, text="0", fg="green")
blueText=tk.Label(master, text="0", fg="blue")
canvas = tk.Canvas(master, width=200, height=100)

# LED control
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
        redButton.grid(row=2,column=2)
    else:
        ser.write('r0'.encode('utf-8'))
        ledStates[RED_LED] = 0
        redButton.configure(bg='white')
        redButton.grid(row=2,column=2)
    
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

def setRedMax(value):
    global maxColorValues
    maxColorValues[RED] = value
    redEntry.delete(0, 5)
    redEntry.insert(0, str(value))

def setGreenMax(value):
    global maxColorValues
    maxColorValues[GREEN] = value
    greenEntry.delete(0, 5)
    greenEntry.insert(0, str(value))

def setBlueMax(value):
    global maxColorValues
    maxColorValues[BLUE] = value
    blueEntry.delete(0, 5)
    blueEntry.insert(0, str(value))

def redMaxEntry():
    global maxColorValues
    tempMax = int(redEntry.get())
    if (tempMax < 1) or (tempMax > MAX_COLOR_VALUE):
        return
    maxColorValues[RED] = tempMax
    redSlider.set(int(maxColorValues[RED]))

def greenMaxEntry():
    global maxColorValues
    tempMax = int(greenEntry.get())
    if (tempMax < 1) or (tempMax > MAX_COLOR_VALUE):
        return
    maxColorValues[GREEN] = tempMax
    greenSlider.set(int(maxColorValues[GREEN]))

def blueMaxEntry():
    global maxColorValues
    tempMax = int(blueEntry.get())
    if (tempMax < 1) or (tempMax > MAX_COLOR_VALUE):
        return
    maxColorValues[BLUE] = tempMax
    blueSlider.set(int(maxColorValues[BLUE]))

def calibrateMax():
    global maxColorValues
    global latestColorValues
    for color in range(3):
        maxColorValues[color] = latestColorValues[color]
        if maxColorValues[color] <= 0:
            maxColorValues[color] = 1
        if maxColorValues[color] > MAX_COLOR_VALUE:
            maxColorValues[color] = MAX_COLOR_VALUE
    redSlider.set(int(maxColorValues[RED]))
    greenSlider.set(int(maxColorValues[GREEN]))
    blueSlider.set(int(maxColorValues[BLUE]))

''' UI Configuration '''
def setupUI():
    buttonRow = 1
    ledText=tk.Label(master, text="LED Control: ")
    ledText.grid(row=buttonRow, column=0)

    whiteButton=tk.Button(master,text="White", command=whiteLED)
    whiteButton.grid(row=buttonRow,column=1)

    redButton=tk.Button(master, text="Red", command=redLED)
    redButton.grid(row=buttonRow,column=2)

    greenButton=tk.Button(master,text="Green", command=greenLED)
    greenButton.grid(row=buttonRow,column=3)

    blueButton=tk.Button(master,text="Blue", command=blueLED)
    blueButton.grid(row=buttonRow,column=4)

    infoRow = 2
    valueText=tk.Label(master, text="Current")
    valueText.grid(row=infoRow, column=1)
    infoText=tk.Label(master, text="Max values")
    infoText.grid(row=infoRow, column=2, columnspan=3)

    redRow=3
    redText.grid(row=redRow, column=0)
    redSlider=tk.Scale(master, from_=1, to=MAX_COLOR_VALUE, orient=tk.HORIZONTAL, command=setRedMax)
    redSlider.set(MAX_COLOR_VALUE)
    redSlider.grid(row=redRow, column=1, columnspan=4, sticky="W")
    redEntry.grid(row=redRow, column=4, columnspan=1)
    redEntryButton=tk.Button(master, text="Set", command=redMaxEntry)
    redEntryButton.grid(row=redRow, column=5)

    greenRow=redRow+1
    greenText.grid(row=greenRow, column=0)
    greenSlider=tk.Scale(master, from_=1, to=MAX_COLOR_VALUE, orient=tk.HORIZONTAL, command=setGreenMax)
    greenSlider.set(MAX_COLOR_VALUE)
    greenSlider.grid(row=greenRow, column=1, columnspan=10, sticky="W")
    greenEntry.grid(row=greenRow, column=4, columnspan=1)
    greenEntryButton=tk.Button(master, text="Set", command=greenMaxEntry)
    greenEntryButton.grid(row=greenRow, column=5)

    blueRow=greenRow+1
    blueText.grid(row=blueRow, column=0)
    blueSlider=tk.Scale(master, from_=1, to=MAX_COLOR_VALUE, orient=tk.HORIZONTAL, command=setBlueMax)
    blueSlider.set(MAX_COLOR_VALUE)
    blueSlider.grid(row=blueRow, column=1, columnspan=10, sticky="W")
    blueEntry.grid(row=blueRow, column=4, columnspan=1)
    blueEntryButton=tk.Button(master, text="Set", command=blueMaxEntry)
    blueEntryButton.grid(row=blueRow, column=5)

    calibrateRow=blueRow+1
    calibrateMaxButton=tk.Button(master, text="Calibrate Max Values", command=calibrateMax)
    calibrateMaxButton.grid(row=calibrateRow, column=0, columnspan=5)
    
    colorRow=calibrateRow+1
    canvas.create_rectangle(30, 10, 120, 80, outline="black", fill ="black")
    canvas.grid(row=colorRow, column=1, columnspan=6)

def pollRGB():
    global maxColorValues
    global latestColorValues
    line = ser.readline().rstrip().decode("utf-8")
    rgb = line.split(',')
    print("{0}/{1},{2}/{3},{4}/{5}".format(int(rgb[RED]), maxColorValues[RED], 
    int(rgb[GREEN]), maxColorValues[GREEN], int(rgb[BLUE]), maxColorValues[BLUE]))
    
    redText.config(text=rgb[RED])
    greenText.config(text=rgb[GREEN])
    blueText.config(text=rgb[BLUE])
    
    for color in range(3):
        rgb[color] = int(rgb[color])
        maxColorValues[color] = int(maxColorValues[color])
        if rgb[color] > int(maxColorValues[color]):
            rgb[color] = maxColorValues[color]
        latestColorValues[color] = rgb[color]
    rgb[RED] = rgb[RED] / int(maxColorValues[RED]) * 255
    rgb[GREEN] = rgb[GREEN] / int(maxColorValues[GREEN]) * 255
    rgb[BLUE] = rgb[BLUE] / int(maxColorValues[BLUE]) * 255
    colorval = "#%02x%02x%02x" % (int(rgb[RED]), int(rgb[GREEN]), int(rgb[BLUE]))
    print(colorval)
    canvas.create_rectangle(30, 10, 120, 80, outline=colorval, fill =colorval)
    canvas.grid(row=7, column=1, columnspan=6)
    
    master.after(RGB_POLL_RATE, pollRGB)

setupUI()
setRedMax(MAX_COLOR_VALUE)
setGreenMax(MAX_COLOR_VALUE)
setBlueMax(MAX_COLOR_VALUE)
master.after(RGB_POLL_RATE, pollRGB)
master.mainloop()