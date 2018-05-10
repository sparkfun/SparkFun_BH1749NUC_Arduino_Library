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
maxColorValues = [MAX_COLOR_VALUE, MAX_COLOR_VALUE, MAX_COLOR_VALUE]
minColorValues = [0, 0, 0]
latestColorValues = [0, 0, 0]
g_pause = 0

''' UI set up '''
master=tk.Tk()
master.winfo_toplevel().title("RGB Sensor Evaluation")
master.grid()

whiteButton=tk.Button()
redButton=tk.Button()
greenButton=tk.Button()
blueButton=tk.Button()

redText=tk.Label(master, text="0", fg="red")
redMaxEntry=tk.Entry(master, width=5)
redMinClick=tk.Button()
redMinEntry=tk.Entry(master, width=5)

greenText=tk.Label(master, text="0", fg="green")
greenMaxEntry=tk.Entry(master, width=5)
greenMinClick=tk.Button()
greenMinEntry=tk.Entry(master, width=5)

blueText=tk.Label(master, text="0", fg="blue")
blueMaxEntry=tk.Entry(master, width=5)
blueMinClick=tk.Button()
blueMinEntry=tk.Entry(master, width=5)

colorMinEntry = [redMinEntry, greenMinEntry, blueMinEntry]

canvas = tk.Canvas(master, width=200, height=100)

rgbLabel = tk.Label(master, text="#000000", fg="black")

pauseButton=tk.Button()

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

def updateRedMin():
    global latestColorValues
    global maxColorValues
    low = latestColorValues[RED]
    high = maxColorValues[RED]
    minColorValues[RED] = low
    maxColorValues[RED] = high
    redMinEntry.delete(0, 5)
    redMinEntry.insert(0, str(low))
    redMaxEntry.delete(0, 5)
    redMaxEntry.insert(0, str(high))

def updateGreenMin():
    global latestColorValues
    global maxColorValues
    low = latestColorValues[GREEN]
    high = maxColorValues[GREEN]
    minColorValues[GREEN] = low
    maxColorValues[GREEN] = high
    greenMinEntry.delete(0, 5)
    greenMinEntry.insert(0, str(low))
    greenMaxEntry.delete(0, 5)
    greenMaxEntry.insert(0, str(high))

def updateBlueMin():
    global latestColorValues
    global maxColorValues
    low = latestColorValues[BLUE]
    high = maxColorValues[BLUE]
    minColorValues[BLUE] = low
    maxColorValues[BLUE] = high
    blueMinEntry.delete(0, 5)
    blueMinEntry.insert(0, str(low))
    blueMaxEntry.delete(0, 5)
    blueMaxEntry.insert(0, str(high))

def setRedMinMax(low, high):
    global maxColorValues
    global minColorValues
    minColorValues[RED] = low
    maxColorValues[RED] = high
    redMinEntry.delete(0, 5)
    redMinEntry.insert(0, str(low))
    redMaxEntry.delete(0, 5)
    redMaxEntry.insert(0, str(high))

def setGreenMinMax(low, high):
    global maxColorValues
    global minColorValues
    minColorValues[GREEN] = low
    maxColorValues[GREEN] = high
    greenMinEntry.delete(0, 5)
    greenMinEntry.insert(0, str(low))
    greenMaxEntry.delete(0, 5)
    greenMaxEntry.insert(0, str(high))

def setBlueMinMax(low, high):
    global maxColorValues
    global minColorValues
    minColorValues[BLUE] = low
    maxColorValues[BLUE] = high
    blueMinEntry.delete(0, 5)
    blueMinEntry.insert(0, str(low))
    blueMaxEntry.delete(0, 5)
    blueMaxEntry.insert(0, str(high))

def updateRedMax():
    global latestColorValues
    setRedMinMax(minColorValues[RED], latestColorValues[RED])

def updateGreenMax():
    global latestColorValues
    setGreenMinMax(minColorValues[GREEN], latestColorValues[GREEN])

def updateBlueMax():
    global latestColorValues
    setBlueMinMax(minColorValues[BLUE], latestColorValues[BLUE])

def calibrateMax():
    global maxColorValues
    global latestColorValues
    for color in range(3):
        maxColorValues[color] = latestColorValues[color]
        if maxColorValues[color] <= 0:
            maxColorValues[color] = 1
        if maxColorValues[color] > MAX_COLOR_VALUE:
            maxColorValues[color] = MAX_COLOR_VALUE

def pause():
    global g_pause
    if g_pause == 0:
        g_pause = 1
    else:
        g_pause = 0

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
    currentText=tk.Label(master, text="Current")
    currentText.grid(row=infoRow, column=0)
    minText=tk.Label(master, text="Min")
    minText.grid(row=infoRow, column=1)
    maxText=tk.Label(master, text="Max")
    maxText.grid(row=infoRow, column=3)
    maxText.grid(row=infoRow, column=2, columnspan=3)

    redRow=3
    redText.grid(row=redRow, column=0)
    redMinEntry.grid(row=redRow, column=1, columnspan=1)
    redMinClick=tk.Button(master, text="rMin", command=updateRedMin)
    redMinClick.grid(row=redRow, column=2, columnspan=1)
    redMaxEntry.grid(row=redRow, column=3, columnspan=1)
    redMaxClick=tk.Button(master, text="rMax", command=updateRedMax)
    redMaxClick.grid(row=redRow, column=4)

    greenRow=redRow+1
    greenText.grid(row=greenRow, column=0)
    greenMinEntry.grid(row=greenRow, column=1, columnspan=1)
    greenMinClick=tk.Button(master, text="gMin", command=updateGreenMin)
    greenMinClick.grid(row=greenRow, column=2, columnspan=1)
    greenMaxEntry.grid(row=greenRow, column=3, columnspan=1)
    greenMaxClick=tk.Button(master, text="gMax", command=updateGreenMax)
    greenMaxClick.grid(row=greenRow, column=4)

    blueRow=greenRow+1
    blueText.grid(row=blueRow, column=0)
    blueMinEntry.grid(row=blueRow, column=1, columnspan=1)
    blueMinClick=tk.Button(master, text="bMin", command=updateBlueMin)
    blueMinClick.grid(row=blueRow, column=2, columnspan=1)
    blueMaxEntry.grid(row=blueRow, column=3, columnspan=1)
    blueMaxClick=tk.Button(master, text="bMax", command=updateBlueMax)
    blueMaxClick.grid(row=blueRow, column=4)

    calibrateRow=blueRow+1
    calibrateMaxButton=tk.Button(master, text="Calibrate Max Values", command=calibrateMax)
    calibrateMaxButton.grid(row=calibrateRow, column=0, columnspan=5)
    
    colorRow=calibrateRow+1
    canvas.create_rectangle(30, 10, 120, 80, outline="black", fill ="black")
    canvas.grid(row=colorRow, column=1, columnspan=6)

    rgbLabelRow=colorRow + 1
    rgbLabel.grid(row=rgbLabelRow, column=1, columnspan=6)

    pauseButtonRow=rgbLabelRow + 1
    pauseButton=tk.Button(master, text="Pause", command=pause)
    pauseButton.grid(row=pauseButtonRow, column=1, columnspan=6)

def pollRGB():
    global maxColorValues
    global latestColorValues
    global g_pause

    if g_pause == 1:
        return

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
    rgbLabel.config(text=colorval)
    canvas.create_rectangle(30, 10, 120, 80, outline=colorval, fill =colorval)
    canvas.grid(row=7, column=1, columnspan=6)
    
    master.after(RGB_POLL_RATE, pollRGB)

setRedMinMax(minColorValues[RED], maxColorValues[RED])
setGreenMinMax(minColorValues[GREEN], maxColorValues[GREEN])
setBlueMinMax(minColorValues[BLUE], maxColorValues[BLUE])

setupUI()
master.after(RGB_POLL_RATE, pollRGB)
master.mainloop()