import numpy as np
import cv2
from PIL import ImageGrab
# import keyboard
import time
import win32api
import win32con
import math
from pynput import mouse
from pynput import keyboard
import pyttsx3


# Velocity of Projectile
V = 117.2
# pi
pi = 3.1415926535
# Scroll Displacement
Scroll_D = 9550
# Zoom Factor
ZoomFactor = 1

# Measuring / Matching mode switch parameters
isMeasuring = False

# index of inputting point
m_point = 1

# Pixel distance % Real Distance
mDistance = 0
realDistance = 0

# Conversion Scale
scale = 0
print('After Pressing M to open the map')
print('Press Space Key to and measure')


def AutoMeasure(k_scale):
    frame = np.array(ImageGrab.grab(bbox=(0, 0, 2560, 1440)))
    # frame = cv2.resize(frame, (1920, 1080))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(np.array(frame), cv2.COLOR_BGR2HSV)
    lower_blue = np.array([55, 150, 150])
    upper_blue = np.array([65, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    result = cv2.bitwise_and(frame, frame, mask=mask)
    result[700:740, 1260:1300, :] = [0, 0, 0]
    result[720:1440, 0:200, :] = [0, 0, 0]
    kernel = np.ones((2, 2), np.uint8)
    result = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel)
    filename = r'D:\Desktop\OutputImg.jpg'
    cv2.imwrite(filename, result)
    # result = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)

    try:
        index = np.nonzero(result)
        index_x_ori = index[1]
        index_x = index_x_ori[0:-1:3]
        index_y_ori = index[0]
        index_y = index_y_ori[0:-1:3]

        mean_x_0 = 1280
        mean_y_0 = 720

        mean_y_1 = np.max(index_y)
        print('mean_y_1', mean_y_1)
        idx = np.where(index_y == mean_y_1)
        print('mean_y_1', mean_y_1)
        mean_x_1 = np.min(index_x[idx])

        point_1 = [mean_x_0, mean_y_0]
        point_2 = [mean_x_1, mean_y_1]
        print(point_1)
        print(point_2)

        pixelDistance = math.sqrt(
            (point_2[0] - point_1[0]) * (point_2[0] - point_1[0]) + (point_2[1] - point_1[1]) * (
                    point_2[1] - point_1[1]))
        print('Dis_pix:', pixelDistance)
        realD = pixelDistance * k_scale
        realDistance_str = str(int(realD))
        print('Real Distance:', realDistance_str)
        engine = pyttsx3.init()
        engine.setProperty('rate', 300)
        engine.say(realDistance_str)
        engine.runAndWait()
        engine.stop()
    except:
        realD = 0
        print('No region')
    return realD

def on_scroll(x, y, dx, dy):
    global ZoomFactor
    global isMeasuring
    if isMeasuring:
        if dy > 0 and isMeasuring:
            print('Up')
            if ZoomFactor < 5:
                ZoomFactor = ZoomFactor + 1
        elif dy < 0 and isMeasuring:
            print('Down')
            if ZoomFactor > 1:
                ZoomFactor = ZoomFactor - 1
        print('zoom factor:', ZoomFactor)


LS = mouse.Listener(on_scroll=on_scroll)
LS.start()


def on_press(key):
    global ZoomFactor
    global isMeasuring
    global scale
    global realDistance
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
        if key.char == 'm':
            if isMeasuring:
                isMeasuring = False
            else:
                isMeasuring = True
            print('isMeasuring:', isMeasuring)

    except AttributeError:
        print('special key {0} pressed'.format(key))
        if not isMeasuring:
            if key == keyboard.Key.up:
                win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, 0, 0, -Scroll_D, 0)
                print('zeroing')
            elif key == keyboard.Key.down:
                if 121 < realDistance < 700:
                    if 500 < realDistance < 700:
                        realDistance = realDistance - 1
                    elif 400 < realDistance < 500:
                        realDistance = realDistance - 2
                    elif 300 < realDistance < 400:
                        realDistance = realDistance - 3
                    elif 200 < realDistance < 300:
                        realDistance = realDistance - 4
                    elif 121 < realDistance < 200:
                        realDistance = realDistance - 5

                    DEG = (pi - math.asin(2 * 9.81 * realDistance / (V * V))) / 2
                    # print(DEG)
                    Scroll_d = int(round((DEG - pi / 4) / (2 / 9 * pi) * Scroll_D, 0))
                    print('Aiming')
                    win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, 0, 0, Scroll_d, 0)
        elif isMeasuring:
            if key == keyboard.Key.space:
                if ZoomFactor == 1:
                    scale = 100 / 18
                elif ZoomFactor == 2:
                    scale = 100 / 37.9
                elif ZoomFactor == 3:
                    scale = 100 / 72.1
                elif ZoomFactor == 4:
                    scale = 100 / 145
                elif ZoomFactor == 5:
                    scale = 100 / 289
                realDistance = AutoMeasure(scale)
                print('Targeting')
                print('scale:', scale)
            elif key == keyboard.Key.esc:
                isMeasuring = False
                print('isMeasuring:', isMeasuring)
            elif key == keyboard.Key.tab:
                isMeasuring = False
                print('isMeasuring:', isMeasuring)


listener = keyboard.Listener(on_press=on_press)
listener.start()



while 1:
    time.sleep(0.1)