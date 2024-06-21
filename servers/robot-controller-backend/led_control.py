# File: /Omega-Code/servers/robot-controller-backend/led_control.py
import sys
import time
from rpi_ws281x import *

# LED strip configuration:
LED_COUNT      = 8      
LED_PIN        = 18     
LED_FREQ_HZ    = 800000  
LED_DMA        = 10     
LED_BRIGHTNESS = 255    
LED_INVERT     = False  
LED_CHANNEL    = 0      

class Led:
    def __init__(self):
        self.ORDER = "RGB"  
        self.strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()

    def LED_TYPR(self,order,R_G_B):
        B=R_G_B & 255
        G=R_G_B >> 8 & 255
        R=R_G_B >> 16 & 255 
        Led_type=["GRB","GBR","RGB", "RBG","BRG","BGR"]
        color = [Color(G,R,B),Color(G,B,R),Color(R,G,B),Color(R,B,G),Color(B,R,G),Color(B,G,R)]
        if order in Led_type:
            return color[Led_type.index(order)]

    def colorWipe(self, color, wait_ms=50):
        color=self.LED_TYPR(self.ORDER,color)
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms/1000.0)

    def theaterChase(self, color, wait_ms=50, iterations=10):
        color=self.LED_TYPR(self.ORDER,color)
        for j in range(iterations):
            for q in range(3):
                for i in range(0,self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, color)
                self.strip.show()
                time.sleep(wait_ms/1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, 0)

    def wheel(self,pos):
        if pos<0 or pos >255:
            r=g=b=0
        elif pos < 85:
            r=pos * 3
            g=255 - pos * 3
            b=0
        elif pos < 170:
            pos -= 85
            r=255 - pos * 3
            g=0
            b=pos * 3
        else:
            pos -= 170
            r=0
            g=pos * 3
            b=255 - pos * 3
        return self.LED_TYPR(self.ORDER,Color(r,g,b))

    def rainbow(self, wait_ms=20, iterations=1):
        for j in range(256*iterations):
            for i in range(self.strip.numPixels()):
                 self.strip.setPixelColor(i, self.wheel((i+j) & 255))
            self.strip.show()
            time.sleep(wait_ms/1000.0)

    def rainbowCycle(self, wait_ms=20, iterations=5):
        for j in range(256*iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, self.wheel((int(i * 256 / self.strip.numPixels()) + j) & 255))
            self.strip.show()
            time.sleep(wait_ms/1000.0)

    def theaterChaseRainbow(self, wait_ms=50):
        for j in range(256):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, self.wheel((i+j) % 255))
                self.strip.show()
                time.sleep(wait_ms/1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, 0)

    def ledIndex(self,index,R,G,B):
        color=self.LED_TYPR(self.ORDER,Color(R,G,B))
        for i in range(8):
            if index & 0x01 == 1:
                self.strip.setPixelColor(i,color)
                self.strip.show()
            index=index >> 1

    def setLed(self, color, mode, pattern, interval):
        if mode == 'single':
            self.colorWipe(Color(color))
        elif mode == 'multi':
            self.theaterChase(Color(color), wait_ms=interval)
        elif mode == 'two':
            self.rainbow(wait_ms=interval)
        else:
            self.colorWipe(Color(0, 0, 0), 10)

led=Led()                 

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print("Usage: python3 led_control.py <color> <mode> <pattern> <interval>")
        sys.exit(1)

    color = int(sys.argv[1], 16)
    mode = sys.argv[2]
    pattern = sys.argv[3]
    interval = int(sys.argv[4])

    led.setLed(color, mode, pattern, interval)

