//KB1UIF (A.Tedds)
//Paint Sketch for Arduino LCD TFT 2.4" Touch Display 8 bit ILI9341.

#include <Adafruit_GFX_AS.h>
#include <Adafruit_ILI9341_8bit_AS.h>
#include <TouchScreen.h>

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

#define YP A1  // must be an analog pin.
#define XM A2  // must be an analog pin.
#define YM 7   // can be a digital pin.
#define XP 6   // can be a digital pin.

#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940

#define BOXSIZE 40
#define PENRADIUS 3

int oldcolor, currentcolor;

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// Its about 300 ohms across the X plate on mine.
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0xFFFF
#define BLUE    0xFFE0
#define GREEN   0xF81F
#define CYAN    0xF800
#define GRAY1   0x8410
#define RED     0x07FF
#define GRAY2   0x4208
#define MAGENTA 0x07E0
#define YELLOW  0x001F
#define WHITE   0x0000

Adafruit_ILI9341_8bit_AS tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

void setup(void) 
{
  //Serial.begin(9600);
  //Serial.println(F("Paint!"));

  tft.reset();

  uint16_t identifier = tft.readID();

  //Serial.print(F("LCD driver chip: "));
  //Serial.println(identifier,HEX);

 // tft.begin(identifier);
  tft.begin(0x9341);
  tft.fillScreen(BLACK);
  tft.setRotation(0);

  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
  tft.fillRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, GREEN);
  tft.fillRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, CYAN);
  tft.fillRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, BLUE);
  tft.fillRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, MAGENTA);

  //Draw a dot for graphic co-ordinations
  //tft.fillCircle(10, 10, PENRADIUS, GREEN); //y,x,size,color
  //tft.fillCircle(230, 310, PENRADIUS, RED); //y,x,size,color

  tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
  currentcolor = RED;

  //pinMode(13, OUTPUT);
}

#define MINPRESSURE 5
#define MAXPRESSURE 1500

void loop()
{
  //digitalWrite(13, HIGH);
  // Recently Point was renamed TSPoint in the TouchScreen library
  // If you are using an older version of the library, use the
  // commented definition instead.
  Point p = ts.getPoint();

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);


  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
  {
    if (p.x < (TS_MINX - 5))
    {
      //Serial.println("erase");
      // press the bottom of the screen to erase
      tft.fillRect(0, BOXSIZE, tft.width(), tft.height() - BOXSIZE, BLACK);
    }

    p.x = map(p.x, TS_MINX, TS_MAXX, 0, 320);
    p.y = map(p.y, TS_MINY, TS_MAXY, 0, 240);


    if (p.x < BOXSIZE)
    {
      oldcolor = currentcolor;

      if (p.y < BOXSIZE)
      {
        currentcolor = RED;
        tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
      } else if (p.y < BOXSIZE * 2)
      {
        currentcolor = YELLOW;
        tft.drawRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, WHITE);
      } else if (p.y < BOXSIZE * 3)
      {
        currentcolor = GREEN;
        tft.drawRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, WHITE);
      } else if (p.y < BOXSIZE * 4)
      {
        currentcolor = CYAN;
        tft.drawRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, WHITE);
      } else if (p.y < BOXSIZE * 5)
      {
        currentcolor = BLUE;
        tft.drawRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, WHITE);
      } else if (p.x < BOXSIZE * 6)
      {
        currentcolor = MAGENTA;
        tft.drawRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, WHITE);
      }

      if (oldcolor != currentcolor)
      {
        if (oldcolor == RED) tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
        if (oldcolor == YELLOW) tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
        if (oldcolor == GREEN) tft.fillRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, GREEN);
        if (oldcolor == CYAN) tft.fillRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, CYAN);
        if (oldcolor == BLUE) tft.fillRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, BLUE);
        if (oldcolor == MAGENTA) tft.fillRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, MAGENTA);
      }
    }
    if (((p.x - PENRADIUS) > BOXSIZE) && ((p.y + PENRADIUS) < tft.height()))
    {
      tft.fillCircle(p.y, p.x, PENRADIUS, currentcolor);
    }
  }
}