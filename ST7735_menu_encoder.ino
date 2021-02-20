// ST7735 Menu with Encoder
// (C)2021 Pawel A. Hernik
// YouTube video: https://youtu.be/mVNaiqcpl4w
// Requires:
// RREFont: https://github.com/cbm80amiga/RREFont
// Arduino_ST7735: https://github.com/cbm80amiga/Arduino_ST7735_Fast

/*
 ST7735 128x160 1.8" LCD pinout (header at the top, from left):
 #1 LED   -> 3.3V
 #2 SCK   -> SCL/D13/PA5
 #3 SDA   -> MOSI/D11/PA7
 #4 A0/DC -> D8/PA1  or any digital
 #5 RESET -> D9/PA0  or any digital
 #6 CS    -> D10/PA2 or any digital
 #7 GND   -> GND
 #8 VCC   -> 3.3V
*/

#define SCR_WD   128
#define SCR_HT   160
#include <SPI.h>
#include <Adafruit_GFX.h>

#if (__STM32F1__) // bluepill
#define TFT_CS  PA2
#define TFT_DC  PA1
#define TFT_RST PA0
// use 3 debouncing capacitors (100nF seems to be enough)
#define encoderPinA     PB4
#define encoderPinB     PB5
#define encoderButton   PB6
//#include <Arduino_ST7735_STM.h>
#else
#define TFT_CS 10
#define TFT_DC  8
#define TFT_RST 9
// use 3 debouncing capacitors (100nF seems to be enough)
#define encoderPinA    2
#define encoderPinB    4
#define encoderButton  3
#include <Arduino_ST7735_Fast.h>
#endif

Arduino_ST7735 lcd = Arduino_ST7735(TFT_DC, TFT_RST, TFT_CS);

#include "RREFont.h"
//#include "rre_kx16x26h.h"
//#include "rre_kx9x14h.h"
#include "rre_fjg_8x16.h"
//#include "rre_vga_8x16.h"
//#include "rre_4x7.h"
#include "rre_5x8.h"

RREFont font;
// needed for RREFont library initialization, define your fillRect
void customRect(int x, int y, int w, int h, int c) { return lcd.fillRect(x, y, w, h, c); }

#include <EEPROM.h>

// -------------------------
volatile int encoderPos=0, encoderPosOld=0, encoderStep=2;

void initEncoder()
{
  encoderPos=0;
  pinMode(encoderPinA,   INPUT_PULLUP); 
  pinMode(encoderPinB,   INPUT_PULLUP); 
  pinMode(encoderButton, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoderInt, CHANGE);  // encoder pin on interrupt 0 = pin 2
  attachInterrupt(digitalPinToInterrupt(encoderButton), buttonInt, CHANGE);  // encoder pin on interrupt 1 = pin 3
}

void buttonInt() {}

void readEncoderInt()
{
  //(digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? encoderPos++ : encoderPos--;
  uint8_t pd = PIND & B10100; // inputs #2 and #4 direct reading
  ((pd == B10100) || (pd == B00000)) ? encoderPos++ : encoderPos--;
}

int readButton()
{
  const long btDebounce = 30;
  static long btTime = 0;
  static int lastState = HIGH;
  int val = 0, state = digitalRead(encoderButton);
  if (state == LOW && lastState == HIGH) { btTime = millis(); val=0; }
  if (state == HIGH && lastState == LOW && millis()-btTime >= btDebounce) { val=1; }
  lastState = state;
  return val;
}

// -------------------------
long readVcc() 
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

float readIntTemp() 
{
  long result;
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  delay(5); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = (result - 125) * 1075;
  return result/10000.0;
}

// -------------------------
#define MAXSIN 255
const uint8_t sinTab[91] PROGMEM = {
0,4,8,13,17,22,26,31,35,39,44,48,53,57,61,65,70,74,78,83,87,91,95,99,103,107,111,115,119,123,
127,131,135,138,142,146,149,153,156,160,163,167,170,173,177,180,183,186,189,192,195,198,200,203,206,208,211,213,216,218,
220,223,225,227,229,231,232,234,236,238,239,241,242,243,245,246,247,248,249,250,251,251,252,253,253,254,254,254,254,254,
255
};

int fastSin(int i)
{
  while(i<0) i+=360;
  while(i>=360) i-=360;
  if(i<90)  return(pgm_read_byte(&sinTab[i])); else
  if(i<180) return(pgm_read_byte(&sinTab[180-i])); else
  if(i<270) return(-pgm_read_byte(&sinTab[i-180])); else
            return(-pgm_read_byte(&sinTab[360-i]));
}

int fastCos(int i)
{
  return fastSin(i+90);
}

char buf[100];

void drawGauge1(int level)
{
  int sx,sy,xs0,ys0,xe0,ye0,xs1,ys1,xe1,ye1;
  int cx=SCR_WD/2;
  int cy=SCR_HT/2;
  int rx0=40, ry0=40;
  int rx1=63, ry1=63;
  int mina=-60;
  int maxa=180+60;
  for(int i=mina; i<maxa; i+=15) {  // 22 pieces
    sx = fastCos(i-180);
    sy = fastSin(i-180);
    xs0 = cx+sx*rx0/MAXSIN;
    ys0 = cy+sy*ry0/MAXSIN;
    xe0 = cx+sx*rx1/MAXSIN;
    ye0 = cy+sy*ry1/MAXSIN;
    sx = fastCos(i-180+10);
    sy = fastSin(i-180+10);
    xs1 = cx+sx*rx0/MAXSIN;
    ys1 = cy+sy*ry0/MAXSIN;
    xe1 = cx+sx*rx1/MAXSIN;
    ye1 = cy+sy*ry1/MAXSIN;
    int l=20*(i-mina)/(maxa-mina);
    uint16_t c=(l<level)?lcd.rgbWheel(512L*l/20):RGBto565(60,60,60);
    lcd.fillTriangle(xs0,ys0,xe0,ye0,xe1,ye1, c);
    lcd.fillTriangle(xs1,ys1,xe1,ye1,xs0,ys0, c);
  }
  snprintf(buf,10,"%02d",level);
  font.setColor(WHITE,BLACK);
  font.printStr(ALIGN_CENTER,SCR_HT/2-7,buf);
}

// -------------------------

char *menuTxt[] = {
  "Set value",     // 0
  "Help",          // 1
  "MCU Temp",      // 2
  "VCC/Battery",   // 3
  "EEPROM dump",   // 4
  "Graph",         // 5
  "Bckgrnd color", // 6
  "Item color",    // 7
  "Frame color",   // 8
  "Slider color",  // 9
  "About",         // 10
  "Reboot"         // 11
};

const int itemHt = 20;
const int numMenus = sizeof(menuTxt) / sizeof(char*);
const int numScrLines = SCR_HT/itemHt;  // 160/20=8
int menuSel=0, menuSelOld=0;
int menuStart = 0;
int menuMode = -1; // -1 -> menu of options, 0..n -> option
int storedPos = 0;

uint16_t bgCol     = RGBto565(30,30,140);
uint16_t frameCol  = RGBto565(255,255,40);
uint16_t itemCol   = RGBto565(220,220,220);
uint16_t sliderCol = RGBto565(20,180,180);

void showHelp()
{
  lcd.fillScreen(RGBto565(150,0,0));
  font.setColor(WHITE);
  font.printStr(ALIGN_CENTER, 4, "Help");
  font.setColor(YELLOW);
  font.setCR(1);
  font.setCharMinWd(0);
  font.printStr(5, 24, "Use encoder to select menu item.\nPress button to exit.");
  font.setCR(0);
}

void showSelected(char *txt)
{
  lcd.fillScreen(RGBto565(150,0,150));
  font.setColor(WHITE);  font.printStr(10, 10, "Selected:");
  font.setColor(YELLOW);  font.printStr(10, 30, txt);
}

void printMenuItem(int y, char *item)
{
  font.setColor(itemCol);
  font.printStr(3,2+y*itemHt,item);
}

void printMenu(int full=0)
{
  int n = numMenus<numScrLines ? numMenus : numScrLines;
  for (int i = 0; i < n; i++) {
    formatMenu(menuTxt[i + menuStart], buf, 14);
    full ? lcd.fillRect(0,i*itemHt,SCR_WD,itemHt,bgCol) : lcd.fillRect(3,2+i*itemHt,120-4,16,bgCol);
    printMenuItem(i,buf);
  }
}

void setMenu(int m)
{
  menuMode = m;
  storedPos = encoderPos;
  encoderPos = 0;
}

void endMenu(int butt)
{
  if(!butt) return;
  menuMode = -1;
  initMenu();
  encoderPos = storedPos;
}

void formatMenu(char *in, char *out, int num)
{
  int j = strlen(in);
  strncpy(out, in, j);
  for (; j < num; j++) out[j] = ' ';
  out[j] = 0;
}

void drawMenuSlider()
{
  //int ht = 10;
  int ht = (SCR_HT-4)/numMenus;
  int n = (SCR_HT-4-ht) * menuSel / (numMenus - 1);
  lcd.drawRect(SCR_WD-6,0,6,SCR_HT,sliderCol);
  lcd.fillRect(SCR_WD-6+2,2,2,SCR_HT-4,bgCol);
  lcd.fillRect(SCR_WD-6+2,n+2,2,ht,sliderCol);
}

void drawFrame(int sel, int stat)
{
  lcd.drawRect(0,(sel-menuStart)*itemHt,120,itemHt-1,stat?frameCol:bgCol);
}

void initMenu()
{
  font.setFont(&rre_fjg_8x16);
  font.setCharMinWd(8);
  font.setSpacing(1);
  font.setColor(WHITE);
  printMenu(1);
  drawMenuSlider();
  drawFrame(menuSel,1);
}

void setValue()
{
  if(encoderPos < 0) encoderPos = 0;
  if(encoderPos > 40) encoderPos = 40;
  drawGauge1(encoderPos/encoderStep);
}

// -------------
int colR,colG,colB;
int colRold,colGold,colBold;
int setRGBMode = 0;
int colBarWd = 96;
int colBarY0 = 5;
int colBarY = 40;
int colBarHt = 30;
int encoderMin = 0;
int encoderMax = 255;

void setColorInit(uint16_t *c)
{
  colR = (*c&0xf800)>>8;
  colG = (*c&0x7e0)>>3;
  colB = (*c&0x1f)<<3;
  colRold = colGold = colBold = 0;
  lcd.fillScreen(BLACK);
  for(int i=0;i<32;i++) { // 96 pixels wide, 32 shades
    lcd.fillRect(2+i*3,colBarY0+colBarY*0,3,colBarHt,RGBto565(i*8,0,0));
    lcd.fillRect(2+i*3,colBarY0+colBarY*1,3,colBarHt,RGBto565(0,i*8,0));
    lcd.fillRect(2+i*3,colBarY0+colBarY*2,3,colBarHt,RGBto565(0,0,i*8));
  }
  lcd.drawRect(1,colBarY0+colBarY*0-1,colBarWd+2,colBarHt+2,RGBto565(128,0,0));
  lcd.drawRect(1,colBarY0+colBarY*1-1,colBarWd+2,colBarHt+2,RGBto565(0,128,0));
  lcd.drawRect(1,colBarY0+colBarY*2-1,colBarWd+2,colBarHt+2,RGBto565(0,0,128));
  lcd.fillRect(128-50-2,160-25-2,50,25,*c);  // cancel
  encoderMax = 4;
}

void setColorAction(uint16_t *col, int butt)
{
  if(encoderPos < encoderMin*encoderStep) encoderPos = encoderMin*encoderStep;
  if(encoderPos > encoderMax*encoderStep) encoderPos = encoderMax*encoderStep;
  int pos = encoderPos/encoderStep;
  int frw = 2+96+2;
  if(butt) {
    if(setRGBMode>0) {
      encoderStep = 2;
      encoderPos = (setRGBMode-1)*encoderStep;
      setRGBMode = 0;
      encoderMax = 4;
    } else {
      if(pos==3 || pos==4) {
        if(pos==3) *col = RGBto565(colR,colG,colB);
        menuMode = -1;
        initMenu();
        encoderPos = storedPos;
        setRGBMode = 0;
        encoderStep = 2;
        return;
      }
      setRGBMode = pos+1;
      encoderMax = 255;
      encoderStep = 1;
      if(setRGBMode==1) { encoderPos = colR; lcd.drawRect(0,colBarY0+colBarY*0-2,frw,colBarHt+4,RGBto565(255,80,80)); } else
      if(setRGBMode==2) { encoderPos = colG; lcd.drawRect(0,colBarY0+colBarY*1-2,frw,colBarHt+4,RGBto565(80,255,80)); } else
      if(setRGBMode==3) { encoderPos = colB; lcd.drawRect(0,colBarY0+colBarY*2-2,frw,colBarHt+4,RGBto565(80,80,255)); }
      return;
    }
  }
  if(setRGBMode==1) colR = pos; else
  if(setRGBMode==2) colG = pos; else
  if(setRGBMode==3) colB = pos;

  font.setColor(WHITE,BLACK);
  int xcol=128-9*3+1;
  if(setRGBMode==0 || setRGBMode==1) { dtostrf(colR,3,0,buf); font.printStr(xcol,colBarY0+colBarY*0+8,buf); }
  if(setRGBMode==0 || setRGBMode==2) { dtostrf(colG,3,0,buf); font.printStr(xcol,colBarY0+colBarY*1+8,buf); }
  if(setRGBMode==0 || setRGBMode==3) { dtostrf(colB,3,0,buf); font.printStr(xcol,colBarY0+colBarY*2+8,buf); }
  font.setColor(WHITE);

  if(colRold!=colR) { 
    lcd.drawFastVLine(2+3*colRold/8,colBarY0+colBarY*0,colBarHt,RGBto565(colRold,0,0)); 
    lcd.drawFastVLine(2+3*colR/8,   colBarY0+colBarY*0,colBarHt,YELLOW); 
    colRold=colR; 
   }
  if(colGold!=colG) { 
    lcd.drawFastVLine(2+3*colGold/8,colBarY0+colBarY*1,colBarHt,RGBto565(0,colGold,0)); 
    lcd.drawFastVLine(2+3*colG/8,   colBarY0+colBarY*1,colBarHt,YELLOW); 
    colGold=colG; 
   }
  if(colBold!=colB) { 
    lcd.drawFastVLine(2+3*colBold/8,colBarY0+colBarY*2,colBarHt,RGBto565(0,0,colBold)); 
    lcd.drawFastVLine(2+3*colB/8,   colBarY0+colBarY*2,colBarHt,YELLOW); 
    colBold=colB; 
   }
  
  lcd.fillRect(       2,160-25-2,50,25,RGBto565(colR,colG,colB));  // ok

  if(setRGBMode==0) {
    lcd.drawRect(0,colBarY0+colBarY*0-2,frw,colBarHt+4,BLACK);
    lcd.drawRect(0,colBarY0+colBarY*1-2,frw,colBarHt+4,BLACK);
    lcd.drawRect(0,colBarY0+colBarY*2-2,frw,colBarHt+4,BLACK);
    lcd.drawRect(       0,160-25-4,50+4,25+4,BLACK);
    lcd.drawRect(128-50-4,160-25-4,50+4,25+4,BLACK);
    switch(pos) {
      case 0: lcd.drawRect(0,colBarY0+colBarY*0-2,frw,colBarHt+4,WHITE); break;
      case 1: lcd.drawRect(0,colBarY0+colBarY*1-2,frw,colBarHt+4,WHITE); break;
      case 2: lcd.drawRect(0,colBarY0+colBarY*2-2,frw,colBarHt+4,WHITE); break;
      case 3: lcd.drawRect(       0,160-25-4,50+4,25+4,WHITE); break;
      case 4: lcd.drawRect(128-50-4,160-25-4,50+4,25+4,WHITE); break;
    }
  }
}

// -------------

uint16_t reqBgCol = RGBto565(0,80,0);
int reqY = 110;

void reqInit()
{
  lcd.fillScreen(reqBgCol);
  font.setColor(WHITE);
  font.printStr(ALIGN_CENTER,50,"Are you sure?");
  font.setColor(YELLOW);
  font.printStr(8,reqY," OK ");
  font.printStr(ALIGN_RIGHT,reqY," CANCEL ");
}

void reqAction()
{
  if(encoderPos < 0) encoderPos = 0;
  if(encoderPos > 1*encoderStep) encoderPos = 1*encoderStep;
  int pos = encoderPos/encoderStep;
  lcd.drawRect(8,reqY-3,4*9-2,20,reqBgCol);
  lcd.drawRect(128-8*9,reqY-3,8*9-4,20,reqBgCol);
  if(pos==0) lcd.drawRect(8,reqY-3,4*9-2,20,WHITE); else
  if(pos==1) lcd.drawRect(128-8*9,reqY-3,8*9-4,20,WHITE);
}

// -------------

void showBattery()
{
  char flt[10];
  long v=readVcc();
  lcd.fillScreen(BLACK);
  dtostrf(v/1000.0,1,3,flt);
  snprintf(buf,90,"Vcc=%sV",flt);
  font.setColor(WHITE);
  font.printStr(16,30,buf);
  lcd.drawRect(10,60,128-20-8,50,WHITE);
  lcd.fillRect(10+128-20-8,60+15,8,20,WHITE);
  int bwd = 128-20-8-8l;
  uint16_t c = YELLOW;
  if(v>3600) c = GREEN;
  if(v<3100) c = RED;
  long fill = constrain(map(v,2900,4200,0,bwd-10),0,bwd-10);
  lcd.fillRect(14,64,fill+10,50-8,c);
}

void showIntTemp()
{
  char flt[10];
  lcd.fillScreen(BLACK);
  dtostrf(readIntTemp(),2,1,flt);
  snprintf(buf,90,"Temp=%s'C",flt);
  font.setColor(WHITE);
  font.printStr(ALIGN_CENTER,70,buf);
}

void dumpEEPROM()
{
  //font.setFont(&rre_4x7); font.setCharMinWd(4);
  font.setFont(&rre_5x8); font.setCharMinWd(5);
  if(encoderPos>=(128-16)*2) encoderPos=(128-16)*2;
  int st = encoderPos/encoderStep;
  for(int j=0;j<16;j++) {
    int ii = st*8+j*8;
    ii&=0x3ff; // max 1kB
    snprintf(buf,8,"%03X",ii);
    font.setColor(YELLOW,BLACK);
    font.printStr(0, j*10, buf);
    for(int i=0;i<8;i++) {
      int v = EEPROM.read(ii+i);
      snprintf(buf,8,"%02X",v);
      font.setColor(WHITE,BLACK);
      font.printStr(5*4+2+i*13, j*10, buf);
    }
  }
}

// -------------

void menuItemInit()
{
  setMenu(menuSel);
  switch(menuMode) {
     case 0: lcd.fillScreen(BLACK); encoderPos=5*encoderStep; break; // for setValue()
     case 1: showHelp(); break;
     case 2: showIntTemp(); break;
     case 3: showBattery(); break;
     case 4: lcd.fillScreen(BLACK); break; // for dumpEEPROM()
     case 6: setColorInit(&bgCol); break;
     case 7: setColorInit(&itemCol); break;
     case 8: setColorInit(&frameCol); break;
     case 9: setColorInit(&sliderCol); break;
     case 11: reqInit(); break;
     default: showSelected(menuTxt[menuSel]);
  }
}

void menuItemAction(int butt)
{
  switch(menuMode) {
     case 0: setValue(); endMenu(butt); break;
     case 4: dumpEEPROM(); endMenu(butt); break;
     case 6: setColorAction(&bgCol,butt); break;
     case 7: setColorAction(&itemCol,butt); break;
     case 8: setColorAction(&frameCol,butt); break;
     case 9: setColorAction(&sliderCol,butt); break;
     case 11: reqAction(); endMenu(butt); break;
     default: endMenu(butt);
  }
}

void handleMenu()
{
  int butt = readButton();
  if(encoderPos < 0) encoderPos = 0;
  if(encoderPosOld == encoderPos && !butt) return;
  encoderPosOld = encoderPos;
  if(menuMode == -1) {
    menuSel = encoderPos / encoderStep;
    if(menuSel >= numMenus) {
      menuSel = numMenus - 1;
      encoderPos = menuSel * encoderStep;
    }
    if(menuSel >= menuStart + numScrLines) {
      menuStart = menuSel - numScrLines + 1;
      printMenu();
    }
    if(menuSel < menuStart) {
      menuStart = menuSel;
      printMenu();
    }
    if(menuSelOld != menuSel) {
      drawFrame(menuSelOld,0);
      drawFrame(menuSel,1);
      drawMenuSlider();
      menuSelOld = menuSel;
    }
    if(butt) menuItemInit();
  } else menuItemAction(butt);
}

void setup() 
{
  Serial.begin(9600);
  lcd.init();
  //lcd.fillScreen(bgCol);
  font.init(customRect, SCR_WD, SCR_HT); // custom fillRect function and screen width and height values
  initEncoder();
  initMenu();
}

void loop()
{
  /*
  if(encoderPosOld!=encoderPos) {
    encoderPosOld=encoderPos;
    snprintf(buf,20,"Encoder %02d",encoderPos);
    font.setColor(WHITE);
    lcd.fillRect(3,140,120,16,BLACK);
    font.printStr(3,140,buf);
  }*/

  handleMenu();
}

