#include <IRremote.h>         //inclui biblioteca IRremote
#include <Thermistor.h>       //inclui biblioteca sensor de Temperatura
#include "U8glib.h"           //inclui biblioteca Display OLED
#include <CapacitiveSensor.h> //inclui biblioteca Touch

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI 

Thermistor temp(0);

IRsend irsend;
IRrecv irrecv(11);
decode_results results;


#define rele   12       //Pino do RELE
byte ledTouchOFF(7);    //Pino do Led Touch ON
byte ledTouchON(8);     //Pino do Led Touch OFF
bool sLedTouch = true;  //vVarialvel LED
int  irValue = 0;       //Esse cara nos que criamos para limpar o valor do IR
byte tempAr = 18;       //Temperatura Minima do Ar
byte modAr = 1;
byte vFan = 1;
byte temperature = 0; 
bool sLampOnled = true;//Estado da Lampada no Display
bool sAr = false;       //Estado do Ar
bool sSwing = false;    //Estado do Swing
bool sJetCool = false;  //Estado do JetColl
bool estadoRele = false;
bool estadoBotao = false; 
bool estadoControle = false; 


CapacitiveSensor capSensor = CapacitiveSensor(4, 2);
int threshold = 3000;
int press = 5000;

static bool estadoBotaoAnt; 
bool bTouch();
void ledTouch();
bool contLampadaC();

void controleAr();
void temperaturaAmb();
void temperaturaAr();
void dispOledArOFF();
void dispOledArON();

//Controle do AR
void ligarAr();
void deslAr();
void tempArMais();
void tempArMenos();
void vFan1();
void vFan2();
void vFan3();
void vFan4();
void swingOnOff();
void jetCoolOn();
void jetCoolOff();
void modoAr();
void modoUmidade();
void energySavingOn();
void energySavingOff();

//const unsigned int bLigarAr[59] PROGMEM = {8392, 4228, 552, 1696, 528, 568, 552, 572, 524, 600, 528, 1692, 528, 592, 532, 568, 556, 568, 524, 596, 532, 568, 552, 572, 528, 592, 532, 568, 552, 572, 524, 596, 532, 568, 552, 572, 524, 596, 528, 1696, 524, 1696, 552, 572, 524, 1696, 552, 572, 528, 1692, 556, 1688, 528, 572, 556, 568, 524, 596, 528};
//const unsigned int bDeslAr[59] PROGMEM = {8416, 4228, 528, 1692, 552, 572, 528, 596, 524, 572, 556, 1692, 528, 568, 580, 544, 528, 596, 528, 1692, 528, 1692, 552, 572, 556, 568, 552, 544, 580, 544, 528, 596, 556, 540, 552, 572, 552, 572, 528, 568, 552, 572, 556, 568, 524, 1696, 524, 600, 552, 1668, 528, 596, 552, 544, 552, 572, 552, 1668, 552};
const unsigned int bTempArMais[59] PROGMEM = {8412, 4232, 580, 1664, 552, 544, 580, 544, 556, 568, 556, 1664, 556, 568, 552, 544, 580, 544, 556, 568, 556, 540, 584, 540, 552, 572, 556, 1664, 552, 572, 556, 540, 580, 544, 556, 1664, 584, 1664, 552, 544, 584, 1664, 556, 540, 576, 1672, 552, 544, 580, 1668, 552, 1668, 556, 568, 548, 1672, 552, 572, 552};
const unsigned int bTempArMenos[59] PROGMEM = {8392, 4224, 556, 1668, 552, 568, 556, 544, 580, 540, 556, 1668, 576, 544, 552, 572, 556, 544, 580, 540, 556, 568, 556, 544, 580, 540, 556, 1664, 580, 544, 552, 572, 552, 548, 580, 1664, 552, 1668, 552, 572, 556, 540, 584, 540, 556, 1664, 584, 540, 556, 1664, 580, 1668, 552, 544, 580, 544, 556, 1664, 580};
const unsigned int bVFan1[59] PROGMEM = {8416, 4204, 552, 1668, 552, 572, 552, 544, 576, 548, 524, 1696, 604, 520, 552, 572, 552, 548, 572, 548, 552, 572, 552, 544, 584, 540, 552, 1668, 576, 548, 552, 572, 552, 548, 572, 548, 552, 572, 552, 1668, 552, 1668, 580, 544, 552, 572, 552, 544, 576, 548, 576, 1648, 552, 568, 552, 1672, 576, 1668, 552};
const unsigned int bVFan2[59] PROGMEM = {8416, 4204, 552, 1668, 552, 572, 552, 544, 576, 548, 524, 1696, 604, 520, 552, 572, 552, 548, 572, 548, 552, 572, 552, 544, 584, 540, 552, 1668, 576, 548, 552, 572, 552, 548, 572, 548, 552, 572, 552, 1668, 552, 1668, 580, 544, 552, 572, 552, 544, 576, 548, 576, 1648, 552, 568, 552, 1672, 576, 1668, 552};
const unsigned int bVFan3[59] PROGMEM = {8416, 4204, 552, 1668, 552, 572, 552, 544, 576, 548, 524, 1696, 604, 520, 552, 572, 552, 548, 572, 548, 552, 572, 552, 544, 584, 540, 552, 1668, 576, 548, 552, 572, 552, 548, 572, 548, 552, 572, 552, 1668, 552, 1668, 580, 544, 552, 572, 552, 544, 576, 548, 576, 1648, 552, 568, 552, 1672, 576, 1668, 552};
const unsigned int bVFan4[59] PROGMEM = {8440, 4228, 548, 1676, 572, 548, 552, 572, 552, 548, 576, 1668, 552, 548, 576, 544, 548, 576, 552, 548, 572, 548, 552, 572, 552, 548, 552, 1692, 552, 548, 568, 552, 552, 572, 528, 572, 576, 544, 552, 1644, 604, 1668, 552, 548, 576, 1668, 552, 548, 576, 1668, 552, 548, 576, 548, 548, 572, 552, 548, 576};
const unsigned int bSwingOn[59] PROGMEM = {8340, 4252, 528, 1692, 556, 568, 528, 596, 528, 572, 552, 1692, 532, 564, 552, 572, 532, 592, 528, 568, 552, 572, 524, 600, 528, 1692, 528, 596, 528, 568, 556, 568, 528, 596, 528, 568, 552, 572, 524, 600, 528, 568, 548, 576, 524, 600, 528, 568, 552, 572, 528, 596, 528, 568, 552, 572, 528, 1692, 556};
const unsigned int bSwingOff[59] PROGMEM = {8360, 4260, 520, 1700, 544, 580, 524, 600, 524, 572, 548, 1700, 516, 580, 548, 576, 520, 604, 520, 576, 548, 576, 520, 604, 520, 1700, 520, 604, 512, 584, 548, 576, 520, 600, 524, 576, 544, 580, 520, 604, 520, 576, 544, 580, 520, 600, 524, 576, 544, 580, 516, 604, 520, 580, 548, 576, 516, 1704, 544};
const unsigned int bJetCoolOn[59] PROGMEM = {8408, 4264, 516, 1708, 540, 580, 516, 604, 524, 576, 540, 1704, 524, 576, 540, 580, 524, 600, 520, 580, 544, 576, 520, 604, 520, 1700, 520, 604, 524, 572, 548, 576, 524, 600, 512, 584, 548, 576, 520, 604, 520, 576, 544, 1704, 520, 576, 548, 576, 520, 604, 524, 1696, 516, 608, 524, 572, 548, 1700, 520};
const unsigned int bJetCoolOff[59] PROGMEM = {8392, 4252, 524, 1700, 540, 580, 528, 596, 520, 576, 548, 1700, 528, 568, 556, 568, 528, 596, 528, 568, 556, 568, 524, 600, 524, 572, 556, 1692, 520, 576, 552, 572, 524, 600, 524, 572, 556, 568, 524, 1696, 556, 1692, 528, 568, 552, 1696, 524, 572, 540, 584, 524, 1696, 556, 1692, 528, 1692, 532, 1688, 556};
const unsigned int bModoAr[59] PROGMEM = {8412, 4232, 552, 1668, 576, 548, 552, 572, 552, 544, 580, 1668, 552, 544, 580, 544, 576, 548, 552, 544, 572, 552, 552, 572, 552, 544, 580, 1668, 552, 544, 584, 540, 552, 572, 552, 544, 580, 544, 552, 1668, 580, 1668, 576, 496, 612, 1660, 576, 520, 580, 1668, 552, 544, 576, 548, 580, 544, 552, 548, 576};
const unsigned int bModoUmidade[59] PROGMEM = {8392, 4228, 576, 1672, 544, 552, 576, 548, 548, 576, 576, 1644, 552, 572, 552, 544, 576, 548, 548, 576, 548, 548, 576, 548, 552, 572, 552, 1668, 552, 572, 548, 548, 572, 1676, 548, 1672, 548, 576, 548, 548, 576, 548, 552, 572, 544, 552, 576, 1672, 548, 548, 576, 548, 552, 572, 548, 1672, 548, 1672, 576};
const unsigned int bEnergySavingOn[59] PROGMEM = {8416, 4256, 528, 1692, 528, 596, 528, 568, 552, 572, 528, 1692, 556, 568, 528, 596, 528, 568, 556, 568, 524, 600, 528, 568, 552, 572, 524, 1696, 552, 572, 528, 596, 524, 572, 556, 568, 528, 1692, 556, 568, 520, 600, 532, 568, 548, 1700, 524, 572, 556, 568, 524, 596, 528, 572, 552, 572, 528, 592, 532};
const unsigned int bEnergySavingOff[59] PROGMEM = {8440, 4204, 576, 1668, 556, 568, 556, 544, 580, 540, 556, 1668, 576, 544, 556, 568, 556, 544, 572, 1672, 552, 1668, 556, 568, 556, 544, 576, 544, 556, 568, 556, 540, 576, 548, 556, 568, 552, 544, 580, 544, 556, 568, 556, 1664, 556, 568, 556, 540, 580, 1668, 556, 540, 580, 1668, 556, 540, 580, 1668, 556};

//CLOUD Width: 52, Height: 35
const unsigned char cloud[245] PROGMEM = {
  0x00, 0x00, 0x00, 0xfe, 0x80, 0x00, 0x00, 
  0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x00, 
  0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 
  0x00, 0x00, 0x7e, 0x00, 0xf8, 0x00, 0x00, 
  0x00, 0x00, 0xf8, 0x00, 0x3c, 0x00, 0x00, 
  0x00, 0x00, 0xe0, 0x00, 0x0e, 0x00, 0x00, 
  0x00, 0x01, 0xc0, 0x00, 0x07, 0x80, 0x00, 
  0x00, 0x03, 0x80, 0x00, 0x03, 0x80, 0x00, 
  0x00, 0x07, 0x80, 0x00, 0x03, 0xc0, 0x00, 
  0x00, 0x07, 0x00, 0x00, 0x01, 0xc0, 0x00, 
  0x00, 0x0e, 0x00, 0x00, 0x00, 0xe0, 0x00, 
  0x00, 0x0e, 0x00, 0x00, 0x00, 0xe0, 0x00, 
  0x00, 0x0c, 0x00, 0x00, 0x00, 0xe0, 0x00, 
  0x00, 0x7c, 0x00, 0x00, 0x00, 0x60, 0x00, 
  0x03, 0xfc, 0x00, 0x00, 0x00, 0x70, 0x00, 
  0x0f, 0xfc, 0x00, 0x00, 0x00, 0x78, 0x00, 
  0x1f, 0x80, 0x00, 0x00, 0x00, 0x7e, 0x00, 
  0x1c, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 
  0x38, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 
  0x70, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 
  0xf0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 
  0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 
  0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 
  0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
  0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 
  0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 
  0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 
  0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 
  0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 
  0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 
  0x1c, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 
  0x0e, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 
  0x07, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 
  0x00, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 
};
//SUM Width: 52, Height: 35
const unsigned char sum[245] PROGMEM = {
  0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x00, 0xe0, 0x04, 0x00, 0x00, 
  0x00, 0x0e, 0x00, 0x40, 0x0c, 0x00, 0x00, 
  0x00, 0x07, 0x00, 0x00, 0x1c, 0x00, 0x00, 
  0x00, 0x03, 0x80, 0x20, 0x38, 0x00, 0x00, 
  0x00, 0x01, 0x8f, 0xfe, 0x38, 0x00, 0x00, 
  0x00, 0x00, 0x1f, 0xfe, 0x10, 0x00, 0x00, 
  0x00, 0x00, 0x3c, 0x0f, 0x80, 0x00, 0x00, 
  0x00, 0x00, 0x70, 0x03, 0xc0, 0x00, 0x00, 
  0x00, 0x00, 0xe0, 0x01, 0xe0, 0x00, 0x00, 
  0x00, 0x00, 0xe0, 0x00, 0xe0, 0x00, 0x00, 
  0x00, 0x00, 0xc0, 0x00, 0xe0, 0x00, 0x00, 
  0x00, 0x00, 0xc0, 0x00, 0x60, 0x00, 0x00, 
  0x00, 0xe8, 0xc0, 0x00, 0x62, 0xe0, 0x00, 
  0x00, 0xfc, 0xc0, 0x00, 0x77, 0xe0, 0x00, 
  0x00, 0xb9, 0xc0, 0x00, 0x61, 0xa0, 0x00, 
  0x00, 0x00, 0xc0, 0x00, 0x70, 0x00, 0x00, 
  0x00, 0x00, 0xe0, 0x00, 0xe0, 0x00, 0x00, 
  0x00, 0x00, 0xe0, 0x00, 0xe0, 0x00, 0x00, 
  0x00, 0x00, 0xf0, 0x01, 0xc0, 0x00, 0x00, 
  0x00, 0x00, 0x38, 0x03, 0x80, 0x00, 0x00, 
  0x00, 0x00, 0x3e, 0x0f, 0x80, 0x00, 0x00, 
  0x00, 0x00, 0x1f, 0xfe, 0x10, 0x00, 0x00, 
  0x00, 0x01, 0x8f, 0xfc, 0x30, 0x00, 0x00, 
  0x00, 0x03, 0xc0, 0x50, 0x78, 0x00, 0x00, 
  0x00, 0x07, 0x80, 0x00, 0x1c, 0x00, 0x00, 
  0x00, 0x0e, 0x00, 0x40, 0x0c, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 
};
//AR Width: 18, Height: 16
const unsigned char ar[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x3f, 0xfe, 0x00, 
  0x60, 0x03, 0x00, 
  0x40, 0x01, 0x00, 
  0x40, 0x01, 0x00, 
  0x43, 0xe1, 0x00, 
  0x40, 0x01, 0x00, 
  0x3f, 0xfe, 0x00, 
  0x00, 0x00, 0x00, 
  0x15, 0x54, 0x00, 
  0x15, 0x54, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
};

//mAr Width: 18, Height: 16
const unsigned char mAr[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0xc0, 0x00, 
  0x00, 0xe0, 0x00, 
  0x10, 0xe0, 0x00, 
  0x14, 0xca, 0x00, 
  0x0c, 0xce, 0x00, 
  0x07, 0xd4, 0x00, 
  0x03, 0xe0, 0x00, 
  0x0f, 0xec, 0x00, 
  0x1c, 0xdc, 0x00, 
  0x1c, 0xca, 0x00, 
  0x10, 0xc0, 0x00, 
  0x01, 0xe0, 0x00, 
  0x00, 0xc0, 0x00, 
  0x00, 0xc0, 0x00, 
  0x00, 0x00, 0x00, 
};
//mUmidade Width: 18, Height: 16
const unsigned char mUmidade[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0x80, 0x00, 
  0x01, 0xc0, 0x00, 
  0x03, 0x60, 0x00, 
  0x02, 0x20, 0x00, 
  0x04, 0x10, 0x00, 
  0x0c, 0x18, 0x00, 
  0x08, 0x08, 0x00, 
  0x08, 0x08, 0x00, 
  0x08, 0x08, 0x00, 
  0x08, 0x08, 0x00, 
  0x08, 0x08, 0x00, 
  0x06, 0x30, 0x00, 
  0x03, 0xe0, 0x00, 
  0x01, 0xc0, 0x00, 
  0x00, 0x00, 0x00, 
};
//mVentilador Width: 18, Height: 16
const unsigned char mVentilador[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x03, 0xe0, 0x00, 
  0x04, 0x30, 0x00, 
  0x04, 0x10, 0x00, 
  0x02, 0x08, 0x00, 
  0x1f, 0x0f, 0x00, 
  0x20, 0x19, 0x80, 
  0x60, 0xc1, 0x80, 
  0x60, 0xc1, 0x80, 
  0x66, 0x01, 0x00, 
  0x3c, 0x62, 0x00, 
  0x04, 0x3c, 0x00, 
  0x06, 0x10, 0x00, 
  0x02, 0x10, 0x00, 
  0x01, 0xf0, 0x00, 
  0x00, 0x00, 0x00, 
};

//fV2 Width: 18, Height: 16
const unsigned char fV1[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x03, 0x00, 0x00, 
  0x33, 0x00, 0x00, 
  0x33, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
};
//fV3 Width: 18, Height: 16
const unsigned char fV2[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x30, 0x00, 
  0x00, 0x30, 0x00, 
  0x03, 0x30, 0x00, 
  0x33, 0x30, 0x00, 
  0x33, 0x30, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
};
//fV4 Width: 18, Height: 16
const unsigned char fV3[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x03, 0x00, 
  0x00, 0x03, 0x00, 
  0x00, 0x03, 0x00, 
  0x00, 0x03, 0x00, 
  0x00, 0x33, 0x00, 
  0x00, 0x33, 0x00, 
  0x03, 0x33, 0x00, 
  0x33, 0x33, 0x00, 
  0x33, 0x33, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
};
//mBrisaLeve Width: 18, Height: 16
const unsigned char mBrisaLeve[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x01, 0xc0, 0x00, 
  0x03, 0x40, 0x00, 
  0x00, 0xcc, 0x00, 
  0x3f, 0x92, 0x00, 
  0x00, 0x02, 0x00, 
  0x3f, 0xfc, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x3f, 0x80, 0x00, 
  0x00, 0xc0, 0x00, 
  0x03, 0x40, 0x00, 
  0x01, 0xc0, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
};

//jetCool Width: 18, Height: 16
const unsigned char jetCool[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0x38, 0x00, 
  0x06, 0x68, 0x00, 
  0x0f, 0x19, 0x80, 
  0xa6, 0x72, 0x40, 
  0x66, 0x00, 0x40, 
  0x16, 0x7f, 0x80, 
  0x0f, 0x00, 0x00, 
  0x0f, 0xbf, 0x80, 
  0x76, 0x00, 0xc0, 
  0xa6, 0x03, 0x40, 
  0x06, 0x01, 0xc0, 
  0x0f, 0x00, 0x00, 
  0x06, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
};

//SWING ON Width: 18, Height: 16
const unsigned char swingON[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x02, 0x00, 
  0x01, 0xff, 0x00, 
  0x01, 0x02, 0x00, 
  0x03, 0x80, 0x00, 
  0x04, 0xe0, 0x00, 
  0x1a, 0x3c, 0x00, 
  0x13, 0x0c, 0x00, 
  0x11, 0x00, 0x00, 
  0x11, 0x80, 0x00, 
  0x10, 0xc0, 0x00, 
  0x10, 0xc0, 0x00, 
  0x38, 0x00, 0x00, 
  0x10, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
};
//SWING OFF Width: 18, Height: 16
const unsigned char swingOFF[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x07, 0xfe, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x20, 0x10, 0x00, 
  0x20, 0x10, 0x00, 
  0x20, 0x10, 0x00, 
  0x20, 0x10, 0x00, 
  0x20, 0x20, 0x00, 
  0x20, 0x40, 0x00, 
  0x27, 0x80, 0x00, 
  0x20, 0x00, 0x00, 
  0x20, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
};

// HOME Width: 18, Height: 16
const unsigned char casa[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x00, 0xc4, 0x00, 
  0x01, 0x2e, 0x00, 
  0x02, 0x14, 0x00, 
  0x04, 0x0c, 0x00, 
  0x08, 0x04, 0x00, 
  0x10, 0x22, 0x00, 
  0x66, 0x79, 0x80, 
  0x44, 0x28, 0x80, 
  0x34, 0x0b, 0x00, 
  0x14, 0x0a, 0x00, 
  0x17, 0xfa, 0x00, 
  0x10, 0x02, 0x00, 
  0x10, 0x02, 0x00, 
  0x1f, 0xfe, 0x00, 
  0x00, 0x00, 0x00, 
};
//LAMPADA Width: 18, Height: 16
const unsigned char lampada[48] PROGMEM = {
  0x00, 0x00, 0x00, 
  0x21, 0xe1, 0x00, 
  0x12, 0x12, 0x00, 
  0x04, 0x88, 0x00, 
  0x09, 0x04, 0x00, 
  0x68, 0x05, 0x80, 
  0x08, 0x04, 0x00, 
  0x04, 0x0c, 0x00, 
  0x16, 0x1a, 0x00, 
  0x33, 0x33, 0x00, 
  0x01, 0x20, 0x00, 
  0x01, 0x20, 0x00, 
  0x01, 0x20, 0x00, 
  0x00, 0xc0, 0x00, 
  0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 
};

void setup(){
  Serial.begin(9600);     //Abre comunicacao da porta serial  
  irrecv.enableIRIn();    //inicia a comunicacaodo receptor IR
  pinMode(rele, OUTPUT);
  pinMode(ledTouchON, OUTPUT);
  pinMode(ledTouchOFF, OUTPUT);
}

void loop(){
  /*MONITORA IR*/
  static unsigned long delayMilis = millis();
  if ((millis() - delayMilis) > 5){
    if(irrecv.decode(&results)) {
      Serial.println(results.value, DEC);
      irValue = results.value;
      irrecv.resume();
    }
    if(estadoBotao != bTouch()){
      alternaRele();
    }
    else if(estadoControle != contLampadaC()){
      alternaRele();
    }
  delayMilis = millis();
  }
  controleAr();
  temperaturaAmb();
  temperaturaAr();
  ledTouch();
  irValue = 0;
  // picture loop  
  u8g.firstPage();  
  do {draw();}
  while(u8g.nextPage());
}

//DESENHA A TELA ON/OFF
void draw(void) {
  switch(sAr) {
    case false: dispOledArOFF();break;
    case true: dispOledArON();break;
  }
}
void dispOledArOFF(){
  u8g.drawBitmapP(8,17,7,35,sum);
  //u8g.drawBitmapP(1,17,7,35,cloud);
  u8g.drawBitmapP(108,16,3,16,casa);
  u8g.setFont(u8g_font_fub30n);
  u8g.setPrintPos(55,50);
  u8g.print(temperature);
  u8g.setFont(u8g_font_6x10);
  char s[2] = " ";  
  s[0] = 176;
  u8g.drawStr(95,24,s);
  u8g.setPrintPos(101,24);
  u8g.print("C");
  if(sLampOnled){
    u8g.drawBitmapP(108,34,3,16,lampada);
  }
}
void dispOledArON(){
  u8g.drawBitmapP(1,17,7,35,cloud);
  //TEMP AR
  u8g.setFont(u8g_font_fub25n);
  u8g.setPrintPos(64,28);
  u8g.print(tempAr);
  u8g.setFont(u8g_font_6x10);
  char s[2] = " ";  
  s[0] = 176;
  u8g.drawStr(98,8,s);
  u8g.setPrintPos(102,8);
  u8g.print("C");
  u8g.drawBitmapP(108,-2,3,16,ar);
  //TEMP AMBIENTE
  u8g.setFont(u8g_font_fub25n);
  u8g.setPrintPos(64,64);
  u8g.print(temperature);
  u8g.setFont(u8g_font_6x10);
  u8g.drawStr(98,42,s);
  u8g.setPrintPos(102,42);
  u8g.print("C");
  u8g.drawBitmapP(108,34,3,16,casa);
  if(sLampOnled){u8g.drawBitmapP(108,50,3,16,lampada);}
  
  if(modAr == 1){u8g.drawBitmapP(0,0,3,16,mAr);}
  else if(modAr == 2){u8g.drawBitmapP(10,0,3,16,mUmidade);}
  else if(modAr == 3){u8g.drawBitmapP(20,0,3,16,mVentilador);}
  if(sJetCool){u8g.drawBitmapP(30,0,3,16,jetCool);}
  
  if(vFan == 1){u8g.drawBitmapP(109,14,3,16,fV1);}
  else if(vFan == 2){u8g.drawBitmapP(109,14,3,16,fV2);}
  else if(vFan == 3){u8g.drawBitmapP(109,14,3,16,fV3);}
  else if(vFan == 4){u8g.drawBitmapP(109,14,3,16,mBrisaLeve);}
  if(sSwing){u8g.drawBitmapP(50,0,3,16,swingON);}
  else{u8g.drawBitmapP(50,0,3,16,swingOFF);}
}

//VERIFICA A TEMPERATURA
void temperaturaAmb() {
  static unsigned long delayMilis = millis();
  if ((millis() - delayMilis) > 1000){
    temperature = temp.getTemp();
    delayMilis = millis();
  }
}
//MOSTRA TEMPEREATURA DO AR
void temperaturaAr() {
  static unsigned long delayMilis = millis();
  if ((millis() - delayMilis) > 1000){delayMilis = millis();}
}

//ACIONA RELE
void alternaRele(){
  estadoRele = !estadoRele;
  if ( estadoRele == true) {
    digitalWrite(rele, HIGH);
    estadoBotao = true;
    estadoControle = true;
    estadoBotaoAnt = true;
    sLampOnled = false;
    sLedTouch = false;

  } else {
    digitalWrite(rele, LOW);
    estadoBotao = false;
    estadoControle = false;
    estadoBotaoAnt = false;
    sLampOnled = true;
    sLedTouch = true;
  }
}
//CONTROLA A LAMPADA PELO CONTROLE
bool contLampadaC(){
  if(irValue == 12 || irValue == 2060){
    irValue = 0;
    return !estadoControle;
  }
    return estadoControle;
}
//CONTROLA BOTAO TOUCH
bool bTouch(){
  long sensorValue = capSensor.capacitiveSensor(30);
  //Serial.println(sensorValue);
  if (sensorValue >= threshold){return !estadoBotao;}
  else if ((sensorValue < threshold) || (sensorValue > press) ){return estadoBotao;}
}
void ledTouch(){
  if (sLedTouch)
  {
    digitalWrite(ledTouchOFF, LOW);
    digitalWrite(ledTouchON, HIGH);
  }
  else{
    digitalWrite(ledTouchOFF, HIGH);
    digitalWrite(ledTouchON, LOW);
  }
}
//CONTROLA O AR
void controleAr(){
//  ON
  if(irValue == 31){
    ligarAr();
    sAr = true;
    Serial.println(F("AR LIGADO"));
  }
//  OFF  
  if(irValue == 2079){  
    deslAr();
    sAr = false;
    Serial.println(F("AR DESLIGADO"));
  }
//  UP
  if((irValue == 43)||(irValue == 2091)){  
    if (tempAr < 30){
      tempArMais();
      tempAr = tempAr +1;
    }
  }
//  DOWN
  if((irValue == 44)||(irValue == 2092)){
    if (tempAr > 18){
      tempArMenos();
      tempAr = tempAr -1;
    }
  }
//  VELOCIDADE DA FAN   - BAIXA -MEDIA-ALTA-PQN BRISA
  if((irValue == 11)||(irValue == 2059)){
    switch (vFan) {
        case 1:
            vFan1();
            Serial.println(vFan);
          break;
        case 2:
            vFan2();
            Serial.println(vFan);
          break;
        case 3:
            vFan3();
            Serial.println(vFan);
          break;
        case 4:
            vFan4();
            Serial.println(vFan);
            vFan=0;
          break;
        default: 
          Serial.println(F("ERRO!!!"));
        break;
    }
    vFan++;
  }
//  MODO   - DESUMIDIFICADOR - AR - VENTILADO
  if((irValue == 56)||(irValue == 2104)){
    switch (modAr) {
        case 1:
            modoAr();
          break;
        case 2:
            modoUmidade();
          break;
        case 3:
            modAr =0;
          break;
        default: 
          Serial.println(F("ERRO!!!"));
        break;
    }
    modAr++;
  }
  //  MODO   - JETCOOL - ON - OFF
  if(irValue == 30){
    jetCoolOn();
    modAr = false;
    vFan = 3;
    sJetCool = true;
    Serial.println(F("JETCOLL ON/OFF"));
  }
  else if(irValue == 2078){jetCoolOff();modAr = true; vFan = 1; sJetCool = false;}
  //  MODO   - SWING - ON - OFF
  if(irValue == 14){
    swingOnOff();
    sSwing = true;
    Serial.println(F("SWING ON/OFF"));
  }
  else if(irValue == 2062){swingOnOff(); sSwing = false;}
}
//CONTROLE DO AR //Padrao sinal RAW Ligar Ar Condicionado LG
void ligarAr(){
  static unsigned long delayMilis = millis();
  const unsigned int bLigarAr [59] PROGMEM =  {8392, 4228, 552, 1696, 528, 568, 552, 572, 524, 600, 528, 1692, 528, 592, 532, 568, 556, 568, 524, 596, 532, 568, 552, 572, 528, 592, 532, 568, 552, 572, 524, 596, 532, 568, 552, 572, 524, 596, 528, 1696, 524, 1696, 552, 572, 524, 1696, 552, 572, 528, 1692, 556, 1688, 528, 572, 556, 568, 524, 596, 528};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bLigarAr, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void deslAr(){
  static unsigned long delayMilis = millis();
  const unsigned int bDeslAr[59] PROGMEM = {8416, 4228, 528, 1692, 552, 572, 528, 596, 524, 572, 556, 1692, 528, 568, 580, 544, 528, 596, 528, 1692, 528, 1692, 552, 572, 556, 568, 552, 544, 580, 544, 528, 596, 556, 540, 552, 572, 552, 572, 528, 568, 552, 572, 556, 568, 524, 1696, 524, 600, 552, 1668, 528, 596, 552, 544, 552, 572, 552, 1668, 552};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bDeslAr, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void tempArMais(){
  static unsigned long delayMilis = millis();
  //const unsigned int bTempArMais[59] PROGMEM = {8412, 4232, 580, 1664, 552, 544, 580, 544, 556, 568, 556, 1664, 556, 568, 552, 544, 580, 544, 556, 568, 556, 540, 584, 540, 552, 572, 556, 1664, 552, 572, 556, 540, 580, 544, 556, 1664, 584, 1664, 552, 544, 584, 1664, 556, 540, 576, 1672, 552, 544, 580, 1668, 552, 1668, 556, 568, 548, 1672, 552, 572, 552};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bTempArMais, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void tempArMenos(){
  static unsigned long delayMilis = millis();
  //const unsigned int bTempArMenos[59] PROGMEM = {8392, 4224, 556, 1668, 552, 568, 556, 544, 580, 540, 556, 1668, 576, 544, 552, 572, 556, 544, 580, 540, 556, 568, 556, 544, 580, 540, 556, 1664, 580, 544, 552, 572, 552, 548, 580, 1664, 552, 1668, 552, 572, 556, 540, 584, 540, 556, 1664, 584, 540, 556, 1664, 580, 1668, 552, 544, 580, 544, 556, 1664, 580};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bTempArMenos, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void vFan1(){
  static unsigned long delayMilis = millis();
  //const unsigned int bVFan1[59] PROGMEM = {8416, 4204, 552, 1668, 552, 572, 552, 544, 576, 548, 524, 1696, 604, 520, 552, 572, 552, 548, 572, 548, 552, 572, 552, 544, 584, 540, 552, 1668, 576, 548, 552, 572, 552, 548, 572, 548, 552, 572, 552, 1668, 552, 1668, 580, 544, 552, 572, 552, 544, 576, 548, 576, 1648, 552, 568, 552, 1672, 576, 1668, 552};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bVFan1, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void vFan2(){
  static unsigned long delayMilis = millis();
  //const unsigned int bVFan2[59] PROGMEM = {8416, 4204, 552, 1668, 552, 572, 552, 544, 576, 548, 524, 1696, 604, 520, 552, 572, 552, 548, 572, 548, 552, 572, 552, 544, 584, 540, 552, 1668, 576, 548, 552, 572, 552, 548, 572, 548, 552, 572, 552, 1668, 552, 1668, 580, 544, 552, 572, 552, 544, 576, 548, 576, 1648, 552, 568, 552, 1672, 576, 1668, 552};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bVFan2, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void vFan3(){
  static unsigned long delayMilis = millis();
  //const unsigned int bVFan3[59] PROGMEM = {8416, 4204, 552, 1668, 552, 572, 552, 544, 576, 548, 524, 1696, 604, 520, 552, 572, 552, 548, 572, 548, 552, 572, 552, 544, 584, 540, 552, 1668, 576, 548, 552, 572, 552, 548, 572, 548, 552, 572, 552, 1668, 552, 1668, 580, 544, 552, 572, 552, 544, 576, 548, 576, 1648, 552, 568, 552, 1672, 576, 1668, 552};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bVFan3, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void vFan4(){
  static unsigned long delayMilis = millis();
  //const unsigned int bVFan4[59] PROGMEM = {8440, 4228, 548, 1676, 572, 548, 552, 572, 552, 548, 576, 1668, 552, 548, 576, 544, 548, 576, 552, 548, 572, 548, 552, 572, 552, 548, 552, 1692, 552, 548, 568, 552, 552, 572, 528, 572, 576, 544, 552, 1644, 604, 1668, 552, 548, 576, 1668, 552, 548, 576, 1668, 552, 548, 576, 548, 548, 572, 552, 548, 576};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bVFan4, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void swingOnOff(){
  static unsigned long delayMilis = millis();
  //const unsigned int bSwingOn[59] PROGMEM = {8340, 4252, 528, 1692, 556, 568, 528, 596, 528, 572, 552, 1692, 532, 564, 552, 572, 532, 592, 528, 568, 552, 572, 524, 600, 528, 1692, 528, 596, 528, 568, 556, 568, 528, 596, 528, 568, 552, 572, 524, 600, 528, 568, 548, 576, 524, 600, 528, 568, 552, 572, 528, 596, 528, 568, 552, 572, 528, 1692, 556};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bSwingOn, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void jetCoolOn(){
  static unsigned long delayMilis = millis();
  //const unsigned int bJetCoolOn[59] PROGMEM = {8408, 4264, 516, 1708, 540, 580, 516, 604, 524, 576, 540, 1704, 524, 576, 540, 580, 524, 600, 520, 580, 544, 576, 520, 604, 520, 1700, 520, 604, 524, 572, 548, 576, 524, 600, 512, 584, 548, 576, 520, 604, 520, 576, 544, 1704, 520, 576, 548, 576, 520, 604, 524, 1696, 516, 608, 524, 572, 548, 1700, 520};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bJetCoolOn, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void jetCoolOff(){
  static unsigned long delayMilis = millis();
  //const unsigned int bJetCoolOff[59] PROGMEM = {8392, 4252, 524, 1700, 540, 580, 528, 596, 520, 576, 548, 1700, 528, 568, 556, 568, 528, 596, 528, 568, 556, 568, 524, 600, 524, 572, 556, 1692, 520, 576, 552, 572, 524, 600, 524, 572, 556, 568, 524, 1696, 556, 1692, 528, 568, 552, 1696, 524, 572, 540, 584, 524, 1696, 556, 1692, 528, 1692, 532, 1688, 556};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bJetCoolOff, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void modoAr(){
  static unsigned long delayMilis = millis();
  //const unsigned int bModoAr[59] PROGMEM = {8412, 4232, 552, 1668, 576, 548, 552, 572, 552, 544, 580, 1668, 552, 544, 580, 544, 576, 548, 552, 544, 572, 552, 552, 572, 552, 544, 580, 1668, 552, 544, 584, 540, 552, 572, 552, 544, 580, 544, 552, 1668, 580, 1668, 576, 496, 612, 1660, 576, 520, 580, 1668, 552, 544, 576, 548, 580, 544, 552, 548, 576};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bModoAr, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void modoUmidade(){
  static unsigned long delayMilis = millis();
  //const unsigned int bModoUmidade[59] PROGMEM = {8392, 4228, 576, 1672, 544, 552, 576, 548, 548, 576, 576, 1644, 552, 572, 552, 544, 576, 548, 548, 576, 548, 548, 576, 548, 552, 572, 552, 1668, 552, 572, 548, 548, 572, 1676, 548, 1672, 548, 576, 548, 548, 576, 548, 552, 572, 544, 552, 576, 1672, 548, 548, 576, 548, 552, 572, 548, 1672, 548, 1672, 576};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bModoUmidade, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void energySavingOn(){
  static unsigned long delayMilis = millis();
  //const unsigned int bEnergySavingOn[59] PROGMEM = {8416, 4256, 528, 1692, 528, 596, 528, 568, 552, 572, 528, 1692, 556, 568, 528, 596, 528, 568, 556, 568, 524, 600, 528, 568, 552, 572, 524, 1696, 552, 572, 528, 596, 524, 572, 556, 568, 528, 1692, 556, 568, 520, 600, 532, 568, 548, 1700, 524, 572, 556, 568, 524, 596, 528, 572, 552, 572, 528, 592, 532};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bEnergySavingOn, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}
void energySavingOff(){
  static unsigned long delayMilis = millis();
  //const unsigned int bEnergySavingOff[59] PROGMEM = {8440, 4204, 576, 1668, 556, 568, 556, 544, 580, 540, 556, 1668, 576, 544, 556, 568, 556, 544, 572, 1672, 552, 1668, 556, 568, 556, 544, 576, 544, 556, 568, 556, 540, 576, 548, 556, 568, 552, 544, 580, 544, 556, 568, 556, 1664, 556, 568, 556, 540, 580, 1668, 556, 540, 580, 1668, 556, 540, 580, 1668, 556};
  if ((millis() - delayMilis) > 500){
    irsend.sendRaw(bEnergySavingOff, 59, 38);
    irrecv.enableIRIn();
    delayMilis = millis();
  }
}