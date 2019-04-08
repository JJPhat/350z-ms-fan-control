// Program: 350z CAN-BUS Shield, receive data, fulfill stock ECM functions


#include <SPI.h>
#include "mcp_can.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>

// You can use any (4 or) 5 pins 
#define sclk 8
#define mosi 7
#define rst  4
#define dc   5
#define cs   6


// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define BURNT_ORANGE    0xB465
#define ROYAL_PURPLE    0x5935
#define ORANGE          0xFCC0
#define LIME            0xB6C4
#define DARK_RED        0x9800
#define DARK_YELLOW     0xC540

Adafruit_SSD1351 tft = Adafruit_SSD1351(cs, dc, rst);
//Adafruit_SSD1351 tft = Adafruit_SSD1351(cs, dc, mosi, sclk, rst);



const int buttonPin = 3; //Inturrupt pin

byte b_index;
byte b_page;
byte flagRecv = 0;
unsigned long currentMillis=0;
char tempchars[11];

int coolant_lo = 95; // temp of coolant to turn fans on low in deg C, stock 350z 95 deg C
int coolant_hi = 100; // temp of coolant to turn fans on high in deg C, stock 350z 100 deg C
int coolant_off = 90; // temp of coolant to turn fans off in deg C

int coolant_lo_f = coolant_lo*1.8+32;
int coolant_hi_f = coolant_hi*1.8+32;
int coolant_off_f = coolant_off*1.8+32;
byte fan_override = 0;
 
byte MAP[2] = {};
byte RPM[2] = {};
byte TPS[2] = {};
byte IAT[2] = {};
byte CLT[2] = {};
byte ADV[2] = {};
byte afrTarget;
byte AFR;
byte COR[2] = {};
byte PW1[2] = {};
byte PW2[2] = {};
byte PWseq1[2] = {};
byte BATVOLT[2] = {};
byte GENSENS1[2] = {};
byte GENSENS2[2] = {};
byte KNK[2] = {};

String FAN;
int Map, Iat, Clt, genSens1, genSens2;
byte Tps, Cor, Pw1, Pw2, PwSeq1, batVolt, knock;
char Adv;
word Rpm;
String gear;

int wheel_sp_FR_hi;
int wheel_sp_FR_lo;
int wheel_sp_FL_hi;
int wheel_sp_FL_lo;
int vehicle_sp_hi;
int vehicle_sp_lo;
int FR_kph;
int FL_kph;
int vehicle_sp_kph;

int wheel_sp_RR_hi;
int wheel_sp_RR_lo;
int wheel_sp_RL_hi;
int wheel_sp_RL_lo;
int RR_kph;
int RL_kph;

unsigned long canId;
byte len = 0;
byte buf[8];
byte stmp[8] = {};  // stors IPDM and fan data
byte stmp1[8] = {}; // stores vehicle speed info and boost control DC

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10

const int SPI_CS_PIN = 9;

MCP_CAN CAN0(SPI_CS_PIN);                                    // Set CS pin


void setup()
{
    Serial.begin(9600);
    if(CAN0.begin(MCP_STD, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.print("MCP2515 Init Okay!!\r\n");
    else Serial.print("MCP2515 Init Failed!!\r\n");
     
    attachInterrupt(digitalPinToInterrupt(2), ISR_mcp2515, FALLING);
    pinMode(buttonPin, INPUT_PULLUP); //pin for push button

    pinMode(14, INPUT_PULLUP);
    pinMode(15, INPUT_PULLUP);
    pinMode(16, INPUT_PULLUP);
    pinMode(17, INPUT_PULLUP);

/*  dec  hex

    1512  5e8 0101 1110 1000
    1513  5e9 0101 1110 1001
    1514  5ea 0101 1110 1010
    1515  5eb 0101 1110 1011

      mask    0101 1110 1011  + 0000 0000 0000 0000 = 0x5EB 0000

    644   284 0010 1000 0100
    645   285 0010 1000 0101

      mask    0010 1000 0101  + 0000 0000 0000 0000 = 0x285 0000     

    CAN0.init_Mask(0,0,0x03FF0000);                // Init first mask...
    CAN0.init_Filt(0,0,0x02840000);                // Init first filter...
    CAN0.init_Filt(1,0,0x02850000);                // Init second filter...
  
    CAN0.init_Mask(1,0,0x7FFF000);                // Init second mask... 
    CAN0.init_Filt(2,0,0x5E80000);                // Init third filter...
    CAN0.init_Filt(3,0,0x5E90000);                // Init fouth filter...
    CAN0.init_Filt(4,0,0x5EA0000);                // Init fifth filter...
    CAN0.init_Filt(5,0,0x5EB0000);                // Init sixth filter...*/
    
    Serial.print("Fan Low Temp deg C = "); Serial.print(coolant_lo); Serial.print("Fan Low Temp deg F = "); Serial.println(coolant_lo_f);
    Serial.print("Fan High Temp deg C = "); Serial.print(coolant_hi); Serial.print("Fan High Temp deg F = "); Serial.println(coolant_hi_f);
    Serial.print("Fan Off Temp deg C = "); Serial.print(coolant_off); Serial.print("Fan Off Temp deg F = "); Serial.println(coolant_off_f);
    
    CAN0.setMode(MCP_NORMAL);  // Change to normal mode to allow messages to be transmitted         
    //CAN0.sendMsgBuf(0x1f9, 0, 8, stmp);

    tft.begin();
    tft.setTextWrap(false);

    tft.fillScreen(BLACK);
    
    //b_index=4;
    //b_page=0;
    
}


void loop()
{

 if(flagRecv)
   {
    flagRecv=0;
    
    if(CAN_MSGAVAIL == CAN0.checkReceive())
    {
        
        CAN0.readMsgBuf(&canId, &len, buf);    // read data,  len: data length, buf: data buffer
        /*if(canId = 0x5e9)
        {
        Serial.println(" ");
        Serial.print("Get Data From id: ");
        Serial.println(canId, HEX);

        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print("0x");
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        }*/

        if(canId == 0x284) //644 dec, data for vss in tunerstudio
        {
          wheel_sp_FR_hi = buf[0];
          wheel_sp_FR_lo = buf[1];
          wheel_sp_FL_hi = buf[2];
          wheel_sp_FL_lo = buf[3];
          vehicle_sp_hi = buf[4];
          vehicle_sp_lo = buf[5];

          FR_kph = (wheel_sp_FR_hi + wheel_sp_FR_lo)/(2);
          FL_kph = (wheel_sp_FL_hi + wheel_sp_FL_lo)/(2);
          vehicle_sp_kph = (vehicle_sp_hi + vehicle_sp_lo)/2;
          stmp1[0] = FR_kph;
          stmp1[1] = FL_kph;
          stmp1[3] = vehicle_sp_kph;
          //break;
        }
        if(canId == 0x285) //645 dec
        {
          wheel_sp_RR_hi = buf[0];
          wheel_sp_RR_lo = buf[1];
          wheel_sp_RL_hi = buf[2];
          wheel_sp_RL_lo = buf[3];

          RR_kph = (wheel_sp_RR_hi + wheel_sp_RR_lo)/(2);
          RL_kph = (wheel_sp_RL_hi + wheel_sp_RL_lo)/(2);
          stmp1[4] = RR_kph;
          stmp1[5] = RL_kph;
        }
        if(canId == 0x5e8)    //1512 data for display
        {  
            MAP[0] = buf[0];
            MAP[1] = buf[1];
            Map = ((MAP[0]*256)+MAP[1])/10;

            RPM[0] = buf[2];
            RPM[1] = buf[3];
            Rpm = ((RPM[0]*256)+RPM[1]);
            
            CLT[0] = buf[4];
            CLT[1] = buf[5];
            Clt = (((CLT[0]*256)+CLT[1])/10);
            Clt = ((Clt-32)/1.8);    //comment out for standard
           
                  
            //Serial.print(Clt);

            TPS[0] = buf[6];
            TPS[1] = buf[7];
            Tps=((TPS[0]*256)+TPS[1])/10;
        }
        if(canId == 0x5e9)
        {
            PW1[0] = buf[0];
            PW1[1] = buf[1];
            Pw1=((PW1[0]*256)+PW1[1])/1000;
            
            PW2[0] = buf[2];
            PW2[1] = buf[3];
            Pw2=((PW2[0]*256)+PW2[1])/1000;
            
            IAT[0] = buf[4];
            IAT[1] = buf[5];
            Iat=(((IAT[0]*256)+IAT[1])/10);
            Iat=(Iat-32)/1.8;    //comment out for standard
            
              
                  
            ADV[0] = buf[6];
            ADV[1] = buf[7];
            Adv=(((ADV[0]*256)+ADV[1])/10);
          }
          if(canId == 0x5ea) //1514 dec
          {
            afrTarget = buf[0];
            AFR = buf[1];
            
            COR[0] = buf[2];
            COR[1] = buf[3];
            Cor=((COR[0]*256)+COR[1]);

            PWseq1[0] = buf[6];
            PWseq1[1] = buf[7];
            PwSeq1=((PWseq1[0]*256)+PWseq1[1])/1000;
          }
          if(canId == 0x5eb)  //1515 dec
          {
            BATVOLT[0] = buf[0];
            BATVOLT[1] = buf[1];
            batVolt=((BATVOLT[0]*256)+BATVOLT[1]);

            GENSENS1[0] = buf[2];
            GENSENS1[1] = buf[3];
            genSens1=((GENSENS1[0]*256)+GENSENS1[1])/10;

            GENSENS2[0] = buf[4];
            GENSENS2[1] = buf[5];
            genSens2=((GENSENS2[0]*256)+GENSENS2[1])/10;

            KNK[0] = buf[6];
            KNK[1] = buf[7];
            knock = ((KNK[0]*256)+KNK[1])/10;
          }
          
       

    }
    else
    {
      Map=0; Tps=0; Iat=0; Adv=0; Cor=0; Clt=0; Rpm=0; Pw1=0; Pw2=0; PwSeq1=0; batVolt=0; genSens1=0; genSens2=0; knock=0; //resent readings to zero when ms is off
    }
   }
    byte b = checkButton();    //poll button pin
    if (b == 1) clickEvent();
    //if (b == 2) doubleClickEvent();
    if (b == 3) holdEvent();
    if (b == 4) longHoldEvent();
    //Serial.println(b);
    
    
    
    if (millis() - currentMillis >= 150) // 5HZ
    {
    fans();
    main_page();
    Gear_Indicator();
    CAN0.sendMsgBuf(0x1f9, 0, 8, stmp);   //ECM to IPDM, needs broadcast all the time
    CAN0.sendMsgBuf(0x5e7, 0, 8, stmp1);  //VSS info for tuner studio
    currentMillis = millis();

    //Serial.println(stmp[0]);
    //Serial.println(stmp1[7]);
    }
    
}

void Gear_Indicator()
{
  int SignalA = digitalRead(14);   //pin A on NSS
  int SignalB = digitalRead(15);   //pin d on NSS
  int SignalC = digitalRead(16);   //pin B on NSS
  int SignalD = digitalRead(17);   //pin c on NSS
  //Serial.print(SignalA);
  //Serial.print(SignalB);
  //Serial.print(SignalC);
  //Serial.println(SignalD);
  if (SignalA == LOW && SignalB == HIGH && SignalC == HIGH && SignalD == LOW)
    gear = 'P';
  if (SignalA == LOW && SignalB == LOW && SignalC == HIGH && SignalD == HIGH)
    gear = 'R';
  if (SignalA == HIGH && SignalB == LOW && SignalC == HIGH && SignalD == LOW)
    gear = 'N';
  if (SignalA == HIGH && SignalB == LOW && SignalC == LOW && SignalD == HIGH)
    gear = '1';
  if (SignalA == LOW && SignalB == LOW && SignalC == LOW && SignalD == LOW)
    gear = '2';
  if (SignalA == LOW && SignalB == HIGH && SignalC == LOW && SignalD == HIGH)
    gear = '3';
  if (SignalA == HIGH && SignalB == HIGH && SignalC == LOW && SignalD == LOW)
    gear = '4';
}

void ISR_mcp2515()
{
  flagRecv = 1;
}

void fans()
{
        if((Clt >= coolant_lo) && (Clt < coolant_hi))
        stmp[0] = 64;
        if(Clt >= coolant_hi)
        stmp[0] = 128;
        if(Clt <= coolant_off)
        stmp[0] = 0;
        if(fan_override == 1)
        stmp[0] = 128;         
}

void clickEvent() {
   b_page ++;
   //tft.fillScreen(BLACK);
}
/*void doubleClickEvent() {
   
}*/
void holdEvent() {
b_page--;
}
void longHoldEvent() {
   fan_override++;
   tft.fillScreen(BLACK);
   tft.setTextSize(3);
   tft.setTextColor(ROYAL_PURPLE, BLACK);
   tft.setCursor(20, 35);
   tft.println("FANS");
   tft.setCursor(0,59 );
   tft.println("CONTROL!");

   delay(1000);
   tft.fillScreen(BLACK);
   b_index --;

   if(fan_override >= 2)
      fan_override = 0;
}

// Button timing variables
int debounce = 20;          // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 100;            // max ms between clicks for a double click event
int holdTime = 1000;        // ms hold period: how long to wait for press+hold event
int longHoldTime = 3000;    // ms long hold period: how long to wait for press+hold event

// Button variables
boolean buttonVal = HIGH;   // value read from button
boolean buttonLast = HIGH;  // buffered value of the button's previous state
boolean DCwaiting = false;  // whether we're waiting for a double click (down)
boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;    // whether it's OK to do a single click
long downTime = -1;         // time the button was pressed down
long upTime = -1;           // time the button was released
boolean ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event
boolean holdEventPast = false;    // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already

int checkButton() {    
   int event = 0;
   buttonVal = digitalRead(buttonPin);
   // Button pressed down
   if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce)
   {
       downTime = millis();
       ignoreUp = false;
       waitForUp = false;
       singleOK = true;
       holdEventPast = false;
       longHoldEventPast = false;
       if ((millis()-upTime) < DCgap && DConUp == false && DCwaiting == true)  DConUp = true;
       else  DConUp = false;
       DCwaiting = false;
   }
   // Button released
   else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce)
   {        
       if (not ignoreUp)
       {
           upTime = millis();
           if (DConUp == false) DCwaiting = true;
           else
           {
               event = 2;
               DConUp = false;
               DCwaiting = false;
               singleOK = false;
           }
       }
   }
   // Test for normal click event: DCgap expired
   if ( buttonVal == HIGH && (millis()-upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
   {
       event = 1;
       DCwaiting = false;
   }
   // Test for hold
   if (buttonVal == LOW && (millis() - downTime) >= holdTime) {
       // Trigger "normal" hold
       if (not holdEventPast)
       {
           event = 3;
           waitForUp = true;
           ignoreUp = true;
           DConUp = false;
           DCwaiting = false;
           //downTime = millis();
           holdEventPast = true;
       }
       // Trigger "long" hold
       if ((millis() - downTime) >= longHoldTime)
       {
           if (not longHoldEventPast)
           {
               event = 4;
               longHoldEventPast = true;
           }
       }
   }
   buttonLast = buttonVal;
   return event;
}

void main_page()
{    
      
    if(b_page == 11)
      b_page = 0;
    if(b_page >11)
      b_page =10;

    byte boost = b_page*10;
      
      /*Rpm = 1000;
      AFR=158;
      Cor=Clt=Iat=Tps= Map= genSens2 =100;
      afrTarget=147;
      batVolt = 126;*/
      tft.setTextColor(WHITE, BLACK);
      
      
        tft.setTextSize(1);
        tft.setCursor(72,0);
        tft.print("Batt ");
        divby10(batVolt);
        tft.print(tempchars);
        
        tft.setCursor(72,16);
        if(Map >= 100)
        {
        tft.print("PSI  ");
        tft.print(Map*145/1000);
        tft.print(" ");
        }
        else
        {
        tft.print("inHG ");
        tft.print(Map*3/10-30);
        tft.print(" ");
        }
        
        tft.setCursor(72,32);
        tft.print("DC%");
        tft.setCursor(72,48);
        tft.print("Gear");
        tft.setCursor(0, 7);
        tft.print("CLT ");
        tft.setCursor(0,23);
        tft.print("IAT ");
        tft.setCursor(0,39);
        tft.print("TPS");
        tft.setCursor(0,55);
        tft.print("MAP");
        tft.setCursor(0,71);
        tft.print("FP ");

        tft.setCursor(74, 80);
        tft.print("Targ ");
        divby10(afrTarget);
        tft.print(tempchars);
        tft.print("  ");
        tft.setCursor(74, 88);
        tft.print("Corr ");
        divby10(Cor);
        tft.print(tempchars);
        tft.print("  ");
        
        tft.setTextSize(2);
        tft.setCursor(24,0);
        tft.print(Clt);
        if(Clt < 10)
        tft.print("  ");
        else if(Clt < 100)
        tft.print(" ");
        tft.setCursor(24,16);
        tft.print(Iat);
        if(Iat < 10)
        tft.print("  ");
        else if(Iat < 100)
        tft.print(" ");
        tft.setCursor(24,32);
        tft.print(Tps);
        if(Tps < 10)
        tft.print("  ");
        else if(Tps < 100)
        tft.print(" ");
        tft.setCursor(24,48);
        tft.print(Map);
        if(Map < 10)
        tft.print("  ");
        else if(Map < 100)
        tft.print(" ");
        tft.setCursor(24,64);
        tft.print(genSens2);
        if(genSens2 < 10)
        tft.print("  ");
        else if(genSens2 < 100)
        tft.print(" ");
        
        tft.setCursor(0,114);
        tft.print("RPM");
        tft.setCursor(92,114);
        tft.print("AFR");
        
        tft.setCursor(81 ,96);    //81
        divby10(AFR);
        tft.print(tempchars);
        if(AFR < 100)
        tft.print("  ");

        tft.setTextSize(1);
        if(boost == 0)            //boost duty cycle
        {
        tft.setCursor(100,32); //76
        tft.print(" ");
        tft.print(boost);
        tft.print(" ");
        }
        else if (boost >= 10 && boost <100)
        {
        tft.setCursor(100,32);
        tft.print(" ");
        tft.print(boost);
        tft.print(" ");
        }
        else if (boost >= 100)
        {
        tft.setCursor(100,32);
        tft.print(boost);
        }
        tft.setTextSize(3);
        tft.setCursor(95,48);   //Gear Indicator
        tft.print(" ");
        tft.print(gear);
        tft.print(" ");

        tft.setCursor(0,88);
        tft.print(Rpm);
        if(Rpm < 1000)
        tft.print(" ");
        else if(Rpm < 100)
        tft.print("  ");
        else if(Rpm < 10)
        tft.print("   ");

        
        
      
    
  
   gauge_bottom();
} //end-----------------------------------------------



void gauge_bottom() {

  tft.setTextSize(1);
  tft.drawRect(37, 116, 52, 12, WHITE);
  tft.setCursor(39, 118);
  tft.setTextColor(WHITE, BLACK);
  tft.print("FAN ");
  
//FAN

  switch(stmp[0])
  {
    case 0:
    tft.setTextColor(WHITE, BLACK);
    tft.print("Off ");
    break;

    case 64:
    tft.setTextColor(BLACK, WHITE);
    tft.print("Low ");
    break;
  
    case 128:
    tft.setTextColor(BLACK, WHITE);
    tft.print("High");
    break;

  }
  


} //end---------------------------------------

void divby10(int val) {
  byte length;
  
  itoa(val, tempchars, 10);
  length=strlen(tempchars);

  //tempchars[length + 1]=tempchars[length]; // null shift right
  tempchars[length]=tempchars[length - 1]; //
  tempchars[length - 1]='.';
}
