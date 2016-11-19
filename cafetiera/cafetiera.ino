#include <TimeLib.h>
#include <OneWire.h>
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

const uint8_t kSensorPin   = 2;

//Lcd stuff
String btnStr = "None";

// Events
enum eEvent { kNone, kBrew, kBrewEnd, kBtnSel, kBtnUp, kBtnDown, kBtnLeft, kBtnRight};
eEvent event = kNone;

/*********** Configuration ****/
struct Configuration
{
  Configuration()
  {
    c0 = 30; cM = 60; cF = 95;
    tM = 4 * 60;
    tF = 3 * 60;
  }
  uint8_t  c0, cM, cF; // initial, middle and final target temperatures in Celsius degrees
  time_t tM, tF;     // desired time (seconds) to reach cM and cF
  uint8_t  tSampling;  // sampling interval
} cfg;

/*********** HEATER *********/
class Heater
{
public:
  Heater()
  { Reset(); }
  
  void  Reset()
  {
    currStatus_ = kRelayOff;
    pinMode(kRelay1Pin, OUTPUT);
    pinMode(kRelay2Pin, OUTPUT);
    digitalWrite(kRelay1Pin, kRelayOff);
    digitalWrite(kRelay2Pin, kRelayOff);
  }

  void  Off()
  {
    if (currStatus_ == kRelayOff)
      return;

    currStatus_ = kRelayOff;
    digitalWrite(kRelay1Pin, kRelayOff);
  }

  void  On()
  {
    if (currStatus_ == kRelayOn)
      return;
      
    currStatus_ = kRelayOn;
    digitalWrite(kRelay1Pin, kRelayOn);
  }

private:
  uint8_t currStatus_;
  const uint8_t kRelay1Pin   = 18;
  const uint8_t kRelay2Pin   = 19;
  const uint8_t kRelayOff    = HIGH;
  const uint8_t kRelayOn     = LOW;
} heater;

/*********** Temperature *********/
OneWire  ds(kSensorPin);  // on pin 2 (a 4.7K resistor is necessary)

bool getTemperature(float& celsius);
void ResetAll();

/*********** Button *********/
class Button
{
public:
  
  enum eType { kSelect, kLeft, kRight, kUp, kDown };
  Button() : currentRead_(kDefault) {}
  
  bool Scan()
  {
    uint16_t x = analogRead(A0); // the buttons are read from the analog0 pin
    // Check if x has changed
    if ((x < currentRead_ - kTol) || (x > currentRead_ + kTol))
    {
      currentRead_ = x;
      
      if (currentRead_ > 700 && currentRead_ < 745)
      { event = kBtnSel; }
      else if (currentRead_ > 450 && currentRead_ < 500)
      { event = kBtnRight; }
      else if (currentRead_ < 10)
      { event = kBtnLeft; }
      else if (currentRead_ > 100 && currentRead_ < 150)
      { event = kBtnUp; }
      else if (currentRead_ > 280 && currentRead_ < 350)
      { event = kBtnDown; }
      
      return true;
    }
    return false;
   }
   
private:
  uint16_t currentRead_;
  const uint16_t kDefault = 1023;
  // tolerance for values - analog input
  // subsequent keys might read values with small difference
  // even if it's the same key
  const uint8_t kTol = 3;
} btn;

/*********** LCD *********/
struct LcdCoffee
{
  void InitLcd()
  {
    // set up the LCD's number of columns and rows: 
    lcd.begin(16, 2);
    pinMode(10, INPUT);
    backlightStatus_ = true;
    ClearOffTimer();
  }

  void Reset()
  {
    DisplayStart();
    ClearOffTimer();
  }
  
  void PrintTemperature(float celsius)
  {
    ClearLcdLine(1);
    lcd.setCursor(0,1);
    lcd.print(celsius, 1);
    lcd.print(" C");
    ClearOffTimer();
  }

  void DisplayBrew()
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Reach: ");
    lcd.print(cfg.c0);
    ClearOffTimer();    
  }

  void DisplayStart()
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Start");
    ClearOffTimer();
  }
  
  void DisplayDone()
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Coffee Done");
    ClearOffTimer();
  }

  void DisplayHeatPhase(uint8_t cStart, uint8_t cEnd, time_t tSecs)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(cStart); lcd.print("->"); lcd.print(cEnd);
    lcd.print("C ");
    DisplayMinSec(tSecs);
    ClearOffTimer();
  }

  void DisplayHeaterStatus(float cExpected, float cNow, time_t deltaBrew)
  {
    ClearLcdLine(1);
    lcd.setCursor(0,1);
    DisplayMinSec(deltaBrew);
    lcd.print(" ");
    lcd.print(cExpected, 1);
    lcd.print(" ");
    lcd.print(cNow, 1);
    ClearOffTimer();
  }

  void UpdateBacklight()
  {
    if (now() - tLastUpdate_ >= kOffTimer)
      Backlight(false);
    else
      Backlight(true);
  }

private:
  void ClearOffTimer()
  { tLastUpdate_ = now(); }
  
  void Backlight(bool on)
  {
    if (on == backlightStatus_)
      return;

    backlightStatus_ = on;
    if (on)
    {
      pinMode(10, INPUT);
    }
    else
    {
      pinMode(10, OUTPUT);
      digitalWrite(10, LOW);
    }
  }
  
  void ClearLcdLine(uint8_t line)
  {
    if (line > 1)
      return;
      
    lcd.setCursor(0, line);
    for (uint8_t i = 0; i < 16; ++i)
      lcd.print(' ');
  }

  void DisplayMinSec(time_t secs)
  {
    if (minute(secs) < 10)
      lcd.print(0);
    lcd.print(minute(secs));
    lcd.print(":");
    if (second(secs) < 10)
      lcd.print(0);
    lcd.print(second(secs));
  }
  
private:
  time_t tLastUpdate_;
  bool   backlightStatus_;
  const static time_t kOffTimer = 30;
} lcdCoffee;

struct Brewer
{
  Brewer() { Reset(); }
  Reset() { state_ = kNone; }
  
  void Init()
  {
    tPrev_ = tBrewPhaseStart_ = 0;
    cCoolDown_ = 0;
    state_ = kReachC0;
    heater.On();
    lcdCoffee.DisplayBrew();
    lcdCoffee.PrintTemperature(10);
  }

  void Step()
  {
    float celsius = 0;
    if (!getTemperature(celsius))
      return;
    // Just in case anything goes wrong and we reach this temperature
    // Abort any calculations and skip to end
    if (celsius >= 98)
    {
      heater.Off();
      event = kBrewEnd;
      return;
    }
    switch (state_)
    {
      case kReachC0:
      {
        // Heat up to cfg.c0 as fast as we can
        lcdCoffee.PrintTemperature(celsius);
        if (celsius < cfg.c0)
          break;
        state_ = kReachCM;
        tBrewPhaseStart_ = tPrev_ = now();
        heater.Off();
        lcdCoffee.DisplayHeatPhase(cfg.c0, cfg.cM, cfg.tM);
        break;
      }
      case kReachCM:
      {
        // Heat up to cfg.cM in cfg.tM seconds
        if (celsius >= cfg.cM)
        {
          // We reached cfg.cM; now determine cool down time for the heater
          tBrewPhaseStart_ = tPrev_ = now();
          heater.Off();
          state_ = kCalcCoolDown;
          break;
        }
        // Calculate if heater should be on or off based on start/end temperature and time to reach end temp
        CheckRefTemp(celsius, cfg.c0, cfg.cM, cfg.tM);
        break;
      }
      case kCalcCoolDown:
      {
        // Calculate how many celsius degrees the heater pumps into the coffee before stabilize
        cCoolDown_ = 5; // hardcode for now
        state_ = kReachCF;
        lcdCoffee.DisplayHeatPhase(cfg.cM, cfg.cF - cCoolDown_, cfg.tF);
        break;
      }
      case kReachCF:
      {
        // Heat up to cfg.cF - cCoolDown in cfg.tF seconds
        if (celsius >= cfg.cF - cCoolDown_)
        {
          // We reached cfg.cF - cCoolDown
          heater.Off();
          event = kBrewEnd;
          break;
        }
        // Calculate if heater should be on or off based on start/end temperature and time to reach end temp
        CheckRefTemp(celsius, cfg.cM, cfg.cF - cCoolDown_, cfg.tF);
        break;
      }
      default:
      { break; }
    }
  }

private:
  // cStart - start temperature; cEnd = end temp; tSec == time to reach temp
  void CheckRefTemp(float cNow, uint16_t cStart, uint16_t cEnd, time_t tSec)
  {
    time_t tCurr = now();
    time_t deltaSample = tCurr - tPrev_;
    tPrev_ = tCurr;
    // Do stuff only once every cfg.tSampling seconds
    if (deltaSample < cfg.tSampling)
      return;

    time_t deltaBrew = tCurr - tBrewPhaseStart_;
    // If heater is not powerful enough, the target temp cEnd is not reached
    // within the time restriction tSec
    // In this case , make deltaBrew equal to tSec and heater will work at max capacity
    if (deltaBrew > tSec)
      deltaBrew = tSec;

    /* linear func tempExpected = factor * time_secs + tempInitial
     *  factor = (tempFinal - tempInitial) / totalTime
     *  factor = (60 - 30) / 120 = 30/120 = 0.25
     *  time_secs = 20 --> tempExpected = 0.25 * 20 + 30 = 5 + 30 = 35;
     */
    float factor = (float)(cEnd - cStart) / tSec;
    float cExpected = factor * deltaBrew + cStart;
    lcdCoffee.DisplayHeaterStatus(cExpected, cNow, deltaBrew);
    if (cNow >= cExpected)
      heater.Off();
    else
      heater.On();
  }
private:
  enum eState { kNone, kReachC0, kReachCM, kCalcCoolDown, kReachCF };
  eState state_;
  time_t tPrev_, tBrewPhaseStart_;
  uint8_t cCoolDown_; // inertial temperature gain of heater
} brewer;

struct Fsm
{
  Fsm() { Reset(); }

  void Reset()
  {
    state_ = eState::kNone;
  }
  void Step()
  {
    Serial.print("state:"); Serial.print(state_);
    Serial.print(" event:"); Serial.println(event);
    eEvent evNow = event;
    event = eEvent::kNone;
    switch (evNow)
    {
      case eEvent::kNone:
      {
        switch (state_)
        {
          case eState::kStartBrew:
          {
            brewer.Init();
            state_ = kBrewing;
            break;
          }
          case eState::kBrewing:
          {
            brewer.Step();
            break;
          }
          default:
          { btn.Scan(); break; }
        }
        break;
      }
      case eEvent::kBtnSel:
      {
        switch (state_)
        {
          case eState::kNone:
          {
            state_ = kStartBrew;
            break;
          }
          case eState::kDone:
          {
            ResetAll();
            state_ = kNone;
            break;
          }
          default:
          { break; }
        }
        break;
      }
      case eEvent::kBrewEnd:
      {
        switch (state_)
        {
          case eState::kBrewing:
          {
            lcdCoffee.DisplayDone();
            state_ = eState::kDone;
            break;
          }
          default:
          { break; }
        }
        break;
      }
      default:
      { break; }
    }
  }

private:
  enum eState { kNone, kStartBrew, kBrewing, kDone };
  eState state_;
} fsm;

void setup(void)
{
  Serial.begin(9600);
  lcdCoffee.InitLcd();
  ResetAll();
}

void loop(void)
{
  fsm.Step();
  lcdCoffee.UpdateBacklight();
  delay(10  );
}

// Returns true if successful
bool getTemperature(float& celsius)
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  bool fahrenheit;

  if ( !ds.search(addr)) {
    ds.reset_search();
    return false;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return false;
  }
  //Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return false;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44);        // start conversion, use ds.write(0x44,1) with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  return true;
}

void ResetAll()
{
  event = eEvent::kNone;
  lcdCoffee.Reset();
  heater.Reset();
  fsm.Reset();
  brewer.Reset();
}

