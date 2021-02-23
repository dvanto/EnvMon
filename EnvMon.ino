
// #define DEBUG_WAKEUPS
//#define DISABLE_LOGGING

#include <limits.h>
#include <WString.h>
// #include <avr/wdt.h>
#include <LowPower.h>               //  https://github.com/rocketscream/Low-Power
#include <EEPROM.h>
#include <ArduinoLog.h>             // https://github.com/thijse/Arduino-Log/
#include <avr/pgmspace.h>
#include <DebounceEvent.h>          // https://github.com/xoseperez/debounceevent
#include "Chrono.h"
#include <LiquidCrystal_PCF8574.h>
//#include <LiquidCrystal_I2C_OLED.h>

//#define INLINE_SENSORKIT
//#define TINY_SENSORKIT

#include <SensorKit.h>

#include "DHT.h"

#define DHTPIN 2 // номер пина, к которому подсоединен датчик
DHT dht(DHTPIN, DHT11);

#include <RotaryEncoder.h>
#define RTR_SW    7
#define RTR_PIN1  8
#define RTR_PIN2  9

RotaryEncoder encoder(RTR_PIN1, RTR_PIN2, RotaryEncoder::LatchMode::TWO03);
DebounceEvent sw_Encoder = DebounceEvent(RTR_SW , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
SensorKit s_fire(2, 12);
//SensorKit s_light(3, 11);
SensorKit s_light(0, 0);


void _P_defines() {} // просто отметка в редакторе

// * 0 - LOG_LEVEL_SILENT     no output
// * 1 - LOG_LEVEL_FATAL      fatal errors
// * 2 - LOG_LEVEL_ERROR      all errors
// * 3 - LOG_LEVEL_WARNING    errors, and warnings
// * 4 - LOG_LEVEL_NOTICE     errors, warnings and notices
// * 5 - LOG_LEVEL_TRACE      errors, warnings, notices & traces
// * 6 - LOG_LEVEL_VERBOSE    all
#define LOG_LEVEL			LOG_LEVEL_TRACE
// #define LOG_LEVEL			LOG_LEVEL_VERBOSE

#define SETFLAG(x) ( ++(x)?(x):(++(x)) )  // если увеличение дало 0, то увеличиваем еще раз

typedef unsigned long	t_time;
/* для работы со сторками из ПЗУ */
typedef __FlashStringHelper* fchar;
typedef const __FlashStringHelper* fchar_;
//const char LOG_AS[] PROGMEM =

//#define FF(a) 			(a)
#define FF(a)       F(a)
// #define FPSTR(a)		(a)

// https://arduino-esp8266.readthedocs.io/en/latest/PROGMEM.html
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))


/* настройки экранчика */
#define	LCD_PORT 						0x27
#define LCD_WIDTH						16
LiquidCrystal_PCF8574 			lcd(LCD_PORT);

//******************************************************************************************
//  EVENT_NONE EVENT_CHANGED EVENT_PRESSED EVENT_RELEASED
//  BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
//  DebounceEvent button = DebounceEvent(BUTTON_PIN, BUTTON_SWITCH | BUTTON_SET_PULLUP);
//*******************************************************************************************
#define LCD_TIMEOUT         2 // s
Chrono LCD_Timeout(Chrono::SECONDS);



volatile uint8_t f_WDT = 0;

void _P_ShutdownMode() {} // просто отметка в редакторе

//******************************************************************************************
typedef enum : char { POWEROFF = 0,
                      STANDBY,
                      ABORT = -127,
                      ERR_FLASH_RESET, // должен быть последним
                      ERR_LAST

                    } t_ShutdownMode;


// statistic
struct t_flash_var_pump {
  uint16_t				sign;
  unsigned int 		pwr;
  unsigned int 		cnt;
  unsigned int 		owfl;
  t_time				 	total_time;
  unsigned long		total_volume;
  byte						errors[ERR_LAST - ABORT];
};

t_flash_var_pump				fv_pump 							= {0, 0, 0, 0, 0, 0};
// t_flash_var_pump				fv_saved;


unsigned 								pump_last_time        = 0;
unsigned								pump_last_volume      = 0;

const  char PROGMEM	STR_sd_CR[]		= ("%s (line:%d)" CR);
const  char PROGMEM	STR_SXd_CR[]	= ("%S %X (line:%d)" CR);

char 										buf[LCD_WIDTH + 1];
char										buf2[LCD_WIDTH + 2];

//******************************************************************************************

void Shutdown(t_ShutdownMode mode);
void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, fchar s1 = NULL); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, const char* s1 ); //, const char*  s2,

/* void log_internal_state(); */

//******************************************************************************************

inline char sw(bool f, char c = 'F')
{
  return f ? c : '_';
}

int freeRam ()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

#define FLASH_SIGNATURE	0xADB0
#define FLASH_OFFSET		0x20*1


//void print_flash(int l)
//{
//  sprintErrors(buf);
//  Log.warning(FF("flash(%d): %x %x %x %x err=|%s|" CR), l,
//              (int)fv_pump.cnt, (int)fv_pump.owfl, (int)fv_pump.total_time, (int)fv_pump.total_volume, buf);
//  // fv_pump.errors[0], fv_pump.errors[1], fv_pump.errors[2], fv_pump.errors[3], fv_pump.errors[4], fv_pump.errors[5],
//  // fv_pump.errors[6], fv_pump.errors[7], fv_pump.errors[8], fv_pump.errors[9]);
//
//}
//
//#define EEPROM_f(f, e_offset, v)		{ EEPROM.f( e_offset, v ); e_offset += sizeof(v); }
//
//void write_flash(uint16_t sign = FLASH_SIGNATURE);
//bool read_flash()
//{
//  int e_offset = FLASH_OFFSET;
//  char *e_buf = (char*)&fv_pump;
//
//  EEPROM_f( get, e_offset, e_buf[0] );
//  EEPROM_f( get, e_offset, e_buf[1] );
//  // Log.notice(STR_SXd_CR, FF("read_flash: sign = "), fv_pump.sign, __LINE__);
//  // Serial.println( fv_pump.sign );
//  // Serial.println( e_offset );
//
//  if ( fv_pump.sign == FLASH_SIGNATURE )
//  {
//    for (size_t i = 2; i < sizeof(fv_pump); i++)
//      EEPROM_f( get, e_offset, e_buf[i] );
//
//    Log.notice(FF("read_flash done %d %d" CR), FLASH_OFFSET, e_offset);
//
//    print_flash(__LINE__);
//    return true;
//  }
//  else
//  {
//    // Log.notice(STR_SXd_CR, FF(""), 0, __LINE__);
//    for (size_t i = 0; i < sizeof(fv_pump.errors); fv_pump.errors[i++] = 0);
//    print_flash(__LINE__);
//    write_flash();
//    return false;
//  }
//
//}
//
//#define EEPROM_W_FUNC	update
//void write_flash(uint16_t sign)
//{
//  int e_offset = FLASH_OFFSET;
//  // t_EEPROM_sign &s = (t_EEPROM_sign&)sign;
//  char *e_buf = (char*)&fv_pump;
//
//  // Serial.println( sign );
//  // Log.warning(STR_SXd_CR, FF("write flash: free mem = "), freeRam(), __LINE__);
//
//  fv_pump.sign = sign;
//  for (size_t i = 0; i < sizeof(fv_pump); i++)
//    EEPROM_f( EEPROM_W_FUNC, e_offset, e_buf[i] );
//
//  Log.notice(FF("write_flash done %d %d" CR), FLASH_OFFSET, e_offset);
//  print_flash(__LINE__);
//
//}
//
//void reset_flash()
//{
//  char *e_buf = (char*)&fv_pump;
//
//  for (size_t i = 0; i < sizeof(fv_pump) - 1; e_buf[i++] = 0); // стирать все кроме последнего errors[RESET];
//
//  delay(100);
//
//  Shutdown(ERR_FLASH_RESET);
//}



void lcd_on(bool timeout = false)
{
  lcd.setBacklight(255);
  lcd.clear();

#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)
  Log.trace(FF("lcd_on %d restart = %c" CR), __LINE__, timeout ? 'R' : '-' );
#endif

  if (timeout)
    LCD_Timeout.restart();
  else
    LCD_Timeout.stop();

}


char* sprintTime4(char* s, unsigned long v)
{
  if (v < 900)
    sprintf(s, ("%03us"), (unsigned)  v);            	// 000s
  else if (v < 6000)
    sprintf(s, ("%03um"), (unsigned) (v / 60) );				// 000m
  /*   else if (v < 600)
      sprintf(s, ("%c:%03um"), (unsigned) (v/60)+'0', (unsigned) (v%60) );         // 0:00	*/
  else if (v < 36000 )
  {
    v /= 360;
    sprintf(s, ("%u.%ch"), (unsigned) v / 10, (char) (v % 10) + '0' );		// 0.0h
  }
  else
    sprintf(s, ("%03uh"), (unsigned) (v / 3600) );			// 000h

  return s;
}


char* sprintTime5(char* s, unsigned long v)
{
  /*
    // Скетч использует 13770 байт (96%) памяти устройства. Всего доступно 14336 байт.
    // Глобальные переменные используют 797 байт (77%) динамической памяти, оставляя 227 байт для локальных переменных. Максимум: 1024 байт.
    if (v < 600)
    sprintf(s, ("%04us"),  (unsigned) v);       // 0000s
    else if ( (v/=6) < 900 )  // < 6000
    sprintf(s, ("%02u.%cm"),  (unsigned) v/10,  (char) (v%10) + '0'); //00.1m
    else if ( (v/=6) < 1000 )
  	sprintf(s, ("%u.%02dh"),  (unsigned) v/100, (unsigned) v%100 );// 0.00h
    else if ( (v/=10) < 1000 )
    sprintf(s, ("%02u.%ch"), 	(unsigned) v/10,  (char) (v%10) + '0' );// 00.0h
    else
    sprintf(s, ("%04uh"),  		(unsigned) v/10 );    // 0000h
  */

  // Скетч использует 13756 байт (95%) памяти устройства. Всего доступно 14336 байт.
  // Глобальные переменные используют 797 байт (77%) динамической памяти, оставляя 227 байт для локальных переменных. Максимум: 1024 байт.
  if (v < 900)
    sprintf(s, ("%04us"),  (unsigned) v);       // 0000s
  /* else if ( v < 5400 )  //   900 сек
    {
  	v /= 6;
    sprintf(s, ("%02u.%cm"),  (unsigned) v/10,  (char) (v%10) + '0'); //00.1m
    }
    else if ( v < 36000 )		// 10 часов
    {
  	v /= 36;
  	sprintf(s, ("%u.%02dh"),  (unsigned) v/100, (unsigned) v%100 );// 0.00h
    } */
  else if ( v < 216000 ) // 60*60*60
  {
    v /= 60;
    sprintf(s, ("%02u:%02uh"), 	(unsigned) v / 60,  (unsigned) (v % 60) ); // 00.0h
  }
  else if ( v < 360000 )   // 100 часов
  {
    v /= 360;
    sprintf(s, ("%02u.%ch"), 	(unsigned) v / 10,  (char) (v % 10) + '0' ); // 00.0h
  }

  else
  {
    sprintf(s, ("%04uh"),  		(unsigned) v / 3600 );     // 0000h
  }


  return s;
}

void lcd_print(char buf[], unsigned int l)
{
  lcd.print(buf);
  Log.notice( FPSTR(STR_sd_CR), buf, l);
}

void lcd_print_P(const char* buf, unsigned int l)
{
  lcd.print( FPSTR(buf) );
  Log.notice( FPSTR(STR_SXd_CR), FPSTR(buf), 0, l);
}



/*
  before:
  Скетч использует 10844 байт (75%) памяти устройства. Всего доступно 14336 байт.
  Глобальные переменные используют 765 байт (74%) динамической памяти, оставляя 259 байт для локальных переменных. Максимум: 1024 байт.

  after:
  Скетч использует 11468 байт (79%) памяти устройства. Всего доступно 14336 байт.
  Глобальные переменные используют 780 байт (76%) динамической памяти, оставляя 244 байт для локальных переменных. Максимум: 1024 байт.
*/

void lcd_statistic(uint8_t mode = 0)
{
  //12    34567890123456
  // char buf[LCD_WIDTH+5] = "T ";//00000 V 000000";
  int sh;
  size_t i;
  char* b1;
  char* b2;
  uint16_t p1;
  uint16_t p2;

  lcd.setCursor(0, 0);



  lcd.setCursor(0, 0);
  lcd_print(buf, __LINE__ + mode * 1000);
  lcd.setCursor(0, 1);
  lcd_print(buf2, __LINE__ + mode * 1000);

  //lcd_on(true);
}

//******************************************************************************************



// уснуть - до следующего поплавка (POWEROFF) или навсегда
void _shutdown(t_ShutdownMode mode = POWEROFF)
{

#ifdef DEBUG
  return;
#endif

  // log_internal_state(-1);

  if (mode == POWEROFF)
  {
    // выключить свет и все
    lcd.setBacklight(0);
  }
  else {
    // FULL Shutdown !!!
    //    pump = PUMP_ABORT;

    // не просыпаться никогда
    // сделать блокировку до полного отключения
    PCICR = 0;
    PCMSK2 = 0;

  }

#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)
  Log.verbose(FF("powerDown %d (%d)" CR), mode, __LINE__);
#endif

  delay(100); // чтобы байтики через serial прошли ...перед глубоким сном

#ifdef DEBUG_WAKEUPS
  wakeupmode = 1;
#endif
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)
  Log.verbose(FF("wakeup %d" CR), __LINE__);
#endif
}


// печатает текст и останавливает устройство
// void Shutdown(t_ShutdownMode mode = POWEROFF, fchar s1 = NULL)

void Shutdown(t_ShutdownMode mode)
{
  // char buf[20] = "empty";
  size_t	l;

  if ( !(mode == POWEROFF || mode == STANDBY) && mode > ERR_LAST) mode = ABORT;


#ifdef WatchDog_h
  WatchDog::stop();
#endif
  f_WDT = 0;

  _shutdown(mode);
}

/*
  void A_Watchdog()
  {
  wdt_disable();
  //f_WDT = 1;
  SETFLAG(f_WDT);
  //PumpOff();
  }
*/


//***********************************************************************************************************************************************************



//***********************************************************************************************************************************************************


void setup()
{
#ifndef DISABLE_LOGGING
  Serial.begin(115200);
  while (!Serial);
  // Serial.println(FF(CR "In the begining...") );
#endif

  // lcd.init();
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd_on(true);
  Log.begin(LOG_LEVEL, &Serial);
  // Log.notice( FF(CR "******************************************" CR) );                     // Info string with Newline

  lcd_print_P(PSTR("Starting 3..."), __LINE__);
  //  lcd.setCursor(0,1);
  //  lcd.print("Starting...");

  dht.begin();
  LCD_Timeout.start();

  s_fire.begin();
  s_light.begin();

#ifdef WatchDog_h
  Log.notice( FF("WatchDog::init(A_Watchdog, OVF_8000MS, STOP);" CR) );
  WatchDog::init(A_Watchdog, OVF_8000MS, STOP);
#endif

  // write_flash(0); // только один раз для платы
  //	read_flash();
  //	fv_pump.pwr++;

  /*
    // разрешение прерываний INT0 и INT1
    //  EIMSK  =  (1<<INT0)  | (1<<INT1);

    // настройка срабатывания прерываний на любому изменению
    //EICRA  =  (0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (1<<ISC00);

  	PORTD |= (1 << PORTD5) | (1 << PORTD6) | (1 << PORTD6);
    PORTB |= (1 << PORTB2);

    // разрешение прерываний с портов B (PCINT[7:0]) и D (PCINT[23:16]), и запрет с порта C (PCINT[14:8])
    PCICR  |= (1 << PCIE2) | (0 << PCIE1) | (1 << PCIE0);

    // маскирование всех ног, кроме PB0 и PD7 - по одной на PCINT0 и PCINT2
    PCMSK0 |= (0 << PCINT7)  | (0 << PCINT6)  | (0 << PCINT5)  | (0 << PCINT4)  | (0 << PCINT3)  | (1 << PCINT2)  | (0 << PCINT1)  | (0 << PCINT0);
    //PCMSK1 |=                (0 << PCINT14) | (0 << PCINT13) | (0 << PCINT12) | (0 << PCINT11) | (0 << PCINT10) | (0 << PCINT9)  | (0 << PCINT8);
    PCMSK2 |= (1 << PCINT23) | (1 << PCINT22) | (1 << PCINT21) | (0 << PCINT20) | (0 << PCINT19) | (0 << PCINT18) | (0 << PCINT17) | (0 << PCINT16);
  */
}


static int pos = 0;
int d = 0;

void update_lcd()
{
  lcd.setBacklight(pos);
  lcd.setCursor(0, 0);
  lcd.print( ((d < 0) ? " << " : "    "));
  lcd.print( pos );
  lcd.print( ((d > 0) ? " >> " : "    "));
}

void update_lcd(SensorKit &s)
{
  //  lcd.setCursor(0, 1);
  int v = s.read();
  bool st = s.status();

  Serial.print("v:");
  Serial.print(v);
  Serial.print(" status:");
  Serial.println(st);

  lcd.print( st ? "\xFF " : "_ " );
  lcd.print( v );
  lcd.print( " ");
}
//***********************************************************************************************************************************************************

char clockwise[] = " \xDF-\x2B\x2A";
char cw_cnt = 0;

void loop()
{
  
  encoder.tick();
  if ( unsigned int event = sw_Encoder.loop()) {
    if ( event == EVENT_PRESSED)
    {
      encoder.setPosition(pos ? (pos = 0) : (pos = 255));
      d = 0;
      //      lcd.print( "BBBB" );
    }
    Serial.print("button:");
    Serial.println(event);

    update_lcd();
  }

  int newPos = encoder.getPosition();
  if (pos != newPos) {
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println(d = (int)(encoder.getDirection()));

    pos = newPos;
    if (pos < 0) encoder.setPosition( pos = 0 );
    else if ( pos > 255) encoder.setPosition( pos = 255 );

    update_lcd();
  }
  //  return;




  if (LCD_Timeout.hasPassed(LCD_TIMEOUT))
  {
    LCD_Timeout.restart();

    //Считываем влажность

    float h = dht.readHumidity();

    // Считываем температуру

    float t = dht.readTemperature();

    // Проверка удачно прошло ли считывание.

    if (isnan(h) || isnan(t)) {
      Serial.println("Не удается считать показания");
    }
    else {
      Serial.print ("Humidity: ");
      Serial.print (h);
      Serial.print ("%\t");
      Serial.print ("Temperature: ");
      Serial.print (t);
      Serial.println (" *C");

      lcd.setCursor(0, 1);
      lcd.print( "T: ");
      lcd.print( t );
      lcd.print( " H: ");
      lcd.print( h );
      lcd.print( "         ");
    }

    lcd.setCursor(0, 0);
    lcd.print( clockwise[ cw_cnt++ ] );
//    lcd.print( sizeof(clockwise) );
    if ( cw_cnt >= sizeof(clockwise)-1 ) cw_cnt=0;
    lcd.print( " " );
    

    update_lcd(s_fire);
    update_lcd(s_light);
  }






}
