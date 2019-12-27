/*
   Управление колодезным насосом

   Закачка воды в бак с двумя датчиками - нижний уровень и верхний
   Ограничение по времени заполнения бака
   Переход в аварийный режим при "непредсказуемом" поведении датчиков
   Вывод диагностической информации на LCD 1602 (16 символов, 2 строки)

   Рабочий режим
   | 1234567890123456 |
   | ABRLHO 000s 0000
   | STA msg456789012

   Доп режм - вкл по кнопке, возврат к рабочему режиму через 10 сек
   mode  0
   | 1234567890123456 |
   | T 0000s L 000000
   | C 00000 OF 00000

*/

// #include <avr/wdt.h>
#include <LowPower.h>               //  https://github.com/rocketscream/Low-Power
#include <EEPROM.h>
#include <ArduinoLog.h>             // https://github.com/thijse/Arduino-Log/
#include <avr/pgmspace.h>
#include <DebounceEvent.h>          // https://github.com/xoseperez/debounceevent
#include "Chrono.h"
#include <LiquidCrystal_PCF8574.h>
//#include <LiquidCrystal_I2C_OLED.h>


// #define DISABLE_LOGGING
// * 0 - LOG_LEVEL_SILENT     no output 
// * 1 - LOG_LEVEL_FATAL      fatal errors 
// * 2 - LOG_LEVEL_ERROR      all errors  
// * 3 - LOG_LEVEL_WARNING    errors, and warnings 
// * 4 - LOG_LEVEL_NOTICE     errors, warnings and notices 
// * 5 - LOG_LEVEL_TRACE      errors, warnings, notices & traces 
// * 6 - LOG_LEVEL_VERBOSE    all 
#define LOG_LEVEL			LOG_LEVEL_TRACE

#define SETFLAG(x) ( ++(x)?(x):(++(x)) )  // если увеличение дало 0, то увеличиваем еще раз


/* для работы со сторками из ПЗУ */
typedef const __FlashStringHelper* fchar;
//const char LOG_AS[] PROGMEM =

//#define FF(a) 			(a)
#define FF(a)       F(a)

// const char STR_D_CR[] PROGMEM  = ("%s (line:%d)" CR);
fchar STR_sd_CR;
fchar STR_SXd_CR;


/* настройки экарнчика */
#define	LCD_PORT 						0x27
#define LCD_WIDTH						16
LiquidCrystal_PCF8574 			lcd(LCD_PORT);


//*******************************************************************************************

#define PIN_OVERFLOW        PD5
#define PIN_HIGHMARK        PD6
#define PIN_LOWMARK         PD7

#define SW_OVERFLOW         5
#define SW_HIGH_MARK        6
#define SW_LOW_MARK         7

#define SW_MOTOR            8
#define SW_VALVE            9

/* 
#define LOWMARK_TIMEOUT     10 // s
#define HIGHMARK_TIMEOUT    427 // s = 0,468л/с 200 литров
#define SPEED_LPS           0.468 // liters per second
#define LCD_TIMEOUT         (LCD_STAT_TIME+10) // s
#define LCD_STAT_TIME       10 // s
#define STAT_DELAY			 		500 // ms
 */
 
#define LOWMARK_TIMEOUT     3 // s
#define HIGHMARK_TIMEOUT    30 // s = 0,468л/с 200 литров
#define SPEED_LPS           0.468 // liters per second
#define LCD_TIMEOUT         (LCD_STAT_TIME+5) // s
#define LCD_STAT_TIME       5 // s
#define STAT_DELAY			 		1000 // ms


//******************************************************************************************
//  EVENT_NONE EVENT_CHANGED EVENT_PRESSED EVENT_RELEASED
//  BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
//  DebounceEvent button = DebounceEvent(BUTTON_PIN, BUTTON_SWITCH | BUTTON_SET_PULLUP);

volatile uint8_t f_Overflow = 0;
volatile uint8_t f_LowMark = 0;
volatile uint8_t f_HighMark = 0;

void btn_Overflow(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_LowMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_HighMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);

DebounceEvent sw_Overflow  = DebounceEvent(SW_OVERFLOW,   btn_Overflow, BUTTON_PUSHBUTTON );
DebounceEvent sw_LowMark   = DebounceEvent(SW_LOW_MARK,   btn_LowMark,  BUTTON_PUSHBUTTON );
DebounceEvent sw_HighMark  = DebounceEvent(SW_HIGH_MARK,  btn_HighMark, BUTTON_PUSHBUTTON );

Chrono TankTimeout(Chrono::SECONDS);
Chrono FlowTimeout(Chrono::SECONDS);
Chrono LCD_Timeout(Chrono::SECONDS);
Chrono StatDelay(Chrono::MILLIS);


volatile uint8_t f_WDT = 0;

void _P_ShutdownMode() {} // просто отметка в редакторе

//******************************************************************************************
typedef enum : char { POWEROFF = 0,
											STANDBY,
											ABORT = -127,
											ERR_OVERFLOW,
											ERR_WATHCDOG,
											
											ERR_NO_FLOW,
											ERR_TANK_TIMEOUT,
											
											ERR_START_ABORTED_PUMP,
											ERR_INVALID_SENSORS,
											ERR_INV_LOWMARK_STATUS,
											ERR_INV_HIGHMARK_STATUS,
											ERR_HIGHMARK_UNATTENDED_RELEASE,
											ERR_LAST
											
										} t_ShutdownMode;
						
// ErrorNames[t_ShutdownMode-ABORT]			
																	// 1234567890123456
const  char ErrorMsg_1[] PROGMEM 	= "FATAL ABORT!";
const  char ErrorMsg_2[] PROGMEM 	= "OVRFLW DETECTED!";
const  char ErrorMsg_3[] PROGMEM 	= "WATCHDOG ALERT!";
const  char ErrorMsg_4[] PROGMEM 	= "NO FLOW! Chk WTR";
const  char ErrorMsg_5[] PROGMEM 	= "Tank Timout!";
const  char ErrorMsg_6[] PROGMEM 	= "Start failed!!!";
const  char ErrorMsg_7[] PROGMEM 	= "INVALID sensors!";
const  char ErrorMsg_8[] PROGMEM 	= "LM FTL! Chk SENS";
const  char ErrorMsg_9[] PROGMEM 	= "HM FTL! Chk SENS";
const  char ErrorMsg_10[] PROGMEM = "HM ERR! Chk SENS";

				
const  char* const  ErrorMsg[] PROGMEM = {
										ErrorMsg_1, ErrorMsg_2, ErrorMsg_3, ErrorMsg_4, ErrorMsg_5,
										ErrorMsg_6, ErrorMsg_7, ErrorMsg_8, ErrorMsg_9, ErrorMsg_10
};


//******************************************************************************************
typedef enum : byte { PUMP_OFF = 0,  PUMP_ABORT = 1, PUMP_ON = 2, PUMP_STANBDY = 3} t_PumpStatus;

t_PumpStatus pump 		= PUMP_OFF;

// statistic
struct t_flash_var_pump {
				int 						cnt;
				int     				owfl;
				unsigned long 	total_time;
				unsigned long		total_volume;
			};
								
t_flash_var_pump				fv_pump 							= {0,0,0,0};
t_flash_var_pump				fv_saved;
byte										fv_errors[ERR_LAST-ABORT];

unsigned 								pump_last_time        = 0;
unsigned								pump_last_volume      = 0;

char 										buf[20];

//******************************************************************************************

void Shutdown(t_ShutdownMode mode);
void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, fchar s1 = NULL); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, const char* s1 ); //, const char*  s2,

void log_internal_state();

//******************************************************************************************

inline char sw(bool f, char c ='F') 
{
  return f ? c : '_';
}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

#define FLASH_SIGNATURE	0xADB3
#define FLASH_OFFSET		0x51

void print_flash(int l)
{
	sprintErrors(buf);
	Log.warning(FF("flash(%d): %x %x %x %x err=|%s|" CR), l,
		(int)fv_pump.cnt, (int)fv_pump.owfl, (int)fv_pump.total_time, (int)fv_pump.total_volume, buf);
		// fv_errors[0], fv_errors[1], fv_errors[2], fv_errors[3], fv_errors[4], fv_errors[5],
		// fv_errors[6], fv_errors[7], fv_errors[8], fv_errors[9]);
	
}

#define EEPROM_f(f, e_offset, v)		{ EEPROM.f( e_offset, v ); e_offset += sizeof(v); }

void write_flash(uint16_t sign = FLASH_SIGNATURE);
bool read_flash()
{
	int e_offset = FLASH_OFFSET;
	uint16_t sign;
	
	EEPROM_f( get, e_offset, sign );
	Log.notice(STR_SXd_CR, FF("read_flash: sign = "), sign, __LINE__);
	
	if ( sign == FLASH_SIGNATURE )
	{
		// Log.notice(STR_SXd_CR, FF(""), 0, __LINE__);
		// EEPROM_f( get, e_offset, fv_pump );
		// EEPROM.get( e_offset, fv_errors );
		EEPROM_f( get, e_offset, fv_pump.cnt );
		EEPROM_f( get, e_offset, fv_pump.owfl );
		EEPROM_f( get, e_offset, fv_pump.total_time );
		EEPROM_f( get, e_offset, fv_pump.total_volume );
	
		for(size_t i=0; i<sizeof(fv_errors); i++ ) 
			EEPROM_f( get, e_offset, fv_errors[i] );
		
		Log.notice(FF("read_flash done %d %d" CR), FLASH_OFFSET, e_offset);
		
		print_flash(__LINE__);
		return true;
	}
	else 
	{
		// Log.notice(STR_SXd_CR, FF(""), 0, __LINE__);
		for (size_t i=0; i<sizeof(fv_errors); fv_errors[i++]=0);
		print_flash(__LINE__);
		write_flash(FLASH_SIGNATURE);
		return false;
	}
	
}


void write_flash(uint16_t sign)
{
	int e_offset = FLASH_OFFSET;
	
	Log.warning(STR_SXd_CR, FF("write flash: free mem = "), freeRam(), __LINE__);
	
	EEPROM_f( put, e_offset, sign );
	
	EEPROM_f( put, e_offset, fv_pump.cnt );
	EEPROM_f( put, e_offset, fv_pump.owfl );
	EEPROM_f( put, e_offset, fv_pump.total_time );
	EEPROM_f( put, e_offset, fv_pump.total_volume );
	
	for(size_t i=0; i<sizeof(fv_errors); i++) 
		EEPROM_f( put, e_offset, fv_errors[i] );
	
	Log.notice(FF("write_flash done %d %d" CR), FLASH_OFFSET, e_offset);
	print_flash(__LINE__);
	
}


void lcd_on(bool timeout = false)
{
  lcd.setBacklight(255);
  lcd.clear();

// #if (LOG_LEVEL == LOG_LEVEL_VERBOSE)
	Log.trace(FF("lcd_on %d restart = %c" CR), __LINE__, timeout?'R':'-' );
// #endif
	
  if (timeout)
    LCD_Timeout.restart();
	else
		LCD_Timeout.stop();

}


char* sprintTime4(char* s, unsigned long v)
{
  if (v < 600)
    sprintf(s, ("%03us"), (unsigned)  v);            // 000s
  else if (v < 6000)
    sprintf(s, ("%03um"), (unsigned) (v/60) );         // 000m
  else if (v < 36000 )
	{
		v /= 360;
    sprintf(s, ("%u.%ch"), (unsigned) v/10, (char) (v%10) + '0' );		// 0.0h
	}
  else
    sprintf(s, ("%03uh"), (unsigned) (v/3600) );       // 000h

  return s;
}


char* sprintTime5(char* s, unsigned long v)
{
  if (v < 600)
	if (v < 600)
    sprintf(s, ("%04us"),  (unsigned) v);       // 0000s
  else if ( (v/=6) < 900 )  // < 6000
    sprintf(s, ("%02u.%cm"),  (unsigned) v/10,  (char) (v%10) + '0'); //00.1m
  else if ( (v/=6) < 1000 )	
		sprintf(s, ("%u.%02dh"),  (unsigned) v/100, (unsigned) v%100 );// 0.00h
  else if ( (v/=10) < 1000 )
    sprintf(s, ("%02u.%ch"), 	(unsigned) v/10,  (char) (v%10) + '0' );// 00.0h
  else
 */

// Скетч использует 13756 байт (95%) памяти устройства. Всего доступно 14336 байт.
// Глобальные переменные используют 797 байт (77%) динамической памяти, оставляя 227 байт для локальных переменных. Максимум: 1024 байт.
  if (v < 600)
    sprintf(s, ("%04us"),  (unsigned) v);       // 0000s
  else if ( v < 5400 )  // < 6000
	{
		v /= 6;
    sprintf(s, ("%02u.%cm"),  (unsigned) v/10,  (char) (v%10) + '0'); //00.1m
  }
	else if ( v < 36000 )	
	{
		v /= 36;
		sprintf(s, ("%u.%02dh"),  (unsigned) v/100, (unsigned) v%100 );// 0.00h
  }
	else if ( v < 360000 )
  {
		v /= 360;
		sprintf(s, ("%02u.%ch"), 	(unsigned) v/10,  (char) (v%10) + '0' );// 00.0h
  }
	else
  {
		sprintf(s, ("%04uh"),  		(unsigned) v/3600 );       // 0000h
	} 

	
  return s;
}


char status_msg[]="INV\0OFF\0ABR\0ON!\0INV";

void lcd_status()
{
  char *s, c1, c2, c3;
  char s_time[5];
  char s_vol[5];
	char buf[LCD_WIDTH+1];

	// update stat
	pump_last_time = TankTimeout.elapsed();
  pump_last_volume = SPEED_LPS * pump_last_time;
	
  lcd.setCursor(0, 0);
  switch (pump)
  {
    case PUMP_OFF:
		case PUMP_STANBDY:
      s = status_msg+4;
      break;
    case PUMP_ABORT:
      s = status_msg+8;  // Abort
      break;
    case PUMP_ON:
      s = status_msg+12;
      break;
    default:
      s = status_msg;  // INVALID
  }
	
	sprintf(buf, "%s%c%c%c %s %04u", 
					s, 
					c1 = sw(sw_Overflow.pressed(), 'O'),
					c2 = sw(sw_HighMark.pressed(), 'F'),
					c3 = sw( sw_LowMark.pressed(), 'E'),
					sprintTime4(s_time, pump_last_time),
					pump_last_volume
					);
	lcd.print(buf);
	Log.notice( STR_sd_CR, buf, __LINE__);
	
  lcd.setCursor(0, 1);
}

int sprintErrors(char* buf)
{		
		char *s = buf;
		for(char i=0; i<sizeof(fv_errors); s+=sprintf(s, "%X", fv_errors[i++]));
		for(char* t = buf; *t; t++) 
			if (*t=='0') *t=' ';
		
		return s-buf;
}

void lcd_statistic(uint8_t mode=0)
{
														//12    34567890123456
	// char buf[LCD_WIDTH+5] = "T ";//00000 V 000000";
	int sh;
	
  switch (mode)
  {
    case 0:
    default:  // пока так
			
      lcd.setCursor(0, 0);
			*buf = 'T';
			*(buf+1)=' ';
			sprintTime5(buf+2, fv_pump.total_time);
			sprintf(buf+7, " V %06d", fv_pump.total_volume);
			
      lcd.print(buf);
			Log.notice( STR_sd_CR, buf, __LINE__);

      lcd.setCursor(0, 1);
			
			sh = sprintf( buf, "%03dE", fv_pump.cnt);
			for( size_t i=sh+sprintErrors( buf + sh ); i<=LCD_WIDTH; buf[i++]=' '); //fv_pump.owfl);
			buf[LCD_WIDTH] = 0;
			
			lcd.print(buf);
			Log.notice( FF("stat (%d): %i sh %i buf %s" CR), __LINE__, sh, buf);
			break;
  }

  //lcd_on(true);
}

//******************************************************************************************

void PumpOff()
{
  Log.notice( FF("Pump OFF - done: %i seconds" CR), pump_last_time );

  //noInterrupts();
  digitalWrite(SW_VALVE, LOW);
  digitalWrite(SW_MOTOR, LOW);
  //interrupts();

  pump = PUMP_OFF;
	
	lcd_on(true);
  lcd_status();
	
  // stat
		
  fv_pump.total_time += pump_last_time;
  fv_pump.total_volume += pump_last_volume;
	write_flash();

  // Log.notice( FF( "done: %i seconds" CR), pump_last_time );
	
	// read_flash();
	

  FlowTimeout.stop();
  TankTimeout.stop();
  
}


// включает двигатель, если все нормально и запускает таймеры переполнения и нижнего датчика
void PumpOn()
{

  if (pump == PUMP_ABORT)
  {
    Shutdown(
      //  1234567890123456
      // FF("Start failed!!!" ),
      ERR_START_ABORTED_PUMP);
    return;
  }

  Log.notice( FF("Pump ON - " ) );

  // stat
  fv_pump.cnt++;

  digitalWrite(SW_VALVE, HIGH);
  digitalWrite(SW_MOTOR, HIGH);

  Log.notice( FF( "done" CR) );

  FlowTimeout.restart();
  TankTimeout.restart();
  StatDelay.restart();
	
  pump = PUMP_ON;

	lcd_on();
  lcd_status();
}


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
    pump = PUMP_ABORT;

    // не просыпаться никогда
    // сделать блокировку до полного отключения
		PCICR = 0;
    PCMSK2 = 0;
	
  }

#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.verbose(FF("powerDown %d (%d)" CR), mode, __LINE__);
#endif

  delay(100); // чтобы байтики через serial прошли ...перед глубоким сном
	
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
	
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.verbose(FF("wakeup %d" CR), __LINE__);
#endif
}

										
// печатает текст и останавливает устройство
// void Shutdown(t_ShutdownMode mode = POWEROFF, fchar s1 = NULL)

void Shutdown(t_ShutdownMode mode)
{
	char buf[20] = "empty";
	
	if ( !(mode == POWEROFF || mode == STANDBY) && mode > ERR_LAST) mode = ABORT;
	if (mode < POWEROFF)
	{
		byte &err = fv_errors[mode - ABORT];
		err += (err^0x0f)?1:0;
		
		pump = PUMP_ABORT;
/* 		
		sprintErrors(buf);
		Log.notice( FF("Shutdown mode %d %X %X %d %s (%d)" CR), mode, &err, fv_errors, sizeof(fv_errors), buf, __LINE__);
 */
	}
	
	if (mode == STANDBY)
		mode = POWEROFF;
	else
	{
		PumpOff();
		
		// lcd_on(pump != PUMP_ABORT);
		lcd_on();
		lcd_status();
		
		strcpy_P(buf, (char*)pgm_read_word(&(ErrorMsg[mode-ABORT])));
		
		lcd.print( buf );
		Log.fatal( STR_sd_CR, buf, __LINE__);
	}
	
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

void btn_LowMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
// #if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace( FF(CR "LowMark: %i %x" CR), f_LowMark, event);
// #endif	
  //f_LowMark = 0;

  switch (event)
  {
    case EVENT_PRESSED:
      if (sw_HighMark.pressed()) {
#ifdef WatchDog_h
        WatchDog::start();
#endif
        PumpOn();
      } else {
        Shutdown(
          //  1234567890123456
          // FF("INVALID sensors!"),
          ERR_INVALID_SENSORS);
      }
      break;
    case EVENT_RELEASED:
      FlowTimeout.stop();
      break;
    default:
      Shutdown(
        //  1234567890123456
        // FF("LM FTL! Chk SENS" ),
        ERR_INV_LOWMARK_STATUS);
  }

}

void btn_HighMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
// #if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace( FF(CR "HighMark: %i %x" CR), f_HighMark, event);
// #endif 
	//f_HighMark = 0;

  //  must be pressed before
  switch (event)
  {
    case EVENT_PRESSED:
      // вода начала кончаться
			// игнорим
			lcd_on(true);
			lcd_status();
			lcd.print(FF("waiting....")); 
			pump = PUMP_STANBDY; // типа выключить экран, но текст не обновлять
      break;
    case EVENT_RELEASED:
      if (pump != PUMP_ON)
      {
        Shutdown(
          //  1234567890123456
          // FF("HM ERR! Chk SENS"),
          ERR_HIGHMARK_UNATTENDED_RELEASE);
      }
      else {
        PumpOff();

        // Shutdown(
        // FF("FULL TANK!" ),
        // FF("Success! :)" ));
      }
      break;
    default:
      Shutdown(
        //  1234567890123456
        // FF("HM FTL! Chk SENS"),
        ERR_INV_HIGHMARK_STATUS);
  }
}

void btn_Overflow(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
// #if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace(FF(CR "OVERFLOW: %i %x" CR), f_Overflow, event);
// #endif

  fv_pump.owfl++;

  // any change is dangerous!!!
  Shutdown(
    //  1234567890123456
    // FF("OVRFLW DETECTED!" ),
    ERR_OVERFLOW);
}

//***********************************************************************************************************************************************************

volatile uint8_t portBhistory = 0x00;     // default is high because the pull-up
volatile uint8_t portChistory = 0x00;
volatile uint8_t portDhistory = 0x00;     // не pool-up
volatile long t = 0;

ISR(PCINT2_vect)
{
  // t = millis();
  uint8_t changedbits;

  //PCIMSK2
  changedbits = PIND ^ portDhistory;
  portDhistory = PIND;

  if (changedbits & (1 << PIN_OVERFLOW))
  {
    SETFLAG(f_Overflow);
  }

  if (changedbits & (1 << PIN_HIGHMARK))
  {
    SETFLAG(f_HighMark);
  }

  if (changedbits & (1 << PIN_LOWMARK))
  {
    SETFLAG(f_LowMark);
  }

}
/* 
ISR(PCINT0_vect)
{
  t = millis();
  uint8_t changedbits;

  // PCIMSK0
  changedbits = PINB ^ portBhistory;
  portBhistory = PINB;

   // if (changedbits & (1 << PIN_OVERFLOW))
   // {
     // SETFLAG(f_Overflow);
   // }
  
   // if (changedbits & (1 << PIN_HIGHMARK))
   // {
     // SETFLAG(f_HighMark);
   // }
  
   // if (changedbits & (1 << PIN_LOWMARK))
   // {
     // SETFLAG(f_LowMark);
   // }

}
 */
 
 /* 
ISR(INT1_vect)
{

}
 */

#ifdef WatchDog_h
//volatile uint8_t  = 0;
ISR(WDT_vect) {
#ifdef WatchDog_h
  if (Wdog1.active())   {
    if (WatchDog::ovfCounter < WatchDog::ovfTop) {
      WatchDog::ovfCounter++;
    } else {
      WatchDog::ovfCounter = 0;
      Wdog1.isrCallback();
    }
  } else
#endif
    wdt_disable();
  //if (++f_WDT == 0) ++f_WDT;
  SETFLAG(f_WDT);
}
#endif

void log_internal_state(int l_cnt)
{
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)
    unsigned long e1 =   FlowTimeout.elapsed();
    unsigned long e2 =   TankTimeout.elapsed();
		unsigned long e3 =   LCD_Timeout.elapsed();
		unsigned long e4 =   StatDelay.elapsed();
						
    Log.trace( FF("l_cnt = %d, t = %l f_WDT %i, f_Overflow %i, f_LowMark %i, f_HighMark %i, " CR
									"        portBhistory %X, portChistory %X, portDhistory %X, " CR
									"        FlowTimeout = %c %l, TankTimeout = %c %l, LCD_Timeout = %c %l, StatDelay = %c %l" CR),
               l_cnt, t, f_WDT, f_Overflow, f_LowMark, f_HighMark,
               portBhistory, portChistory, portDhistory, 
							 sw( FlowTimeout.isRunning(), 'R') , e1, sw( TankTimeout.isRunning(), 'R'), e2, 
							 sw( LCD_Timeout.isRunning(), 'R'), e3, sw( StatDelay.isRunning(), 'R'), e4
		);
#endif
}


//***********************************************************************************************************************************************************


void setup() 
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println(FF(CR "In the begining...") );

  // lcd.init();
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd_on(true);
  lcd.print(FF("Starting..."));
  //  lcd.setCursor(0,1);
  //  lcd.print("Starting...");

	STR_sd_CR = FF("%s (line:%d)" CR);
	STR_SXd_CR = FF("%S %X (line:%d)" CR);
	
  Log.begin(LOG_LEVEL, &Serial);
  Log.notice( FF(CR "******************************************" CR) );                     // Info string with Newline

#ifdef WatchDog_h
	Log.notice( FF("WatchDog::init(A_Watchdog, OVF_8000MS, STOP);" CR) );
  WatchDog::init(A_Watchdog, OVF_8000MS, STOP);
#endif
	
	// write_flash(0); // только один раз для платы
	read_flash();

  pinMode(SW_MOTOR, OUTPUT);
  pinMode(SW_VALVE, OUTPUT);
	
  PumpOff();

  lcd_status();
  lcd.print( FF("READY!!!") );
	StatDelay.start();

	PORTD |= (1 << PORTD5) | (1 << PORTD6) | (1 << PORTD6);
  
  // разрешение прерываний INT0 и INT1
  //  EIMSK  =  (1<<INT0)  | (1<<INT1);
	
  // настройка срабатывания прерываний на любому изменению
  //EICRA  =  (0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (1<<ISC00);
	
  // разрешение прерываний с портов B (PCINT[7:0]) и D (PCINT[23:16]), и запрет с порта C (PCINT[14:8])
  PCICR  |= (1 << PCIE2) | (0 << PCIE1) | (0 << PCIE0);
	
  // маскирование всех ног, кроме PB0 и PD7 - по одной на PCINT0 и PCINT2
  //PCMSK0 |= (0 << PCINT7)  | (0 << PCINT6)  | (0 << PCINT5)  | (0 << PCINT4)  | (0 << PCINT3)  | (0 << PCINT2)  | (0 << PCINT1)  | (1 << PCINT0);
  //PCMSK1 |=                (0 << PCINT14) | (0 << PCINT13) | (0 << PCINT12) | (0 << PCINT11) | (0 << PCINT10) | (0 << PCINT9)  | (0 << PCINT8);
  PCMSK2 |= (1 << PCINT23) | (1 << PCINT22) | (1 << PCINT21) | (0 << PCINT20) | (0 << PCINT19) | (0 << PCINT18) | (0 << PCINT17) | (0 << PCINT16);

}

//***********************************************************************************************************************************************************
uint8_t l_cnt = 0;
#define CNT_DIV 3

void loop() 
{
	char s_time[20];
	pump_last_time += 3;
  pump_last_volume = SPEED_LPS * pump_last_time;
	
	fv_pump.cnt++;
	fv_pump.total_time += 13;
	
	sprintf(buf, "%s %04u", 
					sprintTime4(s_time, pump_last_time),
					pump_last_volume
					);
	// lcd.print(buf);
	Log.notice( STR_sd_CR, buf, __LINE__);
	
	lcd_statistic();
	delay(100);

	
/* 
#ifdef LowPower_h
	// дремать если мотор выключен
  if ( (pump == PUMP_OFF || pump == PUMP_STANBDY) && 
			!LCD_Timeout.isRunning() &&
			!StatDelay.isRunning() &&
			!TankTimeout.isRunning() &&
			!FlowTimeout.isRunning()
		  )
  {
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
    Log.verbose(FF("Sleep %d" CR), __LINE__);
#endif		
		delay(10);	// чтобы байтики через serial прошли ...перед сном
		LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
    //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_ON, TWI_OFF);
    //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
    Log.verbose(FF("wakeup %d" CR), __LINE__);
#endif		
  }
#else
	delay(100); // если нет LowPower
#endif

	l_cnt++;

  if ( StatDelay.hasPassed(STAT_DELAY, true) )
  {
		// log_internal_state(l_cnt);
					
		if (pump == PUMP_ON) 
		{
			lcd_status();
			// дописать текущий тайминг
		}
  }


  // if (0 & f_WDT) 
  // {
    // Shutdown(
      // //  1234567890123456
      // // FF("WATCHDOG ALERT!" ),
      // ERR_WATHCDOG);
  // }



	if (LCD_Timeout.hasPassed(LCD_TIMEOUT)) 
	{
		// if (pump == PUMP_OFF)
		{
			// lcd_off:
			lcd.clear();
			lcd_status();
			lcd.print(("READY! cnt=")); lcd.print(fv_pump.cnt);
			pump = PUMP_STANBDY;		
		}
		
		LCD_Timeout.stop();
		LCD_Timeout.add( -LCD_Timeout.elapsed() );
		lcd.setBacklight(0);
			

		Shutdown(STANDBY);

	}
	else if ((pump == PUMP_OFF) && LCD_Timeout.hasPassed(LCD_STAT_TIME))
	{
		pump = PUMP_STANBDY;
		lcd_statistic();
	}


  if (FlowTimeout.hasPassed(LOWMARK_TIMEOUT)) 
  {
    Shutdown(
      //  1234567890123456
      // FF("NO FLOW! Chk WTR" ),
      ERR_NO_FLOW);
  }

  if (TankTimeout.hasPassed(HIGHMARK_TIMEOUT)) 
  {
    Shutdown(
      //  1234567890123456
      // FF("Tank Timout!" ),
      ERR_TANK_TIMEOUT);
  }

  // put your main code here, to run repeatedly:
 */
 
  sw_Overflow.loop();
  sw_HighMark.loop();
  sw_LowMark.loop();

  if (sw_LowMark.pressed() && !sw_HighMark.pressed()) 
  {
    Shutdown(
      //  1234567890123456
      // FF("Sensors error!" ),
      ERR_INVALID_SENSORS);
  }

}
