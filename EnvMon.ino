/* 
	Логер температуры и влажности 
		на DHT11 + термисторе 
		
		
todo
	+ глубоко спать
	+ периодически просыапться
	+ просыпаться по кнопке
	+ писать в лог реже , если изменения малы
	подключить RTC
	подключить mSD и EEPROM
	повесить конденсатор на питание
	сделать внешнее питание от батарейки
	Читать по wifi другие датчики

*/



/* 
void _P_defines() {} // просто отметка в редакторе
*/
//*******************************************************************************************

//  ..\libraries\ArduinoLog\ArduinoLog.h
// #define		LOG_LEVEL			LOG_LEVEL_NOTICE
#define		LOG_LEVEL			LOG_LEVEL_TRACE
// #define		LOG_LEVEL			LOG_LEVEL_VERBOSE

// #define		DEBUG_WAKEUPS
//#define		DISABLE_LOGGING

// digital pins (_DPINS)
#define		DHT_DPIN			8 // номер пина, к которому подсоединен датчик DTH
#define		BEEPER_DPIN			2
#define		BUTTON_UP_DPIN		4
#define		BUTTON_DOWN_DPIN	5

// analog pins (_APINS)
#define		NTC_APIN	3 // пин термистора

// настройки  SensorKit
//#define		INLINE_SENSORKIT
//#define		TINY_SENSORKIT

/* настройки экранчика */
#define		LCD_PORT		0x27
#define		LCD_WIDTH		16
#define		LCD_TIMEOUT		10 // s

// задержка долгого нажатия
#define		BTN_DELAY		1700

// задержка интервала сна
#define		SLEEP_DELAY		20 // s

// частота опроса датчиковы
#define		MSR_TIMEOUT		4
#define		MSR_DELTA_LIMIT	10  // отклонение на градус
#define		MSR_TIME_LIMIT	(3600UL*2*10)	//  3600 сек * (2 градуса * 10)  - сохранять раз в час, если отклоненния меньше 2х градучовы


/* 
void _P_includes() {} // просто отметка в редакторе
*/
//*******************************************************************************************

#include <limits.h>
#include <WString.h>

#include <LowPower.h>               //  https://github.com/rocketscream/Low-Power

#include <EEPROM.h>
#include <avr/EEPROM.h>
#include <avr/pgmspace.h>

#include <ArduinoLog.h>             //  ..\libraries\ArduinoLog\ArduinoLog.h	https://github.com/thijse/Arduino-Log/

#include <DebounceEvent.h>          // https://github.com/xoseperez/debounceevent


#include "Measurements.h"
#include <SensorKit.h>
#include <DHT.h>
#include <thermistor.h>

#include <misctools.h>

#include <WatchDog.h>

// #include <avr/wdt.h>
// #include <Chrono.h>
// #include <LiquidCrystal_PCF8574.h>
// #include <LiquidCrystal_I2C_OLED.h>
// #include <FlashStorage.h>



/* 
void _P_global_vars() {} 
*/
//*******************************************************************************************
struct	Config
{
	int			lowest_alarm_temp;	//		= 550;	// 55c
	int			highest_alarm_temp;	// 		= 900;	// 90C
	int			lcd_timeout;	//			= LCD_TIMEOUT;
	int			save_interval;	//			= SLEEP_DELAY;
	int			_reserve[4];
};

Measurements	msr( NULL, sizeof(Config));
Chrono			msr_timeout(Chrono::SECONDS);
t_time			old_ts = 0;
int				old_msr[20];

DHT				dht(DHT_DPIN, DHT11);
THERMISTOR		temp_sens(NTC_APIN,        // Analog pin
                      10000,          // Nominal resistance at 25 ºC
                      3950,           // thermistor's beta coefficient
                      10100);


// SensorKit s_fire(2, 12);
// SensorKit s_light(3, 11);
// SensorKit s_light(0, 0);


// DebounceEvent		btn_Up		= DebounceEvent(BUTTON_UP_DPIN , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
// DebounceEvent		btn_Down	= DebounceEvent(BUTTON_DOWN_DPIN , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
LongPressBtn		btn_Up		= LongPressBtn(BUTTON_UP_DPIN , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
LongPressBtn		btn_Down	= LongPressBtn(BUTTON_DOWN_DPIN , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
volatile uint8_t	f_btn_up=0;
volatile uint8_t	f_btn_down=0;
// bool				btn_d_wait_longpress=false;
// bool				btn_u_wait_longpress=false;




Clockwise		clockwise;


FSTR(snd_Barbie,	"Barbie girl:d=4,o=5,b=125:8g#,8e,8g#,8c#6,a,p,8f#,8d#,8f#,8b,g#,8f#,8e,p,8e,8c#,f#,c#,p,8f#,8e,g#,f# ");
FSTR(snd_BootUp,	"snd_BootUp:d=10,o=6,b=180:c,e,g");
// FSTR(snd_BootUp,	"snd_BootUp:d=10,o=6,b=180,c,e,g");
FSTR(snd_alarm,		"alarm:d=8,o=6,b=180:c,b,c,b,c,b,c,b,c,b,c,b,2p,c,b,2p,c,b,2p,c,b,2p,c,b,2p,c,b,2p,c,b,2p,c,b,c,b,c,b,c,b,c,b,c,b");
FSTR(snd_howmuch,	"howmuch:d=4,o=6,b=50:16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16a#5,16d#6,16d#6,32c#6,16c.6,16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16c6,16g#5,8a#.5,16c6,16c#6,16d#6,8f6,16f.6,16f#.6,16d#6,8f.6,16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16a#5,16d#6,16d#6,32c#6,16c.6,16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16c6,16g#5,16a#.5");
FSTR(snd_mi,		"MissionImp:d=16,o=6,b=95:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,a#5,8c,2p,32p,a#5,g5,2f#,32p,a#5,g5,2f,32p,a#5,g5,2e,d#,8d"  );

Alarm			a1(BEEPER_DPIN);
Alarm			a2(BEEPER_DPIN);


enum InputMode	{
	NONE,
	CHOOSE,
	CHG_LOWEST,
	ITEM_FIRST = CHG_LOWEST,
	CHG_HIGHEST,
	CHG_LCD_TIMEOUT,
	CHG_SAVE_INTERVAL,
	ITEM_LAST = CHG_SAVE_INTERVAL,
}				inputMode	= NONE;
int				inputItem	= 0;

enum InputCommand	{
	CMD_NONE,
	CMD_UP,
	CMD_DOWN,
	CMD_SEL,
	CMD_EXIT
}				inputCmd;

Chrono			inputTimeout(Chrono::SECONDS);

/* struct	Config{
	int			lowest_alarm_temp		= 650;	// 55c
	int			highest_alarm_temp 		= 900;	// 90C
	int			lcd_timeout				= LCD_TIMEOUT;
	int			save_interval			= SLEEP_DELAY;
	int			_reserve[4];
}; */

Config			cfg = { 550, 900, LCD_TIMEOUT, SLEEP_DELAY };
int				cfg_limits[][ITEM_LAST-ITEM_FIRST+1]	= {
					// lowest,	highest,	lcd_timeout,	save_interval
					{	0,		500,		5,				10	},		// low limit
					{	1500,	3000,		30,				32000	},	// high limit
					{	10,		10,		 	1,				1	},		// output divider
					{	10,		10,		 	1,				10	}		// step
				};
				
int				*cfg_item				= (int*)&cfg;
bool			cfg_changed 			= false;

/* 
const char menu0[] PROGMEM = "choice0";
const char menu1[] PROGMEM = "choice1";
const char menu2[] PROGMEM = "choice2";
const char menu3[] PROGMEM = "choice3";
const char menu4[] PROGMEM = "choice4";
const char menu5[] PROGMEM = "choice5";

const char *menu[] PROGMEM = {menu0, menu1, menu2, menu3, menu4, menu5};
 */

const char		itemText[] PROGMEM = 
		"Low limit:      " "\x0" 
		"High limit:     " "\x0" 
		"LCD timeout:    " "\x0" 
		"M.interv:       " "\x0";
#define			ITEMTEXT_SIZE	17	
#define			ITEMTEXT		(itemText+inputItem*ITEMTEXT_SIZE)
/* 
		// lcd.print( (ITEMTEXT) );			//	так не работает
		lcd.print( FPSTR(ITEMTEXT) );		//	-норм!
		lcd.printP( (ITEMTEXT), __LINE__);	//	-норм!
		// Log.notice( FF("inputItem =%d itemText = %s	>>>>>=%S" CR), inputItem, FPSTR(ITEMTEXT), ITEMTEXT );				%s не работает
		Log.notice( FF("inputItem =%d itemText =%S" CR), inputItem, ITEMTEXT );			//	-норм!
		Serial.println( FPSTR(ITEMTEXT) );	//	-норм!
 */
 
LCD_misc		lcd(LCD_PORT, cfg.lcd_timeout); // экран с выключением подсветки по таймеру

char			buf[LCD_WIDTH + 1];
char			buf2[LCD_WIDTH + 1];
char			buf3[LCD_WIDTH + 1];

volatile		uint8_t f_WDT = 0;
//*******************************************************************************************

/* 
void _P_ShutdownMode() {} // просто отметка в редакторе
*/
//******************************************************************************************

ISR (PCINT2_vect)
{
	volatile uint8_t portDhistory = 0x00;     // не pool-up
	uint8_t changedbits;

	changedbits = PIND ^ portDhistory;
	portDhistory = PIND;

	if (changedbits & (1 << BUTTON_UP_DPIN))
	{
		SETFLAG(f_btn_up);
	}

	if (changedbits & (1 << BUTTON_DOWN_DPIN))
	{
		SETFLAG(f_btn_down);
	}

}

volatile bool f_dog = false;
void dog()
{
	f_dog = true;
	Log.verbose( FF("gav-gav-gav" CR) );
}

void update_lcd(char cw=0);

//******************************************************************************************


typedef enum : char { POWEROFF = 0,
                      STANDBY,
                      ABORT = -127,
                      ERR_FLASH_RESET, // должен быть последним
                      ERR_LAST

                    } t_ShutdownMode;



//******************************************************************************************

void Shutdown(t_ShutdownMode mode);
void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);

//******************************************************************************************


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
  lcd.print(buf, __LINE__ + mode * 1000);
  lcd.setCursor(0, 1);
  lcd.print(buf2, __LINE__ + mode * 1000);

  //lcd_on(true);
}

//******************************************************************************************



// уснуть - до следующего поплавка (POWEROFF) или навсегда
void _shutdown(t_ShutdownMode mode = POWEROFF)
{
#ifdef DEBUG
  return;
#endif

  if (mode == POWEROFF)
  {
    // выключить свет и все
    lcd.setBacklight(0);
  }
  else {
    // FULL Shutdown !!!
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



//***********************************************************************************************************************************************************


void setup()
{
#ifndef DISABLE_LOGGING
	Serial.begin(115200);
	while (!Serial);
	// Serial.println(FF(CR "In the begining...") );
#endif
	Log.begin(LOG_LEVEL, &Serial);

	// lcd.init();
	lcd.begin(16, 2);
	lcd.noAutoscroll();
	lcd.on(true);
	lcd.printP( PSTR("Starting..." __TIME__ " " __DATE__ ), __LINE__);

	

	// Инциализация платы
	// EEPROM.put(0, cfg); while(1);
	
	// EEPROM.get(0, cfg);
	Log.notice( FF(CR "config la=%d ha=%d lt=%d si=%d" CR CR) , cfg.lowest_alarm_temp, cfg.highest_alarm_temp, cfg.lcd_timeout, cfg.save_interval);
	
	a1.alarm(snd_BootUp);
	a1.armed(1, snd_howmuch);
	// a2.armed(3, snd_mi);
	a2.armed(1, snd_alarm);
	
	pciSetup(BUTTON_UP_DPIN);
	pciSetup(BUTTON_DOWN_DPIN);
	
	btn_Down.setLongpressDelay(BTN_DELAY);
	btn_Up.setLongpressDelay(BTN_DELAY);
	
	dht.begin();

	// s_fire.begin();
	// s_light.begin();
	msr.findFreeSpace();
	msr.addSensors(3);

	msr_timeout.stop();
	if ( update_msr() )
		write_msr();

	inputTimeout.stop();
	

/*	
	msr_timeout.restart();
	
 	while (1)
	{
		Log.notice( FF("msr_timeout r=%d p=%d" CR) , msr_timeout.isRunning(), msr_timeout.hasPassed(2));
		
		if (msr_timeout.hasPassed(2))
		{
			for(int i=0; i<10; i++)
			{
				Log.notice( FF("msr_timeout r=%d p=%d" CR) , msr_timeout.isRunning(), msr_timeout.hasPassed(2));
			}
			
			msr_timeout.stop();
			for(int i=0; i<10; i++)
			{
				Log.notice( FF("msr_timeout r=%d p=%d" CR) , msr_timeout.isRunning(), msr_timeout.hasPassed(2));
			}
			
			while (1);

		}
	}
	 */
	Log.notice( FF("Wdog1.init(dog, %d);" CR), SLEEP_DELAY);
	Wdog1.init(dog, cfg.save_interval*1000);
}


//***********************************************************************************************************************************************************


void loop()
{
	Log.verbose( FF("lcd.on() fd=%d w=%d u=%d d=%d" CR) , f_dog, Wdog1.status(), f_btn_up, f_btn_down);
	
  	if ( Wdog1.status() && !f_dog) // таймер собаки активен и не сработал, т.е. ...
	{
		// проснулись по кнопке
		// if ( (f_btn_up||f_btn_down) ) 
		// if ( !lcd.timeout().isRunning())
		if ( (f_btn_up||f_btn_down) && !lcd.timeout().isRunning())
		{
			lcd.on(START);
			lcd.timeout().restart();
			Log.verbose( FF("lcd.on() fd=%d w=%d u=%d d=%d" CR) , f_dog, Wdog1.status(), f_btn_up, f_btn_down);
			update_lcd();
		}
		else
		{
			// нахрена проснулись?
			// пошли оратно спатьы
		}
		// 
	}
	else
	{		
		f_dog=0;//
		
		Log.trace( FF("WATCHDOG >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" CR) );
		
		// тупо вышел таймаут
		if ( update_msr() )
			write_msr();
	
		
		Wdog1.init(dog, cfg.save_interval*1000, STOP);
	}

	if (a1.updateMelody() || a2.updateMelody() )	// пока играет меловдия подсвечивать экран
	{
		lcd.timeout().restart();
	}
	else
	if (lcd.timeout().isRunning() && lcd.timeout().hasPassed(LCD_TIMEOUT))	// потухнуть, если вышел таймаут
	{
		Log.trace( FF("lcd.timeout().hasPassed(LCD_TIMEOUT)" CR) );
		lcd.setBacklight(LOW);
		lcd.timeout().stop();
		
		if (cfg_changed)
		{
			// EEPROM.put(0, cfg);
			Log.notice( FF(CR "FAKE config saved la=%d ha=%d lt=%d si=%d >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" CR CR) , 
						cfg.lowest_alarm_temp, cfg.highest_alarm_temp, cfg.lcd_timeout, cfg.save_interval);
		}
	}

	if (lcd.timeout().isRunning())
	{		
		if (f_btn_up||f_btn_down)
		{	
			Log.notice( FF("f_btn_up=%d f_btn_down=%d" CR) , f_btn_up, f_btn_down);
			
			lcd.timeout().restart();
			f_btn_up = f_btn_down = 0;
		}
		
		char e1, e2;
		
		if ( input(
				e1=btn_Up.loop()	| (btn_Up.pressed()		?EVENT_YETPRESSED:EVENT_NONE), 
				e2=btn_Down.loop()	| (btn_Down.pressed()	?EVENT_YETPRESSED:EVENT_NONE) 
			) ) // || inputTimeout.isRunning())
		{
			Log.verbose( FF("update_input_lcd  e1=%d e2=%d itк=%d" CR) , e1, e2, inputTimeout.isRunning());
			update_input_lcd();
		}
		else
		{
			Log.verbose( FF("LCD.ON UPDATE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" CR) );
			if (update_msr()) 	
				update_lcd();
		}

			
		// if ( e1 ) update_cw('0'+e1);
		// if ( e2 ) update_cw('5'+e2);
	
	}
	else 
	{
		Log.trace( FF("Wdog1.start() and sleep" CR) );
		// msr_timeout.stop();
		Wdog1.start();
		delay(500);
		LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
		
	}

}

//***********************************************************************************************************************************************************

static int pos = 0;
int d = 0;

inline int abs_f( int a ) { return a>0?a:-a; }

void update_lcd(char cw)// cw=0)
{
	lcd.clear(0);
	lcd.setCursor(0, 0);
	{
		lcd.print( "T: ");
		lcd.print( buf2 );
		lcd.print( " H: ");
		lcd.print( buf );
		lcd.print( "         ");
	}

	lcd.setCursor(0, 1);

	{
		lcd.print("N: ");
		lcd.print( buf3 );
	} 
	
	// update_lcd(s_fire);
	// update_lcd(s_light);
	
	update_cw( cw );
	
}


void update_cw(char cw)
{
	lcd.setCursor(15, 1);
	lcd.print( cw?cw:clockwise.getNext() );	
}


void update_lcd(SensorKit &s)
{
   // lcd.setCursor(0, 1);
	int v = s.read();
	bool st = s.status();
	char sts[] = "_ ";

	Log.notice( FF("s=%X v=%d status=%d" CR), &s, v, st );

	lcd.print( (*sts=sw(st),sts) );
	lcd.print( v );
	lcd.print( " ");
}

bool update_msr()
{
	// msr_timeout(MSR_TIMEOUT);
	
	if ( msr_timeout.isRunning() )
	{
		if (!msr_timeout.hasPassed(MSR_TIMEOUT, true)) 
			return false;
			
		Log.trace(FF("update_msr msr_timeout.isRunning() and hasPassed=%d" CR), msr_timeout.elapsed());
	}
	else
		msr_timeout.restart();

	// протирка данных сенсоров в INT_MIN
	msr.prepare();
	
	//Считываем влажность и температуру
	float h = dht.readHumidity();
	float t = dht.readTemperature();
	
	// Проверка удачно прошло ли считывание.
	if (isnan(h) || isnan(t)) 
	{
		Log.notice(FF("Не удается считать показания" CR));
		buf[0] = buf2[0] = '_';
		buf[1] = buf2[1] = 0;
	}
	else 
	{
		Log.notice (FF("Humidity: %s%% Temperature: %s*C " ), printDecF( buf, msr[0] = h*10 ), printDecF( buf2, msr[1] = t*10 ) );
	}
	msr[2] = temp_sens.read();
	Log.notice (FF(" NTC = %s     " CR), printDecF( buf3, msr[2]) );
		
	// check_alarm();
	
	return true;
}

void write_msr()
{

	int msr_need_save = 0;
	int msr_delta = 0;
	
	for (int i=0; i< msr.data.datalen; i++)
	{
		msr_need_save += ( ( msr_delta += abs_f( old_msr[i] - msr[i] ) / MSR_DELTA_LIMIT) );
	}
	
	Log.notice( FF("update_msr(); need to save = %d d=%d ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" CR) , msr_need_save, msr_delta);

	if ( msr_need_save || (old_ts - millis())/1000 * msr_delta > MSR_TIME_LIMIT )
	{
		if ( msr.write() )  
		{
			// msr.flush();
		}
		
		old_ts = millis();
		for (int i=0; i< msr.data.datalen; i++)
		{
			old_msr[i] = msr[i] ;
		}	
	}
	
	check_alarm();
	update_lcd('W');
	
}


void check_alarm()
{
	Log.trace( FF("check_alarm(); =%d" CR) , msr[2]);
	if ( msr[2] < cfg.lowest_alarm_temp ) 
	{
		lcd.on(START);
		update_lcd();
	Log.notice( FF("		a1.alarmOnce()" CR) );
		a1.alarmOnce(NULL, false);
	}
	else
		a1.armed(3);
		
	if (msr[2] > cfg.highest_alarm_temp)
	{
		lcd.on(START);
		update_lcd();
	Log.notice( FF("		a2.alarmOnce()" CR) );
		a2.alarmOnce(NULL, false);
	}	
	else
		a2.armed(3);
}

void setInputStep( int& step, int scaler)
{
	if (abs_f(scaler) <= 100)
		step = 10;
	else if (abs_f(scaler) <= 900)
		step = 100;
	else
		step = 900;
}						
				
Chrono	input_delay(Chrono::MILLIS);	

bool input(char e_up, char e_down)
{
	volatile bool	need_rest_longpress = false;
	inputCmd = CMD_NONE;
	
	bool	e_up_yp = (e_up & EVENT_YETPRESSED);
	bool	e_down_yp = (e_down & EVENT_YETPRESSED);
	e_up &= ~EVENT_YETPRESSED;
	e_down &= ~EVENT_YETPRESSED;
	
	if ( e_up_yp && e_down_yp)
	{
		inputCmd = CMD_EXIT;
		need_rest_longpress = true;
	}
	else
	{
		switch ( e_up ) 
		{
			case EVENT_LONGPRESS:	
				if (need_rest_longpress) break; 
				if (inputMode<=CHOOSE) 
				{
					inputCmd = CMD_SEL;	
					need_rest_longpress = true;
					break;
				}
				else
					if (!input_delay.hasPassed(250)) 
						break;;

			case EVENT_RELEASED:	
				if (need_rest_longpress) 
					need_rest_longpress = false;
				else
					inputCmd = CMD_UP;		
				break;
		}
		switch ( e_down ) 
		{
			case EVENT_LONGPRESS:	
				if (need_rest_longpress) break; 
				if (inputMode<=CHOOSE) 
				{
					inputCmd = CMD_EXIT;	
					need_rest_longpress = true;
					break;
				}
				else
					if (!input_delay.hasPassed(250)) 
						break;;
					
			case EVENT_RELEASED:	
				if (need_rest_longpress) 
					need_rest_longpress = false;
				else
					inputCmd = CMD_DOWN;	
				break;
		}
		if ( inputTimeout.isRunning() && inputTimeout.hasPassed(5, inputMode != CHOOSE) )
			inputCmd = CMD_EXIT;
	}
		
	if (inputCmd)
		Log.trace( FF("input(); e1=%d e2=%d iCMD=%d iMODE=%d iITEM=%d CFG_V=%d 				itr=%d ite=%d" CR) , e_up, e_down, inputCmd, inputMode, inputItem, *cfg_item, inputTimeout.isRunning(), inputTimeout.elapsed());
	
	switch( inputCmd )
	{
		case CMD_UP:
			switch (inputMode)
			{
				// case NONE:
				case CHOOSE:
					decItem( inputItem, 0 );
					cfg_item = (int*)&cfg + inputItem;
					break;
					
				case CHG_SAVE_INTERVAL:
					setInputStep(cfg_limits[3][inputItem], *cfg_item+1);	
				case CHG_LOWEST:
				case CHG_HIGHEST:
				case CHG_LCD_TIMEOUT:
					decItem( *cfg_item, cfg_limits[0][inputItem], cfg_limits[3][inputItem]);
					cfg_changed = true;
					break;
			}	
			break;
			
		case CMD_DOWN:
			switch (inputMode)
			{
				// case NONE:
				case CHOOSE:
					incItem( inputItem, ITEM_LAST-ITEM_FIRST);
					cfg_item = (int*)&cfg + inputItem;
					break;
					
				case CHG_SAVE_INTERVAL:
					setInputStep(cfg_limits[3][inputItem], *cfg_item);	
				case CHG_LOWEST:
				case CHG_HIGHEST:
				case CHG_LCD_TIMEOUT:
					incItem( *cfg_item, cfg_limits[1][inputItem], cfg_limits[3][inputItem]);
					cfg_changed = true;
					break;
			}	
			break;
		
		case CMD_SEL:
			switch (inputMode)
			{
				case NONE:
					inputMode = CHOOSE;
					break;

				case CHOOSE:
					inputMode = (InputMode)(inputItem + ITEM_FIRST);
					break;
					
				case CHG_LOWEST:
				case CHG_HIGHEST:
				case CHG_LCD_TIMEOUT:
				case CHG_SAVE_INTERVAL:
					inputMode = CHOOSE;
					break;
					
			}	
			break;
			

		case CMD_EXIT:
			switch (inputMode)
			{
				case CHOOSE:
					inputMode = NONE;
					break;

				case CHG_LOWEST:
				case CHG_HIGHEST:
				case CHG_LCD_TIMEOUT:
				case CHG_SAVE_INTERVAL:
					inputMode = CHOOSE;
					break;
			}	
			break;
			
	}	
	
	Log.verbose( FF("input() returns; iCMD=%d iMODE=%d iITEM=%d CFG_V=%d R=%t ir=%d ie=%d" CR) , inputCmd, inputMode, inputItem, *cfg_item, inputMode != NONE, inputTimeout.isRunning(), inputTimeout.elapsed());

	if ( inputMode == NONE && !inputTimeout.isRunning() ||
		 inputTimeout.isRunning() && inputTimeout.hasPassed(5) )
	{
		inputMode = NONE;
		inputTimeout.stop();
		Log.verbose( FF("return false; inputTimeout.stop(); line=%d" CR), __LINE__);
		return false;

	}		
	
	if (inputCmd)
		Log.trace( FF("input()   returns; iCMD=%d iMODE=%d iITEM=%d CFG_V=%d r=%t 			ir=%d ie=%d" CR) , inputCmd, inputMode, inputItem, *cfg_item, inputMode != NONE, inputTimeout.isRunning(), inputTimeout.elapsed());

	Log.verbose( FF("return true; inputTimeout.restart(); line=%d" CR), __LINE__);
	// delay(1000);
	inputTimeout.restart();
	return true;	
}

Chrono	input_refesh(Chrono::MILLIS);
Chrono	lcd_blink(Chrono::MILLIS);
bool	blink	=	false;

void update_input_lcd()
{
	if (inputCmd == CMD_NONE && !input_refesh.hasPassed(500, true)) return;
	// if (inputCmd == NONE)
		// if (input_refesh.isRunning())
			// if( !input_refesh.hasPassed(500, true)) return;
		// else	
			// input_refesh.restart();

	lcd.clear(1);
	lcd.setCursor(0, 0);
	
	if (lcd_blink.hasPassed(1000, true)) blink != blink;
	
	if (inputMode <= CHOOSE || blink)
	{
		lcd.print( FF("Select  :      \x7E") );
		lcd.setCursor(7, 0);
		lcd.print( inputItem );	
	}
	else
	{
		lcd.print( FF("               \x7E") );
	}
	
	char buf[LCD_WIDTH+1];
	lcd.setCursor(0, 1);
	lcd.print( FPSTR(ITEMTEXT) );
	lcd.setCursor(LCD_WIDTH - sprintf( buf, "%d", *cfg_item/cfg_limits[2][inputItem]) -2, 1 );
	lcd.print( buf );

	Log.notice( FF("update_input_lcd(); iCMD=%d iMODE=%d iITEM=%d is=%S ci=%d" CR) , inputCmd, inputMode, inputItem, ITEMTEXT, *cfg_item);
	
	update_cw( '\x7F' );

}


/* 
#include "ESP8266.h" //  для работы с esp8266
#include <SoftwareSerial.h> //  чтобы добавить больше пинов UART 
#include <math.h> // чтобы высчитать логарифм
 
#define SSID     "Arduino" //  имя вашего wi-fi
#define PASSWORD  "12345678" //пароль вашего wi-fi
void sendWiFi()
{
	wifi.joinAP(SSID, PASSWORD);
	if (wifi.createTCP("www.dweet.io", 80)) { // если нам удалось создать TCP соединение
		String data = "GET /dweet/for/" + name + "?"; // создаем переменную data  в виде строки (заполняем GET-запрос)
		data += "temperatur_C=" + String(temperatur) + " HTTP/1.1\r\n"; 
		data += "Host: dweet.io\r\n\r\n"; // закрываем GET-запрос в строке
		wifi.send(data.c_str(), data.length()); // отправляем данные в массиве (строка с-стиля) и общее количество байтов
		wifi.releaseTCP(); // закрываем TCP соеденение
	}
  
}
 */


