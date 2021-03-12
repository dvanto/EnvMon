/* 
	Логер температуры и влажности 
		на DHT11 + термисторе 
		
		
todo
	глубоко спать
	периодически просыапться
	просыпаться по кнопке
	писать в лог реже , если изменения малы
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
#define		LOG_LEVEL			LOG_LEVEL_NOTICE
// #define LOG_LEVEL			LOG_LEVEL_TRACE
// #define LOG_LEVEL			LOG_LEVEL_VERBOSE

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
#define		LCD_TIMEOUT		2 // s

#define		BTN_DELAY		1700

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


// #include <avr/wdt.h>
// #include <Chrono.h>
// #include <LiquidCrystal_PCF8574.h>
// #include <LiquidCrystal_I2C_OLED.h>
// #include <FlashStorage.h>



/* 
void _P_global_vars() {} 
*/
//*******************************************************************************************

Measurements	msr;

DHT				dht(DHT_DPIN, DHT11);
THERMISTOR		temp_sens(NTC_APIN,        // Analog pin
                      10000,          // Nominal resistance at 25 ºC
                      3950,           // thermistor's beta coefficient
                      10100);


// SensorKit s_fire(2, 12);
SensorKit s_light(3, 11);
// SensorKit s_light(0, 0);


// DebounceEvent		btn_Up		= DebounceEvent(BUTTON_UP_DPIN , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
// DebounceEvent		btn_Down	= DebounceEvent(BUTTON_DOWN_DPIN , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
LongPressBtn		btn_Up		= LongPressBtn(BUTTON_UP_DPIN , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
LongPressBtn		btn_Down	= LongPressBtn(BUTTON_DOWN_DPIN , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
volatile uint8_t	f_btn_up=0;
volatile uint8_t	f_btn_down=0;
// bool				btn_d_wait_longpress=false;
// bool				btn_u_wait_longpress=false;


LCD_misc		lcd(LCD_PORT, LCD_TIMEOUT); // экран с выключением подсветки по таймеру

char			buf[LCD_WIDTH + 1];
char			buf2[LCD_WIDTH + 2];

Clockwise		clockwise;

FSTR(snd_Barbie,	"Barbie girl:d=4,o=5,b=125:8g#,8e,8g#,8c#6,a,p,8f#,8d#,8f#,8b,g#,8f#,8e,p,8e,8c#,f#,c#,p,8f#,8e,g#,f# ");
FSTR(snd_BootUp,	"snd_BootUp:d=10,o=6,b=180,c,e,g");
// FSTR(snd_BootUp,	"snd_BootUp:d=10,o=6,b=180,c,e,g");
FSTR(snd_howmuch,	"howmuch:d=4,o=6,b=50:16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16a#5,16d#6,16d#6,32c#6,16c.6,16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16c6,16g#5,8a#.5,16c6,16c#6,16d#6,8f6,16f.6,16f#.6,16d#6,8f.6,16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16a#5,16d#6,16d#6,32c#6,16c.6,16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16c6,16g#5,16a#.5");
FSTR(snd_mi,		"MissionImp:d=16,o=6,b=95:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,a#5,8c,2p,32p,a#5,g5,2f#,32p,a#5,g5,2f,32p,a#5,g5,2e,d#,8d"  );

Alarm			a1(BEEPER_DPIN);
Alarm			a2(BEEPER_DPIN);

int				lowest_alarm_temp		= 550;	// 55c
int				highest_alarm_temp 		= 900;	// 90C

volatile		uint8_t f_WDT = 0;
//*******************************************************************************************

/* 
void _P_ShutdownMode() {} // просто отметка в редакторе
*/
//******************************************************************************************
volatile uint8_t portDhistory = 0x00;     // не pool-up

ISR (PCINT2_vect)
{
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

  // lcd.init();
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd.on(true);
  Log.begin(LOG_LEVEL, &Serial);

  lcd.printP( PSTR("Starting..." __TIME__ " " __DATE__ ), __LINE__);

  dht.begin();

  // s_fire.begin();
  s_light.begin();
  msr.findFreeSpace();
  msr.addSensors(3);
  

#ifdef WatchDog_h
  Log.notice( FF("WatchDog::init(A_Watchdog, OVF_8000MS, STOP);" CR) );
  WatchDog::init(A_Watchdog, OVF_8000MS, STOP);
#endif

  // write_flash(0); // только один раз для платы
  //	read_flash();
  //	fv_pump.pwr++;


  
	a1.alarm(snd_BootUp);
	a1.armed(3, snd_howmuch);
	a2.armed(3, snd_mi);
	
	pciSetup(BUTTON_UP_DPIN);
	pciSetup(BUTTON_DOWN_DPIN);
	
	btn_Down.setLongpressDelay(BTN_DELAY);
	btn_Up.setLongpressDelay(BTN_DELAY);
	
}

//***********************************************************************************************************************************************************

static int pos = 0;
int d = 0;

void update_lcd()
{
  lcd.setBacklight(pos);
  lcd.setCursor(0, 0);
  lcd.print( ((d < 0) ? " << " : "    ") );
  lcd.print( pos );
  lcd.print( ((d > 0) ? " >> " : "    ") );
}

void update_lcd(SensorKit &s)
{
  //  lcd.setCursor(0, 1);
	int v = s.read();
	bool st = s.status();
	char sts[] = "_ ";

	Log.notice( FF("s=%X v=%d status=%d" CR), &s, v, st );

	lcd.print( (*sts=sw(st),sts) );
	lcd.print( v );
	lcd.print( " ");
}
//***********************************************************************************************************************************************************


void loop()
{
  
	switch (btn_Up.loop() )
	{
	case EVENT_PRESSED:
	
		Log.notice( FF("btn_Up.loop() == EVENT_PRESSED" CR) );
		break;
		
	case EVENT_LONGPRESS:
		Log.notice( FF("btn_Up.loop() longpress" CR));
		break;
	}

	switch (btn_Down.loop() )
	{
	case EVENT_PRESSED:
	
		Log.notice( FF("btn_Down.loop() == EVENT_PRESSED" CR) );
		break;
		
	case EVENT_LONGPRESS:
		Log.notice( FF("btn_Down.loop() longpress" CR));
		break;
	}


	
	if (lcd.timeout().hasPassed(LCD_TIMEOUT))
	{
		lcd.timeout().restart();

		// протирка данных сенсоров в INT_MIN
		msr.prepare();
		
		//Считываем влажность и температуру
		float h = dht.readHumidity();
		float t = dht.readTemperature();
		
		// Проверка удачно прошло ли считывание.
		if (isnan(h) || isnan(t)) {
			Log.notice(FF("Не удается считать показания" CR));
			buf[0] = buf2[0] = '_';
			buf[1] = buf2[1] = 0;
		}
		else {
			Log.notice (FF("Humidity: %s%% Temperature: %s*C " ), printDecF( buf, msr[0] = h*10 ), printDecF( buf2, msr[1] = t*10 ) );
		}

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
			msr[2] = temp_sens.read();
			Log.notice (FF(" NTC = %s " CR), printDecF( buf, msr[2]) );
			
			lcd.print("N: ");
			lcd.print( buf );
		} 
		
		// update_lcd(s_fire);
		// update_lcd(s_light);
		
		lcd.setCursor(15, 1);
		lcd.print( clockwise.getNext() );
		
		// msr.write();
		if (msr.write())  
		{
			// msr.flush();
		}
		
		if ( msr[2] < lowest_alarm_temp ) 
		{
			a1.alarmOnce();
		}
		else
			a1.armed(3);
			
		if (msr[2] > highest_alarm_temp)
		{
			a2.alarmOnce();
		}	
		else
			a2.armed(3);
			// while(1) delay(1000);
		// lcd.print( " " );
	}
  

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


