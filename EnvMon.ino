
void _P_defines() {} // просто отметка в редакторе
//*******************************************************************************************

//  ..\libraries\ArduinoLog\ArduinoLog.h
#define		LOG_LEVEL			LOG_LEVEL_NOTICE
// #define LOG_LEVEL			LOG_LEVEL_TRACE
// #define LOG_LEVEL			LOG_LEVEL_VERBOSE

// #define		DEBUG_WAKEUPS
//#define		DISABLE_LOGGING

// digital pins (_DPINS)
#define		DHT_DPIN	8 // номер пина, к которому подсоединен датчик DTH
#define		BEEPER_DPIN	2

// analog pins (_APINS)
#define		NTC_APIN	3 // пин термистора

// настройки  SensorKit
//#define		INLINE_SENSORKIT
//#define		TINY_SENSORKIT

/* настройки экранчика */
#define		LCD_PORT 						0x27
#define		LCD_WIDTH						16
#define		LCD_TIMEOUT         2 // s


void _P_includes() {} // просто отметка в редакторе
//*******************************************************************************************

#include <limits.h>
#include <WString.h>

#include <LowPower.h>               //  https://github.com/rocketscream/Low-Power

#include <EEPROM.h>
#include <avr/EEPROM.h>
#include <avr/pgmspace.h>

#include <ArduinoLog.h>             //  ..\libraries\ArduinoLog\ArduinoLog.h	https://github.com/thijse/Arduino-Log/

#include <DebounceEvent.h>          // https://github.com/xoseperez/debounceevent

#include <SensorKit.h>
#include <DHT.h>
#include <thermistor.h>

#include <misctools.h>
#include "Measurements.h"

// #include <avr/wdt.h>
// #include <Chrono.h>
// #include <LiquidCrystal_PCF8574.h>
// #include <LiquidCrystal_I2C_OLED.h>
// #include <FlashStorage.h>



void _P_global_vars() {}
//*******************************************************************************************

DHT dht(DHT_DPIN, DHT11);
THERMISTOR 	temp_sens(NTC_APIN,        // Analog pin
                      10000,          // Nominal resistance at 25 ºC
                      3950,           // thermistor's beta coefficient
                      10100);

// DebounceEvent sw_Encoder = DebounceEvent(3 , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
// SensorKit s_fire(2, 12);
//SensorKit s_light(3, 11);
// SensorKit s_light(0, 0);


LCD_misc	lcd(LCD_PORT, LCD_TIMEOUT); // экран с выключением подсветки по таймеру

char		buf[LCD_WIDTH + 1];
char		buf2[LCD_WIDTH + 2];


FSTR(snd_Barbie, "Barbie girl:d=4,o=5,b=125:8g#,8e,8g#,8c#6,a,p,8f#,8d#,8f#,8b,g#,8f#,8e,p,8e,8c#,f#,c#,p,8f#,8e,g#,f# ");
FSTR(snd_BootUp, "snd_BootUp:d=10,o=6,b=180,c,e,g");
FSTR(snd_howmuch, "howmuch:d=4,o=6,b=50:16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16a#5,16d#6,16d#6,32c#6,16c.6,16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16c6,16g#5,8a#.5,16c6,16c#6,16d#6,8f6,16f.6,16f#.6,16d#6,8f.6,16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16a#5,16d#6,16d#6,32c#6,16c.6,16d#6,32d#6,32c#6,16c6,8c#6,8a#5,16c6,16g#5,16a#.5");

Alarm a1(BEEPER_DPIN);
Alarm a2(BEEPER_DPIN);

int	lowest_alarm_temp		= 550;	// 55c
int	highest_alarm_temp 		= 900;	// 90C

volatile uint8_t f_WDT = 0;
//*******************************************************************************************


void _P_ShutdownMode() {} // просто отметка в редакторе
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

  // lcd.init();
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd.on(true);
  Log.begin(LOG_LEVEL, &Serial);

  lcd.printP( PSTR("Starting..." __TIME__ " " __DATE__ ), __LINE__);

  dht.begin();

  // s_fire.begin();
  // s_light.begin();
  msr.findFreeSpace();
  msr.addSensors(3);
  

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
	a1.alarm(snd_BootUp);
	a1.armed(3, snd_howmuch);
	a2.armed(3);
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

  Serial.print("v:");
  Serial.print(v);
  Serial.print(" status:");
  Serial.println(st);

  lcd.print( (*sts=sw(st),sts) );
  lcd.print( v );
  lcd.print( " ");
}
//***********************************************************************************************************************************************************
Clockwise clockwise;

void loop()
{
  
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
			Log.notice (FF("Humidity: %s%% Temperature: %s*C" CR), printDecF( buf, msr[0] = h*10 ), printDecF( buf2, msr[1] = t*10 ) );
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


