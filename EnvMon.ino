
// #define DEBUG_WAKEUPS
//#define DISABLE_LOGGING

#include <limits.h>
#include <WString.h>
// #include <avr/wdt.h>
#include <LowPower.h>               //  https://github.com/rocketscream/Low-Power

#include <EEPROM.h>
#include <avr/EEPROM.h>
#include <avr/pgmspace.h>

#include <ArduinoLog.h>             //  ..\libraries\ArduinoLog\ArduinoLog.h	https://github.com/thijse/Arduino-Log/
#include <RTCx.h>
#include <Rtttl.h>

#include <DebounceEvent.h>          // https://github.com/xoseperez/debounceevent
//#define INLINE_SENSORKIT
//#define TINY_SENSORKIT
#include <SensorKit.h>
#include <DHT.h>
#include <thermistor.h>

#include <misctools.h>
// #include <Chrono.h>
// #include <LiquidCrystal_PCF8574.h>
// #include <LiquidCrystal_I2C_OLED.h>

// #include <FlashStorage.h>


#define DHT_DPIN 8 // номер пина, к которому подсоединен датчик DTH
#define NTC_APIN	3

DHT dht(DHT_DPIN, DHT11);
THERMISTOR 	temp_sens(NTC_APIN,        // Analog pin
                      10000,          // Nominal resistance at 25 ºC
                      3950,           // thermistor's beta coefficient
                      10100);

// DebounceEvent sw_Encoder = DebounceEvent(3 , BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
// SensorKit s_fire(2, 12);
//SensorKit s_light(3, 11);
// SensorKit s_light(0, 0);

//*******************************************************************************************
/* настройки экранчика */
#define		LCD_PORT 						0x27
#define		LCD_WIDTH						16
#define		LCD_TIMEOUT         2 // s
// LigthChrono 		LCD_Timeout();
// LCD_misc	lcd(LCD_PORT, &LCD_Timeout, LCD_TIMEOUT); // экран с выключением подсветки по таймеру
LCD_misc	lcd(LCD_PORT, LCD_TIMEOUT); // экран с выключением подсветки по таймеру

char		buf[LCD_WIDTH + 1];
char		buf2[LCD_WIDTH + 2];

//*******************************************************************************************

void _P_defines() {} // просто отметка в редакторе

//  ..\libraries\ArduinoLog\ArduinoLog.h
#define LOG_LEVEL			LOG_LEVEL_NOTICE
// #define LOG_LEVEL			LOG_LEVEL_VERBOSE

#define SETFLAG(x) ( ++(x)?(x):(++(x)) )  // если увеличение дало 0, то увеличиваем еще раз

typedef unsigned long	t_time;

//const char LOG_AS[] PROGMEM =

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
struct t_flash_data {
  uint16_t				sign;
  
};

t_flash_data				fv_data 							= {0};
// t_flash_var_pump				fv_saved;




//******************************************************************************************

void Shutdown(t_ShutdownMode mode);
void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, fchar s1 = NULL); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, const char* s1 ); //, const char*  s2,

/* void log_internal_state(); */

//******************************************************************************************

#define FLASH_SIGNATURE	0xADB0
#define FLASH_OFFSET		0x20*1

// Stream &operator (&int);

struct Measurements
{  
private:
	RTCx*	_rtc;
	
	uint16_t	_index;
	uint16_t	_length;
	
	const uint8_t		_align = 16;
	uint8_t		_data_size;

	int		_sensor_num;
	char*	_sens_types;
	int 	_NAN = INT_MIN;
	
	bool	_writeConfig;
	bool	_endOfEPPROM;
	
public:
	struct DataHeader {
		RTCx::tm 	timestamp;
		int			sensor_num; // <0 то определяет структуру -sensor_num - кол-во сенсоров, далее их тип (TBD)
		
		DataHeader(): sensor_num(0) {}
	};
		
	struct Data: public DataHeader {
		int			datalen;
		int* 		data;
		
		Data(): DataHeader(), datalen(0), data(NULL) {}
	} data;
	
	Measurements(RTCx *__rtc = NULL)
		: _rtc (__rtc?__rtc:&rtc)
		, _index(0)
		, _length(EEPROM.length())
		, _sens_types(NULL)
		, _writeConfig(false)
		, _endOfEPPROM(false)
		, _data_size( sizeof(DataHeader) )
		, data()
	{
		// data.datalen = 0;
		// data.data = NULL ;
	}
	
	bool	findFreeSpace()
	{
		while ( read() );
		
		Log.notice( FF("Measurements.findFreeSpace = %X %d" CR), _index, _index);
		
		return !endOfEPPROM();

	}
	
	void	prepare(bool clear_all=true)
	{
		_rtc->readClock(data.timestamp);
		// _rtc->printIsotime( Serial, data.timestamp );
		data.sensor_num = _sensor_num;
		
		// чистим все данные
		if (clear_all)
			for(char i=0; i < data.datalen; data.data[i++] = _NAN);
	}
	

	Data*	read() 	// NULL if read fail
	{
		if (endOfEPPROM()) return NULL;
		
		Log.trace( FF("read %X " ), _index);
		
		EEPROM.get( _index, (DataHeader&) data );
				
		{
			char i=0;	
			for (char* a=(char*)&data.timestamp; i < sizeof(RTCx::tm) && a[i] != 0xFF; i++) 
				Log.verbose( FF(" %x " ), a[i]);

			if (i < 8) // начало структуры не должно содержать -1
				return NULL;
				
			if (data.sensor_num == -1 )  // тоже фигня
				return NULL;
		}
		
		if ( data.sensor_num < 0 )
		{
			data.sensor_num *= -1;
			// read configuration
			readConfig();
		}
		else if ( data.sensor_num > 0 )
		{
			// read data	
			if ( data.datalen < data.sensor_num )
				expandData(data.sensor_num);

			eeprom_read_block(data.data, dataOffset(), data.datalen*sizeof(*data.data));
			
		}
		align();

		Log.trace( FF(" %X " CR), _index);
		
		return &data;
	}
	
	
	bool	write()	// true if need to flash from EEPROM, next save will overwrite data
	{
		if (endOfEPPROM()) return true;
		
		// если нулевая запись - сохраняем конфиг
		if (_index == 0 && !_writeConfig && writeConfig()) return true;
		
		Log.trace( FF("write %X %t " ), _index, _writeConfig);
		_writeConfig = false;
		
		// пишем заголовок
		// перед сохраненнием должен быть вызван prepare()
		EEPROM.put( _index, (DataHeader&) data );
		
		// пишем данные только по числу сенсоров
		eeprom_update_block(data.data, dataOffset(), data.sensor_num*sizeof(*data.data));

		align();
		
		Log.trace( FF(" %X " CR), _index);
	
		return endOfEPPROM();
	}
	
	void flush()
	{
		// переключиться на начало памяти
		_index = 0;
		_endOfEPPROM = false;
		writeConfig();
		
	}
	
	void init()
	{
		if (rtc.autoprobe()) {
			// Found something, hopefully a clock.
			Serial.print("Autoprobe found ");
			Serial.print(rtc.getDeviceName());
			Serial.print(" at 0x");
			Serial.println(rtc.getAddress(), HEX);
		}
		else {
			// Nothing found at any of the addresses listed.
			Serial.println("No RTCx found, cannot continue");
			while (1)
				;
		}

		// Enable the battery backup. This happens by default on the DS1307
		// but needs to be enabled on the MCP7941x.
		rtc.enableBatteryBackup();

		// rtc.clearPowerFailFlag();

		// Ensure the oscillator is running.
		rtc.startClock();

		if (rtc.getDevice() == RTCx::MCP7941x) {
			Serial.print("Calibration: ");
			Serial.println(rtc.getCalibration(), DEC);
			// rtc.setCalibration(-127);
		}

		rtc.setSQW(RTCx::freq4096Hz);

		resetEEPROM();
		while(1) delay(1000);
	}
	
	void resetEEPROM()
	{
		for( uint16_t i = 0 ; i < EEPROM.length(); EEPROM[i++] = -1);
		_index = 0;
		_endOfEPPROM = false;
	}
	
	inline int& operator []( const int idx ) 
	{ 
		// if (!data.data) return _NAN;
		// Log.notice(FF( "MSR[%d] = %d " CR), idx, data.data[idx]);
		return data.data?data.data[idx]:_NAN; 
	}
	
	inline unsigned char sensNum() { return _sensor_num; }
	inline uint16_t offset() { return _index; }
	void addSensors(unsigned char max_sensors) 
	{ 
		if (max_sensors > data.datalen)
			expandData(max_sensors);
	}
	
private:

	void readConfig()
	{
		if (_sens_types) delete _sens_types;
		
		_sensor_num = data.sensor_num;
		_sens_types = new typeof(_sens_types)[_sensor_num];
		
		for(unsigned char i=0; i < _sensor_num;  i++) _sens_types[i] = data.data[i];

		Log.notice( FF("Measurements.readConfig = %d" CR), _sensor_num);

	}

	bool writeConfig()
	{
		_writeConfig = true;
		prepare(false);
		for(unsigned char i=0; i < data.datalen; i++) data.data[i] = _sens_types[i];
		return write();

	}
	
	bool endOfEPPROM()
	{
		if (!_endOfEPPROM && _index + _data_size > _length ) 
		{
			_endOfEPPROM = true;
				
			Log.verbose( FF(CR CR "Measurements.endOfEPPROM i=%X i=%d h=%X sn=%X sd=%X EL=%X " CR), _index, _index, sizeof(DataHeader), data.sensor_num, sizeof(*data.data), _length);
		}


		return _endOfEPPROM;
	}
	
	void align()
	{	
		Log.verbose( FF(CR "Measurements.align _index = %X sn=%X h=%X a=%X ~a=%X " ), _index, data.sensor_num, sizeof(DataHeader), (_align-1), ~(_align-1));
		
		_index += sizeof(DataHeader) + data.sensor_num*sizeof(*data.data) + (_align-1);
		Log.verbose( FF(" %X " ), _index);	
		
		_index &= ~(_align-1);
		
		Log.verbose( FF(" %X " CR), _index);

	}
	
	// unsigned char alignedNum(unsigned char max_idx)
	
	
	void expandData(unsigned char size)
	{
		// sizeof(DataHeader) + data.sensor_num*sizeof(*data.data)
		_data_size = sizeof(DataHeader) + size*sizeof(*data.data);
	
		unsigned char aligned_size = (_data_size + (_align-1)) & ~(_align-1);
		Log.verbose( FF("Measurements.alignedNum h=%X s=%X sd=%X a=%X as=%X ~a=%X " ), sizeof(DataHeader), size, sizeof(*data.data), (_align-1), aligned_size, ~(_align-1));
		aligned_size = (aligned_size-sizeof(DataHeader))/sizeof(*data.data);
		Log.verbose( FF("Measurements.alignedNum i=%X d=%X fs=%X h=%X" CR), size, aligned_size, aligned_size*2+sizeof(DataHeader), sizeof(DataHeader));
	
		int* newdata = new int[ aligned_size ];
		if (data.data)
		{
			unsigned char i;
			for(i=0; i < data.datalen; i++) newdata[i] = data.data[i];
			for(; i < aligned_size; newdata[i++] = _NAN);
			delete data.data;
		}
		
		Log.notice( FF("Measurements.expandData = %d -> %d -> %d" CR), data.datalen, size, aligned_size);
		
		data.datalen = aligned_size;
		data.data = newdata;
		
	}
	
	inline void* dataOffset() { return (void*)(_index+sizeof(DataHeader)); }
		
};
Measurements msr;


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
  // Log.notice( FF(CR "******************************************" CR) );                     // Info string with Newline

  lcd.printP( PSTR("Starting..."  __DATE__ " " __TIME__), __LINE__);

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

		//Считываем влажность

		float h = dht.readHumidity();

		// Считываем температуру

		float t = dht.readTemperature();

		// протирка данных сенсоров в INT_MIN
		msr.prepare();
		
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
		
		msr.write();
		// if (msr.write())  while(1) delay(1000);
		// lcd.print( " " );
	}
  

}




