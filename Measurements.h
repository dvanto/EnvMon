#ifndef	_MEASUREMENTS_H_
#define _MEASUREMENTS_H_

// #define DEBUG_WAKEUPS
//#define DISABLE_LOGGING

#include <limits.h>
#include <WString.h>

#include <EEPROM.h>
#include <avr/EEPROM.h>
#include <avr/pgmspace.h>

#include <ArduinoLog.h>             //  ..\libraries\ArduinoLog\ArduinoLog.h	https://github.com/thijse/Arduino-Log/
#include <RTCx.h>
#include <Rtttl.h>

//#define FF(a) 	(a)		// для отладки, переключение на строки в RAM
#define FF(a)       F(a)


struct Measurements
{  
private:
	RTCx*			_rtc;
	
	uint16_t		_index			= 0;
	uint16_t		_length			= EEPROM.length();
	
	uint8_t			_data_size		= sizeof(DataHeader);
	const uint8_t	_align 			= 16;

	int				_sensor_num		= 0;
	char*			_sens_types		= NULL;
	int 			_NAN 			= INT_MIN;
	
	bool			_configSaved	= false;
	bool			_writeConfig	= false;
	bool			_endOfEPPROM	= false;
	
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
	
	Measurements(RTCx *__rtc = NULL): _rtc (__rtc?__rtc:&rtc)
		// , _index(0)
		// , _length(EEPROM.length())
		// , _sens_types(NULL)
		// , _writeConfig(false)
		// , _endOfEPPROM(false)
		// , _data_size( sizeof(DataHeader) )
		// , data()
	{
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

			eeprom_read_block(data.data, dataOffset(), data.sensor_num*sizeof(*data.data));
			for(int i=data.sensor_num; i < data.datalen; data.data[i++] = INT_MIN); // очистить хвост
			
		}
		align();

		Log.trace( FF(" %X " CR), _index);
		
		return &data;
	}
	
	
	bool	write()	// true if need to flash from EEPROM, next save will overwrite data
	{
		if (endOfEPPROM()) return true;
		
		// если нулевая запись - сохраняем конфиг
		// if (_index == 0 && !_writeConfig && writeConfig()) return true;
		if (!_configSaved && !_writeConfig && writeConfig()) return true;
		
		Log.trace( FF("write %X %t %X " ), _index, _writeConfig, dataOffset());
		_writeConfig = false;
		
		// пишем заголовок
		// перед сохраненнием должен быть вызван prepare()
		EEPROM.put( _index, (DataHeader&) data );
		
		// пишем данные только по числу сенсоров
		eeprom_update_block(data.data, dataOffset(), data.sensor_num*sizeof(*data.data));
		
		{
			Log.verbose(FF(CR "write check " CR));
			for(int i=0; i < data.sensor_num; i++)
				Log.verbose(FF("%x "), data.data[i] ); 
			Log.verbose(FF(CR "verify " CR));
			eeprom_read_block(data.data, dataOffset(), data.sensor_num*sizeof(*data.data));
			for(int i=0; i < data.sensor_num; i++)
			Log.verbose(FF("%x "), data.data[i] ); 
		}

		align();
		
		Log.trace( FF(" %X " CR), _index);
	
		return endOfEPPROM();
	}
	
	void flush()
	{
		// переключиться на начало памяти
		_index = 0;
		_endOfEPPROM = false;
		
		Serial.println();
		Serial.println();
		while ( read() )
		{
			_rtc->printIsotime(Serial, data.timestamp);
						
			if (data.data)
				for(int i=0; i < data.sensor_num; i++) 
				{
					Serial.print(',');
					Serial.print(data.data[i]);
				}
			else
				Serial.print(",nodata");
				
			Serial.println();
		}
		
		_index = 0;
		_endOfEPPROM = false;
		if (!_configSaved) writeConfig();
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
		{
			expandData(max_sensors);
		}
		if (max_sensors > data.sensor_num)
		{
			expandSensors(max_sensors);
		}
	}
	
private:

	void readConfig()
	{
		if (_sens_types) delete _sens_types;
		
		_sensor_num = data.sensor_num;
		_sens_types = new typeof(_sens_types)[_sensor_num];
		
		for(unsigned char i=0; i < _sensor_num;  i++) _sens_types[i] = data.data[i];
		_configSaved = true;
		
		Log.notice( FF("Measurements.readConfig = %d" CR), _sensor_num);

	}

	bool writeConfig()
	{
		_writeConfig = true;
		prepare(false);
		for(unsigned char i=0; i < data.datalen; i++) data.data[i] = _sens_types[i];
		_configSaved = true;
		
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
	
	void expandSensors(unsigned char size)
	{
		typeof(_sens_types) new_sens_types = new typeof(_sens_types)[ size ];
		if (_sens_types)
		{
			unsigned char i;
			for(i=0; i < _sensor_num; i++) new_sens_types[i] = _sens_types[i];
			for(; i < size; new_sens_types[i++] = _NAN);
			delete _sens_types;
		}
		_sens_types = new_sens_types;
		_sensor_num = size;
		_configSaved = false;
	}	
	
	inline void* dataOffset() { return (void*)(_index+sizeof(DataHeader)); }
		
};




#endif // _MEASUREMENTS_H_
