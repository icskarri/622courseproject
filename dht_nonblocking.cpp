#include "dht_nonblocking.h"

#define DHT_IDLE                  0
#define DHT_BEGIN_MEASUREMENT     1
#define DHT_BEGIN_MEASUREMENT_2   2
#define DHT_DO_READING            3
#define DHT_COOLDOWN              4

#define COOLDOWN_TIME  2000

DHT_nonblocking::DHT_nonblocking( uint8_t pin, uint8_t type )
	: _pin( pin ),
	  _type( type ),
	  _bit( digitalPinToBitMask( pin ) ),
	  _port( digitalPinToPort( pin ) ),
	  _maxcycles( microsecondsToClockCycles( 1000 ) )
{
  dht_state = DHT_IDLE;

  pinMode( _pin, INPUT );
  digitalWrite( _pin, HIGH );
}

bool DHT_nonblocking::measure( float *temperature, float *humidity )
{
  if( read_nonblocking( ) == true )
  {
    *temperature = read_temperature( );
    *humidity    = read_humidity( );
    return( true );
  }
  else
  {
    return( false );
  }
}

float DHT_nonblocking::read_temperature( ) const
{
  int16_t value;
  float   to_return;

  switch( _type )
  {
  case DHT_TYPE_11:
    value = data[ 2 ];
    to_return = (float) value;
    break;

  case DHT_TYPE_21:
  case DHT_TYPE_22:
    value = ( data[ 2 ] & 0x7f ) << 8;
    value |= data[ 3 ];
    if( ( data[ 2 ] & 0x80 ) != 0 )
    {
      value = -value;
    }
    to_return = ( (float) value ) / 10.0;
    break;

  default:
    to_return = NAN;
    break;
  }

  return( to_return );
}

float DHT_nonblocking::read_humidity( ) const
{
  uint16_t value;
  float   to_return;

  switch( _type )
  {
  case DHT_TYPE_11:
    value =  data[ 0 ];
    to_return = (float) value;
    break;

  case DHT_TYPE_21:
  case DHT_TYPE_22:
    value =  data[ 0 ] << 8;
    value |= data[ 1 ];
    to_return = (float)value / 10.0;
    break;

  default:
    to_return = NAN;
    break;
  }

  return( to_return );
}

uint32_t DHT_nonblocking::expect_pulse(bool level) const
{
  uint32_t count = 0;
  #ifdef __AVR
    uint8_t portState = level ? _bit : 0;
    while ((*portInputRegister(_port) & _bit) == portState) {
      if (count++ >= _maxcycles) {
        return 0;
      }
    }
  #else
    while (digitalRead(_pin) == level) {
      if (count++ >= _maxcycles) {
        return 0;
      }
    }
  #endif

  return count;
}

bool DHT_nonblocking::read_nonblocking( )
{
  bool status = false;
  switch( dht_state )
  {
  case DHT_IDLE:
    dht_state = DHT_BEGIN_MEASUREMENT;
    break;

  case DHT_BEGIN_MEASUREMENT:
    digitalWrite( _pin, HIGH );
    data[ 0 ] = data[ 1 ] = data[ 2 ] = data[ 3 ] = data[ 4 ] = 0;
    dht_timestamp = millis( );
    dht_state = DHT_BEGIN_MEASUREMENT_2;
    break;

  case DHT_BEGIN_MEASUREMENT_2:
    if( millis( ) - dht_timestamp > 250 )
    {
      pinMode( _pin, OUTPUT );
      digitalWrite( _pin, LOW );
      dht_timestamp = millis( );
      dht_state = DHT_DO_READING;
    }
    break;

  case DHT_DO_READING:
    if( millis( ) - dht_timestamp > 20 )
    {
      dht_timestamp = millis( );
      dht_state = DHT_COOLDOWN;
      status = read_data( );

    }
    break;

  case DHT_COOLDOWN:
    if( millis( ) - dht_timestamp > COOLDOWN_TIME )
    {
      dht_state = DHT_IDLE;
    }
    break;

  default:
    break;
  }

  return( status );
}

bool DHT_nonblocking::read_data( )
{
  uint32_t cycles[ 80 ];

  {
    volatile DHT_interrupt interrupt;

    digitalWrite( _pin, HIGH );
    delayMicroseconds( 40 );

    pinMode( _pin, INPUT );
    delayMicroseconds( 10 );

    if( expect_pulse( LOW ) == 0 )
    {
      return( false );
    }
    if( expect_pulse( HIGH ) == 0 )
    {
      return( false );
    }

    for( int i = 0; i < 80; i += 2 )
    {
      cycles[ i     ] = expect_pulse( LOW );
      cycles[ i + 1 ] = expect_pulse( HIGH );
    }
  }

  for( int i = 0; i < 40; ++i )
  {
    uint32_t low_cycles  = cycles[ 2 * i     ];
    uint32_t high_cycles = cycles[ 2 * i + 1 ];
    if( ( low_cycles == 0 ) || ( high_cycles == 0 ) )
    {
      return( false );
    }
    data[ i / 8 ] <<= 1;
    if( high_cycles > low_cycles )
    {
      data[ i / 8 ] |= 1;
    }
  }

  if( data[ 4 ] == ( ( data[ 0 ] + data[ 1 ] + data[ 2 ] + data[ 3 ]) & 0xFF ) )
  {
    return( true );
  }
  else
  {
    return( false );
  }
}

