#define BIT_GET(a, b) (a & _BV(b))
#define BIT_SET(a, b) (a |= _BV(b))
#define BIT_CLEAR(a, b) (a &= ~_BV(b))

#define RADIO_ON BIT_SET(PORT, PIN_RADIO)
#define RADIO_OFF BIT_CLEAR(PORT, PIN_RADIO)

#define LED_ON BIT_SET(PORT, PIN_LED)
#define LED_OFF BIT_CLEAR(PORT, PIN_LED)

#define RADIO_ON BIT_SET(PORT, PIN_RADIO)
#define RADIO_OFF BIT_CLEAR(PORT, PIN_RADIO)
