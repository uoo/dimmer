// configurables
#define USE25            1          // use all 25 I/O pins for 24 dimmers
#define USESERIAL        1          // serial port control (lose 2 dimmers)
#define NCONSEC          5          // consecutive reads for noise cancelling
#define ZEROPIN          5          // pin to read for zero crossing
#ifdef USE25
#define MAXPIN          24          // maximum valid pin
#else
#define MAXPIN          23          // maximum valid pin
#endif

#define BPS             115200      // serial bits per second

// port indices for circuit I/O calculations
#define BOFF            0           // port B index
#define COFF            1           // port C index
#define DOFF            2           // port D index
#define FOFF            3           // port F index
#define EOFF            4           // port E index
#define NPORTS          5           // number of ports
#define END             0xff        // end of data marker value

// Renard protocol
// http://www.doityourselfchristmas.com/wiki/index.php?title=Renard
#define RENARD_PAD      0x7d        // Renard pad byte - ignore
#define RENARD_SYNC     0x7e        // begin of a Renard packet
#define RENARD_ESCAPE   0x7f        // Renard escape sequence
#define RENARD_THIS     0x80        // this controller address
#define ESCAPE_OFFSET   (RENARD_ESCAPE - 0x31)

// communication state machine
#define STATE_NONE      0           // no state yet
#define STATE_PIN       1           // reading pin number
#define STATE_BRIGHT    2           // reading brightness value
#define STATE_COMMAND   3           // waiting for Renard command
#define STATE_RENARD    4           // waiting for Renard data

// stream flags
#define STREAM_NONE     0           // no command in progress
#define STREAM_USB      1           // USB command in progress
#define STREAM_SERIAL   2           // serial command in progress

// constants
#define NBITS            8          // bits in a byte
#define BADDIGIT        -1          // invalid digit character

// macros
#define BIT(n)          (1 << n)    // convert bit to bitmask

// 8333us (100000 / 120) per halfcycle
// 32.6 us per 1/255 brightness
// so to convert from us to brightness, divide by 32.6
// we approximate this faster by right shifting 5 bits

#define DIFF2COUNT(n)   (n >> 5)    // convert time difference to dim count

// circuit description structure
struct circuit {
    unsigned char   dim;
    unsigned char   off;
    unsigned char   portBit;
};

// define each dimmer output
static struct circuit  circuits[] = {
                            // port A pin
    { 0, BOFF, BIT(0) },    // PB0  0  2
    { 0, BOFF, BIT(1) },    // PB1  1  3
    { 0, BOFF, BIT(2) },    // PB2  2  4
    { 0, BOFF, BIT(3) },    // PB3  3  5
    { 0, BOFF, BIT(7) },    // PB7  4  6
    { 0, DOFF,     0  },    // PD0  5  7 used as input as it is also INT0
    { 0, DOFF, BIT(1) },    // PD1  6  8
#ifdef  USESERIAL
    { 0, DOFF,     0  },    // PD2  7  9    serial RX
    { 0, DOFF,     0  },    // PD3  8 10    serial TX
#else
    { 0, DOFF, BIT(2) },    // PD2  7  9
    { 0, DOFF, BIT(3) },    // PD3  8 10
#endif
    { 0, COFF, BIT(6) },    // PC6  9 11
    { 0, COFF, BIT(7) },    // PC7 10 12
    { 0, DOFF, BIT(6) },    // PD6 11 13
    { 0, DOFF, BIT(7) },    // PD7 12 14
    { 0, BOFF, BIT(4) },    // PB4 13 15
    { 0, BOFF, BIT(5) },    // PB5 14 16
    { 0, BOFF, BIT(6) },    // PB6 15 17
    { 0, FOFF, BIT(7) },    // PF7 16 18
    { 0, FOFF, BIT(6) },    // PF6 17 19
    { 0, FOFF, BIT(5) },    // PF5 18 20
    { 0, FOFF, BIT(4) },    // PF4 19 21
    { 0, FOFF, BIT(1) },    // PF1 20 22
    { 0, FOFF, BIT(0) },    // PF0 21 22
    { 0, DOFF, BIT(4) },    // PD4 22 end
    { 0, DOFF, BIT(5) },    // PD5 23 end
#ifdef USE25
    { 0, EOFF, BIT(6) },    // PE6 24 inner
#endif
    { 0, END, 0 }
};

// global/static variables
static boolean          escape   = false;               // Renard escape seq
static unsigned char    pin      = 0;                   // pin accumulator
static unsigned char    bright   = 0;                   // brightness accum
static unsigned char    state    = STATE_NONE;          // comms state machine
static unsigned char    stream   = STREAM_NONE;         // current cmd stream
volatile unsigned char  nhigh    = 0;                   // noise canceller count
static HardwareSerial   Uart     = HardwareSerial();    // serial port
volatile unsigned long  zeroTime = 0;                   // last zero cross time

// called once at startup - initialize

void
setup()
{
    unsigned char       ddrBits[NPORTS];
    struct circuit *    cptr;
    
    // configure interrupt on zero crossing detection
    attachInterrupt(ZEROPIN, zero, RISING);
    
    // initialize comms
    Serial.begin(BPS);  // USB speed is actually fixed
    Serial1.begin(BPS);

    // configure dimmer control pins as outputs 
    for (cptr = circuits; cptr->off != END; ++cptr) {
        ddrBits[cptr->off] |= cptr->portBit;
    }
    
    DDRB = ddrBits[BOFF];
    DDRC = ddrBits[COFF];
    DDRD = ddrBits[DOFF];
    DDRF = ddrBits[FOFF];
#ifdef USE25
    DDRE = ddrBits[EOFF];
#endif

#ifndef USESERIAL
    stream = STREAM_USB;
#endif

    // announce ourself
    Serial.println("AVR Dimmer");
#ifdef USESERIAL
    Serial1.println("AVR Dimmer");
#endif
}

// called continuously - actual operation loop

void
loop()
{
    // check if pulse is stable
    if (nhigh < NCONSEC) {
        // do noise cancelling counting
        if (digitalRead(ZEROPIN)) {
            // (still) high, increment counter
            ++nhigh;
            
            if (nhigh >= NCONSEC) {
                // stable zero crossing pulse, start dimmer timing
                zeroTime = micros();
                // clear outputs
                PORTB    = 0xff;
                PORTC    = 0xff;
                PORTD    = 0xff;
                PORTF    = 0xff; 
#ifdef USE25
                PORTE    = 0xff;
#endif
            }
        } else {
            // pulse unstable, reset counter
            nhigh = 0;
        }
    } else {
        // dimmer cycle underway
        dodim();
    }
    
    docomms();
}

// implement dimming

static void
dodim()
{
    unsigned char       portBits[NPORTS];   // output accumulators
    unsigned long       lastZero;           // non-volatile zero crossing copy
    unsigned long       nonce;              // non-volatile current time copy
    unsigned long       counts;             // dimmer counts
    struct circuit *    cptr;               // circuit pointer
    
    lastZero = zeroTime;    // non-volatile copy of zero crossing time
    nonce    = micros();    // non-volatile copy of current time
    
    if (lastZero > nonce) {
        // whoops, updated zero crossing after "now"
        // avoid wraparound subtraction
        return;
    }
    
    // calculate current dimmer count
    counts = DIFF2COUNT(nonce - lastZero);
    
    // calculate updated port bits

    for (cptr = circuits; cptr->off != END; ++cptr) {
        if (counts > cptr->dim) {
            portBits[cptr->off] |= cptr->portBit;
        }
    }

    // copy current port bits to output ports

    PORTB = ~portBits[BOFF];
    PORTC = ~portBits[COFF];
    PORTD = ~portBits[DOFF];
    PORTF = ~portBits[FOFF];
#ifdef USE25
    PORTE = ~portBits[EOFF];
#endif
}

// deal with communications

static void
docomms()
{
#ifdef USESERIAL
    if (stream != STREAM_USB) {
        stream = STREAM_SERIAL;
    
        while (Serial1.available()) {
            processChar(Serial1.read());
        }
        
        if (state == STATE_NONE) {
            stream = STREAM_NONE;
        }
    }

    if (stream != STREAM_SERIAL) {
        stream = STREAM_USB;

#endif
        while (Serial.available()) {
            processChar(Serial.read());
        }
#ifdef USESERIAL
    
        if (state == STATE_NONE) {
            stream = STREAM_NONE;
        }
    }
#endif
}

static void
processChar(
    int     ch)     // current character
{
    char    digit;  // digit value

    switch (state) {
        case STATE_NONE:
            if (ch == RENARD_SYNC) {
                state = STATE_COMMAND;
                break;
            }
            
            if (ch == RENARD_PAD) {
                break;
            }
            
            state = STATE_PIN;
            /* fall thru */
        case STATE_PIN: // we're looking for pin number digits or =
            if (ch == '=') {
                // got = character, check pin number for validity
                if ((pin > 0) && (pin <= MAXPIN)) {
                    // valid pin number, update state machine
                    state = STATE_BRIGHT;
                } else {
                    // invalid pin number, return error and reset
                    print("bogus pin ");
                    println(pin, DEC);
                    pin = 0;
                }
                break;
            }

            // convert digit to value if valid
            digit = getdigit(ch);

            if (digit == BADDIGIT) {
                // invalid digit, return error and reset
                println("bad pin digit");
                pin = 0;
            } else {
                // valid digit, accumulate value
                pin *= 10;
                pin += digit;
            }        
            break;

        case STATE_BRIGHT:  // we're looking for brightness or newline
            if (ch == '\n') {
                // got newline, echo command
                print("pin ");
                print(pin, DEC);
                print(" bright ");
                println(bright, HEX);
                // update brightness
                circuits[pin].dim = 0xff - bright;
                // reset state machine
                pin               = 0;
                bright            = 0;
                state             = STATE_PIN;
                break;
            }

            // convert digit to value if valid
            digit = gethexdigit(ch);

            if (digit == BADDIGIT) {
                // invalid digit, return error and reset
                println("bad brightness hex");
                pin    = 0;
                bright = 0;
                state  = STATE_PIN;
            } else {
                // valid digit, accumulate value
                bright *= 16;
                bright += digit;
            }
            break;
        
        case STATE_COMMAND:
            if (ch == RENARD_PAD) {
                break;
            }
            
            if (ch < RENARD_THIS) {
                print("unsupported Renard command ");
                println(ch, HEX);
                state = STATE_NONE;
                break;
            }
            
            if (ch == RENARD_THIS) {
                state = STATE_RENARD;
                pin   = 0;
                break;
            }
            
            print("unsupported Renard downstream ");
            println(ch, HEX);
            state = STATE_NONE;
            break;
            
        case STATE_RENARD:
            if (ch == RENARD_ESCAPE) {
                // process following character specially
                escape = true;
                break;
            }
            
            if (ch == RENARD_PAD) {
                // pad byte - ignore
                break;
            }
            
            if (escape) {
                // this is an escape character
                ch    += ESCAPE_OFFSET;
                escape = false;
            }
            
            // update current pin with brightness

            circuits[pin].dim = 0xff - ch;
            
            // have we updated all pins?
            
            if (pin == MAXPIN) {
                state = STATE_NONE;
                println("OK");
            }
            
            // increment pin
            
            ++pin;
            break;
    }
}

// get decimal digit value from character

static char
getdigit(
  int  ch)
{
    if ((ch >= '0') && (ch <= '9')) {
        // it's a valid digit, return value
        return(ch - '0');
    }

    // invalid digit, return error
    return(BADDIGIT);
}

// get hexadecimal digit value from character

static char
gethexdigit(
  int  ch)
{
    if ((ch >= 'a') && (ch <= 'f')) {
        // valid lowercase hex digit, return value
        return(ch + 10 - 'a');
    }

    if ((ch >= 'A') && (ch <= 'F')) {
        // valid uppercase hex digit, return value
        return(ch + 10 - 'A');
    }

    // it's either a valid decimal digit or not
    return(getdigit(ch));
}

// called at rising edge of zero crossing pulse

void
zero()
{
    // reset pulse width counter
    nhigh = 0;
}

void
print(
    const char *    msg)
{
    if (stream == STREAM_SERIAL) {
        Serial1.print(msg);
        return;
    }

    Serial.print(msg);
}

void
println(
    const char *    msg)
{
    if (stream == STREAM_SERIAL) {
        Serial1.println(msg);
        return;
    }

    Serial.println(msg);
}

void
print(
    int n,
    int type)
{
    if (stream == STREAM_SERIAL) {
        Serial1.print(n, type);
        return;
    }

    Serial.print(n, type);
}

void
println(
    int n,
    int type)
{
    if (stream == STREAM_SERIAL) {
        Serial1.println(n, type);
        return;
    }

    Serial.println(n, type);
}
