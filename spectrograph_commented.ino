
#include <util/delay_basic.h>

#ifdef ARDUINO_AVR_MEGA2560  // only use the definitions below if we're running on a mega
#define LAMP 0x20   
#define SH 0x40
#define ICG 0x80
#define MCLK 0x10
#else                        // we're not on a mega, so the following defs apply
#define LAMP 0x01     // 0x01 = (bits) 0000 0001
#define SH 0x02       // 0x02 = (bits) 0000 0010
#define ICG 0x04      // 0x04 = (bits) 0000 0100
#define MCLK 0x08     // 0x08 = (bits) 0000 1000
#endif
// These defs mean that, for example, from now on all occurrences
// of LAMP in the code are replaced by the compiler with 0x01.

#define CLOCK PORTB        // all occurrences of CLOCK below are replaced with PORTB,
                           // not sure why he does this, it's not any shorter!
                           // PORTB is a single-byte register. Bits 0-5 map to digital
                           // pins 8,9,10,11,12,13, and the highest two bits are unusable, 
                           // so the register PORTB effectively looks like this:
                           //      x, x, 13, 12, 11, 10, 9, 8
                           // Bit-mangling example: performing logical OR with PORTB and, 
                           // say, the number 4 (hex notation: 0x04, bits: 0000 0100) 
                           // would make sure digital output pin 10 is set to high.
                           // Logical AND with the negation of 4 (~4, bits: 1111 1011) 
                           // would leave the state of all pins untouched except for digital 
                           // pin 10, which will be pulled low.

uint8_t buffer[800];       // declare variable 'buffer' which will hold 800 values of
                           // type uint8_t, i.e. unsigned 8-bit integer  (for the CCD data)
uint8_t avg = 0;
char cmdBuffer[16];
int cmdIndex;
int exposureTime = 20;

void setup()
{
  uint8_t val;

  // Initialize the clocks.
  // So DDRB is another single-byte register whose bits correspond to the same pins
  // as mapped to by PORTB, however DDRB's purpose is not to read or write directly 
  // to/from pins as PORTB does, but rather to set pins to being outputs or inputs --- 
  // this is a prerequisite for reading/writing from/to pins.
  // What happens here then?:
  //    LAMP is 0x01 = (bits) 0000 0001
  //    SH   is 0x02 = (bits) 0000 0010
  //    ICG  is 0x04 = (bits) 0000 0100
  //    MCLK is 0x08 = (bits) 0000 1000
  //   Now, what is LAMP | SH = ?
  //        0000 0001 _OR_ 0000 0010 = 0000 0011
  //   and the chain  LAMP | SH | ICG | MCLK  comes to  0000 1111.
  //   The expression DDRB |= ...  is a short form for  DDRB = DDRB | ..., just as
  //   one can write a += b instead of a = a + b.
  //   So this next line basically just pulls the lowest four bits in DDRB high, 
  //   which means digital pins 8,9,10,11 are declared as output pins.
  DDRB |= (LAMP | SH | ICG | MCLK);	// Set the clock lines to outputs

  // Now we're using the same operation to set digital pin 10 high:
  CLOCK |= ICG;				// Set the integration clear gate high.

  // Enable the serial port.
  Serial.begin(115200);

  // Setup timer2 to generate a 470kHz frequency on D11
  // Arduino timers require serious black magic, but are a very powerful tool, as 
  // they enable you to do tasks at exact frequencies, etc..  
  // Some useful spellbooks on timers are:
  //    https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
  //    https://arduino-info.wikispaces.com/Timers-Arduino
  //    http://www.instructables.com/id/Arduino-Timer-Interrupts/?ALLSTEPS
  // and the actual bible on this is of course the data sheet:
  //    http://www.atmel.com/images/doc2545.pdf  
  //    (pages 144--164, budget half a day to chew through this; worth reading)
  //
  // In brief, the easy way to get PWM output on a digital pin is to use 
  // the 'analogWrite' function, which can set the duty cycle, but doesn't give 
  // you the frequency control we need. The hard way is to directly control the 
  // three hardware timers which the Arduino has for this purpose. TCCR2A, TCCR2B, 
  // OCR2A, TCNT2, etc, are registers which provide control interfaces.
  // Timer 2 (which we use here) is an 8-bit timer, which means it counts 
  // perpetually from 0 to 255, and can set output pins low/high depending on where 
  // it currently counted to. At each count the timer checks if its current value is
  // equal to the number stored in the registers OCR2A and OCR2B, and if this is the 
  // case flag bits are set (which may trigger further events and interrupts).
  // The counting rate is essentially the system clock of 16 MHz, but there's a 
  // prescaler for this (clock select, CS) which is used to access a list of 
  // predefined dividers for that frequency (8,32,64,128,256,1024 for timer2).
  // Now, labels like COM2A1, COM2A0, WGM21, WGM20, etc, somewhat confusingly do not 
  // refer to registers, but to specific bits in the control registers;  the 8 bits 
  // of register TCCR2A are [ COM2A1, COM2A0, COM2B1, COM2B0, -, -, WGM21, WGM20 ].
  // Note that the timer has two outputs A and B, each configured with two bits 
  // ([COM2A1,COM2A0] and [COM2B1,COM2B0]) to control output behaviour, which is 
  // further dependent on the setting of the waveform generator bits WGM21 and WGM20. 
  // Now, you'd think the bit labels are something like COM2A1=128, COM2A0=64, etc, 
  // so that you can set the register as before via TCCR2A |= COM2A1 | COM2A0 ...  
  // but you'd be _wrong_ --- the labels contain the position of the bit in the 
  // register byte, so COM2A1=7, COM2A0=6,..., WGM20=0. 
  // How do you set these bits? ---via bit-shift operators! For example, we start 
  // with a one and shift it to the left by, say three digits:  
  //                1 << 3  =  0000 0001 << 3  =  0000 1000   
  // (Left bit-shift, <<, will discard the bits which are shifted too far left 
  // (out of the byte), and new bits appearing on the right will be zeros.)
  // So, to set the COM2A0 bit in some byte, you can write  1 << COM2A0. To not set
  // it, you could write  0 << COM2A1, or just zero, or just forget about that byte,
  // but I guess it makes sense to write this as 0 << COM2A1 to explicitly state 
  // that this bit is not set.
  // With all this, we can decode what the next line does:  
  TCCR2A =  + (0 << COM2A1) | (1 << COM2A0) | (1 << WGM21) | (0 << WGM20);
  // TCCR2A = 0000 0000 | 0100 0000 | 0000 0010 | 0000 0000
  //        = 0100 0010
  // This sets the COM2A0 and WGM21 bits. The waveform generator has a third bit,
  // WGM22, which is located in the second control register, called TCCR2B. We do
  // not set this, but set the clock-select CS20 bit which is located there.
  TCCR2B = (0 << WGM22) | (1 << CS20);
  // Just setting CS20, not CS21 or CS22, means we use the internal 16 MHz clock
  // without any scaling. 
  // The waveform generator bits [WGM22, WGM21, WGM20] are now set to [0, 1, 0], 
  // which means (see table 18-8, page 160 in the data sheet) we have set the 
  // timer to clear-timer on compare (CTC) mode: the timer will count up until 
  // comparison with OCR2A shows a match, and reset itself to zero. Upon match,
  // the value on pin 11 is toggled from high to low or vice-versa.
  OCR2A = 20;  
  // Page 150 in the data sheet has the formula for computing the frequency given 
  // the prescaler (here =1) and the value of OCR2A (here =20). In our case:
  //   f = 16 MHz / 2 / (1+OCR2A) 
  // The factor two is always there, because the waveform on pin 11 comes from
  // toggling high/low. So we have a resulting frequency of 8 MHz/21 = 380 KHz. 
  // Can we check this on an oscilloscope? Dave Allmon says he's setting 470 KHz 
  // here, but I believe we'd need OCR2A=16 for that---maybe I'm missing something?
  TCNT2 = 1;
  // We start the timer at =1, but I think it will reset to and count from 0 
  // each time it has hit OCR2A.
  // Whatever the frequency, this will be the master clock provided to the CCD.

  // Set the ADC clock to sysclk/32
  // ADCSRA is a control register of the ADC unit, and the lowest few bits 
  // are the prescaler bits for the ADC frequency, which we touch here:
  // First line here pulls down the ADPS0,1,2 bits, and the second line sets
  // just ADPS bit 0 and 2, so we have [1,0,1], which set a frequency divider 
  // of 32 (see table 24-4, page 263).
  ADCSRA &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);
  
  pinMode(12,OUTPUT);
}

void readCCD(void)
{
  int x;
  uint8_t result;

  // To understand the motivation behind the following sequence we need to 
  // look into the manual for the CCD (TCP1304AP), specifically the timing
  // diagrams which contain how to tell the CCD that we want to read the
  // spectrum...  (haven't done that yet, but it's not rocket surgery).
  CLOCK &= ~ICG;          // ICG --> low
  _delay_loop_1(12);      // short delay: should be 3 cycles per loop, so here ~3 μs.
                          // not sure why he doesn't just use delayMicroseconds(3) instead
  CLOCK |= SH;            // SH  --> high
  delayMicroseconds(5);   // 
  CLOCK &= ~SH;           // SH  --> low
  delayMicroseconds(15);  // 
  CLOCK |= ICG;           // ICG --> high
  delayMicroseconds(1);

  for (x = 0; x < 800; x++)
  {
    CLOCK |= SH;          // SH  --> high,  we're reading a pixel value now
    if (x == 0)        // if this is the first pixel we read, then
    {
      // Read A0 using the ADC. The result is bit-shifted to the right by two bits, 
      // which means we're discarding the lowest two bits of the 10-bit result, for the
      // convenience of being able to cast the result into a single byte (declared as 
      // unsigned integer).
      // We also use the variables 'avg' and 'result', reading A0 twice. The use of 
      // 'avg' is not as an average, strangely, it acts as a baseline, see below.
      avg = (uint8_t)(analogRead(A0) >> 2);
      result = (uint8_t)(analogRead(A0) >> 2);
    }
    else
    {
      result = (uint8_t)(analogRead(A0) >> 2);   // read A0 into result
      // if the result is less than the first-pixel baseline stored in avg,
      // then we presume it's nonsensical and set it to zero.
      if (result < avg)       
      {
        result = 0;
      }
      else
      {
	// the result is above the first-pixel baseline, so we subtract the baseline
        result -= avg;
      }
      buffer[x] = result;   // store in buffer
      delayMicroseconds(20);   // give CCD some time before we go to the next pixel
    }
    CLOCK &= ~SH;     // SH  --> low, we're done reading this pixel
  }
}

uint16_t centroid()
{
  // This function simply works out at which pixel half the total collected
  // intensity has occurred in the spectrum
  uint16_t x;
  uint32_t sum = 0;
  uint32_t so_far = 0;
  uint32_t half_max;

  for (x = 0; x < sizeof(buffer); ++x)
  {
    sum += buffer[x];
  }
  half_max = sum / 2;
  for (x = 0; x < sizeof(buffer); ++x)
  {
    so_far += buffer[x];
    if (so_far >= half_max)
    {
      return x;
    }
  }
}

void sendData(void)
{
  // This function pushes the contents of buffer (which contains
  // a spectrum after readCCD() ) out over the serial port.

  int x;
  for (x = 0; x < 800; ++x)
  {
    Serial.println(buffer[x]);
  }
}

void loop()
{
  int x;

  if (Serial.available())
  {
    cmdBuffer[cmdIndex++] = Serial.read();
  }
  if (cmdBuffer[0] == 'r')
  {
    //digitalWrite(12,HIGH);
    //delay(500);
    //digitalWrite(12,LOW);  
    //Serial.println("hurro");
    sendData();
  }
  else if (cmdBuffer[0] == 'l')
  {
    CLOCK &= ~LAMP;
  }
  else if (cmdBuffer[0] == 'L')
  {
    CLOCK |= LAMP;
  }
  else if (cmdBuffer[0] == 'e')
  {
    if (--exposureTime < 0) exposureTime = 0;
    Serial.print("Exposure time ");
    Serial.println(exposureTime);
  }
  else if (cmdBuffer[0] == 'E')
  {
    if (++exposureTime > 200) exposureTime = 200;
    Serial.print("Exposure time ");
    Serial.println(exposureTime);
  }
  else if (cmdBuffer[0] == 'c')
  {
    Serial.print("Centroid position: ");
    Serial.println(centroid());
  }
  cmdBuffer[0] = '\0';
  cmdIndex = 0;

  readCCD();
  delay(exposureTime);
}
