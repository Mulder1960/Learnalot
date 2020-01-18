//////////////////////////////////////////////////////////////////////
// Test om de powderdown van de Arduino te testen
// Meest relevante info uit de Atmeg datasheet, Belangrijke registers zijn SMCR en PRR
// SMCR Sleep Mode Control Register, only Low nibble is in use
// PRR Power reduction Register
//////////////////////////////////////////////////////////////////////

#define PDTimerCounter0 0 // Power Down Timer Counter 0
#define PDTimerCounter1 1 // Power Down Timer Counter 1
#define PDTimerCounter2 2 // Power Down Timer Counter 2
#define PDADC           3 // Power Down AD Converter
#define PDUSART         4 // Power Down USART
#define PDSPI           5 // Power Down SPI 
#define PDTWI           6 // Power Down TWI (Two Wire Interface)


void PowerDownd(int sel)
{
  switch (sel)
  {
  case PDTimerCounter0:
    PRR = PRR | B00100000; // set bit 5 in Power Reduction Register
    break;

  case PDTimerCounter1:
    PRR = PRR | B00001000; // set bit 3 in Power Reduction Register
    break;

  case PDTimerCounter2:
    PRR = PRR | B01000000; // set bit 6 in Power Reduction Register
    break;

  case PDADC:
    PRR = PRR | B00000001; // set bit 0 in Power Reduction Register
    break;

  case PDUSART:
    PRR = PRR | B00000010; // set bit 1 in Power Reduction Register
    break;

  case PDSPI:
    PRR = PRR | B00000100; // set bit 2 in Power Reduction Register
    break;

  case PDTWI:
    PRR = PRR | B10000000; // set bit 7 in Power Reduction Register
    break;

  }
}

#define IDLE
#define POWERSAVE 

void setup()
{
  // SMCR Sleep Mode Control Register, only Low nibble is in use
  // PRR Power reduction Register


  PRR = 0;
}

void loop()
{
}


