
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bcm2835.h"
#include "serial.h"
#include "../threads/interrupt.h"
#include "../threads/thread.h"
#include "../threads/synch.h"


extern void PUT32 ( unsigned int, unsigned int );
extern unsigned int GET32 ( unsigned int );
extern void dummy ( unsigned int );
extern void enable_irq ( void );
extern void enable_fiq ( void );

#define GPFSEL1 0x20200004
#define GPSET0  0x2020001C
#define GPCLR0  0x20200028
//#define GPPUD       0x20200094
//#define GPPUDCLK0   0x20200098

#define AUX_ENABLES     0x20215004
#define AUX_MU_IO_REG   0x20215040
#define AUX_MU_IER_REG  0x20215044
#define AUX_MU_IIR_REG  0x20215048
#define AUX_MU_LCR_REG  0x2021504C
#define AUX_MU_MCR_REG  0x20215050
#define AUX_MU_LSR_REG  0x20215054
#define AUX_MU_MSR_REG  0x20215058
#define AUX_MU_SCRATCH  0x2021505C
#define AUX_MU_CNTL_REG 0x20215060
#define AUX_MU_STAT_REG 0x20215064
#define AUX_MU_BAUD_REG 0x20215068

#define IRQ_BASIC 0x2000B200
#define IRQ_PEND1 0x2000B204
#define IRQ_PEND2 0x2000B208
#define IRQ_FIQ_CONTROL 0x2000B210
#define IRQ_ENABLE1 0x2000B210
#define IRQ_ENABLE2 0x2000B214
#define IRQ_ENABLE_BASIC 0x2000B218
#define IRQ_DISABLE1 0x2000B21C
#define IRQ_DISABLE2 0x2000B220
#define IRQ_DISABLE_BASIC 0x2000B224

static volatile unsigned int rxhead;
static volatile unsigned int rxtail;
#define RXBUFMASK 0xFFF
static volatile unsigned char rxbuffer[RXBUFMASK+1];
static struct semaphore io_sem;

void serial_init(void) {
  test_serial();
}
void serial_putc (char character) {
  //uart_putc(96);
  //uart_putc('B');
  uart_putc(character);
}
void serial_flush (void) {
	// TODO Implement the method.
}

void serial_notify (void) {
	// TODO Implement the method.
}

static inline void mmio_write(uint32_t reg, uint32_t data)
{
  uint32_t *ptr = (uint32_t*) reg;
  asm volatile("str %[data], [%[reg]]" : : [reg]"r"(ptr), [data]"r"(data));
}

static inline uint32_t mmio_read(uint32_t reg)
{
  uint32_t *ptr = (uint32_t*)reg;
  uint32_t data;
  asm volatile("ldr %[data], [%[reg]]" : [data]"=r"(data) : [reg]"r"(ptr));
  return data;
}

/* Loop <delay> times in a way that the compiler won't optimize away. */
static inline void delay(int32_t count)
{
  asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
     : : [count]"r"(count) : "cc");
}

enum
{
    // The GPIO registers base address.
    GPIO_BASE = 0x20200000,

    // The offsets for reach register.

    // Controls actuation of pull up/down to ALL GPIO pins.
    GPPUD = (GPIO_BASE + 0x94),

    // Controls actuation of pull up/down for specific GPIO pin.
    GPPUDCLK0 = (GPIO_BASE + 0x98),

    // The base address for UART.
    UART0_BASE = 0x20201000,

    // The offsets for reach register for the UART.
    UART0_DR     = (UART0_BASE + 0x00),
    UART0_RSRECR = (UART0_BASE + 0x04),
    UART0_FR     = (UART0_BASE + 0x18),
    UART0_ILPR   = (UART0_BASE + 0x20),
    UART0_IBRD   = (UART0_BASE + 0x24),
    UART0_FBRD   = (UART0_BASE + 0x28),
    UART0_LCRH   = (UART0_BASE + 0x2C),
    UART0_CR     = (UART0_BASE + 0x30),
    UART0_IFLS   = (UART0_BASE + 0x34),
    UART0_IMSC   = (UART0_BASE + 0x38),
    UART0_RIS    = (UART0_BASE + 0x3C),
    UART0_MIS    = (UART0_BASE + 0x40),
    UART0_ICR    = (UART0_BASE + 0x44),
    UART0_DMACR  = (UART0_BASE + 0x48),
    UART0_ITCR   = (UART0_BASE + 0x80),
    UART0_ITIP   = (UART0_BASE + 0x84),
    UART0_ITOP   = (UART0_BASE + 0x88),
    UART0_TDR    = (UART0_BASE + 0x8C),
};

//void uart_init()
//{
//  // Disable UART0.
//  mmio_write(UART0_CR, 0x00000000);
//  // Setup the GPIO pin 14 && 15.
//
//  // Disable pull up/down for all GPIO pins & delay for 150 cycles.
//  mmio_write(GPPUD, 0x00000000);
//  delay(150);
//
//  // Disable pull up/down for pin 14,15 & delay for 150 cycles.
//  mmio_write(GPPUDCLK0, (1 << 14) | (1 << 15));
//  delay(150);
//
//  // Write 0 to GPPUDCLK0 to make it take effect.
//  mmio_write(GPPUDCLK0, 0x00000000);
//
//  // Clear pending interrupts.
//  mmio_write(UART0_ICR, 0x7FF);
//
//  // Set integer & fractional part of baud rate.
//  // Divider = UART_CLOCK/(16 * Baud)
//  // Fraction part register = (Fractional part * 64) + 0.5
//  // UART_CLOCK = 3000000; Baud = 115200.
//
//  // Divider = 3000000 / (16 * 115200) = 1.627 = ~1.
//  // Fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40.
//  mmio_write(UART0_IBRD, 1);
//  mmio_write(UART0_FBRD, 40);
//
//  // Enable FIFO & 8 bit data transmission (1 stop bit, no parity).
//  mmio_write(UART0_LCRH, (1 << 4) | (1 << 5) | (1 << 6));
//
//  // Mask all interrupts.
//  mmio_write(UART0_IMSC, (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
//                         (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));
//
//  // Enable UART0, receive & transfer part of UART.
//  mmio_write(UART0_CR, (1 << 0) | (1 << 8) | (1 << 9));
//}


void uart_init ( void )
{
    unsigned int ra;

    PUT32(AUX_ENABLES,1);
    PUT32(AUX_MU_IER_REG,0);
    PUT32(AUX_MU_CNTL_REG,0);
    PUT32(AUX_MU_LCR_REG,3);
    PUT32(AUX_MU_MCR_REG,0);
    PUT32(AUX_MU_IER_REG,0x5); //enable rx interrupts
    PUT32(AUX_MU_IIR_REG,0xC6);
    PUT32(AUX_MU_BAUD_REG,270);

    ra=GET32(GPFSEL1);
    ra&=~(7<<12); //gpio14
    ra|=2<<12;    //alt5
    ra&=~(7<<15); //gpio15
    ra|=2<<15;    //alt5
    PUT32(GPFSEL1,ra);

    PUT32(GPPUD,0);
    for(ra=0;ra<150;ra++) dummy(ra);
    PUT32(GPPUDCLK0,(1<<14)|(1<<15));
    for(ra=0;ra<150;ra++) dummy(ra);
    PUT32(GPPUDCLK0,0);

    PUT32(AUX_MU_CNTL_REG,3);
    sema_init(&io_sem, 0);
}

void uart_putc ( unsigned int c )
{
  if (c == '\n'||c == '\r') {
      uart_putc_helper('\n');
      uart_putc_helper('\r');
    } else {
      uart_putc_helper(c);
    }

}


void c_irq_handler ( struct interrupts_stack_frame *stack_frame )
{
    unsigned int rb,rc;
//    printf("\ninto keyboard handler");
//    set_current_interrupts_stack_frame(stack_frame);
    //an interrupt has occurred, find out why
    while(1) //resolve all interrupts to uart
    {
        rb=GET32(AUX_MU_IIR_REG);
        if((rb&1)==1) break; //no more interrupts
        if((rb&6)==4)
        {
//            printf("\nreceive character");
            //receiver holds a valid byte
            rc=GET32(AUX_MU_IO_REG); //read byte from rx fifo
            rxbuffer[rxhead]=rc&0xFF;
            rxhead=(rxhead+1)&RXBUFMASK;
        }
    }
    sema_up(&io_sem);

}

void keyboard_init() {
  printf("\nInitializing keyboard.....");
  interrupts_register_irq(IRQ_29, c_irq_handler, "Keyboard Interrupt");
}

int echo ()
{
    unsigned int ra;
    unsigned int rb;
    unsigned int rc;
    unsigned int rx;

//    PUT32(IRQ_DISABLE1,1<<29);
//    uart_init();
//    PUT32(IRQ_ENABLE1,1<<29);
//    enable_irq();
//    sema_init(&io_sem,0);
    while(1)
    {
        while(rxtail!=rxhead)
        {
            uart_putc(rxbuffer[rxtail]);
            rxtail=(rxtail+1)&RXBUFMASK;
        }
        //if tail == head, no data
//        sema_down(&io_sem);
    }
    return(0);
}

//void uart_putc_helper(unsigned char byte) {
//  // Wait for UART to become ready to transmit.
//  while ( mmio_read(UART0_FR) & (1 << 5) ) { }
//  mmio_write(UART0_DR, byte);
//}

void uart_putc_helper(unsigned int c) {
  PUT32(AUX_MU_IO_REG,c);
}


//void uart_putc(unsigned char byte)
//{
//  if (byte == '\n') {
//    uart_putc_helper('\n');
//    uart_putc_helper('\r');
//  } else {
//    uart_putc_helper(byte);
//  }
//}

// Test using screen /dev/cu.PL2303-00001004 115200

unsigned char uart_getc()
{
    // Wait for UART to have recieved something.
//    while ( mmio_read(UART0_FR) & (1 << 4) ) { }
//    return mmio_read(UART0_DR);
    while(1)
        {
          if(rxtail != rxhead){
            while(rxtail!=rxhead)
            {
                uart_putc(rxbuffer[rxtail]);
                rxtail=(rxtail+1)&RXBUFMASK;
            }
            return rxbuffer[rxtail];
          }else{
            //if tail == head, no data
            sema_down(&io_sem);
          }
        }
}
unsigned char * uart_gets()
{
    // Wait for UART to have recieved something.
//    while ( mmio_read(UART0_FR) & (1 << 4) ) { }
//    return mmio_read(UART0_DR);
    unsigned char buf[128];
    int i = 0;
    memset(buf, 0 ,128);
    while(1)
        {
          if(rxtail != rxhead){
            while(rxtail!=rxhead)
            {
                uart_putc(rxbuffer[rxtail]);
                buf[i++] = rxbuffer[rxtail];
                rxtail=(rxtail+1)&RXBUFMASK;
            }
            return buf;
          }else{
            //if tail == head, no data
            sema_down(&io_sem);
          }
        }
}

void uart_write(const unsigned char* buffer, size_t size)
{
  size_t i;
  for ( i = 0; i < size; i++ )
    uart_putc(buffer[i]);
}

void uart_puts(const char* str)
{
  uart_write((const unsigned char*) str, strlen(str));
}

void test_serial() {

  uart_init();
}
