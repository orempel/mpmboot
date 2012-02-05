/***************************************************************************
*   Copyright (C) 01/2012 by Olaf Rempel                                  *
*   razzor@kopf-tisch.de                                                  *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; version 2 of the License,               *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

/*
* atmega32:
* Fuse H: 0xda (1024 words bootloader)
* Fuse L: 0xd4 (8Mhz internal RC-Osz.)
*/

#if defined (__AVR_ATmega32__)
#define F_CPU               8000000
#define VERSION_STRING      "MPMBOOT m32v1.0"
#define SIGNATURE_BYTES     0x1E, 0x95, 0x02

#else
#error MCU not supported
#endif

/* 25ms @8MHz */
#define TIMER_RELOAD            (0xFF - 195)

/* 40 * 25ms */
#define TIMEOUT                 40

#define EN_TX                   (1<<PORTD2)
#define LED_GN                  (1<<PORTD3)

#define BAUDRATE                115200
#define MPM_ADDRESS             0x11

#define EEPROM_SUPPORT          1

#define CMD_WAIT                0x00
#define CMD_SWITCH_MODE         0x01
#define CMD_GET_VERSION         0x02
#define CMD_GET_CHIPINFO        0x03
#define CMD_READ_MEMORY         0x11
#define CMD_WRITE_MEMORY        0x12

#define CAUSE_SUCCESS           0x00
#define CAUSE_NOT_SUPPORTED     0xF0
#define CAUSE_INVALID_PARAMETER 0xF1
#define CAUSE_UNSPECIFIED_ERROR 0xFF

#define BOOTMODE_BOOTLOADER     0x00
#define BOOTMODE_APPLICATION    0x80

#define MEMTYPE_FLASH           0x01
#define MEMTYPE_EEPROM          0x02

#define BOOTWAIT_EXPIRED        0x00
#define BOOTWAIT_RUNNING        0x01
#define BOOTWAIT_INTERRUPTED    0x02

#define UART_CALC_BAUDRATE(baudRate) (((uint32_t)F_CPU) / (((uint32_t)baudRate)*16) -1)

/*
 * LED_GN blinks with 20Hz (while bootloader is running)
 * LED_RT blinks on activity
 *
 * general protocol:
 * =================
 * req: <device> <cmd> <length> <data>*
 * rsp: <cmd> <cause> <length> <data>*
 *
 * device is MPM device address and has 9th bit set, all other
 * bytes in request & response have 9th bit NOT set
 *
 * length is 16bit, MSB first
 *
 * rsp-cause codes:
 * 0x00 - ok
 * 0xF0 - command not supported (unknown to bootloader)
 * 0xF1 - parameter error (invalid memtype/address)
 * 0xFF - unspecified error
 *
 * CMD switch mode:
 * ================
 * req: <cmd:0x01>,<length:0x0001>,<mode>
 * rsp: <cmd:0x01>,<cause:0x00>,<length:0x0000>
 *
 * mode codes:
 * 0x00 - bootloader
 * 0x80 - application
 *
 * CMD get bootloader version:
 * ===========================
 * req: <cmd:0x02>,<length:0x0000>
 * rsp: <cmd:0x02>,<cause:0x00>,<length[2]>,<data[x]>
 *
 * response data is bootloader version string with implementation specific length
 *
 * CMD get chip info:
 * ==================
 * req: <cmd:0x03>,<length:0x0000>
 * rsp: <cmd:0x03>,<cause:0x00>,<length:0x0008>,<signature[3]><pagesize[1]><flashsize[2]><eepromsize[2]>
 *
 * CMD read memory:
 * ================
 * req: <cmd:0x11>,<length:0x0004>,<memtype>,<address[2]>,<size[2]>
 * rsp: <cmd:0x11>,<cause:0x00>,<length[2]>,<data[x]>
 *
 * memtype codes:
 * 0x01 - flash memory
 * 0x02 - eeprom memory
 *
 * CMD write memory:
 * =================
 * req: <cmd:0x12>,<length[2]>,<memtype>,<address[2]>,<size[2]>,<data[x]>
 * rsp: <cmd:0x12>,<cause:0x00>,<length:0x0000>
 *
 * memtype codes:
 * 0x01 - flash memory
 * 0x02 - eeprom memory
 *
 */

const static uint8_t info[16] = VERSION_STRING;
const static uint8_t chipinfo[8] = {
    SIGNATURE_BYTES,

    SPM_PAGESIZE,

    ((BOOTLOADER_START) >> 8) & 0xFF,
    (BOOTLOADER_START) & 0xFF,
    ((E2END +1) >> 8 & 0xFF),
    (E2END +1) & 0xFF
};

/* wait 40 * 25ms = 1s */
static uint8_t boot_timeout = TIMEOUT;
volatile static uint8_t boot_wait = BOOTWAIT_RUNNING;

static uint8_t rx_addressed;

static uint16_t rx_bcnt;
static uint8_t rx_cmd;
static uint16_t rx_length;

static uint16_t tx_bcnt;
static uint8_t tx_cmd;
static uint8_t tx_cause;
static uint16_t tx_length;

static uint8_t para_mode;
static uint8_t para_memtype;
static uint16_t para_address;
static uint16_t para_size;

/* write buffer */
static uint8_t pagebuf[SPM_PAGESIZE];

static void write_flash_page(uint16_t pagestart, uint8_t *data, uint8_t size)
{
    uint16_t address = pagestart;
    uint8_t pagesize = SPM_PAGESIZE;

    boot_page_erase(pagestart);
    boot_spm_busy_wait();

    do {
        uint16_t dataword;

        dataword  =  (size-- != 0) ? *data++ : 0xFF;
        dataword |= ((size-- != 0) ? *data++ : 0xFF) << 8;
        boot_page_fill(address, dataword);

        address += 2;
        pagesize -= 2;
    } while (pagesize);

    boot_page_write(pagestart);
    boot_spm_busy_wait();
    boot_rww_enable();
}

#if (EEPROM_SUPPORT)
static uint8_t read_eeprom_byte(uint16_t address)
{
    EEARL = address;
    EEARH = (address >> 8);
    EECR |= (1<<EERE);
    return EEDR;
}

static void write_eeprom_page(uint16_t address, uint8_t *data, uint16_t size)
{
    while (size--) {
        EEARL = address;
        EEARH = (address >> 8);
        address++;
        EEDR = *data++;
#if defined (__AVR_ATmega32__)
        EECR |= (1<<EEMWE);
        EECR |= (1<<EEWE);
#else
#error write_eeprom_page(): access not defined
#endif
        eeprom_busy_wait();
    }
}
#endif /* (EEPROM_SUPPORT) */

ISR(USART_RXC_vect)
{
    uint8_t data = UDR;

    if (rx_addressed == 0) {
        /* own address, disable MPM mode and receive following bytes */
        if (data == MPM_ADDRESS) {
            boot_wait = BOOTWAIT_INTERRUPTED;
            /* enable LED */
            PORTD &= ~(LED_GN);

            UCSRA &= ~(1<<MPCM);
            rx_addressed = 1;
            rx_bcnt = 0;
        }

    } else {
        /* byte 0 is command */
        if (rx_bcnt == 0) {
            rx_cmd = data;

        /* byte 1/2 is payload length */
        } else if (rx_bcnt == 1 || rx_bcnt == 2) {
            rx_length = (rx_length << 8) | data;

        /* byte >= 3 is payload */
        } else if ((rx_bcnt -3) < rx_length) {
            uint16_t pos = rx_bcnt -3;

            if ((rx_cmd == CMD_SWITCH_MODE) && (pos == 0)) {
                para_mode = data;

            } else if ((rx_cmd == CMD_READ_MEMORY) || (rx_cmd == CMD_WRITE_MEMORY)) {
                switch (pos) {
                    case 0:
                        para_memtype = data;
                        break;

                    case 1:
                    case 2:
                        para_address = (para_address << 8) | data;
                        break;

                    case 3:
                    case 4:
                        para_size = (para_size << 8) | data;
                        break;

                    default:
                        pos -= 5;
                        if ((rx_cmd == CMD_WRITE_MEMORY) && (pos < sizeof(pagebuf))) {
                            pagebuf[pos] = data;
                        }
                        break;
                }
            }
        }

        /* last byte received */
        if ((rx_bcnt -2) == rx_length) {
            /* setup response */
            tx_bcnt   = 0;
            tx_cmd    = rx_cmd;
            tx_cause  = CAUSE_SUCCESS;
            tx_length = 0;

            switch (tx_cmd) {
                case CMD_SWITCH_MODE:
                    if ((para_mode != BOOTMODE_APPLICATION) && (para_mode != BOOTMODE_BOOTLOADER)) {
                        tx_cause = CAUSE_INVALID_PARAMETER;
                    }
                    break;

                case CMD_GET_VERSION:
                    tx_length = sizeof(info);
                    break;

                case CMD_GET_CHIPINFO:
                    tx_length = sizeof(chipinfo);
                    break;

                case CMD_READ_MEMORY:
                    tx_length = para_size;
                    /* no break */
                case CMD_WRITE_MEMORY:
                    if (para_memtype == MEMTYPE_FLASH) {
                        /* only access application area */
                        if (para_address > (BOOTLOADER_START - SPM_PAGESIZE)) {
                            tx_length = 0;
                            tx_cause  = CAUSE_INVALID_PARAMETER;

                        /* writes must pagesize aligned */
                        } else if (tx_cmd == CMD_WRITE_MEMORY) {
                            if (((para_address & (SPM_PAGESIZE -1)) == 0x00) &&
                                (para_size <= SPM_PAGESIZE)
                               ) {
                                write_flash_page(para_address, pagebuf, para_size);
                                para_address += para_size;

                            } else {
                                tx_cause = CAUSE_INVALID_PARAMETER;
                            }
                        }
#if (EEPROM_SUPPORT)
                    } else if (para_memtype == MEMTYPE_EEPROM) {
                        if ((para_address > (E2END +1)) || ((para_address + para_size) > (E2END +1))) {
                            tx_cause = CAUSE_INVALID_PARAMETER;

                        } else if (tx_cmd == CMD_WRITE_MEMORY) {
                            write_eeprom_page(para_address, pagebuf, para_size);
                            para_address += para_size;
                        }
#endif /*(EEPROM_SUPPORT) */
                    } else {
                        tx_length = 0;
                        tx_cause = CAUSE_INVALID_PARAMETER;
                    }
                    break;

                default:
                    tx_cause = CAUSE_NOT_SUPPORTED;
                    break;
            }

            /* kickoff transmit */
            UCSRB    |= (1<<UDRIE);
        }

        rx_bcnt++;
    }
}

ISR(USART_UDRE_vect)
{
    if (tx_bcnt == 0) {
        /* enable RS485 transmitter */
        PORTD |= EN_TX;

        UCSRB &= ~(1<<TXB8);
        UDR    = tx_cmd;

    } else if (tx_bcnt == 1) {
        UDR = tx_cause;

    } else if (tx_bcnt == 2) {
        UDR = (tx_length >> 8);

    } else if (tx_bcnt == 3) {
        UDR = (tx_length & 0xFF);

    } else if ((tx_bcnt -4) < tx_length) {
        uint16_t pos  = tx_bcnt -4;
        uint8_t  data = 0xFF;

        if (tx_cmd == CMD_GET_VERSION) {
            data = info[pos];

        } else if (tx_cmd == CMD_GET_CHIPINFO) {
            data = chipinfo[pos];

        } else if (tx_cmd == CMD_READ_MEMORY) {
            if (para_memtype == MEMTYPE_FLASH) {
                data = pgm_read_byte_near(para_address++);
#if (EEPROM_SUPPORT)
            } else if (para_memtype == MEMTYPE_EEPROM) {
                data = read_eeprom_byte(para_address++);
#endif /* (EEPROM_SUPPORT) */
            }
        }

        UDR = data;

    } else {
        /* stop transmit */
        UCSRB &= ~(1<<UDRIE);
    }

    tx_bcnt++;
}

ISR(USART_TXC_vect)
{
    /* disable LED */
    PORTD |= LED_GN;

    /* disable RS485 transmitter */
    PORTD &= ~(EN_TX);

    /* enable MP mode again */
    UCSRA |= (1<<MPCM);
    rx_addressed = 0;

    /* switch to application after everything is transmitted */
    if ((tx_cmd == CMD_SWITCH_MODE) && (para_mode == BOOTMODE_APPLICATION)) {
        boot_wait = BOOTWAIT_EXPIRED;
    }
}

ISR(TIMER0_OVF_vect)
{
    /* restart timer */
    TCNT0 = TIMER_RELOAD;

    if (boot_wait == BOOTWAIT_RUNNING) {
        PORTD ^= LED_GN;

        boot_timeout--;
        if (boot_timeout == 0) {
            boot_wait = BOOTWAIT_EXPIRED;
        }
    }
}

#if 0
static void uart_send(char *p)
{
    while (*p) {
        while (!(UCSRA & (1<<UDRE)));
        UDR = *p++;
    }
}
#endif

static void (*jump_to_app)(void) __attribute__ ((noreturn)) = 0x0000;

int main(void) __attribute__ ((noreturn));
int main(void)
{
    /* LED and TXEN are outputs */
    DDRD |= LED_GN | EN_TX;

#if defined(OSCCAL_VALUE)
    OSCCAL = OSCCAL_VALUE;
#endif

    /* move interrupt-vectors to bootloader */
    /* timer0: running with F_CPU/1024, OVF interrupt */
#if defined (__AVR_ATmega32__)
    GICR = (1<<IVCE);
    GICR = (1<<IVSEL);

    TCCR0 = (1<<CS02) | (1<<CS00);
    TIMSK = (1<<TOIE0);
#endif

    /* USART config */
    /* Multi Drop Mode, 9n1 */
    UCSRA = (1<<MPCM);
    UCSRB = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE) | (1<<TXCIE) | (1<<UCSZ2);
    UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);
    UBRRH = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
    UBRRL = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

    /* 11.354ms for 109 bits @9600 */
    /* 946.18us for 109 bits @115200 */
//    uart_send("1234567890");

    sei();

    while (boot_wait != BOOTWAIT_EXPIRED);

    cli();

    /* disable timer0 */
    /* move interrupt vectors back to application */
#if defined (__AVR_ATmega32__)
    TCCR0 = 0x00;
    TIMSK = 0x00;

    GICR = (1<<IVCE);
    GICR = (0<<IVSEL);
#endif

    /* disable LED */
    PORTD |= LED_GN;

    uint16_t wait = 0x0000;
    do {
        __asm volatile ("nop");
    } while (--wait);

    jump_to_app();
}
