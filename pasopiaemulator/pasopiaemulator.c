//  Toshiba PASOPIA PA-7010 & PASOPIA7 PA-7007 emulator
//
//  GP0: HSYNC
//  GP1: VSYNC
//  GP2: Blue0
//  GP3: Red1
//  GP4: Green0
//  GP6: Audio

// Select Machine
#define MACHINE_PA7010
//#define MACHINE_PA7007

//#define USE_FDC
#define USE_RAMPAC

//#define USE_DEBUG
#define DRAW_ON_DEMAND  // NEVER turn off

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"

#include "tusb.h"
#include "bsp/board.h"

#include "vga16_graphics.h"

#include "pasopiakeymap.h"
#include "Z80.h"
#include "pasopiamisc.h"

#include "lfs.h"

#include "pasopiarom.h"

#ifdef USE_DEBUG
#include "pasopiatest.h"
#endif

// VGAout configuration

#define DOTCLOCK 25000
#define CLOCKMUL 10
// Pico does not work at CLOCKMUL=7 (175MHz).

#define VGA_PIXELS_X 640
#define VGA_PIXELS_Y 200

#define VGA_CHARS_X 80
#define VGA_CHARS_Y 25

#define VRAM_PAGE_SIZE (VGA_PIXELS_X*VGA_PIXELS_Y/8)

extern unsigned char vga_data_array[];
volatile uint8_t fbcolor,cursor_x,cursor_y,video_mode;

volatile uint32_t video_hsync,video_vsync,scanline,vsync_scanline,video_framecount;

struct repeating_timer timer,timer2;

// PC configuration

static Z80 cpu;
uint32_t cpu_clocks=0;
uint32_t cpu_ei=0;
uint32_t cpu_cycles=0;
uint32_t cpu_hsync=0;
uint8_t readmap[8];
uint8_t writemap[8];

uint8_t mainram[0x10000];
uint8_t ioport[0x100];
uint8_t fontrom[0x800]; // COPY FONT to RAM (for speed up)
uint8_t vram[0xc000];
uint8_t atram[0x4000];
uint8_t menuram[0x1000];  // 80x25 monochrome

uint8_t ram_ioaccess=0;
uint8_t lastattr=0;
uint8_t color_palette[16];
uint8_t color_palette_table[256];
uint8_t color_mask_table[256];
uint32_t nmi_enable_irq=0;

#ifdef USE_RAMPAC
uint8_t rampac[0x8000];
#endif

uint32_t renderbuffer[81];  // 80x4
uint32_t gvrenderbuffer[81];  
uint32_t maskbuffer[81];



uint32_t pacmode=0;     // PAC 1:RAMPAC 2:KANJI 3:JOYPAC
uint32_t pacptr=0;
uint8_t pacload=0;

static const uint8_t rampac_header[16] = {
	0xaa, 0x1f, 0x04, 0x00, 0x04, 0x80, 0x00, 0x01, 0x04, 0x04, 0x01, 0x03, 0x08, 0x00, 0x00, 0x00
};

uint16_t vramptr;
uint8_t crtc[16];
uint8_t crtc_mode;

uint8_t timer_enable_irq=0;
uint8_t timer_reset=0;

volatile uint8_t redraw_flag=0;

volatile uint8_t keypressed=0;  //last pressed usbkeycode
uint8_t keymap[16];
uint32_t key_repeat_flag=1;
uint32_t key_repeat_count=0;
uint32_t key_caps=0;
uint32_t key_kana=0;
uint32_t key_hirakata=0;
uint32_t lastmodifier=0; 
uint8_t  keysum=0xff;

uint32_t keyboard_enable_irq=0;

// BEEP & PSG

uint32_t pwm_slice_num;
volatile uint32_t sound_tick=0;
volatile uint32_t beep_enable=0;
uint32_t beep_interval=0;
uint32_t beep_count=0;
uint32_t sound_mute=0;

#define PSG_NUMBERS 2

uint16_t psg_register[8 * PSG_NUMBERS];
uint32_t psg_osc_interval[3 * PSG_NUMBERS];
uint32_t psg_osc_counter[3 * PSG_NUMBERS];

uint32_t psg_noise_interval[PSG_NUMBERS];
uint32_t psg_noise_counter[PSG_NUMBERS];
uint8_t psg_noise_output[PSG_NUMBERS];
uint32_t psg_noise_seed[PSG_NUMBERS];
uint32_t psg_freq_write[PSG_NUMBERS];
// uint32_t psg_envelope_interval[PSG_NUMBERS];
// uint32_t psg_envelope_counter[PSG_NUMBERS];
//uint32_t psg_master_clock = PSG_CLOCK2;
//uint32_t psg_master_clock = (3579545/2);
uint32_t psg_master_clock = 2000000;    // ???
uint16_t psg_master_volume = 0;

// TESUTO
uint32_t psg_note_count;

//const uint16_t psg_volume[] = { 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04,
//        0x05, 0x06, 0x07, 0x08, 0x09, 0x0b, 0x0d, 0x10, 0x13, 0x17, 0x1b, 0x20,
//        0x26, 0x2d, 0x36, 0x40, 0x4c, 0x5a, 0x6b, 0x80, 0x98, 0xb4, 0xd6, 0xff };

const uint16_t psg_volume[] = { 0xFF,0xCB,0xA1,0x80,0x66,0x51,0x40,0x33,0x28,0x20,0x1A,0x14,0x10,0x0D,0x0A,0x00};
  
//#define SAMPLING_FREQ 22050                           
#define SAMPLING_FREQ 15625                             // Timer tick (64us)


#define TIME_UNIT 100000000                           // Oscillator calculation resolution = 10nsec
#define SAMPLING_INTERVAL (TIME_UNIT/SAMPLING_FREQ) 

// Tape

uint32_t tape_ready=0;
uint32_t tape_ptr=0;
uint32_t tape_phase=0;
uint32_t tape_count=0;

uint32_t tape_read_wait=0;
uint32_t tape_leader=0;
uint32_t tape_autoclose=0;          // Default value of TAPE autoclose
uint32_t tape_skip=0;               // Default value of TAPE load accelaration
uint32_t tape_cycles;

#define TAPE_LONG   2800
#define TAPE_SHORT  1400

#define TAPE_WAIT 2200
#define TAPE_WAIT_SHORT 400

#define TAPE_THRESHOLD 200000

uint8_t uart_rx[32];
uint8_t uart_nibble=0;
uint8_t uart_count=0;
volatile uint8_t uart_write_ptr=0;
volatile uint8_t uart_read_ptr=0;
uint32_t uart_cycle;

uint8_t pioa[4],piob[4];
volatile uint32_t pioa_enable_irq=0;
volatile uint32_t piob_enable_irq=1;
uint32_t pioa_next_mask,pioa_next_iocontrol;
uint32_t piob_next_mask,piob_next_iocontrol;
uint32_t pio_irq_processing=0;

uint8_t ctc_next_iocontrol[4];
uint8_t ctc_next_timeconstant[4];
uint8_t ctc_timeconstant[4];
uint16_t ctc_counter[4];
uint32_t ctc_internal_counter[4];
uint8_t ctc_irq_vector[4];
uint8_t ctc_control[4];
volatile uint8_t ctc_enable_irq[4];

uint32_t timer_ticks;

//#define CTC_CLOCK   3.9936000
#define CTC_CLOCK   4
#define CTC_TICK    (256/CTC_CLOCK)      // in usec

#ifdef USE_FDC
#include "fdc.h"
uint8_t diskbuffer[0x400];
unsigned char fd_filename[16];
#endif

// UI

volatile uint32_t menumode=0;
uint32_t menuitem=0;

// USB

hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released
extern void hid_app_task(void);

uint32_t usbcheck_count=0;
uint32_t kbhit=0;            // 4:Key pressed (timer stop)/3&2:Key depressed (timer running)/1:no key irq triggerd
uint8_t hid_dev_addr=255;
uint8_t hid_instance=255;
uint8_t hid_led;

extern volatile uint8_t gamepad_info; 
uint8_t gamepad_select;

#define USB_CHECK_INTERVAL 30 // 31.5us*30=1ms

// Define the flash sizes
// This is setup to read a block of the flash from the end 
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)
// for 1M flash pico
//#define HW_FLASH_STORAGE_BASE   (1024*1024 - HW_FLASH_STORAGE_BYTES) 
//#define HW_FLASH_STORAGE_BYTES  (512 * 1024)
// for 2M flash
// #define HW_FLASH_STORAGE_BYTES  (1024 * 1024)
#define HW_FLASH_STORAGE_BYTES  (1536 * 1024)
#define HW_FLASH_STORAGE_BASE   (PICO_FLASH_SIZE_BYTES - HW_FLASH_STORAGE_BYTES) 
// for 16M flash
//#define HW_FLASH_STORAGE_BYTES  (15872 * 1024)
//#define HW_FLASH_STORAGE_BASE   (1024*1024*16 - HW_FLASH_STORAGE_BYTES) 

lfs_t lfs;
lfs_file_t lfs_file,rampac_file;

#define FILE_THREHSOLD 20000000
#define LFS_LS_FILES 9

volatile uint32_t load_enabled=0;
volatile uint32_t save_enabled=0;
//uint32_t file_cycle=0;

unsigned char filename[16];
unsigned char tape_filename[16];
unsigned char rampac_filename[16];

static inline unsigned char tohex(int);
static inline unsigned char fromhex(int);
static inline void video_print(uint8_t *);
uint8_t fdc_find_sector(void);

void __not_in_flash_func(draw_framebuffer_menu)(uint32_t line);
void __not_in_flash_func(draw_framebuffer_mode0)(uint32_t line);
void __not_in_flash_func(draw_framebuffer_mode1)(uint32_t line);
void __not_in_flash_func(draw_framebuffer_mode2)(uint32_t line);
void __not_in_flash_func(draw_framebuffer7_text)(uint32_t line);
void __not_in_flash_func(draw_framebuffer7_graphic)(uint32_t line);
void __not_in_flash_func(mix_framebuffer7)(void);

#define BASEPOS -2

#ifdef DRAW_ON_DEMAND
// *REAL* H-Sync for emulation
void __not_in_flash_func(hsync_handler)(void) {

    pio_interrupt_clear(pio0, 0);

    if((scanline!=0)&&(gpio_get(1)==0)) { // VSYNC
        scanline=0;
        video_vsync=1;
        video_framecount++;
    } else {
        scanline++;
    }

    if((scanline%2)==0) {
        video_hsync=1;

        if((scanline>=(73+BASEPOS))&&(scanline<=(472+BASEPOS))) {        
//        if((scanline>=73)&&(scanline<=472)) {
//        if((scanline>=81)&&(scanline<=464)) {

            if(menumode) {
                draw_framebuffer_menu((scanline-(73+BASEPOS))/2);
                memcpy(vga_data_array +320*(((scanline-(73+BASEPOS))/2)%4),renderbuffer,320);    
            } else {
#if defined(MACHINE_PA7007)
                if(ioport[0x8]&0x80) {
                    draw_framebuffer7_graphic((scanline-(73+BASEPOS))/2);  // Fine Graphics                    
                } else {
                    draw_framebuffer7_text((scanline-(73+BASEPOS))/2);
                }
#else
                if(ioport[0x8]&0x40) {
                    draw_framebuffer_mode1((scanline-(73+BASEPOS))/2);
                } else {
                    if(ioport[0x8]&0x80) {
                        draw_framebuffer_mode2((scanline-(73+BASEPOS))/2);                    
                    } else {
                        draw_framebuffer_mode0((scanline-(73+BASEPOS))/2);
                    }
                }
                memcpy(vga_data_array +320*(((scanline-(73+BASEPOS))/2)%4),renderbuffer,320);
#endif
            }
        }

    } else {

#if defined(MACHINE_PA7007)
        if(menumode==0) {

            if((scanline>=(74+BASEPOS))&&(scanline<=(473+BASEPOS))) {
//                if((scanline>=74)&&(scanline<=473)) {


                mix_framebuffer7();
                memcpy(vga_data_array +320*(((scanline-(74+BASEPOS))/2)%4),renderbuffer,320);
                         
            }

        }
#endif

    }

    return;

}


#else
// Virtual H-Sync for emulation
bool __not_in_flash_func(hsync_handler)(struct repeating_timer *t) {

    if(scanline%262==0) {
        video_vsync=1;
        video_framecount++;
    }

    scanline++;

    video_hsync=1;

    return true;

}
#endif


#if 0
// BEEP and PSG emulation
bool __not_in_flash_func(sound_handler)(struct repeating_timer *t) {

    uint16_t timer_diffs;
    uint32_t pon_count;
    uint16_t master_volume;

    uint8_t tone_output[3], noise_output[3], envelope_volume;

//    pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,psg_master_volume);

    // PSG

    master_volume = 0;

    // psg_master_volume = master_volume / 3;    // Add beep

    // if (psg_master_volume > 255)
    //     psg_master_volume = 255;

    return true;
}
#endif

// CTC emulation
bool __not_in_flash_func(ctc_handler)(struct repeating_timer *t) {

    uint16_t timer_diffs;
    uint32_t pon_count;
    uint16_t master_volume;
    uint8_t downcount;
    static uint8_t beep;

    uint32_t beep_on,beep_volume;
    uint8_t tone_output[3 * PSG_NUMBERS], noise_output[3 * PSG_NUMBERS];

    // CTC

    for(int i=0;i<4;i++) {

        
        if(ctc_control[i]&0x20) {
            downcount=1;
        } else {
            downcount=16;
        }

        // 1 tick = master clock / 256 (15.6kHz)
        // TC0 -> 256

            if(ctc_counter[i]!=0) {

                if(ctc_counter[i] >= downcount) {
                    ctc_counter[i]-=downcount;
                } else {
                    ctc_counter[i]=0;
                }

            } else {  // count down

//                if(ctc_control[i]&8==0) {

                    if(ctc_timeconstant[i]==0) {
                        ctc_counter[i]=256;
                    } else {
                        ctc_counter[i]=ctc_timeconstant[i];
                    }

//                } 

                if(ctc_control[i]&0x80) {
                    ctc_enable_irq[i]=1;
                }

                // if((i==1) && (beep_enable)) {
                //     //  beep for channel 1
                //     if(beep) {
                //         beep=0;
                //         pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,0);
                //     } else {
                //         beep=1;
                //         pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,255);
                //     }
                // }
            }

    }

    if(beep_enable) {
            if(beep_count>16) {
                beep_count-=16;
            } else {
                beep_count+=beep_interval;
                if(beep_count>16) {
                    beep_count-=16;
                } else {    // ??

                }
                if(beep) {
                    beep=0;
#if defined(MACHINE_PA7010)
                    pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,0);
#endif
                } else {
                    beep=1;
#if defined(MACHINE_PA7010)
                    pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,255);
#endif
                }                
            }
    }

    // PSG Emulation
#if defined(MACHINE_PA7007)

    if(!sound_mute) {
        pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,psg_master_volume);
    }

        // Run Noise generator

        for (int i = 0; i < PSG_NUMBERS; i++) {

            psg_noise_counter[i] += SAMPLING_INTERVAL;
            if (psg_noise_counter[i] > psg_noise_interval[i]) {
                psg_noise_seed[i] = (psg_noise_seed[i] >> 1)
                        | (((psg_noise_seed[i] << 14) ^ (psg_noise_seed[i] << 16))
                                & 0x10000);
                psg_noise_output[i] = psg_noise_seed[i] & 1;
                psg_noise_counter[i] -= psg_noise_interval[i];
            }
    
        }
    
        // Run Oscillator
    
        for (int i = 0; i < 3 * PSG_NUMBERS; i++) {
            pon_count = psg_osc_counter[i] += SAMPLING_INTERVAL;
            if (pon_count < (psg_osc_interval[i] / 2)) {
    //            tone_output[i] = psg_tone_on[i];
                tone_output[i] = 1;
            } else if (pon_count > psg_osc_interval[i]) {
                psg_osc_counter[i] -= psg_osc_interval[i];
    //            tone_output[i] = psg_tone_on[i];
                tone_output[i] = 1;
            } else {
                tone_output[i] = 0;
            }
        }
    
        // Mixer
    
        master_volume = 0;
        psg_note_count=0;
    
        for (int i = 0; i < PSG_NUMBERS; i++) {
            for (int j = 0; j < 3; j++) {
                if(tone_output[j+i*3]) {
                    master_volume+=psg_volume[psg_register[j*2+i*8+1]];
    //                master_volume+=psg_volume[32];
                } 
            }
            if(psg_noise_output[i]) {
                master_volume+=psg_volume[psg_register[7+i*8]];
            }
        }
    
        // count enable channels
    
        for (int i = 0; i < PSG_NUMBERS; i++) {
            for (int j = 0; j < 4; j++) {
                if(psg_register[j*2+i*8+1]!=0xf) {
                        psg_note_count++;
                }            
            }
        }
    
        if(beep_enable) {
            psg_note_count++;
            master_volume+=(beep*256);
        }
    
    //    psg_master_volume = master_volume / (3 * PSG_NUMBERS);
        psg_master_volume = master_volume / psg_note_count;

#endif

    return true;

}

// PSG virtual registers
// 0: CH0 Freq
// 1: CH0 Volume
// 6: Noise Freq
// 7: Noise Volume

void psg_write(uint32_t psg_no,uint32_t data) {

    uint32_t channel,freqdiv,freq;

    if(data&0x80) {

        channel=(data&0x60)>>5;
        psg_freq_write[psg_no]=0;

        switch((data&0x70)>>4) {

            // Frequency

            case 0:
            case 2:
            case 4:

                psg_register[psg_no*8+channel*2]=data&0xf;
                psg_freq_write[psg_no]=channel;
                break;

            case 6:  // WIP
                psg_register[psg_no*8+6]=data&0xf;
                switch(data&3){
                    case 0:
                        freqdiv=512;
                        break;
                    case 1:
                        freqdiv=1024;
                        break;
                    case 2:
                        freqdiv=2048;
                        break;
                    case 3:
                        freqdiv=psg_register[psg_no*8+4];
                }


                if(freqdiv==0) {
                    psg_noise_interval[psg_no]=UINT32_MAX;
                    return;
                }

                freq= psg_master_clock / freqdiv;
                freq>>=5;

                if(freq==0) {
                    psg_noise_interval[psg_no]=UINT32_MAX; 
                } else {
                    psg_noise_interval[psg_no]= TIME_UNIT/freq;
                    psg_noise_counter[psg_no]=0;
                }

                break;

            // volume

            case 1:
            case 3:
            case 5:
            case 7:
            
                psg_register[psg_no*8+channel*2+1]=data&0xf;

                break;

        }

    } else {

        uint32_t noise_flag=psg_register[psg_no*8+6]&3;
        
        channel=psg_freq_write[psg_no];
        psg_register[psg_no*8+channel*2]|=(data&0x3f)<<4;

        freqdiv=psg_register[psg_no*8+channel*2];

        if(freqdiv==0) {
            psg_osc_interval[psg_no*3+channel]=UINT32_MAX;
            if(noise_flag==3) {
                psg_noise_interval[psg_no]=UINT32_MAX;
            }
            return;
        }

        freq= psg_master_clock / freqdiv;
        freq>>=5;

        if(freq==0) {
            psg_osc_interval[psg_no*3+channel]=UINT32_MAX; 
            if(noise_flag==3) {
                psg_noise_interval[psg_no]=UINT32_MAX;
            }
        } else {
            psg_osc_interval[psg_no*3+channel]= TIME_UNIT/freq;
            psg_osc_counter[psg_no*3+channel]=0;
            if(noise_flag==3) {
                psg_noise_interval[psg_no]=TIME_UNIT/freq;
                psg_noise_counter[psg_no]=0;
            }

        }

    }    
}

void __not_in_flash_func(uart_handler)(void) {

    uint8_t ch;

    if(uart_is_readable(uart0)) {
        ch=uart_getc(uart0);
        if(uart_count==0) {
            uart_nibble=fromhex(ch)<<4;
            uart_count++;
        } else {
            ch=fromhex(ch)+uart_nibble;
            uart_count=0;

            if(uart_read_ptr==uart_write_ptr+1) {  // buffer full
                return;
            }
            if((uart_read_ptr==0)&&(uart_write_ptr==31)) {
                return;
            }

            uart_rx[uart_write_ptr]=ch;
            uart_write_ptr++;
            if(uart_write_ptr>31) {
                uart_write_ptr=0;
            }
        }
    }

}

// PASOPIA TAPE format ( MAX 256Bytres/block for BASIC )
// 1 x ?
// Preample (AA*10)
// Size (2)
// DATA (MAX256)
// Checksum (1)
// Postample (AA*1)
// 0 x ?

uint8_t tapein() {

    static uint32_t tape_diff_cycles;
    static uint8_t tape_bits,tape_file_data;
    static uint8_t tape_signal;
    static uint16_t tape_data;
    static uint16_t tape_byte;
    static uint32_t tape_header_bits;
    static uint8_t tape_baud;
    static uint8_t tape_last_bits;
    static uint16_t tape_byte_count,tape_block_size;
    static uint8_t tape_block_end;

    if(tape_ready==0) {
        return 0;
    }
    
    if(load_enabled==0) {
        return 0;
    }
    
    load_enabled=2;
    
    tape_diff_cycles=cpu_cycles-tape_cycles;
    //    tape_cycles=cpu_cycles;
    //    tape_last_bits=data;
    
    if(tape_phase==1) {
    
        if((tape_diff_cycles<TAPE_LONG)&&(tape_last_bits==1)) {
//printf("%d %d %d ",tape_signal,tape_last_bits,tape_diff_cycles);
            return tape_signal;
        }
        if((tape_diff_cycles<TAPE_SHORT)&&(tape_last_bits==0)) {
//    printf("B %d %d %d ",tape_signal,tape_last_bits,tape_diff_cycles);
            return tape_signal;
        }
    
//        printf("[D:%d,%d,%d,%d,%x]",tape_diff_cycles,tape_signal,tape_last_bits,tape_bits,tape_file_data);
    
        tape_cycles=cpu_cycles;
    
        if(tape_bits==0) { // start bit
            if(tape_signal==0) {   // 0 -> 1
                tape_signal=1;
                tape_bits++;
                tape_last_bits=(tape_file_data&1);
                return 1;
            } else {            // 1 -> 0
                tape_signal=0;
                return 0;
            }
        }
        if(tape_bits<9) {
            if(tape_signal==0) {   // 0 -> 1
                tape_signal=1;
                if(tape_bits<8) {
                    tape_last_bits=tape_file_data>>tape_bits;
                    tape_last_bits&=1;
                    tape_bits++;
                    return 1;
                } else {
                    tape_last_bits=1;
                    tape_bits++;
                    return 1;                    
                }
                return 1;
            } else {            // 1 -> 0
                tape_signal=0;
                return 0;
            }
        }
    
                if(tape_signal==0) {   // 1 -> 0
    
                    // if(tape_last_bits) {
                    //     if(tape_half_bit==0) {
                    //         tape_half_bit++;
                    //         tape_signal=0;
                    //         return 0;
                    //     }
                    // }
                    if(tape_bits==9) {
                        tape_last_bits=0;
                        tape_bits=0;
                        tape_signal=1;
                        
                        if(tape_byte_count>=tape_block_size+14) {
                            tape_last_bits=1;
                            tape_bits=1;
                            tape_signal=1;
                            tape_phase=2;
                            return 1;
                        }

                        lfs_file_read(&lfs,&lfs_file,&tape_file_data,1);
                        tape_data=tape_file_data;
                        tape_ptr++;

                        tape_byte_count++;
                        
//printf("[%x:%x]",tape_byte_count,tape_file_data);

                        if (tape_byte_count==11) {
                            tape_block_size&=0xff00;
                            tape_block_size+=tape_file_data;
                        } 
                        if (tape_byte_count==12) {
                            tape_block_size&=0xff;
                            tape_block_size+=(tape_file_data)<<8;

//                        printf("[BS:%x]",tape_block_size);

                        }                         


                        return 1;                    
                    }
                } else {            // 1 -> 0
                    tape_signal=0;
                    return 0;
                }
            
        } else if (tape_phase==0) {
            // Header 
            // Return '1'
    
            if((tape_diff_cycles)>TAPE_LONG*10) { // First 'h'
                tape_cycles=cpu_cycles;
                tape_signal=0;
                tape_header_bits=0;
                return 0;
            }
            if(tape_diff_cycles>TAPE_LONG) {
    
//        printf("[H:%d,%d,%d,%d]",tape_diff_cycles,tape_signal,tape_header_bits,tape_phase);
    
                tape_cycles=cpu_cycles;
                if(tape_signal==0) {
                    tape_signal=1;
                    tape_header_bits++;
                    if(tape_header_bits>800) {

                        if(tape_ptr!=0) {
                            lfs_file_read(&lfs,&lfs_file,&tape_file_data,1);                            
                        }

                        lfs_file_read(&lfs,&lfs_file,&tape_file_data,1);

                        tape_bits=0;
                        tape_ptr++;
                        tape_last_bits=0;
                        tape_phase++;
                        tape_header_bits=0;

                        tape_byte_count=1;
                        tape_block_size=0;

                    }
                    return 1;
                } else {
                    tape_signal=0;
                    return 0;
                }
            } else {
                return tape_signal;
            }
        } else { 

            // Footer 
            // Return '0'
    
            if(tape_diff_cycles>TAPE_SHORT) {
    
    //    printf("[H:%d,%d,%d,%d]",tape_diff_cycles,tape_signal,tape_header_bits,tape_phase);
    
                tape_cycles=cpu_cycles;
                if(tape_signal==0) {
                    tape_signal=1;
                    tape_header_bits++;
                    if(tape_header_bits>800) {
                        tape_phase=0;                    
                    }
                    return 1;
                } else {
                    tape_signal=0;
                    return 0;
                }
            } else {
                return tape_signal;
            }
        }
        
        
}

void tapeout(uint8_t data) {

    static uint32_t tape_diff_cycles;
    static uint8_t tape_bits,tape_file_data;
    static uint16_t tape_data;
    static uint16_t tape_byte;
    static uint8_t tape_baud;
    static uint8_t tape_last_bits;
    
    if(tape_ready==0) {
        return;
    }

    if(tape_last_bits!=data) {

        tape_diff_cycles=cpu_cycles-tape_cycles;
        tape_cycles=cpu_cycles;
        tape_last_bits=data;

//        printf("[%d:%d]",tape_diff_cycles,data);
        
        if(tape_diff_cycles>TAPE_LONG*10) {   // First bit
            tape_phase=0;
            tape_bits=0;
            tape_byte=0;
        } 

        if(tape_phase==0) { // Header
            if(data==0) {
                if(tape_diff_cycles<(TAPE_LONG+TAPE_SHORT)/2) {  // First Zero
                    tape_phase=1;
                    tape_bits=1;
                    tape_data=0;
                }
            }
        } else if(tape_phase==1) {
            if(data==0) {
                tape_bits++;
                tape_data>>=1;
                if(tape_diff_cycles>(TAPE_LONG+TAPE_SHORT)/2) {
                    tape_data|=0x100;
                }

//        printf("[%d:%x]",tape_bits,tape_data);

                if(tape_bits==10) {
                    if((tape_data&0x100)==0) { // Postample
                        tape_phase=2;
                        return;
                    }
                    tape_file_data=(tape_data&0xff);
                    if(save_enabled) {
                        save_enabled=2;
                        tape_count++;
                        lfs_file_write(&lfs,&lfs_file,&tape_file_data,1);
                    } else {
                        printf("%02x",tape_file_data);
                    }
                    tape_bits=0;
                    tape_data=0;
                }
            }
        }

    }

}



static inline void video_cls() {
        memset(menuram,0,0x800);
}

static inline void video_scroll() {

//    memmove(vga_data_array, vga_data_array + VGA_PIXELS_X*10, (VGA_PIXELS_X*(VGA_PIXELS_X-10)));
//    memset(vga_data_array + (VGA_CHARS_X*(VGA_PIXELS_X-10)), 0, VGA_PIXELS_X*10);

}

static inline void video_print(uint8_t *string) {

    int len;
    uint8_t fdata;
    uint32_t vramindex;

    len = strlen(string);

    for (int i = 0; i < len; i++) {

        uint8_t ch=string[i];
        menuram[cursor_x+cursor_y*VGA_CHARS_X]=ch;
        menuram[cursor_x+cursor_y*VGA_CHARS_X+0x800]=fbcolor;

        cursor_x++;
        if (cursor_x >= VGA_CHARS_X) {
            cursor_x = 0;
            cursor_y++;
            if (cursor_y >= VGA_CHARS_Y) {
                video_scroll();
                cursor_y = VGA_CHARS_Y - 1;
            }
        }
    }

}

void draw_menu(void) {

    cursor_x=2;
    cursor_y=2;
    fbcolor=7;
      video_print("                                    ");
    for(int i=3;i<19;i++) {
        cursor_x=2;
        cursor_y=i;
        video_print("                                    ");
    }

    cursor_x=2;
    cursor_y=19;
    fbcolor=7;
    video_print("                                    ");

}

int draw_files(int num_selected,int page) {

    lfs_dir_t lfs_dirs;
    struct lfs_info lfs_dir_info;
    uint32_t num_entry=0;
    unsigned char str[16];

    int err= lfs_dir_open(&lfs,&lfs_dirs,"/");

    if(err) return -1;

    // for(int i=0;i<LFS_LS_FILES;i++) {
    //     cursor_x=22;
    //     cursor_y=i+3;
    //     fbcolor=7;
    //     video_print("             ");
    // }

    while(1) {

        int res= lfs_dir_read(&lfs,&lfs_dirs,&lfs_dir_info);
        if(res<=0) {

            if(num_entry>=LFS_LS_FILES*(page+1)) {
                break;
            }

            if((num_entry%LFS_LS_FILES)!=(LFS_LS_FILES-1)) {
                for(int i=num_entry%LFS_LS_FILES;i<LFS_LS_FILES;i++) {
                    cursor_x=22;
                    cursor_y=i+3;
                    fbcolor=7;
                    video_print("                  ");                    
                }
            }

            break;
        }

        cursor_x=28;
        cursor_y=18;
        fbcolor=7;
        sprintf(str,"Page %02d",page+1);

        video_print(str);

        switch(lfs_dir_info.type) {

            case LFS_TYPE_DIR:
                break;
            
            case LFS_TYPE_REG:

                if((num_entry>=LFS_LS_FILES*page)&&(num_entry<LFS_LS_FILES*(page+1))) {

                    cursor_x=22;
                    cursor_y=num_entry%LFS_LS_FILES+3;

                    if(num_entry==num_selected) {
                        fbcolor=0x70;
                        memcpy(filename,lfs_dir_info.name,16);
                    } else {
                        fbcolor=7;
                    }

                    snprintf(str,16,"%s            ",lfs_dir_info.name);
                    video_print(str);
//                    video_print(lfs_dir_info.name);

                }

                num_entry++;

                break;

            default:
                break; 

        }

    }

    lfs_dir_close(&lfs,&lfs_dirs);

    return num_entry;

}

int file_selector(void) {

    uint32_t num_selected=0;
    uint32_t num_files=0;
    uint32_t num_pages=0;

    num_files=draw_files(-1,0);

    if(num_files==0) {
         return -1;
    }

    while(1) {

        while(video_vsync==0) ;
        video_vsync=0;

        draw_files(num_selected,num_selected/LFS_LS_FILES);

        tuh_task();

        if(keypressed==0x52) { // up
            keypressed=0;
            if(num_selected>0) {
                num_selected--;
            }
        }

        if(keypressed==0x51) { // down
            keypressed=0;
            if(num_selected<num_files-1) {
                num_selected++;
            }
        }

        if(keypressed==0x4b) { // Pageup
            keypressed=0;
            if(num_selected>=LFS_LS_FILES) {
                num_selected-=LFS_LS_FILES;
            }
        }

        if(keypressed==0x4e) { // Pagedown
            keypressed=0;
            if(num_selected<num_files-LFS_LS_FILES) {
                num_selected+=LFS_LS_FILES;
            }
        }

        if(keypressed==0x28) { // Ret
            keypressed=0;

            return 0;
        }

        if(keypressed==0x29 ) {  // ESC

            return -1;

        }

    }
}

int enter_filename() {

    unsigned char new_filename[16];
    unsigned char str[32];
    uint8_t keycode;
    uint32_t pos=0;

    memset(new_filename,0,16);

    while(1) {

        sprintf(str,"Filename:%s  ",new_filename);
        cursor_x=3;
        cursor_y=18;
        video_print(str);

        while(video_vsync==0) ;
        video_vsync=0;

        tuh_task();

        if(keypressed!=0) {

            if(keypressed==0x28) { // enter
                keypressed=0;
                if(pos!=0) {
                    memcpy(filename,new_filename,16);
                    return 0;
                } else {
                    return -1;
                }
            }

            if(keypressed==0x29) { // escape
                keypressed=0;
                return -1;
            }

            if(keypressed==0x2a) { // backspace
                keypressed=0;

                cursor_x=3;
                cursor_y=18;
                video_print("Filename:          ");

                new_filename[pos]=0;

                if(pos>0) {
                    pos--;
                }
            }

            if(keypressed<0x4f) {
                keycode=usbhidcode[keypressed*2];
                keypressed=0;

                if(pos<7) {

                    if((keycode>0x20)&&(keycode<0x5f)&&(keycode!=0x2f)) {

                        new_filename[pos]=keycode;
                        pos++;

                    }

                }
            }

        }
    }

}

// MENU mode
// output to render buffer
void __not_in_flash_func(draw_framebuffer_menu)(uint32_t line) {

    uint32_t scany,scanyy;
    uint32_t fontw1,fontw2;
    uint16_t attribute,vramaddr,ch;
    uint8_t screenwidth,scanx,bgcolor,fontdata,fgcolor;

    scany=line/8;
    scanyy=line%8;

    vramaddr=scany*VGA_CHARS_X;

    // check cursor position

    for(int i=0;i<VGA_CHARS_X;i++) {

        attribute=menuram[vramaddr+0x800];
        ch=menuram[vramaddr++];

        bgcolor=(attribute&0xf0)>>4;
        fgcolor=attribute&0xf;

        fontdata=fontrom[ch*8+scanyy];
            
        fontw1=bitexpand80[fontdata*2]*fgcolor+bitexpand80[fontdata*2+1]*bgcolor;
        renderbuffer[i]=fontw1;

    }
}

// mode 0
// TEXT mode
// output to render buffer
void __not_in_flash_func(draw_framebuffer_mode0)(uint32_t line) {

    uint32_t scany,scanyy;
    uint32_t fontw1,fontw2;
    uint16_t attribute,vramaddr,startaddr,cursoraddr;
    uint8_t screenwidth,screenwide,screenhight,scanx,bgcolor,fontdata;
    uint8_t ch,ch9;
    static uint8_t fgcolor;

    // check ctrc 

    startaddr=(crtc[12]<<8) + crtc[13];
    cursoraddr=(crtc[0xe]<<8) + crtc[0xf];

    screenhight=crtc[9]+1;

    scany=line/8;
    scanyy=line%8;

    if(scany>=crtc[6]) {
        memset(renderbuffer,0,320);
        return;
    }

    // check cursor positon

    if(crtc[10]&0x40) { // Blinks
        if(crtc[10]&0x20) {
            if((video_framecount%32)>16) {
                cursoraddr=0xffff;
            }
        } else {
            if((video_framecount%16)>8) {
                cursoraddr=0xffff;
            }
        }
    } else {
        if(crtc[10]&0x20) { // Not displayed
            cursoraddr=0xffff;
        }
    }

    if((scanyy<(crtc[10]&0x7))||(crtc[11]<scanyy)) {
            cursoraddr=0xffff;
    }

    attribute=0;
    bgcolor=ioport[8]&7;

    if(ioport[8]&0x20) {    // 80 chars
        screenwide=1;
    } else {
        screenwide=0;
    }

    screenwidth=crtc[1];

    //

    vramaddr=scany*screenwidth+startaddr;
    scanx=0;

    if(screenwide) {

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[(vramaddr)&0x7ff];
            ch=vram[(vramaddr)&0x7ff];

            if(((ch&0xf8)==0xf8)&&(ch9==0)) { // change attribute
                fgcolor=ch&7;
                if(scanx)  renderbuffer[scanx++]=0;
            } else {
                // cursor position check

                if(vramaddr==cursoraddr) {
                    fontdata=0xff;
                } else {
                    fontdata=fontrom[ch*8+scanyy];
                }
                
                if(ch9) fontdata=~fontdata;

                fontw1=bitexpand80[fontdata*2]*fgcolor+bitexpand80[fontdata*2+1]*bgcolor;
                renderbuffer[scanx++]=fontw1;

            }

            vramaddr++;
        }

    } else {

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[(vramaddr)&0x7ff];
            ch=vram[(vramaddr)&0x7ff];

            if((ch&0xf8)==0xf8) { // change attribute
                fgcolor=ch&7;
                if(scanx)  renderbuffer[scanx++]=0;
                if(scanx)  renderbuffer[scanx++]=0;
            } else {
                // cursor position check

                if(vramaddr==cursoraddr) {
                    fontdata=0xff;
                } else {
                    fontdata=fontrom[ch*8+scanyy];
                }
                
                if(ch9) fontdata=~fontdata;

                fontw1=bitexpand40[fontdata*4+1]*fgcolor+bitexpand40[fontdata*4+3]*bgcolor;
                renderbuffer[scanx++]=fontw1;
                fontw1=bitexpand40[fontdata*4]*fgcolor+bitexpand40[fontdata*4+2]*bgcolor;
                renderbuffer[scanx++]=fontw1;

            }

            vramaddr++;
        }

    }
}

// mode 1 (and 1.5)
// Graphic mode
// output to render buffer
void __not_in_flash_func(draw_framebuffer_mode1)(uint32_t line) {

    uint32_t scany,scanyy,scany2;
    uint32_t fontw1,fontw2;
    uint16_t attribute,vramaddr,startaddr,vramoffset,cursoraddr;
    uint8_t screenwidth,screenwide,screenhight,scanx,bgcolor,fontdata;
    uint8_t ch,ch9;
    static uint8_t fgcolor;

    // check ctrc 

    startaddr=(crtc[12]<<8) + crtc[13];
    cursoraddr=(crtc[0xe]<<8) + crtc[0xf];

    screenhight=crtc[9]+1;

    scany=line/8;
    scanyy=line%8;

    if(scany>=crtc[6]) {
        memset(renderbuffer,0,320);
        return;
    }

    // check cursor position

    if(crtc[10]&0x40) { // Blinks
        if(crtc[10]&0x20) {
            if((video_framecount%32)>16) {
                cursoraddr=0xffff;
            }
        } else {
            if((video_framecount%16)>8) {
                cursoraddr=0xffff;
            }
        }
    } else {
        if(crtc[10]&0x20) { // Not displayed
            cursoraddr=0xffff;
        }
    }

    if((scanyy<(crtc[10]&0x7))||(crtc[11]<scanyy)) {
            cursoraddr=0xffff;
    }

    attribute=0;
    bgcolor=ioport[8]&7;

    if(ioport[8]&0x20) {    // 80 chars
        screenwide=1;
    } else {
        screenwide=0;
    }

    screenwidth=crtc[1];

    //

    if((ioport[8]&0x80)==0) {
        vramoffset=(scanyy>>1)*0x1000;
    } else {            // Mode 1.5
        vramoffset=scanyy*0x800;
    }

    vramaddr=scany*screenwidth+startaddr;

    scanx=0;

    if(screenwide) {

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[((vramaddr)&0x7ff)+vramoffset];
            ch=vram[((vramaddr)&0x7ff)+vramoffset];

            if(((ch&0xf8)==0xf8)&&(ch9==0)) { // change attribute
                fgcolor=ch&7;
                if(scanx)  renderbuffer[scanx++]=0;
            } else {
                // cursor position check

                if(ch9) {   // Graphics
                    fontw1=((ch&0x70)>>4)*0x1111;
                    fontw1|=(ch&0x7)*0x11110000;
                    renderbuffer[scanx++]=fontw1;
                } else {
                    if(vramaddr==cursoraddr) {
                        fontdata=0xff;
                    } else {
                        fontdata=fontrom[ch*8+scanyy];
                    }

                    fontw1=bitexpand80[fontdata*2]*fgcolor+bitexpand80[fontdata*2+1]*bgcolor;
                    renderbuffer[scanx++]=fontw1;
                }
            }

            vramaddr++;
        }

    } else {

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[((vramaddr)&0x7ff)+vramoffset];
            ch=vram[((vramaddr)&0x7ff)+vramoffset];

            if(((ch&0xf8)==0xf8)&&(ch9==0)) { // change attribute
                fgcolor=ch&7;
                if(scanx)  renderbuffer[scanx++]=0;
                if(scanx)  renderbuffer[scanx++]=0;
            } else {

                // cursor position check

                if(ch9) { // Graphics

                    fontw1=((ch&0x70)>>4)*0x11111111;
                    renderbuffer[scanx++]=fontw1;

                    fontw1=(ch&0x7)*0x11111111;
                    renderbuffer[scanx++]=fontw1;

                } else {

                    if(vramaddr==cursoraddr) {
                        fontdata=0xff;
                    } else {
                        fontdata=fontrom[ch*8+scanyy];
                    }
                
                    fontw1=bitexpand40[fontdata*4+1]*fgcolor+bitexpand40[fontdata*4+3]*bgcolor;
                    renderbuffer[scanx++]=fontw1;
                    fontw1=bitexpand40[fontdata*4]*fgcolor+bitexpand40[fontdata*4+2]*bgcolor;
                    renderbuffer[scanx++]=fontw1;

                }
            }

            vramaddr++;
        }
    }

    // fill blank

}

// mode 2
// Find Graphic mode
// output to render buffer
void __not_in_flash_func(draw_framebuffer_mode2)(uint32_t line) {

    uint32_t scany,scanyy,scany2;
    uint32_t fontw1,fontw2;
    uint16_t attribute,vramaddr,startaddr,vramoffset,cursoraddr;
    uint8_t screenwidth,screenwide,screenhight,scanx,bgcolor,fontdata;
    uint8_t ch,ch9;
    static uint8_t fgcolor;

    // check ctrc 

    startaddr=(crtc[12]<<8) + crtc[13];
    cursoraddr=(crtc[0xe]<<8) + crtc[0xf];

    screenhight=crtc[9]+1;

    scany=line/8;
    scanyy=line%8;

    if(scany>=crtc[6]) {
        memset(renderbuffer,0,320);
        return;
    }

    // check cursor position

    if(crtc[10]&0x40) { // Blinks
        if(crtc[10]&0x20) {
            if((video_framecount%32)>16) {
                cursoraddr=0xffff;
            }
        } else {
            if((video_framecount%16)>8) {
                cursoraddr=0xffff;
            }
        }
    } else {
        if(crtc[10]&0x20) { // Not displayed
            cursoraddr=0xffff;
        }
    }

    if((scanyy<(crtc[10]&0x7))||(crtc[11]<scanyy)) {
            cursoraddr=0xffff;
    }

    attribute=0;
    bgcolor=ioport[8]&7;

    if(ioport[8]&0x20) {    // 80 chars
        screenwide=1;
    } else {
        screenwide=0;
    }

    screenwidth=crtc[1];

    //

    vramoffset=scanyy*0x800;

    vramaddr=scany*screenwidth+startaddr;

    scanx=0;

    if(screenwide) {

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[((vramaddr)&0x7ff)+vramoffset];
            ch=vram[((vramaddr)&0x7ff)+vramoffset];

            if(((ch&0xf8)==0xf8)&&(ch9==0)) { // change attribute
                fgcolor=ch&7;
                if(scanx)  renderbuffer[scanx++]=0;
            } else {
                // cursor position check

                if(vramaddr==cursoraddr) {
                    fontdata=0xff;
                } else {
                    if(ch9) {
                        fontdata=ch;
                    } else {
                        fontdata=fontrom[ch*8+scanyy];
                    }
                }

                fontw1=bitexpand80[fontdata*2]*fgcolor+bitexpand80[fontdata*2+1]*bgcolor;
                renderbuffer[scanx++]=fontw1;
                
            }

            vramaddr++;

        }

    } else {

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[((vramaddr)&0x7ff)+vramoffset];
            ch=vram[((vramaddr)&0x7ff)+vramoffset];

            if(((ch&0xf8)==0xf8)&&(ch9==0)) { // change attribute
                fgcolor=ch&7;
                if(scanx)  renderbuffer[scanx++]=0;
                if(scanx)  renderbuffer[scanx++]=0;
            } else {

                // cursor position check

                if(vramaddr==cursoraddr) {
                    fontdata=0xff;
                } else {
                    if(ch9) {
                        fontdata=ch;
                    } else {
                        fontdata=fontrom[ch*8+scanyy];
                    }
                }
                
                fontw1=bitexpand40[fontdata*4+1]*fgcolor+bitexpand40[fontdata*4+3]*bgcolor;
                renderbuffer[scanx++]=fontw1;
                fontw1=bitexpand40[fontdata*4]*fgcolor+bitexpand40[fontdata*4+2]*bgcolor;
                renderbuffer[scanx++]=fontw1;
                
            }

            vramaddr++;

        }
    }

}

// PA7007
// TEXT mode
// output to render buffer & gv render buffer
void __not_in_flash_func(draw_framebuffer7_text)(uint32_t line) {

    uint32_t scany,scanyy;
    uint32_t fontw1,fontw2,fontw3;
    uint16_t attribute,vramaddr,gvramaddr,startaddr,cursoraddr,crtcaddr;
    uint8_t screenwidth,screenwide,screenhight,scanx,bgcolor,fontdata,blinkmode;
    uint8_t ch,ch9,colormode,monofgcolor,datab,datar,gvcolor,gvcolor1,gvcolor2,txtcolor;
    static uint8_t fgcolor;

    // check ctrc 

    startaddr=(crtc[12]<<8) + crtc[13];
    cursoraddr=(crtc[0xe]<<8) + crtc[0xf];

    screenhight=crtc[9]+1;

    scany=line/8;
    scanyy=line%8;

    if(scany>=crtc[6]) {
        memset(renderbuffer,0,320);
        return;
    }

    // check cursor positon

    if(crtc[10]&0x40) { // Blinks
        if(crtc[10]&0x20) {
            if((video_framecount%32)>16) {
                cursoraddr=0xffff;
            }
        } else {
            if((video_framecount%16)>8) {
                cursoraddr=0xffff;
            }
        }
    } else {
        if(crtc[10]&0x20) { // Not displayed
            cursoraddr=0xffff;
        }
    }

    if((scanyy<(crtc[10]&0x7))||(crtc[11]<scanyy)) {
            cursoraddr=0xffff;
    }

    attribute=0;
//    bgcolor=ioport[8]&7;
    bgcolor=8;      // Background color is always black in PA7007

    if(ioport[8]&0x20) {    // 80 chars
        screenwide=1;
    } else {
        screenwide=0;
    }

    screenwidth=crtc[1];

    //

    vramaddr=scany*(screenwidth*8)+((ioport[0xd]&0x70)>>4);
    gvramaddr=scany*(screenwidth*8)+scanyy;
    if((startaddr==0x400)&&(screenwide==0)) {
        vramaddr+=0x2000;
        gvramaddr+=0x2000;
    }
    crtcaddr=scany*screenwidth+startaddr;

    gvramaddr&=0x3fff;
    vramaddr&=0x3fff;

    scanx=0;

    colormode=ioport[0x8]&8;
    monofgcolor=ioport[0x8]&7;

    blinkmode=ioport[0xe]&0x20;

    if(screenwide) {

        // for TEXT

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[vramaddr];
            ch=vram[vramaddr];

            // if(crtcaddr==cursoraddr) {
            //     fontdata=0xff;
            // } else {
            //     fontdata=fontrom[ch*8+scanyy];
            // }

            // Select color

            if(colormode) {
                fgcolor=monofgcolor;
                if((blinkmode)&&(!(ch9&4))) {
                    fontdata=0;
                } else {
                    if(crtcaddr==cursoraddr) {
                        fontdata=0xff;
                    } else {
                        fontdata=fontrom[ch*8+scanyy];
                    }
                }
            } else {
                fgcolor=ch9&7;
                if(crtcaddr==cursoraddr) {
                    fontdata=0xff;
                } else {
                    fontdata=fontrom[ch*8+scanyy];
                }
            }

            if(ch9&8) fontdata=~fontdata;

            fontw1=bitexpand80[fontdata*2]*fgcolor;
            fontw3=bitexpand80[fontdata*2+1]*0xf;

            datab=vram[gvramaddr+0x8000];
            datar=vram[gvramaddr+0x4000];

            fontw2=(bitexpand80[datar*2]<<1) | bitexpand80[datab*2];
//            fontw2=bitexpand80[datab*2];
     
            
            renderbuffer[scanx]=fontw1;
            maskbuffer[scanx]=fontw3;
            gvrenderbuffer[scanx]=fontw2;
            scanx++;
            
            vramaddr+=8;
            gvramaddr+=8;
            crtcaddr++;
        }

    } else {

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[vramaddr];
            ch=vram[vramaddr];

            // if(crtcaddr==cursoraddr) {
            //     fontdata=0xff;
            // } else {
            //     fontdata=fontrom[ch*8+scanyy];
            // }

            if(colormode) {
                fgcolor=monofgcolor;
                if((blinkmode)&&(!(ch9&4))) {
                    fontdata=0;
                } else {
                    if(crtcaddr==cursoraddr) {
                        fontdata=0xff;
                    } else {
                        fontdata=fontrom[ch*8+scanyy];
                    }
                }               
            } else {
                fgcolor=ch9&7;
                if(crtcaddr==cursoraddr) {
                    fontdata=0xff;
                } else {
                    fontdata=fontrom[ch*8+scanyy];
                }
            }
   
            if(ch9&8) fontdata=~fontdata;

            fontw1=bitexpand40[fontdata*4+1]*fgcolor;
            fontw3=bitexpand40[fontdata*4+3]*0xf;
            
            datab=vram[gvramaddr+0x8000];
            datar=vram[gvramaddr+0x4000];

            fontw2=(bitexpand40[datar*4+1]<<1)+bitexpand40[datab*4+1];
            
            renderbuffer[scanx]=fontw1;
            maskbuffer[scanx]=fontw3;
            gvrenderbuffer[scanx]=fontw2;
            scanx++;

            fontw1=bitexpand40[fontdata*4]*fgcolor;
            fontw3=bitexpand40[fontdata*4+2]*0xf;
            fontw2=(bitexpand40[datar*4]<<1)+bitexpand40[datab*4];

            renderbuffer[scanx]=fontw1;
            maskbuffer[scanx]=fontw3;
            gvrenderbuffer[scanx]=fontw2;
            scanx++;

            vramaddr+=8;
            gvramaddr+=8;
            crtcaddr++;
        }

    }
}

// PA7007
// FineGraphic mode
// output to render buffer & gv render buffer
void __not_in_flash_func(draw_framebuffer7_graphic)(uint32_t line) {

    uint32_t scany,scanyy;
    uint32_t fontw1,fontw2,fontw3;
    uint16_t attribute,vramaddr,gvramaddr,startaddr,cursoraddr,crtcaddr;
    uint8_t screenwidth,screenwide,screenhight,scanx,bgcolor,fontdata,blinkmode;
    uint8_t ch,ch9,colormode,monofgcolor,datab,datar,gvcolor,gvcolor1,gvcolor2,txtcolor;
    static uint8_t fgcolor;

    // check ctrc 

    startaddr=(crtc[12]<<8) + crtc[13];
    cursoraddr=(crtc[0xe]<<8) + crtc[0xf];

    screenhight=crtc[9]+1;

    scany=line/8;
    scanyy=line%8;

    if(scany>=crtc[6]) {
        memset(renderbuffer,0,320);
        return;
    }

    // check cursor positon

    if(crtc[10]&0x40) { // Blinks
        if(crtc[10]&0x20) {
            if((video_framecount%32)>16) {
                cursoraddr=0xffff;
            }
        } else {
            if((video_framecount%16)>8) {
                cursoraddr=0xffff;
            }
        }
    } else {
        if(crtc[10]&0x20) { // Not displayed
            cursoraddr=0xffff;
        }
    }

    if((scanyy<(crtc[10]&0x7))||(crtc[11]<scanyy)) {
            cursoraddr=0xffff;
    }

    attribute=0;
//    bgcolor=ioport[8]&7;
    bgcolor=8;      // Background color is always black in PA7007

    if(ioport[8]&0x20) {    // 80 chars
        screenwide=1;
    } else {
        screenwide=0;
    }

    screenwidth=crtc[1];

    //

//    vramaddr=scany*(screenwidth*8)+startaddr;
    gvramaddr=scany*(screenwidth*8)+scanyy;
    crtcaddr=scany*screenwidth+startaddr;

    if((startaddr==0x400)&&(screenwide==0)) {
        gvramaddr+=0x2000;
    }

    gvramaddr&=0x3fff;
//    vramaddr&=0x3fff;

    scanx=0;

    colormode=ioport[0x8]&8;
    monofgcolor=ioport[0x8]&7;

    blinkmode=ioport[0xe]&0x20;

    if(screenwide) {

        // for TEXT

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[gvramaddr];
            ch=vram[gvramaddr];

            // if(crtcaddr==cursoraddr) {
            //     fontdata=0xff;
            // } else {
            //     fontdata=fontrom[ch*8+scanyy];
            // }

            if(ch9&8) { // 3-plane graphics

                if(crtcaddr==cursoraddr) {
                    fontw1=0xffffffff;
                    fontw3=0;
                } else {
                    fontw1=0;
                    fontw3=0xffffffff;
                }

                datab=vram[gvramaddr+0x8000];
                datar=vram[gvramaddr+0x4000];
    
                fontw2= (bitexpand80[ch*2] <<2) | (bitexpand80[datar*2]<<1) | bitexpand80[datab*2];
                
                renderbuffer[scanx]=fontw1;
                maskbuffer[scanx]=fontw3;

                gvrenderbuffer[scanx]=fontw2;
                scanx++;

            } else {

                if(colormode) {
                    fgcolor=monofgcolor;
                    if((blinkmode)&&(!(ch9&4))) {
                        fontdata=0;
                    } else {
                        if(crtcaddr==cursoraddr) {
                            fontdata=0xff;
                        } else {
                            fontdata=fontrom[ch*8+scanyy];
                        }
                    }
                } else {
                    fgcolor=ch9&7;
                    if(crtcaddr==cursoraddr) {
                        fontdata=0xff;
                    } else {
                        fontdata=fontrom[ch*8+scanyy];
                    }
                }
    
                fontw1=bitexpand80[fontdata*2]*fgcolor;
                fontw3=bitexpand80[fontdata*2+1]*0xf;
    
                datab=vram[gvramaddr+0x8000];
                datar=vram[gvramaddr+0x4000];
    
                fontw2=(bitexpand80[datar*2]<<1) | bitexpand80[datab*2];
    //            fontw2=bitexpand80[datab*2];
         
                
                renderbuffer[scanx]=fontw1;
                maskbuffer[scanx]=fontw3;
                gvrenderbuffer[scanx]=fontw2;
                scanx++;

            }
            
//            vramaddr+=8;
            gvramaddr+=8;
            crtcaddr++;
        }

    } else {

        for(int i=0;i<screenwidth;i++) {

            ch9=atram[gvramaddr];
            ch=vram[gvramaddr];

            // if(crtcaddr==cursoraddr) {
            //     fontdata=0xff;
            // } else {
            //     fontdata=fontrom[ch*8+scanyy];
            // }

            if(ch9&8) { // 3-plane graphics

                if(crtcaddr==cursoraddr) {
                    fontw1=0xffffffff;
                    fontw3=0;
                } else {
                    fontw1=0;
                    fontw3=0xffffffff;
                }

                datab=vram[gvramaddr+0x8000];
                datar=vram[gvramaddr+0x4000];
    
                fontw2= (bitexpand40[ch*4+1] <<2) | (bitexpand40[datar*4+1]<<1) | bitexpand40[datab*4+1];
                
                renderbuffer[scanx]=fontw1;
                maskbuffer[scanx]=fontw3;
                gvrenderbuffer[scanx]=fontw2;
                scanx++;

                fontw2= (bitexpand40[ch*4] <<2) | (bitexpand40[datar*4]<<1) | bitexpand40[datab*4];
                
                renderbuffer[scanx]=fontw1;
                maskbuffer[scanx]=fontw3;
                gvrenderbuffer[scanx]=fontw2;
                scanx++;


            } else {

                if(colormode) {
                    fgcolor=monofgcolor;
                    if((blinkmode)&&(!(ch9&4))) {
                        fontdata=0;
                    } else {
                        if(crtcaddr==cursoraddr) {
                            fontdata=0xff;
                        } else {
                            fontdata=fontrom[ch*8+scanyy];
                        }
                    }
                } else {
                    fgcolor=ch9&7;
                    if(crtcaddr==cursoraddr) {
                        fontdata=0xff;
                    } else {
                        fontdata=fontrom[ch*8+scanyy];
                    }
                }

                fontw1=bitexpand40[fontdata*4+1]*fgcolor;
                fontw3=bitexpand40[fontdata*4+3]*0xf;
                
                datab=vram[gvramaddr+0x8000];
                datar=vram[gvramaddr+0x4000];

                fontw2=(bitexpand40[datar*4+1]<<1)+bitexpand40[datab*4+1];
                
                renderbuffer[scanx]=fontw1;
                maskbuffer[scanx]=fontw3;
                gvrenderbuffer[scanx]=fontw2;
                scanx++;

                fontw1=bitexpand40[fontdata*4]*fgcolor;
                fontw3=bitexpand40[fontdata*4+2]*0xf;
                fontw2=(bitexpand40[datar*4]<<1)+bitexpand40[datab*4];

                renderbuffer[scanx]=fontw1;
                maskbuffer[scanx]=fontw3;
                gvrenderbuffer[scanx]=fontw2;
                scanx++;

            }

//            vramaddr+=8;
            gvramaddr+=8;
            crtcaddr++;
        }

    }
}

// TEXT & Graphics Mixing
// Render buffer <= render buffer + gv render buffer
void __not_in_flash_func(mix_framebuffer7)(void) {

    uint32_t fontw,maskw,mixmask,pixeldata;
    uint8_t gvcolor,gvcolor1,gvcolor2,txtcolor;

    union bytemember {
        uint32_t w;
        uint8_t b[4];
    };

    union bytemember graphw1,graphw2,gvmask;

    for(int scanx=0;scanx<80;scanx++) {

        fontw=renderbuffer[scanx];
        maskw=maskbuffer[scanx];
        graphw1.w=gvrenderbuffer[scanx];

        // // Apply color palette
        graphw2.b[0]=color_palette_table[graphw1.b[0]];
        graphw2.b[1]=color_palette_table[graphw1.b[1]];
        graphw2.b[2]=color_palette_table[graphw1.b[2]];
        graphw2.b[3]=color_palette_table[graphw1.b[3]];

        // graphics mask
        gvmask.b[0]=color_mask_table[graphw1.b[0]];
        gvmask.b[1]=color_mask_table[graphw1.b[1]];
        gvmask.b[2]=color_mask_table[graphw1.b[2]];
        gvmask.b[3]=color_mask_table[graphw1.b[3]];

        mixmask=maskw | gvmask.w;
        fontw&=~gvmask.w;        
        pixeldata = fontw + (graphw2.w & mixmask);

//        pixeldata = fontw + (graphw1.w & mixmask);

        renderbuffer[scanx]=pixeldata;

    }

}

// Rebuild color palette table and gv mask table
void rebuild_color_palette(uint8_t index,uint8_t value) {

    uint8_t ttindex;

    if(index&8) { // odd

        index&=7;

        for(int i=0;i<16;i++) {

            ttindex=index+(i*0x10);
            color_palette_table[ttindex]&=0xf0;
            color_palette_table[ttindex]|=value;

            color_mask_table[ttindex]&=0xf0;
            if(value&8) {
                color_mask_table[ttindex]|=0xf;
            }
        }

    } else { // even

        index&=7;

        for(int i=0;i<16;i++) {

            ttindex=i+(index*0x10);
            color_palette_table[ttindex]&=0xf;
            color_palette_table[ttindex]|=value<<4;

            color_mask_table[ttindex]&=0xf;
            if(value&8) {
                color_mask_table[ttindex]|=0xf0;
            }
        }
    }

}



static inline void redraw(void){

}

//----------------------------------------------------------------------------------------------------

static inline unsigned char tohex(int b) {

    if(b==0) {
        return '0';
    } 
    if(b<10) {
        return b+'1'-1;
    }
    if(b<16) {
        return b+'a'-10;
    }

    return -1;

}

static inline unsigned char fromhex(int b) {

    if(b=='0') {
        return 0;
    } 
    if((b>='1')&&(b<='9')) {
        return b-'1'+1;
    }
    if((b>='a')&&(b<='f')) {
        return b-'a'+10;
    }

    return -1;

}

// LittleFS

int pico_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t fs_start = XIP_BASE + HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] READ: %p, %d\n", addr, size);
    
    memcpy(buffer, (unsigned char *)addr, size);
    return 0;
}

int pico_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] WRITE: %p, %d\n", addr, size);
        
    uint32_t ints = save_and_disable_interrupts();
    multicore_lockout_start_blocking();     // pause another core
    flash_range_program(addr, (const uint8_t *)buffer, size);
    multicore_lockout_end_blocking();
    restore_interrupts(ints);
        
    return 0;
}

int pico_erase(const struct lfs_config *c, lfs_block_t block)
{           
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t offset = fs_start + (block * c->block_size);
    
//    printf("[FS] ERASE: %p, %d\n", offset, block);
        
    uint32_t ints = save_and_disable_interrupts();   
    multicore_lockout_start_blocking();     // pause another core
    flash_range_erase(offset, c->block_size);  
    multicore_lockout_end_blocking();
    restore_interrupts(ints);

    return 0;
}

int pico_sync(const struct lfs_config *c)
{
    return 0;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config PICO_FLASH_CFG = {
    // block device operations
    .read  = &pico_read,
    .prog  = &pico_prog,
    .erase = &pico_erase,
    .sync  = &pico_sync,

    // block device configuration
    .read_size = FLASH_PAGE_SIZE, // 256
    .prog_size = FLASH_PAGE_SIZE, // 256
    
    .block_size = BLOCK_SIZE_BYTES, // 4096
    .block_count = HW_FLASH_STORAGE_BYTES / BLOCK_SIZE_BYTES, // 352
    .block_cycles = 16, // ?
    
    .cache_size = FLASH_PAGE_SIZE, // 256
    .lookahead_size = FLASH_PAGE_SIZE,   // 256    
};

// Keyboard

static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}


void process_kbd_report(hid_keyboard_report_t const *report) {

    int usbkey;
    uint8_t newkeysum;

    if(menumode==0) { // Emulator mode

        for(int i=0;i<16;i++) {  
            keymap[i]=0xff;
        }

        keyboard_enable_irq=0;

        if(report->modifier&0x22) {  // SHIFT
            keymap[0]&=0xfd;
        }

        if(report->modifier&0x11) {  // CTRL
            keymap[0]&=0xef;
        }

        if(report->modifier&0x44) {  // ALT = Graph
            keymap[0]&=0xfe;
        }

        for(int i=0;i<6;i++) {

            if ( report->keycode[i] ) {

                usbkey=report->keycode[i];
                if(pasopiausbcode[usbkey*2]) {
                    keymap[pasopiausbcode[usbkey*2+1]] &= ~pasopiausbcode[usbkey*2];
                    keyboard_enable_irq=1;
                }

#ifdef USE_DEBUG
            // Load TEST Program
            if(usbkey==0x44) {
                memcpy(mainram+testexec,testprog,sizeof(testprog));
            }  
#endif

            // Enter Menu
                if(usbkey==0x45) {
                    prev_report=*report;
                    menumode=1;
                    keypressed=0;
                }  
            }
        }

    prev_report=*report;

    // INT for key depressed
    // CHECK IF NEED FOR 7010

//#if defined(MACHINE_PA7007)

        newkeysum=0xff;
        for(int i=0;i<12;i++) {
            newkeysum&=keymap[i];
        }
        if(newkeysum!=keysum) {
            keyboard_enable_irq=1;
        }
        keysum=newkeysum;
//#endif

} else {  // menu mode

    for(uint8_t i=0; i<6; i++)
    {
        if ( report->keycode[i] )
        {
        if ( find_key_in_report(&prev_report, report->keycode[i]) )
        {
            // exist in previous report means the current key is holding
        }else
        {
            keypressed=report->keycode[i];
        }
        }
    } 
    prev_report = *report;
    }

}

//

void rampac_save(void) {

    uint8_t file_match;
    uint8_t file_data;

#ifdef USE_RAMPAC
    // compare RAM and RAMPAC File

    lfs_file_rewind(&lfs,&rampac_file);

    file_match=0;

    for(int i=0;i<32768;i++) {
        lfs_file_read(&lfs,&rampac_file,&file_data,1);
        if(rampac[i]!=file_data) {
            file_match=1;
        }
    }

    if(file_match==0) { // NO need to save
        return;
    }

    lfs_file_rewind(&lfs,&rampac_file);

    lfs_file_write(&lfs,&rampac_file,rampac,32768);

#endif
    return;

}

void init_rampac(void) {
#ifdef USE_RAMPAC
            memset(rampac, 0, sizeof(rampac));
            memcpy(rampac, rampac_header, sizeof(rampac_header));
            memset(rampac + 0x20, 0xff, 0x200);
            memset(rampac + 0x300, 0xfe, 0x004);
            memset(rampac + 0x304, 0xff, 0x0fc);
    
            return;
#endif
}
    

//

static uint8_t mem_read(void *context,uint16_t address)
{

    if(address>=0x8000) {

#if defined(MACHINE_PA7007)
        if(address<0xc000) {
            if(ioport[0x3c]&4) { // VRAM

                if(ioport[0xe]&0x8) {   // Pallet selected
                    return 0xff;
                }

                if(ioport[0x0c]&0x1) {  // Blue 
                    return vram[(address&0x3fff)+0x8000];
                }
                if(ioport[0x0c]&0x2) {  // Red
                    return vram[(address&0x3fff)+0x4000]; 
                }
                if(ioport[0x0c]&0x4) {  // Green
//                    if(ioport[0xe]&0x10) {  // Read attribute 
                        lastattr=atram[address&0x3fff];
//                    }    
                    return vram[(address&0x3fff)];
                }
                return 0xff;    // ?
            }

        } 

//            printf("{%x:%x:%x}",Z80_PC(cpu),address,mainram[address]);


#endif
        return mainram[address];
    }

    if(ioport[0x3c]&2) {    // RAM
        return mainram[address];
    } else {  // BASIC ROM or BIOS
        if(ioport[0x3c]&1) { 
#if defined(MACHINE_PA7007)
            if(address<0x4000) {
                return basicrom[address];
            } else {
                return biosrom[address&0x3fff]; // BIOS (7007)
            }
#else        
            return 0xff;    // PAC1 (7010)
#endif
        } else {            // BASIC
            return basicrom[address];
        }
    }

}

static void mem_write(void *context,uint16_t address, uint8_t data)
{

    uint8_t bank,permit,bankno;
    uint16_t bankprefix;

#if defined(MACHINE_PA7007)
    if((address>=0x8000)&&(address<0xc000)) {
        if(ioport[0x3c]&0x4) {
            if((ioport[0xe]&0xc)==0xc) { // Write color Pallet

                color_palette[address&0xf]=data&0xf;
                rebuild_color_palette(address&0xf,data);

                return;
            }
//            if((ioport[0xe]&8)==0) {
                if(ioport[0xc]&0x10) {  // Blue
                    if(ioport[0xc]&1) {
                        vram[(address&0x3fff)+0x8000]=data;
                    } else {
                        vram[(address&0x3fff)+0x8000]=0xff;                        
                    }
                }
                if(ioport[0xc]&0x20) {  // Red
                    if(ioport[0xc]&2) {
                        vram[(address&0x3fff)+0x4000]=data;                        
                    } else {
                        vram[(address&0x3fff)+0x4000]=0xff;
                    }
                }
                if(ioport[0xc]&0x40) {  // Green
                    if(ioport[0xc]&4) {
                        vram[(address&0x3fff)]=data;
                    } else {
                        vram[(address&0x3fff)]=0xff;
                    }
                    if(ioport[0xe]&0x10) {
                        atram[address&0x3fff]=lastattr;
                    } else {
                        atram[address&0x3fff]=ioport[0xd]&0xf;                        
                    }

                }
                return;
  //          }
        }
    }

//    printf("[%x:%x:%x]",Z80_PC(cpu),address,data);

#endif

        mainram[address]=data;

    return;

}

static uint8_t io_read(void *context, uint16_t address)
{
    uint8_t data = ioport[address&0xff];
    uint8_t b;
    uint32_t kanji_addr;

        // if((address&0xf0)==0xe0) {
        // printf("[IOR:%04x:%02x]",Z80_PC(cpu),address&0xff);
        // }

#if defined(MACHINE_PA7007)

        if(ram_ioaccess) {         // Need check
            ram_ioaccess=0;
            return mainram[address];
        }
    
#endif

    switch(address&0xff) {

        case 0x2: // VRAM READ

            // may be read control
//        if(ioport[0xa]&0x40)

            return (vram[vramptr]&0xff);

        case 0x9: // CRT info

            b=0x10;
#if defined(MACHINE_PA7007)
            if(lastattr&8) b|=0x80;
            if(lastattr&4) b|=0x4;
            if(lastattr&2) b|=0x2;
            if(lastattr&1) b|=0x1;
            if(video_hsync) b|=0x8;
#else
            if(atram[vramptr]) b|=0x80;
            if(video_hsync) b|=0x40;
#endif
            if(video_vsync) b|=0x20;


            return b;

        case 0x11: // CRTC
            return crtc[ioport[0x10]];

        case 0x18:  // PAC
        case 0x19:
        case 0x1a:
        case 0x1b:

            if(pacmode==0) {
                return 0xff;
#ifdef USE_RAMPAC                
            } else if(pacmode==1) {     // RAMPAC
                return rampac[pacptr&0x7fff];
#endif
            } else if(pacmode==2) {     // KANJIPAC
                return kanjirom[pacptr&0x1ffff];
            } else if(pacmode==3) {     // JOYPAC

                if((address&0xff)==0x1a) {  // JOYSTICK 1
                    return gamepad_info;
                } else {
                    return 0xff;
                }

            }

            return 0xff;

        case 0x21:  // CMT/LPT read

            b=0;

            if(tapein()) {
                b|=0x20;
            }

            return b;


        case 0x22:  // BANK select info

            b=0;
#if defined(MACHINE_PA7007)

            if(ioport[0x3c]&1) b|=0x1;
            if(ioport[0x3c]&2) b|=0x2;

#else

            if(ioport[0x3c]&1) b|=0x80;
            if(ioport[0x3c]&2) b|=0x40;

#endif


            return b;

        case 0x28:  // CTC counter
        case 0x29:
        case 0x2a:
        case 0x2b:

            b=address&3;

//            printf("[CR:%d:%x]",b,ctc_counter[b]);

            return ctc_counter[b];

        case 0x31:  // PIO B data (Keyscan)

            if(ioport[0x30]==0x7f) {
                b=0xff;
                for(int i=0;i<12;i++) {
                    b&=keymap[i];
                }

                return b;
            }

            b=0;
            if((ioport[0x30]&0x70)==0x20) {
                b=4;
            } else if((ioport[0x30]&0x70)==0x40) {
                b=8;
            }

            if(ioport[0x30]&2) {
                b+=1;
            } else if(ioport[0x30]&4) {
                b+=2;
            } else if(ioport[0x30]&8) {
                b+=3;
            }

            return keymap[b];

        case 0xe4:  // FDC
        case 0xe5:
            return 0xff;

        default:
            return ioport[address&0xff];

    }

//   return 0xff;
}

static void io_write(void *context, uint16_t address, uint8_t data)
{

    uint8_t b;

    // if((address&0xf0)==0xf0) {
    // printf("[IOW:%04x:%02x:%02x->%02x]",Z80_PC(cpu),address&0xff,ioport[address&0xff],data);
    // }

#if defined(MACHINE_PA7007)

    if(ram_ioaccess) {         // Need check
        mainram[address]=data;
        ram_ioaccess=0;
        return;
    }

#endif

    switch(address&0xff) {

        case 0: // set low byte of VRAM ptr

            vramptr&=0xff00;
            vramptr|=data;
            return;

        case 0xa: // set high byte of VRAM ptr

            vramptr&=0xff;
            vramptr|=(data&0x3f)<<8;

//            if((ioport[0x0a]&0x40)&&((data&0x40)==0)) {  // Write VRAM
            if((data&0x40)==0) {  // Write VRAM
                vram[vramptr]=ioport[1];
                if(data&0x80) {
                    atram[vramptr]=1;
                } else {
                    atram[vramptr]=0;
                }

            }
            ioport[0x0a]=data;

            return;

#if defined(MACHINE_PA7007)

            case 0xc:
            ioport[0xc]=data;
            return;


            case 0xd:
            lastattr=data&0xf;
            ioport[0xd]=data;
            return;


        case 0xf:   // i8255 control ?

            if((data&0x80)==0) { // Bit operation

                b=(data&0x0e)>>1;

                if(data&1) {
                    ioport[0xe]|= 1<<b;
                } else {
                    ioport[0xe]&= ~(1<<b);
                }

            }


            return;

#endif

        case 0x11: // CRTC

//       if((ioport[0x10]==10)||(ioport[0x10]==11)) {
    //    if((ioport[0x10]<10)) {
    //    printf("[CRTC:%x:%x]",ioport[0x10],data);
    //    }
    //    if((ioport[0x10]==12)||(ioport[0x10]==13)) {  // START ADDR
    //     printf("[CRTC:%x:%x]",ioport[0x10],data);
    //     }
        // if((ioport[0x10]==14)||(ioport[0x10]==15)) {  // CURSOR
        //     printf("[CRTC:%x:%x]",ioport[0x10],data);
        // }

            crtc[ioport[0x10]]=data;
            return;

        case 0x18:  // PAC (Low)

            pacptr&=0xffff00;
            pacptr|=data;
            return;

        case 0x19:  // PAC (Middle)

            pacptr&=0xff00ff;
            pacptr|=data<<8;
            return;

        case 0x1a:  // PAC (High or DATA write) 

#ifdef USE_RAMPAC
            if(pacmode==1) {
                rampac[pacptr&0x7fff]=data;
                return;
            }
#endif 
            pacptr&=0xffff;
            pacptr|=data<<16;
            return;

        case 0x20:  // 8255 for CMT/LPT

            if(((ioport[0x20]&0x20)!=0)&&((data&0x20)==0)) {
//                printf("[CMT ON]");
                tape_ready=1;
                tape_phase=0;
            }
            if(((ioport[0x20]&0x20)==0)&&((data&0x20)!=0)) {
//                printf("[CMT OFF]");
                tape_ready=0;
            }           

            if(data&0x10) {
                tapeout(1);
            } else {
                tapeout(0);
            }

            if(data&2) {    // Sound MUTE
                sound_mute=1;                
            } else {
                sound_mute=0;
            }

            if(data&1) {    // RESET NMI
                nmi_enable_irq=0;
            }

            ioport[0x20]=data;

            return;

        case 0x30:

            if(data&0x80) {
                beep_enable=1;
            } else {
                beep_enable=0;
                pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,0);
            }

            if(data==0x7f) {
                b=0xff;
                for(int i=0;i<12;i++) {
                    b&=keymap[i];
                }

                if(b!=0xff){
                    keyboard_enable_irq=1;
                }
            }

            ioport[0x30]=data;
            return;


        case 0x32: // PIOA control

            if(pioa_next_mask) {
                pioa[3]=data;
                pioa_next_mask=0;
                return;
            }
            if(pioa_next_iocontrol) {
                pioa_next_iocontrol=0;
                return;
            }

            switch(data&0xf) {

                case 0:
                case 2:
                case 4:
                case 8:
                case 0xa:
                case 0xc:
                case 0xe:

                    pioa[0]=data;
                    return;
                
                case 3: // Intrrupt disable

                    if(data&0x80) {
                        pioa[2]|=0x80;
                    } else {
                        pioa[2]&=0x7f;
                    }

                    return;

                case 7: // Interrupt control

                    pioa[2]=data;
                    if(data&0x10) {
                        pioa_next_mask=1;
                    }
                    return;

                case 0xf: // Mode control

                    if((data&0xc0)==0xc0) { // Mode 3
                        pioa_next_iocontrol=1;
                    }


                default:                

                    return;
            }

        case 0x33: // PIO B Control

            if(piob_next_mask) {
                piob[3]=data;
                piob_next_mask=0;
                return;
            }
            if(piob_next_iocontrol) {
                piob_next_iocontrol=0;
                return;
            }

            switch(data&0xf) {

                case 0:
                case 2:
                case 4:
                case 8:
                case 0xa:
                case 0xc:
                case 0xe:

                    piob[0]=data;
                    return;
                
                case 3: // Intrrupt disable

                    if(data&0x80) {
                        piob[2]|=0x80;
                    } else {
                        piob[2]&=0x7f;
                    }

                    return;

                case 7: // Interrupt control

                    piob[2]=data;
                    if(data&0x10) {
                        piob_next_mask=1;
                    }
                    return;

                case 0xf: // Mode control

                    if((data&0xc0)==0xc0) { // Mode 3
                        piob_next_iocontrol=1;
                    }

                default:                

                    return;
            }

        case 0x28:  // CTC
        case 0x29:
        case 0x2a:
        case 0x2b:

            b=address&3;

            // if(b==1) {
            // printf("[T:%d:%x]",b,data);
            // }

            if(ctc_next_timeconstant[b]) {
                ctc_timeconstant[b]=data;
                ctc_next_timeconstant[b]=0;

                if(b==1) {  // set beep frequency

                    beep_interval=data;
                    if(ctc_control[b]&0x20) {
                        beep_interval*=16;
                    }

                }

                return;
            }

            if(data&1) {   // control word
                ctc_control[b]=data;
                if(data&2) {    // RESET counter
                    if(ctc_timeconstant[b]==0) {
                        ctc_counter[b]=256;
                    } else {
                        ctc_counter[b]=ctc_timeconstant[b];
                    }
                }
                if(data&4) {
                    ctc_next_timeconstant[b]=1;
                }
            } else {    // interrupt vector
                if(b==0) {
                    ctc_irq_vector[0]=data&0xf8;
                    ctc_irq_vector[1]=ctc_irq_vector[0]+2;
                    ctc_irq_vector[2]=ctc_irq_vector[0]+4;
                    ctc_irq_vector[3]=ctc_irq_vector[0]+6;
                }
            }

            return;

#if defined(MACHINE_PA7007)

            case 0x3a:  // DPSG 1
                psg_write(0,data);
                return;
    
            case 0x3b:  // DPSG 2
                psg_write(1,data);
                return;
#endif


        case 0x3c: // Bank select & reset

            ioport[address&0xff]=data;

#if defined(MACHINE_PA7007)

            if(data&8) {    // VRAM acess via IO port
                ram_ioaccess=1;
            }
#endif

#if defined(MACHINE_PA7070)
            if(data&4) {    // RESET
                z80_power(&cpu,true);
            }
#endif

return;

        default:
            ioport[address&0xff]=data;
            return;

    }


}

static uint8_t ird_read(void *context,uint16_t address) {

//  printf("INT:%d:%d:%d:%02x:%02x\n\r",cpu.im,pioa_enable_irq,pio_irq_processing,cpu.i,pioa[0]);

    if(cpu.im==2) { // mode 2

//  printf("[INT:%x:%d:%d:%d]",Z80_PC(cpu),subcpu_ird,vsync_enable_irq,timer_enable_irq);
//  printf("[INT]");

        if(ctc_enable_irq[0]) {
            ctc_enable_irq[0]=0;
            z80_int(&cpu,FALSE);
            return ctc_irq_vector[0];   
        }

        if(ctc_enable_irq[1]) {
            ctc_enable_irq[1]=0;
            z80_int(&cpu,FALSE);
            return ctc_irq_vector[1];   
        }

        if(ctc_enable_irq[2]) {
            ctc_enable_irq[2]=0;
            z80_int(&cpu,FALSE);
            return ctc_irq_vector[2];   
        }

        if(ctc_enable_irq[3]) {
            ctc_enable_irq[3]=0;
            z80_int(&cpu,FALSE);
            return ctc_irq_vector[3];   
        }

        if(keyboard_enable_irq) {
            keyboard_enable_irq=0;
            z80_int(&cpu,FALSE);
            return piob[0];   
        }


        if(timer_enable_irq) {
//            printf("[!]");  
            timer_enable_irq=0;
            z80_int(&cpu,FALSE);
            return ioport[0xf7];    
        }
    } 

    return 0xff;

}

#if 0
static void reti_callback(void *context) {



}
#endif

void init_emulator(void) {
//  setup emulator 

    key_kana=0;
    key_caps=0;
    key_hirakata=0;

    tape_ready=0;
    tape_leader=0;

    gamepad_info=0xff;

    beep_enable=0;

    pioa_next_mask=0;
    piob_next_mask=0;
    pioa_next_iocontrol=0;
    piob_next_iocontrol=0;

    for(int i=0;i<16;i++) {
        keymap[i]=0xff;
    }

    for(int i=0;i<4;i++) {
        ctc_next_timeconstant[i]=0;
        ctc_control[i]=0;
        ctc_counter[i]=0;
        ctc_internal_counter[i]=0;
        ctc_timeconstant[i]=0;
        ctc_enable_irq[i]=0;
    }

    keysum=0xff;

#if defined(MACHINE_PA7007)

    memset(color_mask_table,0,256);
    memset(color_palette_table,0,256);

//    ioport[0x3c]=1;
    nmi_enable_irq=1;
#endif

}

void main_core1(void) {

//    uint32_t redraw_start,redraw_length;

    multicore_lockout_victim_init();

    scanline=0;

#ifdef DRAW_ON_DEMAND
    irq_set_exclusive_handler (PIO0_IRQ_0, hsync_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled (pio0, pis_interrupt0 , true);
#else
    // set virtual Hsync timer
    add_repeating_timer_us(63,hsync_handler,NULL  ,&timer);
#endif

    while(1) {

#ifndef DRAW_ON_DEMAND
        if(menumode) {
            draw_framebuffer_menu(scanline%200);
        } else {
            if(ioport[0x8]&0x40) {
                draw_framebuffer_mode1(scanline%200);
            } else {
                if(ioport[0x8]&0x80) {
                    draw_framebuffer_mode2(scanline%200);                    
                } else {
                    draw_framebuffer_mode0(scanline%200);
                }
            }
        }
        memcpy(vga_data_array +320*(scanline%200),renderbuffer,320);
        while(video_hsync==0) ;
        video_hsync=0;
#endif    
    }
}

int main() {

    uint32_t menuprint=0;
    uint32_t filelist=0;
    uint32_t subcpu_wait;

    static uint32_t hsync_wait,vsync_wait;

    set_sys_clock_khz(DOTCLOCK * CLOCKMUL ,true);

    stdio_init_all();

    uart_init(uart0, 115200);

    initVGA();

    gpio_set_function(12, GPIO_FUNC_UART);
    gpio_set_function(13, GPIO_FUNC_UART);

    // gpio_set_slew_rate(0,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(1,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(2,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(3,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(4,GPIO_SLEW_RATE_FAST);

    gpio_set_drive_strength(2,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(3,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(4,GPIO_DRIVE_STRENGTH_2MA);


    // Beep & PSG


    gpio_set_function(6,GPIO_FUNC_PWM);
 //   gpio_set_function(11,GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(6);

    pwm_set_wrap(pwm_slice_num, 256);
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0);
//    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_B, 0);
    pwm_set_enabled(pwm_slice_num, true);

    // set PSG timer

//    add_repeating_timer_us(1000000/SAMPLING_FREQ,sound_handler,NULL  ,&timer2);

    add_repeating_timer_us(CTC_TICK,ctc_handler,NULL  ,&timer2);

    tuh_init(BOARD_TUH_RHPORT);

    memcpy(fontrom,cgrom,0x800);    // copy FONT to RAM (for speed up)

    video_cls();

    video_hsync=0;
    video_vsync=0;

    // video_mode=0;
    // fbcolor=0x7;

// uart handler

    // irq_set_exclusive_handler(UART0_IRQ,uart_handler);
    // irq_set_enabled(UART0_IRQ,true);
    // uart_set_irq_enables(uart0,true,false);

    multicore_launch_core1(main_core1);

    multicore_lockout_victim_init();

    sleep_ms(1);

// mount littlefs
    if(lfs_mount(&lfs,&PICO_FLASH_CFG)!=0) {
       cursor_x=0;
       cursor_y=0;
       fbcolor=7;
       video_print("Initializing LittleFS...");
       // format
       lfs_format(&lfs,&PICO_FLASH_CFG);
       lfs_mount(&lfs,&PICO_FLASH_CFG);
   }

    menumode=1;  // Pause emulator

    init_emulator();
    init_rampac();

    cpu.read = mem_read;
    cpu.write = mem_write;
    cpu.in = io_read;
    cpu.out = io_write;
	cpu.fetch = mem_read;
    cpu.fetch_opcode = mem_read;
//    cpu.reti = reti_callback;
    cpu.inta = ird_read;

    z80_power(&cpu,true);
    z80_instant_reset(&cpu);

    cpu_hsync=0;
    cpu_cycles=0;

#ifdef USE_FDC
    lfs_handler=lfs;
    fdc_init(diskbuffer);

    fd_drive_status[0]=0;
#endif

    // start emulator
    
    menumode=0;

    while(1) {

        if(menumode==0) { // Emulator mode

        cpu_cycles += z80_run(&cpu,1);
        cpu_clocks++;

// #ifdef USE_SR
//                  printf("[%04x]",Z80_PC(cpu));


//             cursor_x=0;
//             cursor_y=19;
//             fbcolor=0x7;
//             uint8_t str[64];
//             sprintf(str,"%04x %04x %04x %04x %02x",Z80_PC(cpu),Z80_BC(cpu),Z80_DE(cpu),Z80_HL(cpu),mainram[Z80_DE(cpu)]);
//             video_print(str);
// #endif

#if defined(MACHINE_PA7007)

        if(nmi_enable_irq) {
            nmi_enable_irq=0;
            z80_nmi(&cpu);
        }

#endif

        if((keyboard_enable_irq)&&(piob[2]&0x80)) {
//            printf(" INT ");
            if((cpu.iff1)&&(cpu.im==2)) {
                z80_int(&cpu,TRUE);
            }
        }

        if(ctc_enable_irq[0]) {
//            printf(" INT C0 ");
            if((cpu.iff1)&&(cpu.im==2)) {
                z80_int(&cpu,TRUE);
            }
        }

        if(ctc_enable_irq[1]) {
            //            printf(" INT C1 ");
            if((cpu.iff1)&&(cpu.im==2)) {
                z80_int(&cpu,TRUE);
            }
        }

        if(ctc_enable_irq[2]) {
//            printf(" INT C2 ");
            if((cpu.iff1)&&(cpu.im==2)) {
                z80_int(&cpu,TRUE);
            }
        }

        if(ctc_enable_irq[3]) {
//            printf(" INT C3 ");
            if((cpu.iff1)&&(cpu.im==2)) {
                z80_int(&cpu,TRUE);
            }
        }

        if(timer_enable_irq) {
            if((cpu.iff1)&&(cpu.im==2)) {
                z80_int(&cpu,TRUE);
            }
        }

        // Wait

#ifdef DRAW_ON_DEMAND 
//        if((cpu_cycles-cpu_hsync)>1 ) { // 63us * 3.58MHz = 227
        if((cpu_cycles-cpu_hsync)>254 ) { // 63us * 4MHz = 254
            while(video_hsync==0) ;
            cpu_hsync=cpu_cycles;
//            video_hsync=0;
        }

        if(video_hsync==1) {
            hsync_wait++;
            if(hsync_wait>30) {
                video_hsync=0;
                hsync_wait=0;
            }
        }

#endif

        if(video_vsync>=2) {
            if(scanline>(vsync_scanline+60)) {
                video_vsync=0;
            }
        }

        if((video_vsync)==1) { // Timer
            tuh_task();
            video_vsync=2;
            vsync_scanline=scanline;

        }

        if((tape_autoclose)&&(save_enabled==2)) {
            if((cpu_cycles-tape_cycles)>TAPE_THRESHOLD) {
                save_enabled=0;
                lfs_file_close(&lfs,&lfs_file);
            }
        }

        if((tape_autoclose)&&(load_enabled==2)) {
            if((cpu_cycles-tape_cycles)>TAPE_THRESHOLD) {
                load_enabled=0;
                lfs_file_close(&lfs,&lfs_file);
            }
        }

        } else { // Menu Mode


            unsigned char str[80];

            fbcolor=7;
            
            if(menuprint==0) {

                draw_menu();
                menuprint=1;
                filelist=0;
            }

            cursor_x=3;
            cursor_y=3;
            video_print("MENU");

            uint32_t used_blocks=lfs_fs_size(&lfs);
            sprintf(str,"Free:%d Blocks",(HW_FLASH_STORAGE_BYTES/BLOCK_SIZE_BYTES)-used_blocks);
            cursor_x=3;
            cursor_y=4;
            video_print(str);

            sprintf(str,"TAPE:%x",tape_ptr);
            cursor_x=3;
            cursor_y=5;
            video_print(str);

            cursor_x=3;            
            cursor_y=6;
            if(menuitem==0) { fbcolor=0x70; } else { fbcolor=7; } 
            if(save_enabled==0) {
                video_print("SAVE: empty");
            } else {
                sprintf(str,"SAVE: %8s",tape_filename);
                video_print(str);
            }
            cursor_x=3;
            cursor_y=7;

            if(menuitem==1) { fbcolor=0x70; } else { fbcolor=7; } 
            if(load_enabled==0) {
                video_print("LOAD: empty");
            } else {
                sprintf(str,"LOAD: %8s",tape_filename);
                video_print(str);
            }

            cursor_x=3;
            cursor_y=8;

            if(menuitem==2) { fbcolor=0x70; } else { fbcolor=7; } 
            if(pacmode==0) {
                 video_print("PAC: Disable");
            } else if(pacmode==1) {
                 video_print("PAC: RAM");
            } else if(pacmode==2) {
                video_print("PAC: KANJI");                
            } else if(pacmode==3) {
                video_print("PAC: JOYSTICK");                
            }


            if(pacmode==1) {

                cursor_x=3;
                cursor_y=9;

                if(menuitem==3) { fbcolor=0x70; } else { fbcolor=7; } 

                if(pacload==0) {
                    video_print("RAMPAC: Empty");                    
                } else {
                    sprintf(str,"RAMPAC: %8s",rampac_filename);
                    video_print(str);                    
                }

                cursor_x=3;
                cursor_y=10;

                if(menuitem==4) { fbcolor=0x70; } else { fbcolor=7; } 

                if(pacload==0) {
                    video_print("NEW RAPMAC");                    
                }

                
            }


#ifdef USE_FDC

            cursor_x=3;
            cursor_y=11;

            if(menuitem==5) { fbcolor=0x70; } else { fbcolor=7; } 
            if(fd_drive_status[0]==0) {
                video_print("FD: empty");
            } else {
                sprintf(str,"FD: %8s",fd_filename);
                video_print(str);
            }
#endif

            cursor_x=3;
            cursor_y=13;

            if(menuitem==6) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("DELETE File");

            cursor_x=3;
            cursor_y=16;

            if(menuitem==7) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("Reset");


            cursor_x=3;
            cursor_y=17;

            if(menuitem==8) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("PowerCycle");

// TEST
            fbcolor=7;
            // cursor_x=3;
            //  cursor_y=17;
            //      sprintf(str,"%04x %x %04x %04x %x %04x %04x",Z80_PC(cpu),i8253[1],i8253_counter[1],i8253_preload[1],i8253[2],i8253_counter[2],i8253_preload[2]);
            //      video_print(str);


#ifdef USE_DEBUG
           cursor_x=3;
             cursor_y=18;
//                 sprintf(str,"%04x %04x %04x %04x %04x",Z80_PC(cpu),Z80_AF(cpu),Z80_BC(cpu),Z80_DE(cpu),Z80_HL(cpu));
                 sprintf(str,"%04x %02x",Z80_PC(cpu),ioport[0xf0]);
//                                  sprintf(str,"%04x",D7752e_GetStatus(voice_instance));
                 video_print(str);
#endif

            // cursor_x=3;
            //  cursor_y=18;
            //      sprintf(str,"%d %d/%d/%d/%d %d",intrcount,vsynccount,vsynccountf1,vsynccountf2,vsynccountf,timercount);
            //      video_print(str);


            // cursor_x=3;
            //  cursor_y=18;
            //  uint16_t sp=Z80_SP(cpu);
            //      sprintf(str,"%04x %04x %04x",mainram[sp]+256*mainram[sp+1],mainram[sp+2]+256*mainram[sp+3],mainram[sp+4]+256*mainram[sp+5]);
            //      video_print(str);

            if(filelist==0) {
                draw_files(-1,0);
                filelist=1;
            }
     
            while(video_vsync==0);

            video_vsync=0;

                tuh_task();

                if(keypressed==0x52) { // Up
                    keypressed=0;
                    if(menuitem>0) menuitem--;
#ifndef USE_FDC
                    if(menuitem==5) menuitem--;
#endif
                    if((menuitem==4)&&(pacmode!=1)) {
                        menuitem-=2;
                    }
                }

                if(keypressed==0x51) { // Down
                    keypressed=0;
                    if(menuitem<8) menuitem++; 
                    if((menuitem==3)&&(pacmode!=1)) {
                        menuitem+=2;
                    }
#ifndef USE_FDC
                    if(menuitem==5) menuitem++;
#endif
                }

                if(keypressed==0x28) {  // Enter
                    keypressed=0;

                    if(menuitem==0) {  // SAVE

                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=enter_filename();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDWR|LFS_O_CREAT);
                                save_enabled=1;
                                // tape_phase=0;
                                tape_ptr=0;
                                // tape_count=0;
                            }

                        } else if (save_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            save_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==1) { // LOAD

                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDONLY);
                                load_enabled=1;
                                // tape_phase=0;
                                tape_ptr=0;
                                // tape_count=0;
//                                file_cycle=cpu.PC;
                            }
                        } else if(load_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            load_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==2) { // PAC

                        pacmode++;
#ifndef USE_RAMPAC
                        if(pacmode==1) {
                            pacmode++;
                        }
#endif

                        if(pacmode>3) pacmode=0;

                        menuprint=0;

                    }

#ifdef USE_RAMPAC
                    if(menuitem==3) {  // RAMPAC mount/unmount

                        if(pacload) { // unmount

                            // check rewrite is needed
                            
                            rampac_save();

                            lfs_file_close(&lfs,&rampac_file);

                            pacload=0;

                        } else { // mount

                            pacload=0;

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(rampac_filename,filename,16);
                                lfs_file_open(&lfs,&rampac_file,rampac_filename,LFS_O_RDWR);

                                // check file is RAMPAC (file size = 32KB & Initial header is correct)

                                if(lfs_file_size(&lfs,&rampac_file)!=32768) {
                                    menuprint=0;
                                    continue;
                                }

                                lfs_file_rewind(&lfs,&rampac_file);
                                lfs_file_read(&lfs,&rampac_file,rampac,32768);

                                if(memcmp(rampac,rampac_header,16)!=0) {
                                    menuprint=0;
                                    continue;
                                }

                                pacload=1;

                            }

                        }
                        menuprint=0;

                    }
                    if(menuitem==4) {
                        if(pacload==0) {

                            uint32_t res=enter_filename();

                            if(res==0) {
                                memcpy(rampac_filename,filename,16);
                                lfs_file_open(&lfs,&rampac_file,rampac_filename,LFS_O_RDWR|LFS_O_CREAT);
                                init_rampac();
                                pacload=1;
                            }


                        }
                        menuprint=0;
                    }
#endif

#ifdef USE_FDC

                    if(menuitem==5) {  // FD
                        if(fd_drive_status[0]==0) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(fd_filename,filename,16);
                                lfs_file_open(&lfs,&fd_drive[0],fd_filename,LFS_O_RDONLY);
                                fdc_check(0);
                            }
                        } else {
                            lfs_file_close(&lfs,&fd_drive[0]);
                            fd_drive_status[0]=0;
                        }
                        menuprint=0;
                    }
#endif
                    // if(menuitem==4) { // colormode

                    //     colormode++;

                    //     if(colormode>2) {
                    //         colormode=0;
                    //     }

                    //     menuprint=0;

                    // }


                    if(menuitem==6) { // Delete

                        if((load_enabled==0)&&(save_enabled==0)) {
                            uint32_t res=enter_filename();

                            if(res==0) {
                                lfs_remove(&lfs,filename);
                            }
                        }

                        menuprint=0;

                    }

                    if(menuitem==7) { // Reset
                        menumode=0;
                        menuprint=0;
                        redraw();
                    
                        init_emulator();

                        redraw();

                        z80_power(&cpu,true);

                    }

                    if(menuitem==8) { // PowerCycle
                        menumode=0;
                        menuprint=0;

                        memset(mainram,0,0x10000);
                        memset(ioport,0,0x100);

                        init_emulator();

                        redraw();

//                        z80_instant_reset(&cpu);
                        z80_power(&cpu,true);

                    }



                }

                if(keypressed==0x45) {
                    keypressed=0;
                    menumode=0;
                    menuprint=0;
                    redraw();
                //  break;     // escape from menu
                }

        }


    }

}
