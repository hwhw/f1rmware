#include <r0ketlib/config.h>
#include <r0ketlib/fonts.h>
#include <r0ketlib/fonts/smallfonts.h>
#include <r0ketlib/display.h>
#include <r0ketlib/print.h>
#include <r0ketlib/render.h>
#include <r0ketlib/keyin.h>
#include <r0ketlib/select.h>
#include <r0ketlib/stringin.h>
#include <r0ketlib/image.h>
#include <r0ketlib/idle.h>
#include <r0ketlib/print.h>
#include <r0ketlib/night.h>
#include <r0ketlib/itoa.h>
#include <rad1olib/setup.h>
#include <rad1olib/pins.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/sdio.h>

inline uint32_t sdio_status_command_fsm(void) {
    return (SDIO_STATUS & SDIO_STATUS_CMDFSMSTATES_MASK) >> SDIO_STATUS_CMDFSMSTATES_SHIFT;
}

inline void sdio_reset_card(void) {
    SDIO_RST_N &= ~SDIO_RST_N_CARD_RESET(1);
}

inline void sdio_clear_interrupts() {
    // clear raw interrupt status
    SDIO_RINTSTS = 0xFFFFFFFF;
}

bool sdio_has_error() {
    if(SDIO_RINTSTS & (
              (1 <<  1) /* response error */
            | (1 <<  6) /* response CRC error */
            | (1 <<  7) /* data CRC error */
            | (1 <<  8) /* response time-out */
            | (1 <<  9) /* data time-out */
            | (1 << 10) /* data starvation by host time-out */
            | (1 << 11) /* FIFO underrun/overrun */
            | (1 << 12) /* hardware locked write */
            | (1 << 13) /* start bit error */
            | (1 << 15) /* end bit error */
            )) return true;
    return false;
}

void sdop_reset_fifo() {
    SDIO_CTRL = SDIO_CTRL_FIFO_RESET(1);
    while(SDIO_CTRL & SDIO_CTRL_FIFO_RESET(1)) {}
}

void sdio_init() {
    // enable clock line for SD/MMC block
    CGU_BASE_SDIO_CLK = CGU_BASE_SDIO_CLK_AUTOBLOCK(1) |
        CGU_BASE_SDIO_CLK_CLK_SEL(CGU_SRC_PLL1);

    // software reset
    SDIO_BMOD = SDIO_BMOD_SWR(1);

    // controller, FIFO, DMA reset
    const uint32_t resetflags = SDIO_CTRL_CONTROLLER_RESET(1) | SDIO_CTRL_FIFO_RESET(1) | SDIO_CTRL_DMA_RESET(1);
    SDIO_CTRL = resetflags;
    while(SDIO_CTRL & resetflags) {}

    // setup config
    // DMA
    SDIO_CTRL = SDIO_CTRL_USE_INTERNAL_DMAC(1) | SDIO_CTRL_INT_ENABLE(1);
    // Interrupt mask
    SDIO_INTMASK = 0;

    sdio_clear_interrupts();

    // timeout
    SDIO_TMOUT = 0xFFFFFFFF;

    // FIFO setup
    SDIO_FIFOTH = SDIO_FIFOTH_DMA_MTS(1) // 4 transfers
        | SDIO_FIFOTH_RX_WMARK(15)
        | SDIO_FIFOTH_TX_WMARK(16);

    // bus mode register
    SDIO_BMOD = SDIO_BMOD_DE(1) // enable DMA
        | SDIO_BMOD_PBL(1)      // programmable burst length 4
        | SDIO_BMOD_DSL(4);     // descriptor lenght 4

    SDIO_CLKENA = 0; // disable clock
    SDIO_CLKSRC = 0; // active clock divider = 0
}

void sdio_enable() {
    // power on
    SDIO_PWREN = 1;
}

void sdio_disable() {
    //TODO disable clock line
    // power off
    SDIO_PWREN = 0;
}

#define DELAYNOPS 2000
#define CMDRESULTTRIES 100
/*
 * @return 0 on success
 */
int sdio_cmd(uint32_t cmd, uint32_t cmdarg) {
    sdio_clear_interrupts();
    SDIO_CMDARG = cmdarg;
    SDIO_CMD = cmd | SDIO_CMD_START_CMD(1);
    for(int c=CMDRESULTTRIES; c>0; c--) {
        if(SDIO_CMD & SDIO_CMD_START_CMD(1)) {
            // start unset, not done yet
            delayNop(DELAYNOPS);
        } else {
            return 0;
        }
    }
    return 1;
}

#define PLL1SPEED 204000000
// set card clock speed
void sdio_set_clock(uint32_t speed) {
    // TODO: use the different dividers
    const uint32_t div = ((PLL1SPEED / speed) >> 1) + 1;
    // first, stop the clock
    SDIO_CLKENA = 0;
    // set active source to divider 0
    SDIO_CLKSRC = 0;

    // command CIU to update clock
    sdio_cmd(SDIO_CMD_UPDATE_CLOCK_REGISTERS_ONLY(1) | SDIO_CMD_WAIT_PRVDATA_COMPLETE(1), 0);

    // set divider 0 to desired value
    SDIO_CLKDIV = SDIO_CLKDIV_CLK_DIVIDER0(div);

    // command CIU to update clock
    sdio_cmd(SDIO_CMD_UPDATE_CLOCK_REGISTERS_ONLY(1) | SDIO_CMD_WAIT_PRVDATA_COMPLETE(1), 0);

    // now enable the clock again, use low-power mode (no clock when idle)
    SDIO_CLKENA = SDIO_CLKENA_CCLK_ENABLE(1); // | SDIO_CLKENA_CCLK_LOW_POWER(1);

    // command CIU to update clock
    sdio_cmd(SDIO_CMD_UPDATE_CLOCK_REGISTERS_ONLY(1) | SDIO_CMD_WAIT_PRVDATA_COMPLETE(1), 0);
}

// see UM10503, 22.7.6 (DMA descriptors)
// descriptor length is set to 4 in sdio_init() above.
// for now, we only implement a single DMA descriptor, which will restrict
// us to a single buffer of 2^13 - 4 bytes size (must be divisible by 4).
typedef struct {
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint32_t DESC2;
    volatile uint32_t DESC3;
} sdio_dma_descriptor;

static sdio_dma_descriptor desc;
#define SDIO_DMADESC_DIC (1 << 1)
#define SDIO_DMADESC_LD (1 << 2)
#define SDIO_DMADESC_FS (1 << 3)
#define SDIO_DMADESC_CH (1 << 4)
#define SDIO_DMADESC_ER (1 << 5)
#define SDIO_DMADESC_CES (1 << 30)
#define SDIO_DMADESC_OWN (1 << 31)

// set up DMA parameters
// addr pointer must be 32bit aligned! (22.7.6.4)
void sdio_set_dma(void *addr, int size) {
    desc.DESC0 = SDIO_DMADESC_LD  // last descriptor
        | SDIO_DMADESC_FS        // first desciptor, too
        | SDIO_DMADESC_OWN;
    desc.DESC1 = (uint32_t) size; //TODO: check if it fits into 13bit for BS1 field
    desc.DESC2 = (uint32_t) addr;
    desc.DESC3 = 0;               // next descriptor address - there is none for now
    SDIO_DBADDR = (uint32_t) &desc;
}

// check if DMA operation is still in progress
int sdio_dma_running() {
    // see UM10503, 22.7.6.7., step 10: OWN flag is cleared by DMA engine when transfer is done
    return desc.DESC0 & SDIO_DMADESC_OWN;
}

// check for card presence
// returns 1 if card detected, 0 otherwise
int sdio_card_detect() {
    return SDIO_CDETECT & SDIO_CDETECT_CARD_DETECT(1);
}

inline void sdio_store_response(uint32_t* target) {
    target[0] = SDIO_RESP0;
    target[1] = SDIO_RESP1;
    target[2] = SDIO_RESP2;
    target[3] = SDIO_RESP3;
}

typedef struct {
    uint32_t rca;
    uint32_t cid[4];
    uint32_t csd[4];
    bool ccs;
} sd_card_desc;

sd_card_desc card;

uint32_t sdio_wait() {
    //TODO: implement variable timeout?
    while(SDIO_RINTSTS == 0) {};
    uint32_t intstatus = SDIO_RINTSTS;
    lcdPrint("w:");
    lcdPrintln(IntToStr(intstatus,10,0));
    return intstatus;
}

uint32_t sdio_status_cdone() {
    return (SDIO_RINTSTS & SDIO_RINTSTS_CDONE(1));
}

uint32_t sdio_status_dto() {
    return (SDIO_RINTSTS & SDIO_RINTSTS_DTO(1));
}

/* 3.2-3.3V and 3.3-3.4V bits: */
#define OCR_VOLTAGE_MASK ((1 << 20) | (1 << 21))
#define OCR_VOLTAGE_MASK ((1 << 20) | (1 << 21))
#define OCR_CCS (1 << 30)
// start up SD memory card
int sd_open() {
    SDIO_TMOUT = 0xFFFFFFFF;

    sdio_set_clock(400000); // 400 kHz

    SDIO_CTYPE = SDIO_CTYPE_CARD_WIDTH0(0)  // 1 bit (0) or 4 bit (1)
               | SDIO_CTYPE_CARD_WIDTH1(0); // or 8 bit (1)?

    if(sdio_cmd(SDIO_CMD_CMD_INDEX(0) | SDIO_CMD_SEND_INITIALIZATION(1), 0)) // GO_IDLE_STATE
        return 1;

    SDIO_RINTSTS = 0xFFFFFFFF;

    // Send Interface Condition
    if(sdio_cmd(SDIO_CMD_CMD_INDEX(8) | SDIO_CMD_RESPONSE_EXPECT(1),
            (1 << 8 /* 2.7-3.6 volt */) | (0xAA /* check pattern */))
        || !sdio_wait() || !sdio_status_cdone() || sdio_has_error())
        return 2;

    if((SDIO_RESP0 & 0x1FF) != ((1 << 8 /* 2.7-3.6v voltage accepted */) | 0xAA))
        return 3;

    // as per SD physical layer spec, we should try for 1 second here:
    int try = 100;
    uint32_t ocr = 0;
    do {
        // ACMD41:
        if(sdio_cmd(SDIO_CMD_CMD_INDEX(55) | SDIO_CMD_RESPONSE_EXPECT(1), 0) ||
            !sdio_wait() || !sdio_status_cdone() || sdio_has_error())
            return 4;
        if(sdio_cmd(SDIO_CMD_CMD_INDEX(41) | SDIO_CMD_RESPONSE_EXPECT(1), (1<<30 /*HCS*/) | ocr) ||
            !sdio_wait() || !sdio_status_cdone() || sdio_has_error())
            return 4;
        if(ocr == 0) {
            ocr = SDIO_RESP0 & OCR_VOLTAGE_MASK;
            if(ocr == 0) {
                /* card does not support voltage range we're operating at */
                return 5;
            }
            // SDHC (or SDXC) card?
            card.ccs = (SDIO_RESP0 & OCR_CCS) ? true : false;
        }
        delayms(10);
        if(try-- == 0) return 6;
    } while(!(SDIO_RESP0 & (1UL << 31)));

    // read CID
    if(sdio_cmd(SDIO_CMD_CMD_INDEX(2) | SDIO_CMD_RESPONSE_EXPECT(1) | SDIO_CMD_RESPONSE_LENGTH(1), 0) ||
        !sdio_wait() || !sdio_status_cdone() || sdio_has_error())
        return 10;
    sdio_store_response(card.cid);

    lcdPrintln(IntToStr(card.cid[0],10,0));
    lcdPrintln(IntToStr(card.cid[1],10,0));
    lcdPrintln(IntToStr(card.cid[2],10,0));
    lcdPrintln(IntToStr(card.cid[3],10,0));

    if(sdio_cmd(SDIO_CMD_CMD_INDEX(3) | SDIO_CMD_RESPONSE_EXPECT(1), 0) ||
        !sdio_wait() || !sdio_status_cdone() || sdio_has_error())
        return 8;
    card.rca = SDIO_RESP0 >> 16;
    lcdPrintln(IntToStr(SDIO_RESP0,10,0));
    //card.rca = 0;

    // read CSD
    if(sdio_cmd(SDIO_CMD_CMD_INDEX(9) | SDIO_CMD_RESPONSE_EXPECT(1) | SDIO_CMD_RESPONSE_LENGTH(1), card.rca << 16) ||
        !sdio_wait() || !sdio_status_cdone() || sdio_has_error())
        return 11;
    sdio_store_response(card.csd);
    lcdPrintln(IntToStr(card.csd[0],10,0));
    lcdPrintln(IntToStr(card.csd[1],10,0));
    lcdPrintln(IntToStr(card.csd[2],10,0));
    lcdPrintln(IntToStr(card.csd[3],10,0));

    return 0;
}

uint64_t sd_size() {
    uint64_t s = ((card.csd[1] >> 30) | ((card.csd[2] & 0x3FF) << 2)) + 1; // C_SIZE + 1
    s = s << (((card.csd[1] >> 15) & 3) + 2); // 2 ^ (C_SIZE_MULT + 2)
    s = s << ((card.csd[2] >> 16) & 0xF);     // 2 ^ READ_BL_LEN
    return s;
}

//# MENU sdcard
void menu_sdcard(void){
    lcdClear();

    SETUPpin(SD_CD);
    SETUPpin(SD_CLK);
    SETUPpin(SD_CMD);
    SETUPpin(SD_DAT0);
    SETUPpin(SD_DAT1);
    SETUPpin(SD_DAT2);
    SETUPpin(SD_DAT3);

    cpu_clock_set(204);

    sdio_init();
    sdio_enable();

    if(sdio_card_detect()) {
        lcdPrintln(IntToStr(sd_open(),3,0));
        //lcdPrintln(IntToStr(SDIO_STATUS & 0xFFFF,6,0));
        //lcdPrintln(IntToStr(SDIO_STATUS >> 16,6,0));
        uint64_t s = sd_size() >> 20;
        uint32_t s1 = s;
        lcdPrintln(IntToStr(s1,10,0));
        /*
        lcdPrintln(IntToStr(csd[0],10,0));
        lcdPrintln(IntToStr(csd[1],10,0));
        lcdPrintln(IntToStr(csd[2],10,0));
        lcdPrintln(IntToStr(csd[3],10,0));
        */
    } else {
        lcdPrintln("no card detected.");
    }

    lcdDisplay();
    while(getInputRaw()!=BTN_ENTER){ /* wait */ };

    sdio_disable();
}

