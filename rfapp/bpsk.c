/* BPSK RX/TX
 *
 * Copyright (C) 2015 Hans-Werner Hilse <hwhilse@gmail.com>
 *
 * some parts (receive/filters) are
 *   Copyright (C) 2013 Jared Boone, ShareBrained Technology, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <rad1olib/setup.h>
#include <r0ketlib/display.h>
#include <r0ketlib/print.h>
#include <r0ketlib/itoa.h>
#include <r0ketlib/keyin.h>
#include <r0ketlib/menu.h>
#include <r0ketlib/select.h>
#include <r0ketlib/idle.h>

#include <rad1olib/pins.h>

#include <common/hackrf_core.h>
#include <common/rf_path.h>
#include <common/sgpio.h>
#include <common/tuning.h>
#include <common/max2837.h>
#include <common/streaming.h>
#include <libopencm3/lpc43xx/sgpio.h>
#include <libopencm3/lpc43xx/m4/nvic.h>
#include <libopencm3/cm3/vector.h>

#include <stddef.h>
#include <portalib/arm_intrinsics.h>
#include <portalib/complex.h>
#include <portalib/fxpt_atan2.h>

#include <lpcapi/cdc/cdc_main.h>
#include <lpcapi/cdc/cdc_vcom.h>

#include "cossin1024.h"

// default to 2496 MHz
#define FREQSTART 2496000000

static int64_t frequency = FREQSTART;

static void my_set_frequency(const int64_t new_frequency, const int32_t offset) {
    const int64_t tuned_frequency = new_frequency + offset;
    ssp1_set_mode_max2837();
    if(set_freq(tuned_frequency)) {
        frequency = new_frequency;
    }
}

static bool lna_enable = true;
static int32_t lna_gain_db = 8;
static int32_t vga_gain_db = 20;
static int32_t txvga_gain_db = 20;

/* set amps */
static void set_rf_params() {
    ssp1_set_mode_max2837(); // need to reset this since display driver will hassle with SSP1
    rf_path_set_lna(lna_enable ? 1 : 0);
    max2837_set_lna_gain(lna_gain_db);     /* 8dB increments */
    max2837_set_vga_gain(vga_gain_db);     /* 2dB increments, up to 62dB */
    max2837_set_txvga_gain(txvga_gain_db); /* 1dB increments, up to 47dB */
}

/* portapack_init plus a bunch of stuff from here and there, cleaned up */
static void rfinit() {
    /* Release CPLD JTAG pins */
    scu_pinmux(SCU_PINMUX_CPLD_TDO, SCU_GPIO_NOPULL | SCU_CONF_FUNCTION4);
    scu_pinmux(SCU_PINMUX_CPLD_TCK, SCU_GPIO_NOPULL | SCU_CONF_FUNCTION0);
    scu_pinmux(SCU_PINMUX_CPLD_TMS, SCU_GPIO_NOPULL | SCU_CONF_FUNCTION0);
    scu_pinmux(SCU_PINMUX_CPLD_TDI, SCU_GPIO_NOPULL | SCU_CONF_FUNCTION0);
    GPIO_DIR(PORT_CPLD_TDO) &= ~PIN_CPLD_TDO;
    GPIO_DIR(PORT_CPLD_TCK) &= ~PIN_CPLD_TCK;
    GPIO_DIR(PORT_CPLD_TMS) &= ~PIN_CPLD_TMS;
    GPIO_DIR(PORT_CPLD_TDI) &= ~PIN_CPLD_TDI;

    hackrf_clock_init();
    rf_path_pin_setup();

    /* Configure external clock in */
    scu_pinmux(SCU_PINMUX_GP_CLKIN, SCU_CLK_IN | SCU_CONF_FUNCTION1);

    /* Disable unused clock outputs. They generate noise. */
    scu_pinmux(CLK0, SCU_CLK_IN | SCU_CONF_FUNCTION7);
    scu_pinmux(CLK2, SCU_CLK_IN | SCU_CONF_FUNCTION7);

    sgpio_configure_pin_functions();

    ON(EN_VDD);
    ON(EN_1V8);
    delayNop(250000); // doesn't work without

    cpu_clock_set(204); // WARP SPEED! :-)
    si5351_init();

    cpu_clock_pll1_max_speed();
    
    ssp1_init();

    rf_path_init();
}

/* ---------------------------- TRANSMIT: ------------------------------- */

/* transmit options */
#define TX_BANDWIDTH  1750000
#define TX_SAMPLERATE 2000000

static volatile uint8_t *tx_pkg;
static volatile uint16_t tx_pkg_len;
static volatile bool transmitting = false;

static void stop_transmit() {
    baseband_streaming_disable();
    /* stop powering the TX LNA */
    rf_path_set_lna(0);
    /* switch everything off */
    rf_path_set_direction(RF_PATH_DIRECTION_OFF);
    /* clear flag */
    transmitting = false;
}

#define STATE_NULL 0
#define STATE_PREAMBLE_ONE 1
#define STATE_PREAMBLE_ZERO 44
#define STATE_PKGLEN 48
#define STATE_DATA 63
#define STATE_FINISH 100
static int8_t get_codec_bit_length() {
    static uint8_t state = STATE_NULL;
    static uint16_t data;

    if(state == STATE_NULL) {
        state++;
        return 0;
    } else if(state < STATE_PREAMBLE_ZERO) {
        /* in STATE_PREAMBLE_ONE */
        state++;
        return 3;
    } else if(state < STATE_PKGLEN) {
        /* in STATE_PREAMBLE_ZERO */
        state++;
        return 2;
    } else if(state == STATE_PKGLEN) {
        data = tx_pkg_len;
        state = STATE_DATA + 9;
    } else if(state == STATE_DATA) {
        if((tx_pkg_len--) == 0) {
            state = STATE_FINISH;
            return 1;
        }
        data = *(tx_pkg++);
        state = STATE_DATA + 9;
    } else if(state == STATE_FINISH) {
        state = STATE_NULL;
        stop_transmit();
        return 0;
    }
    state--;
    data <<= 1;
    return (data & 0x200) ? 3 : 2;
}

static uint32_t get_rf_sample() {
    /* this is 2x (127/127) samples */
    static uint32_t sample = 0x7F007F00;
    static int8_t samples_to_send = 0;

    /* keep sending current sample? */
    samples_to_send--;
    if(samples_to_send > 0) return sample;
    /* send 3x16 samples for a 1 or 2x16 samples for a 0 */
    samples_to_send = get_codec_bit_length();
    /* special case: 0/0 sample */
    if(samples_to_send == 0) return 0;
    /* shift phase (127/0 vs -128/0) */
    sample ^= 0xFF00FF00;
    return sample;
}

/*
 * the following follows the example of hackrf's sgpio_isr
 * which triggers write to SGPIO on 8 planes
 * will be triggered with FREQ/16
 **/
void bpsk_sgpio_isr_tx() {
    static uint32_t sample = 0;

    /* this is shortened to send one single sample value 16x (8x2x) */
	__asm__(
		"ldr r0, [%[p], #0]\n\t"
		"str r0, [%[SGPIO_REG_SS], #44]\n\t"
		"str r0, [%[SGPIO_REG_SS], #20]\n\t"
		"str r0, [%[SGPIO_REG_SS], #40]\n\t"
		"str r0, [%[SGPIO_REG_SS], #8]\n\t"
		"str r0, [%[SGPIO_REG_SS], #36]\n\t"
		"str r0, [%[SGPIO_REG_SS], #16]\n\t"
		"str r0, [%[SGPIO_REG_SS], #32]\n\t"
		"str r0, [%[SGPIO_REG_SS], #0]\n\t"
		:
		: [SGPIO_REG_SS] "l" (SGPIO_PORT_BASE + 0x100),
		  [p] "l" (&sample)
		: "r0"
	);
    sample = get_rf_sample();
	SGPIO_CLR_STATUS_1 = (1 << SGPIO_SLICE_A);
}

static void transmit(uint8_t *data, uint16_t length) {
    /* wait for completion of last transfer */
    while(transmitting) {};
    transmitting = true;

    /* if in receive mode, stop generating samples */
    baseband_streaming_disable();
    /* set up TX mode */
    vector_table.irq[NVIC_SGPIO_IRQ] = bpsk_sgpio_isr_tx;
    /* set TX frequency */
    my_set_frequency(frequency, 0);
    /* set up RF path */
    rf_path_set_direction(RF_PATH_DIRECTION_TX);
    /* this is needed to get the RF switches right: */
    set_rf_params();
    /* set TX sample rate */
    sample_rate_frac_set(TX_SAMPLERATE * 2, 1);
    /* and LPF */
    baseband_filter_bandwidth_set(TX_BANDWIDTH);

    tx_pkg = data;
    tx_pkg_len = length;

    baseband_streaming_enable();
}
/* ---------------------------- RECEIVE: -------------------------------- */

/* frequency offset is -500kHz because of the shift done by the first filter
 * (to get the DC peak out of the way)
 */
#define RX_FREQOFFSET (-500000+15000)
#define RX_BANDWIDTH  7000000
#define RX_SAMPLERATE 8000000
#define RX_DECIMATION 4 /* effective sample rate is 2000000 */

/* packets are assembled by the ISR and its subroutines, then a flag is
 * set to handle the packet data outside of the ISR.
 */
#define MAX_PACKET_LEN 255
/* packet buffer */
static volatile uint8_t rx_pkg[MAX_PACKET_LEN+1];
/* length of received data */
static volatile uint32_t rx_pkg_len;
static volatile bool rx_pkg_flag = false;

/* filter functions from portapack C code, slightly modified */
static void my_fir_cic3_decim_2_s16_s16(
	complex_s16_t* const src,
	complex_s16_t* const dst,
	const size_t sample_count
) {
	/* Complex non-recursive 3rd-order CIC filter (taps 1,3,3,1).
	 * Gain of 8.
	 * Consumes 16 bytes (4 s16:s16 samples) per loop iteration,
	 * Produces  8 bytes (2 s16:s16 samples) per loop iteration.
	 */
	int32_t n = sample_count;
	static uint32_t t1 = 0;
	static uint32_t t2 = 0;
	uint32_t t3, t4;
	uint32_t taps = 0x00000003;
	uint32_t* s = (uint32_t*)src;
	uint32_t* d = (uint32_t*)dst;
	uint32_t i, q;
	for(; n>0; n-=4) {
		i = __SXTH(t1, 0);			/* 1: I0 */
		q = __SXTH(t1, 16);			/* 1: Q0 */
		i = __SMLABB(t2, taps, i);	/* 1: I1*3 + I0 */
		q = __SMLATB(t2, taps, q);	/* 1: Q1*3 + Q0 */

		t3 = *(s++);				/* 3: Q2:I2 */
		t4 = *(s++);				/*    Q3:I3 */

		i = __SMLABB(t3, taps, i);	/* 1: I2*3 + I1*3 + I0 */
		q = __SMLATB(t3, taps, q);	/* 1: Q2*3 + Q1*3 + Q0 */
		i = __SXTAH(i, t4, 0);		/* 1: I3 + Q2*3 + Q1*3 + Q0 */
		q = __SXTAH(q, t4, 16);		/* 1: Q3 + Q2*3 + Q1*3 + Q0 */
		i = __BFI(i, q, 16, 16);	/* 1: D2_Q0:D2_I0 */
		*(d++) = i;					/* D2_Q0:D2_I0 */

		i = __SXTH(t3, 0);			/* 1: I2 */
		q = __SXTH(t3, 16);			/* 1: Q2 */
		i = __SMLABB(t4, taps, i);	/* 1: I3*3 + I2 */
		q = __SMLATB(t4, taps, q);	/* 1: Q3*3 + Q2 */

		t1 = *(s++);				/* 3: Q4:I4 */
		t2 = *(s++);				/*    Q5:I5 */

		i = __SMLABB(t1, taps, i);	/* 1: I4*3 + I3*3 + I2 */
		q = __SMLATB(t1, taps, q);	/* 1: Q4*3 + Q3*3 + Q2 */
		i = __SXTAH(i, t2, 0);		/* 1: I5 + Q4*3 + Q3*3 + Q2 */
		q = __SXTAH(q, t2, 16);		/* 1: Q5 + Q4*3 + Q3*3 + Q2 */
		i = __BFI(i, q, 16, 16);	/* 1: D2_Q1:D2_I1 */
		*(d++) = i;					/* D2_Q1:D2_I1 */
	}
}

static void my_translate_fs_over_4_and_decimate_by_2_cic_3_s8_s16(
	complex_s8_t* const src_and_dst,
	const size_t sample_count
) {
	/* Translates incoming complex<int8_t> samples by -fs/4,
	 * decimates by two using a non-recursive third-order CIC filter.
	 */
	int32_t n = sample_count;
	static uint32_t q1_i0 = 0;
	static uint32_t q0_i1 = 0;
	uint32_t k_3_1 = 0x00030001;
	uint32_t* p = (uint32_t*)src_and_dst;
	for(; n>0; n-=4) {
		const uint32_t q3_i3_q2_i2 = p[0];							// 3
		const uint32_t q5_i5_q4_i4 = p[1];

		const uint32_t i2_i3 = __SXTB16(q3_i3_q2_i2, 16);			// 1: (q3_i3_q2_i2 ror 16)[23:16]:(q3_i3_q2_i2 ror 16)[7:0]
		const uint32_t q3_q2 = __SXTB16(q3_i3_q2_i2,  8);			// 1: (q3_i3_q2_i2 ror  8)[23:16]:(q3_i3_q2_i2 ror  8)[7:0]
		const uint32_t i2_q3 = __PKHTB(i2_i3, q3_q2, 16);			// 1: Rn[31:16]:(Rm>>16)[15:0]
		const uint32_t i3_q2 = __PKHBT(q3_q2, i2_i3, 16);			// 1:(Rm<<16)[31:16]:Rn[15:0]

		// D_I0 = 3 * (i2 - q1) + (q3 - i0)
		const uint32_t i2_m_q1_q3_m_i0 = __QSUB16(i2_q3, q1_i0);	// 1: Rn[31:16]-Rm[31:16]:Rn[15:0]-Rm[15:0]
		const uint32_t d_i0 = __SMUAD(k_3_1, i2_m_q1_q3_m_i0);		// 1: Rm[15:0]*Rs[15:0]+Rm[31:16]*Rs[31:16]

		// D_Q0 = 3 * (q2 + i1) - (i3 + q0)
		const uint32_t i3_p_q0_q2_p_i1 = __QADD16(i3_q2, q0_i1);	// 1: Rn[31:16]+Rm[31:16]:Rn[15:0]+Rm[15:0]
		const uint32_t d_q0 = __SMUSDX(i3_p_q0_q2_p_i1, k_3_1);		// 1: Rm[15:0]*Rs[31:16]–Rm[31:16]*RsX[15:0]
		const uint32_t d_q0_i0 = __PKHBT(d_i0, d_q0, 16);			// 1: (Rm<<16)[31:16]:Rn[15:0]

		const uint32_t i5_i4 = __SXTB16(q5_i5_q4_i4,  0);			// 1: (q5_i5_q4_i4 ror  0)[23:16]:(q5_i5_q4_i4 ror  0)[7:0]
		const uint32_t q4_q5 = __SXTB16(q5_i5_q4_i4, 24);			// 1: (q5_i5_q4_i4 ror 24)[23:16]:(q5_i5_q4_i4 ror 24)[7:0]
		const uint32_t q4_i5 = __PKHTB(q4_q5, i5_i4, 16);			// 1: Rn[31:16]:(Rm>>16)[15:0]
		const uint32_t q5_i4 = __PKHBT(i5_i4, q4_q5, 16);			// 1: (Rm<<16)[31:16]:Rn[15:0]

		// D_I1 = (i2 - q5) + 3 * (q3 - i4)
		const uint32_t i2_m_q5_q3_m_i4 = __QSUB16(i2_q3, q5_i4);	// 1: Rn[31:16]-Rm[31:16]:Rn[15:0]-Rm[15:0]
		const uint32_t d_i1 = __SMUADX(i2_m_q5_q3_m_i4, k_3_1);		// 1: Rm[15:0]*Rs[31:16]+Rm[31:16]*Rs[15:0]

		// D_Q1 = (i5 + q2) - 3 * (q4 + i3)
		const uint32_t q4_p_i3_i5_p_q2 = __QADD16(q4_i5, i3_q2);	// 1: Rn[31:16]+Rm[31:16]:Rn[15:0]+Rm[15:0]
		const uint32_t d_q1 = __SMUSD(k_3_1, q4_p_i3_i5_p_q2);		// 1: Rm[15:0]*Rs[15:0]–Rm[31:16]*Rs[31:16]
		const uint32_t d_q1_i1 = __PKHBT(d_i1, d_q1, 16);			// 1: (Rm<<16)[31:16]:Rn[15:0]

		q1_i0 = q5_i4;
		q0_i1 = q4_i5;

		*(p++) = d_q0_i0;											// 3
		*(p++) = d_q1_i1;
	}
}

/* we use a peamble of 0xFFF0 - so mostly 1 bits - in order to keep the
 * phase constant as long as possible during the time the PLL is still about
 * to lock. The PLL should lock when a few other 1-bits (0xFFFF) are sent
 * before sending this peamble. So when sending, just send 0xFFFFFFF0.
 *
 * the row of 1 bits will put a running decoding out of sync if
 * encountered for whatever reason during data decode
 */
#define PREAMBLE 0xFFF0
static void decoder(uint8_t bit) {
    /* shift register for received data */
    static uint16_t shift = 0;
    /* flag to determine whether we have seen a header and are in sync
     * also, counter for received data bits (+1)
     */
    static uint8_t sync = 0;
    /* number of bytes in packet to receive */
    static int16_t pkglen;

    shift <<= 1;
    shift |= bit;
    if(sync == 0) {
        /* wait for sync */
        if(shift == PREAMBLE) {
            pkglen = -1;
            sync = 1;
        }
    } else if(sync == 9) {
        /* the first of every 9 bits must be 0, otherwise, lose sync.*/
        if(shift & 0x100)
            goto desync;

        /* start new byte */
        sync = 1;

        /* check if we're at the start of a new packet */
        if(pkglen == -1) {
            /* we have already received one byte */
            pkglen = shift & 0xFF;
            if(pkglen > MAX_PACKET_LEN)
                goto desync;

            rx_pkg_len = 0;
            return;
        }
        if(rx_pkg_len < pkglen) {
            rx_pkg[rx_pkg_len++] = shift & 0xFF;
            if(rx_pkg_len == pkglen)
                rx_pkg_flag = true;
        }
        return;
    } else {
        sync++;
    }
    return;

desync:
    sync = 0;
    pkglen = -1;
    return;
}

#define ABS(x) ((x<0)?(-x):x)
static void pll(complex_s16_t *in) {
    static uint16_t phase = 0;
    static int freq = 0;

    /* duration since last phase shift for bit decoder */
    static int dur = 0;

    const complex_s8_t pll_osc = cos_sin[phase >> 6];

    /* calculate error vector
     * this is a result of the multiplication of the input
     * value and the conjugate of our own value
     */
    const int32_t err_q = in->q * pll_osc.q - in->i * (-pll_osc.i);
    const int32_t err_i = in->q * (-pll_osc.i) + in->i * pll_osc.q;

    /* calculate resulting phase error */
    int16_t err = fxpt_atan2(err_i, err_q);

    /* this PLL is special: it is phase-shift tolerant, i.e. it
     * explicitly tests for very large phase-shifts and will act
     * specially in such a case
     */
    if(ABS(err) > 16384) {
        /* rotate by 180 degrees */
        phase += 32768;
        err = 0;

        /* decode: */
        if(dur >= 6) {
            if(dur <= 14) decoder(dur > 10 ? 1 : 0);
            dur = 0;
        }
    }
    dur++;

    /* correct the pll osc's frequency and phase */
    const int16_t fshift = err >> 8;
    freq += fshift;
    /* clamp to -4000..4000 range */
    if((freq < -4000) || (freq > 4000)) freq -= fshift;

    phase += freq + (err >> 4);
}

/*
 * the following follows the example of hackrf's sgpio_isr
 * which triggers read from SGPIO on 8 planes
 *
 * we read 32 bytes, i.e. 16 8bit complex samples (8bit q, 8bit i)
 * and we will immediately send it through the filter chain
 **/
void bpsk_sgpio_isr_rx() {
    SGPIO_CLR_STATUS_1 = (1 << SGPIO_SLICE_A);
    static uint32_t buffer[8];
    __asm__(
        "ldr r0, [%[SGPIO_REG_SS], #44]\n\t"
        "str r0, [%[buffer], #0]\n\t"
        "ldr r0, [%[SGPIO_REG_SS], #20]\n\t"
        "str r0, [%[buffer], #4]\n\t"
        "ldr r0, [%[SGPIO_REG_SS], #40]\n\t"
        "str r0, [%[buffer], #8]\n\t"
        "ldr r0, [%[SGPIO_REG_SS], #8]\n\t"
        "str r0, [%[buffer], #12]\n\t"
        "ldr r0, [%[SGPIO_REG_SS], #36]\n\t"
        "str r0, [%[buffer], #16]\n\t"
        "ldr r0, [%[SGPIO_REG_SS], #16]\n\t"
        "str r0, [%[buffer], #20]\n\t"
        "ldr r0, [%[SGPIO_REG_SS], #32]\n\t"
        "str r0, [%[buffer], #24]\n\t"
        "ldr r0, [%[SGPIO_REG_SS], #0]\n\t"
        "str r0, [%[buffer], #28]\n\t"
        :
        : [SGPIO_REG_SS] "l" (SGPIO_PORT_BASE + 0x100),
          [buffer] "l" (buffer)
        : "r0"
    );

    /* 3.072MHz complex<int8> */
    my_translate_fs_over_4_and_decimate_by_2_cic_3_s8_s16((complex_s8_t*) buffer, 16);
    /* 1.544MHz complex<int16>[N/2] */
    complex_s16_t* const buf16 = (complex_s16_t*) buffer;
    my_fir_cic3_decim_2_s16_s16(buf16, buf16, 8);
    /* 768kHz complex<int16>[N/4] */
    for(int n=0; n < 4; n++) {
        pll(&buf16[n]);
    }
}

static void receive() {
    /* wait for completion of last transfer */
    while(transmitting) {};

    /* set up RX mode */
    vector_table.irq[NVIC_SGPIO_IRQ] = bpsk_sgpio_isr_rx;
    /* set RX frequency */
    my_set_frequency(frequency, RX_FREQOFFSET);
    /* set up RF path */
    rf_path_set_direction(RF_PATH_DIRECTION_RX);
    /* this is needed to get the RF switches right: */
    set_rf_params();
    /* set TX sample rate */
    sample_rate_frac_set(RX_SAMPLERATE * 2, 1);
    /* and LPF */
    baseband_filter_bandwidth_set(RX_BANDWIDTH);
    /* set up decimation in CPLD */
    sgpio_cpld_stream_rx_set_decimation(RX_DECIMATION);

    rx_pkg_flag = false;

    baseband_streaming_enable();
}

/* ------------------------------------------------------------------- */

void sendstring(char *str, uint16_t len) {
    transmit((uint8_t*)str, len);
    getInputWaitRelease();
    receive();
}

//# MENU BPSK
void bpsk_menu() {
    lcdClear();
    lcdPrintln("ENTER to go back");
    lcdPrintln("L/R/U/D to xmit");
    lcdDisplay();
    getInputWaitRelease();

    cpu_clock_set(204);

    rfinit();
    receive();

    while(1) {
        switch (getInputRaw()) {
            case BTN_UP:
                sendstring("up", 2);
                break;
            case BTN_DOWN:
                sendstring("down", 4);
                break;
            case BTN_RIGHT:
                sendstring("right", 5);
                break;
            case BTN_LEFT:
                sendstring("left", 4);
                break;
            case BTN_ENTER:
                goto stop;
        }
        if(rx_pkg_flag) {
            rx_pkg_flag = false;
            rx_pkg[rx_pkg_len] = 0; /* ensure string termination */
            lcdPrintln((char*)rx_pkg);
            lcdDisplay();
        }
    }
stop:
    baseband_streaming_disable();
    OFF(EN_1V8);
    OFF(EN_VDD);
    return;
}
