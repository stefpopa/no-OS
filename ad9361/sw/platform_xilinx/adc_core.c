/***************************************************************************//**
 *   @file   adc_core.c
 *   @brief  Implementation of ADC Core Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <xil_cache.h>
#include <xil_io.h>
#include "adc_core.h"
#include "parameters.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
//#define FMCOMMS5

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief adc_read
*******************************************************************************/
void adc_read(adc_state *adc, uint32_t regAddr, uint32_t *data)
{
	*data = Xil_In32(adc->ad9361_rx_baseaddr + regAddr);
}

/***************************************************************************//**
 * @brief adc_write
*******************************************************************************/
void adc_write(adc_state *adc, uint32_t regAddr, uint32_t data)
{
	Xil_Out32((adc->ad9361_rx_baseaddr + regAddr), data);
}

/***************************************************************************//**
 * @brief adc_dma_read
*******************************************************************************/
void adc_dma_read(uint32_t regAddr, uint32_t *data)
{
	*data = Xil_In32(CF_AD9361_RX_DMA_BASEADDR + regAddr);
}

/***************************************************************************//**
 * @brief adc_dma_write
*******************************************************************************/
void adc_dma_write(uint32_t regAddr, uint32_t data)
{
	Xil_Out32(CF_AD9361_RX_DMA_BASEADDR + regAddr, data);
}

/***************************************************************************//**
 * @brief axiadc_init
*******************************************************************************/
int32_t axiadc_init(struct axiadc_state **state, axiadc_state_init axiadc_init)
{
	struct axiadc_state *st;
	st = (struct axiadc_state *)malloc(sizeof(*st));
	if (!st)
		return -1;
	st->cnv = (struct axiadc_converter *)malloc(sizeof(*st->cnv));
	if (!st->cnv) {
		return -1;
	}

	st->cnv->chip_info = (struct axiadc_chip_info *)malloc(sizeof(*st->cnv->chip_info));
	if (!st->cnv->chip_info) {
		return -1;
	}

	st->adc_st = (adc_state *)malloc(sizeof(*st->adc_st));
	if (!st->adc_st) {
		return -1;
	}

	st->adc_st->id_no = axiadc_init.id_no;
	switch (st->adc_st->id_no)
	{
	case 0:
		st->adc_st->ad9361_rx_baseaddr = AD9361_RX_0_BASEADDR;
		break;
	case 1:
		st->adc_st->ad9361_rx_baseaddr = AD9361_RX_1_BASEADDR;
		break;
	default:
		break;
	}
	st->adc_st->start_address = axiadc_init.start_address;
	st->adc_st->num_tx_channels = axiadc_init.num_tx_channels;

	adc_write(st->adc_st, ADC_REG_RSTN, 0);
	adc_write(st->adc_st, ADC_REG_RSTN, ADC_RSTN);
	adc_write(st->adc_st, ADC_REG_CHAN_CNTRL(0),
		ADC_IQCOR_ENB | ADC_FORMAT_SIGNEXT | ADC_FORMAT_ENABLE | ADC_ENABLE);
	adc_write(st->adc_st, ADC_REG_CHAN_CNTRL(1),
		ADC_IQCOR_ENB | ADC_FORMAT_SIGNEXT | ADC_FORMAT_ENABLE | ADC_ENABLE);
	if(st->adc_st->num_tx_channels == 2)
	{
		adc_write(st->adc_st, ADC_REG_CHAN_CNTRL(2),
			ADC_IQCOR_ENB | ADC_FORMAT_SIGNEXT | ADC_FORMAT_ENABLE | ADC_ENABLE);
		adc_write(st->adc_st, ADC_REG_CHAN_CNTRL(3),
			ADC_IQCOR_ENB | ADC_FORMAT_SIGNEXT | ADC_FORMAT_ENABLE | ADC_ENABLE);
	}
	else
	{
		adc_write(st->adc_st, ADC_REG_CHAN_CNTRL(2), 0);
		adc_write(st->adc_st, ADC_REG_CHAN_CNTRL(3), 0);
	}

	st->pcore_version = axiadc_read(st, ADI_REG_VERSION);
	st->cnv->chip_info = &axiadc_chip_info_tbl[(st->adc_st->num_tx_channels == 2) ? ID_AD9361 : ID_AD9364];

	*state = st;

	return 0;
}

/***************************************************************************//**
 * @brief axiadc_ad9361_alloc
*******************************************************************************/
int32_t axiadc_ad9361_alloc(struct ad9361_rf_phy *phy, struct axiadc_state *st)
{
	phy->adc_conv = (struct axiadc_converter *)zmalloc(sizeof(*phy->adc_conv));
	if (!phy->adc_conv) {
		return -1;
	}

	phy->adc_state = (struct axiadc_state *)zmalloc(sizeof(*phy->adc_state));
	if (!phy->adc_state) {
		return -1;
	}

	phy->adc_state = st;
	phy->adc_conv = st->cnv;

	return 0;
}

/***************************************************************************//**
 * @brief axiadc_ad9361_dealloc
*******************************************************************************/
void axiadc_ad9361_dealloc(struct ad9361_rf_phy *phy)
{
	free(phy->adc_conv);
	free(phy->adc_state);
}

/***************************************************************************//**
 * @brief axiadc_post_setup
*******************************************************************************/
int axiadc_post_setup(struct ad9361_rf_phy *phy)
{
	return ad9361_post_setup(phy);
}

/***************************************************************************//**
 * @brief axiadc_read
*******************************************************************************/
unsigned int axiadc_read(struct axiadc_state *st, unsigned long reg)
{
	uint32_t val;

	adc_read(st->adc_st, reg, &val);

	return val;
}

/***************************************************************************//**
 * @brief axiadc_write
*******************************************************************************/
void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	adc_write(st->adc_st, reg, val);
}

/***************************************************************************//**
 * @brief axiadc_set_pnsel
*******************************************************************************/
int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel)
{
	unsigned reg;

	uint32_t version = axiadc_read(st, 0x4000);

	if (PCORE_VERSION_MAJOR(version) > 7) {
		reg = axiadc_read(st, ADC_REG_CHAN_CNTRL_3(channel));
		reg &= ~ADC_ADC_PN_SEL(~0);
		reg |= ADC_ADC_PN_SEL(sel);
		axiadc_write(st, ADC_REG_CHAN_CNTRL_3(channel), reg);
	} else {
		reg = axiadc_read(st, ADC_REG_CHAN_CNTRL(channel));

		if (sel == ADC_PN_CUSTOM) {
			reg |= ADC_PN_SEL;
		} else if (sel == ADC_PN9) {
			reg &= ~ADC_PN23_TYPE;
			reg &= ~ADC_PN_SEL;
		} else {
			reg |= ADC_PN23_TYPE;
			reg &= ~ADC_PN_SEL;
		}

		axiadc_write(st, ADC_REG_CHAN_CNTRL(channel), reg);
	}

	return 0;
}

/***************************************************************************//**
 * @brief axiadc_idelay_set
*******************************************************************************/
void axiadc_idelay_set(struct axiadc_state *st,
				unsigned lane, unsigned val)
{
	if (PCORE_VERSION_MAJOR(st->pcore_version) > 8) {
		axiadc_write(st, ADC_REG_DELAY(lane), val);
	} else {
		axiadc_write(st, ADC_REG_DELAY_CNTRL, 0);
		axiadc_write(st, ADC_REG_DELAY_CNTRL,
				ADC_DELAY_ADDRESS(lane)
				| ADC_DELAY_WDATA(val)
				| ADC_DELAY_SEL);
	}
}

/***************************************************************************//**
 * @brief adc_capture
*******************************************************************************/
int32_t adc_capture(adc_state *adc_st, uint32_t size)
{
	uint32_t reg_val;
	uint32_t transfer_id;
	uint32_t length;

	length = (size * adc_st->num_tx_channels * sizeof(uint32_t));

	adc_dma_write(AXI_DMAC_REG_CTRL, 0x0);
	adc_dma_write(AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE);

	adc_dma_write(AXI_DMAC_REG_IRQ_MASK, 0x0);

	adc_dma_read(AXI_DMAC_REG_TRANSFER_ID, &transfer_id);
	adc_dma_read(AXI_DMAC_REG_IRQ_PENDING, &reg_val);
	adc_dma_write(AXI_DMAC_REG_IRQ_PENDING, reg_val);

	adc_dma_write(AXI_DMAC_REG_DEST_ADDRESS, adc_st->start_address);
	adc_dma_write(AXI_DMAC_REG_DEST_STRIDE, 0x0);
	adc_dma_write(AXI_DMAC_REG_X_LENGTH, length - 1);
	adc_dma_write(AXI_DMAC_REG_Y_LENGTH, 0x0);

	adc_dma_write(AXI_DMAC_REG_START_TRANSFER, 0x1);
	/* Wait until the new transfer is queued. */
	do {
		adc_dma_read(AXI_DMAC_REG_START_TRANSFER, &reg_val);
	}
	while(reg_val == 1);

	/* Wait until the current transfer is completed. */
	do {
		adc_dma_read(AXI_DMAC_REG_IRQ_PENDING, &reg_val);
	}
	while(reg_val != (AXI_DMAC_IRQ_SOT | AXI_DMAC_IRQ_EOT));
	adc_dma_write(AXI_DMAC_REG_IRQ_PENDING, reg_val);

	/* Wait until the transfer with the ID transfer_id is completed. */
	do {
		adc_dma_read(AXI_DMAC_REG_TRANSFER_DONE, &reg_val);
	}
	while((reg_val & (1 << transfer_id)) != (1 << transfer_id));

	return 0;
}

/***************************************************************************//**
 * @brief adc_set_calib_scale_phase
*******************************************************************************/
int32_t adc_set_calib_scale_phase(adc_state *adc_st,
								  uint32_t phase,
								  uint32_t chan,
								  int32_t val,
								  int32_t val2)
{
	uint32_t fract;
	uint64_t llval;
	uint32_t tmp;

	switch (val) {
	case 1:
		fract = 0x4000;
		break;
	case -1:
		fract = 0xC000;
		break;
	case 0:
		fract = 0;
		if (val2 < 0) {
			fract = 0x8000;
			val2 *= -1;
		}
		break;
	default:
		return -1;
	}

	llval = (uint64_t)val2 * 0x4000UL + (1000000UL / 2);
	do_div(&llval, 1000000UL);
	fract |= llval;

	adc_read(adc_st, ADC_REG_CHAN_CNTRL_2(chan), &tmp);

	if (!((chan + phase) % 2)) {
		tmp &= ~ADC_IQCOR_COEFF_1(~0);
		tmp |= ADC_IQCOR_COEFF_1(fract);
	} else {
		tmp &= ~ADC_IQCOR_COEFF_2(~0);
		tmp |= ADC_IQCOR_COEFF_2(fract);
	}

	adc_write(adc_st, ADC_REG_CHAN_CNTRL_2(chan), tmp);

	return 0;
}

/***************************************************************************//**
 * @brief adc_get_calib_scale_phase
*******************************************************************************/
int32_t adc_get_calib_scale_phase(adc_state *adc_st,
								  uint32_t phase,
								  uint32_t chan,
								  int32_t *val,
								  int32_t *val2)
{
	uint32_t tmp;
	int32_t sign;
	uint64_t llval;

	adc_read(adc_st, ADC_REG_CHAN_CNTRL_2(chan), &tmp);

	/* format is 1.1.14 (sign, integer and fractional bits) */

	if (!((phase + chan) % 2)) {
		tmp = ADC_TO_IQCOR_COEFF_1(tmp);
	} else {
		tmp = ADC_TO_IQCOR_COEFF_2(tmp);
	}

	if (tmp & 0x8000)
		sign = -1;
	else
		sign = 1;

	if (tmp & 0x4000)
		*val = 1 * sign;
	else
		*val = 0;

	tmp &= ~0xC000;

	llval = tmp * 1000000ULL + (0x4000 / 2);
	do_div(&llval, 0x4000);
	if (*val == 0)
		*val2 = llval * sign;
	else
		*val2 = llval;

	return 0;
}

/***************************************************************************//**
 * @brief adc_set_calib_scale
*******************************************************************************/
int32_t adc_set_calib_scale(adc_state *adc_st,
							uint32_t chan,
							int32_t val,
							int32_t val2)
{
	return adc_set_calib_scale_phase(adc_st, 0, chan, val, val2);
}

/***************************************************************************//**
 * @brief adc_get_calib_scale
*******************************************************************************/
int32_t adc_get_calib_scale(adc_state *adc_st,
							uint32_t chan,
							int32_t *val,
							int32_t *val2)
{
	return adc_get_calib_scale_phase(adc_st, 0, chan, val, val2);
}

/***************************************************************************//**
 * @brief adc_set_calib_phase
*******************************************************************************/
int32_t adc_set_calib_phase(adc_state *adc_st,
							uint32_t chan,
							int32_t val,
							int32_t val2)
{
	return adc_set_calib_scale_phase(adc_st, 1, chan, val, val2);
}

/***************************************************************************//**
 * @brief adc_get_calib_phase
*******************************************************************************/
int32_t adc_get_calib_phase(adc_state *adc_st,
							uint32_t chan,
							int32_t *val,
							int32_t *val2)
{
	return adc_get_calib_scale_phase(adc_st, 1, chan, val, val2);
}
