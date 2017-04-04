/***************************************************************************//**
 *   @file   dac_core.c
 *   @brief  Implementation of DAC Core Driver.
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
#include <xil_cache.h>
#include <xil_io.h>
#include "dac_core.h"
#include "parameters.h"
#include "util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
//#define FMCOMMS5

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
const uint16_t sine_lut[128] = {
		0x000, 0x064, 0x0C8, 0x12C, 0x18F, 0x1F1, 0x252, 0x2B1,
		0x30F, 0x36B, 0x3C5, 0x41C, 0x471, 0x4C3, 0x512, 0x55F,
		0x5A7, 0x5ED, 0x62E, 0x66C, 0x6A6, 0x6DC, 0x70D, 0x73A,
		0x763, 0x787, 0x7A7, 0x7C2, 0x7D8, 0x7E9, 0x7F5, 0x7FD,
		0x7FF, 0x7FD, 0x7F5, 0x7E9, 0x7D8, 0x7C2, 0x7A7, 0x787,
		0x763, 0x73A, 0x70D, 0x6DC, 0x6A6, 0x66C, 0x62E, 0x5ED,
		0x5A7, 0x55F, 0x512, 0x4C3, 0x471, 0x41C, 0x3C5, 0x36B,
		0x30F, 0x2B1, 0x252, 0x1F1, 0x18F, 0x12C, 0xC8,  0x64,
		0x000, 0xF9B, 0xF37, 0xED3, 0xE70, 0xE0E, 0xDAD, 0xD4E,
		0xCF0, 0xC94, 0xC3A, 0xBE3, 0xB8E, 0xB3C, 0xAED, 0xAA0,
		0xA58, 0xA12, 0x9D1, 0x993, 0x959, 0x923, 0x8F2, 0x8C5,
		0x89C, 0x878, 0x858, 0x83D, 0x827, 0x816, 0x80A, 0x802,
		0x800, 0x802, 0x80A, 0x816, 0x827, 0x83D, 0x858, 0x878,
		0x89C, 0x8C5, 0x8F2, 0x923, 0x959, 0x993, 0x9D1, 0xA12,
		0xA58, 0xAA0, 0xAED, 0xB3C, 0xB8E, 0xBE3, 0xC3A, 0xC94,
		0xCF0, 0xD4E, 0xDAD, 0xE0E, 0xE70, 0xED3, 0xF37, 0xF9B
};

const uint32_t sine_lut_iq[128] = {
		0x00002660, 0x01E02650, 0x03C02630, 0x05A025F0, 0x077025A0,
		0x09502530, 0x0B2024B0, 0x0CE02420, 0x0EB02370, 0x106022B0,
		0x121021D0, 0x13B020E0, 0x15501FE0, 0x16D01ED0, 0x18501DA0,
		0x19C01C70, 0x1B201B20, 0x1C7019C0, 0x1DA01850, 0x1ED016D0,
		0x1FE01550, 0x20E013B0, 0x21D01210, 0x22B01060, 0x23700EB0,
		0x24200CE0, 0x24B00B20, 0x25300950, 0x25A00770, 0x25F005A0,
		0x263003C0, 0x265001E0, 0x26600000, 0x2650FE10, 0x2630FC30,
		0x25F0FA50, 0x25A0F880, 0x2530F6A0, 0x24B0F4D0, 0x2420F310,
		0x2370F150, 0x22B0EF90, 0x21D0EDE0, 0x20E0EC40, 0x1FE0EAA0,
		0x1ED0E920, 0x1DA0E7A0, 0x1C70E630, 0x1B20E4D0, 0x19C0E390,
		0x1850E250, 0x16D0E120, 0x1550E010, 0x13B0DF10, 0x1210DE20,
		0x1060DD40, 0x0EB0DC80, 0x0CE0DBD0, 0x0B20DB40, 0x0950DAC0,
		0x0770DA50, 0x05A0DA00, 0x03C0D9C0, 0x01E0D9A0, 0x0000D990,
		0xFE10D9A0, 0xFC30D9C0, 0xFA50DA00, 0xF880DA50, 0xF6A0DAC0,
		0xF4D0DB40, 0xF310DBD0, 0xF150DC80, 0xEF90DD40, 0xEDE0DE20,
		0xEC40DF10, 0xEAA0E010, 0xE920E120, 0xE7A0E250, 0xE630E390,
		0xE4D0E4D0, 0xE390E630, 0xE250E7A0, 0xE120E920, 0xE010EAA0,
		0xDF10EC40, 0xDE20EDE0, 0xDD40EF90, 0xDC80F150, 0xDBD0F310,
		0xDB40F4D0, 0xDAC0F6A0, 0xDA50F880, 0xDA00FA50, 0xD9C0FC30,
		0xD9A0FE10, 0xD9900000, 0xD9A001E0, 0xD9C003C0, 0xDA0005A0,
		0xDA500770, 0xDAC00950, 0xDB400B20, 0xDBD00CE0, 0xDC800EB0,
		0xDD401060, 0xDE201210, 0xDF1013B0, 0xE0101550, 0xE12016D0,
		0xE2501850, 0xE39019C0, 0xE4D01B20, 0xE6301C70, 0xE7A01DA0,
		0xE9201ED0, 0xEAA01FE0, 0xEC4020E0, 0xEDE021D0, 0xEF9022B0,
		0xF1502370, 0xF3102420, 0xF4D024B0, 0xF6A02530, 0xF88025A0,
		0xFA5025F0, 0xFC302630, 0xFE102650
};
/***************************************************************************//**
 * @brief dac_read
*******************************************************************************/
void dac_read(dds_state *dds, uint32_t regAddr, uint32_t *data)
{
	*data = Xil_In32(dds->ad9361_tx_baseaddr + regAddr);
}

/***************************************************************************//**
 * @brief dac_write
*******************************************************************************/
void dac_write(dds_state *dds, uint32_t regAddr, uint32_t data)
{
	Xil_Out32(dds->ad9361_tx_baseaddr + regAddr, data);
}

/***************************************************************************//**
 * @brief dac_dma_read
*******************************************************************************/
void dac_dma_read(uint32_t regAddr, uint32_t *data)
{
	*data = Xil_In32(CF_AD9361_TX_DMA_BASEADDR + regAddr);
}

/***************************************************************************//**
 * @brief dac_dma_write
*******************************************************************************/
void dac_dma_write(uint32_t regAddr, uint32_t data)
{
	Xil_Out32(CF_AD9361_TX_DMA_BASEADDR + regAddr, data);
}

/***************************************************************************//**
 * @brief dds_default_setup
*******************************************************************************/
int32_t dds_default_setup(dds_state *dds,
						  uint32_t chan, uint32_t phase,
						  uint32_t freq, int32_t scale)
{
	dds_set_phase(dds, chan, phase);
	dds_set_frequency(dds, chan, freq);
	dds_set_scale(dds, chan, scale);
	dds[dds->id_no].cached_freq[chan] = freq;
	dds[dds->id_no].cached_phase[chan] = phase;
	dds[dds->id_no].cached_scale[chan] = scale;

	return 0;
}

/***************************************************************************//**
 * @brief dac_stop
*******************************************************************************/
void dac_stop(dds_state *dds)
{
	if (PCORE_VERSION_MAJOR(dds[dds->id_no].pcore_version) < 8)
	{
		dac_write(dds, DAC_REG_CNTRL_1, 0);
	}
}

/***************************************************************************//**
 * @brief dac_start_sync
*******************************************************************************/
void dac_start_sync(dds_state *dds, bool force_on)
{
	if (PCORE_VERSION_MAJOR(dds[dds->id_no].pcore_version) < 8)
	{
		dac_write(dds, DAC_REG_CNTRL_1, (dds[dds->id_no].enable || force_on) ? DAC_ENABLE : 0);
	}
	else
	{
		dac_write(dds, DAC_REG_CNTRL_1, DAC_SYNC);
	}
}

/***************************************************************************//**
 * @brief dac_write_custom_data
*******************************************************************************/
void dac_write_custom_data(dds_state *dds,
					  	   const uint32_t *custom_data_iq,
						   uint32_t custom_tx_count)
{
	uint32_t index;
	uint32_t index_mem = 0;
	uint32_t length;
	uint8_t chan;

	for(index = 0; index < custom_tx_count; index++)
	{
		for (chan = 0; chan < dds->num_tx_channels; chan++)
		{
			Xil_Out32(dds->dac_ddr_baseaddr + index_mem * 4, custom_data_iq[index]);
			index_mem++;
		}
	}
	Xil_DCacheFlushRange(dds->dac_ddr_baseaddr, index_mem * 4);

	length = index_mem * 4;

	dac_dma_write(AXI_DMAC_REG_CTRL, 0);
	dac_dma_write(AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE);
	dac_dma_write(AXI_DMAC_REG_SRC_ADDRESS, dds->dac_ddr_baseaddr);
	dac_dma_write(AXI_DMAC_REG_SRC_STRIDE, 0x0);
	dac_dma_write(AXI_DMAC_REG_X_LENGTH, length - 1);
	dac_dma_write(AXI_DMAC_REG_Y_LENGTH, 0x0);
	dac_dma_write(AXI_DMAC_REG_START_TRANSFER, 0x1);
}

/***************************************************************************//**
 * @brief dac_init
*******************************************************************************/
int32_t dac_init(dds_state **dds_st,
				 enum dds_data_select data_sel,
				 uint32_t *dac_clk,
				 dds_state_init dds_init)
{
	dds_state *dds;
	uint32_t reg_ctrl_2;

	dds = (dds_state *)malloc(sizeof(*dds));
	if (!dds)
		return -1;

	dds->id_no = dds_init.id_no;
	switch (dds->id_no)
	{
	case 0:
		dds->ad9361_tx_baseaddr = AD9361_TX_0_BASEADDR;
		break;
	case 1:
		dds->ad9361_tx_baseaddr = AD9361_TX_1_BASEADDR;
		break;
	default:
		break;
	}

	dds->dac_ddr_baseaddr = dds_init.dac_ddr_baseaddr;

	dac_write(dds, DAC_REG_RSTN, 0x0);
	dac_write(dds, DAC_REG_RSTN, DAC_RSTN | DAC_MMCM_RSTN);

	dds[dds->id_no].dac_clk = dac_clk;
	dac_read(dds, DAC_REG_CNTRL_2, &reg_ctrl_2);

	dds->num_tx_channels = dds_init.num_tx_channels;
	switch (dds->num_tx_channels)
	{
	case 1:
		dds[dds->id_no].num_buf_channels = 2;
		dac_write(dds, DAC_REG_RATECNTRL, DAC_RATE(1));
		reg_ctrl_2 |= DAC_R1_MODE;
		break;
	case 2:
		dds[dds->id_no].num_buf_channels = 4;
		dac_write(dds, DAC_REG_RATECNTRL, DAC_RATE(3));
		reg_ctrl_2 &= ~DAC_R1_MODE;
		break;
	default:
		break;
	}

	dac_write(dds, DAC_REG_CNTRL_2, reg_ctrl_2);
	dac_read(dds, DAC_REG_VERSION, &dds[dds->id_no].pcore_version);
	dac_stop(dds);

	if (DATA_SEL_DDS == data_sel) {
		dds_default_setup(dds, DDS_CHAN_TX1_I_F1, 90000, 1000000, 250000);
		dds_default_setup(dds, DDS_CHAN_TX1_I_F2, 90000, 1000000, 250000);
		dds_default_setup(dds, DDS_CHAN_TX1_Q_F1, 0, 1000000, 250000);
		dds_default_setup(dds, DDS_CHAN_TX1_Q_F2, 0, 1000000, 250000);

		if(dds->num_tx_channels == 2)
		{
			dds_default_setup(dds, DDS_CHAN_TX2_I_F1, 90000, 1000000, 250000);
			dds_default_setup(dds, DDS_CHAN_TX2_I_F2, 90000, 1000000, 250000);
			dds_default_setup(dds, DDS_CHAN_TX2_Q_F1, 0, 1000000, 250000);
			dds_default_setup(dds, DDS_CHAN_TX2_Q_F2, 0, 1000000, 250000);
		}
	}

	dac_datasel(dds, -1, data_sel);
	dds[dds->id_no].enable = dds_init.enable;
	dac_start_sync(dds, 0);

	*dds_st = dds;

	return 0;
}

/***************************************************************************//**
 * @brief dds_set_frequency
*******************************************************************************/
void dds_set_frequency(dds_state *dds, uint32_t chan, uint32_t freq)
{
	uint64_t val64;
	uint32_t reg;

	dds[dds->id_no].cached_freq[chan] = freq;
	dac_stop(dds);
	dac_read(dds, DAC_REG_CHAN_CNTRL_2_IIOCHAN(chan), &reg);
	reg &= ~DAC_DDS_INCR(~0);
	val64 = (uint64_t) freq * 0xFFFFULL;
	do_div(&val64, *dds[dds->id_no].dac_clk);
	reg |= DAC_DDS_INCR(val64) | 1;
	dac_write(dds, DAC_REG_CHAN_CNTRL_2_IIOCHAN(chan), reg);
	dac_start_sync(dds, 0);
}

/***************************************************************************//**
 * @brief dds_get_frequency
*******************************************************************************/
void dds_get_frequency(dds_state *dds, uint32_t chan, uint32_t *freq)
{
	*freq = dds[dds->id_no].cached_freq[chan];
}

/***************************************************************************//**
 * @brief dds_set_phase
*******************************************************************************/
void dds_set_phase(dds_state *dds, uint32_t chan, uint32_t phase)
{
	uint64_t val64;
	uint32_t reg;

	dds[dds->id_no].cached_phase[chan] = phase;
	dac_stop(dds);
	dac_read(dds, DAC_REG_CHAN_CNTRL_2_IIOCHAN(chan), &reg);
	reg &= ~DAC_DDS_INIT(~0);
	val64 = (uint64_t) phase * 0x10000ULL + (360000 / 2);
	do_div(&val64, 360000);
	reg |= DAC_DDS_INIT(val64);
	dac_write(dds, DAC_REG_CHAN_CNTRL_2_IIOCHAN(chan), reg);
	dac_start_sync(dds, 0);
}

/***************************************************************************//**
 * @brief dds_get_phase
*******************************************************************************/
void dds_get_phase(dds_state *dds, uint32_t chan, uint32_t *phase)
{
	*phase = dds[dds->id_no].cached_phase[chan];
}

/***************************************************************************//**
 * @brief dds_set_phase
*******************************************************************************/
void dds_set_scale(dds_state *dds, uint32_t chan, int32_t scale_micro_units)
{
	uint32_t scale_reg;
	uint32_t sign_part;
	uint32_t int_part;
	uint32_t fract_part;

	if (PCORE_VERSION_MAJOR(dds[dds->id_no].pcore_version) > 6)
	{
		if(scale_micro_units >= 1000000)
		{
			sign_part = 0;
			int_part = 1;
			fract_part = 0;
			dds[dds->id_no].cached_scale[chan] = 1000000;
			goto set_scale_reg;
		}
		if(scale_micro_units <= -1000000)
		{
			sign_part = 1;
			int_part = 1;
			fract_part = 0;
			dds[dds->id_no].cached_scale[chan] = -1000000;
			goto set_scale_reg;
		}
		dds[dds->id_no].cached_scale[chan] = scale_micro_units;
		if(scale_micro_units < 0)
		{
			sign_part = 1;
			int_part = 0;
			scale_micro_units *= -1;
		}
		else
		{
			sign_part = 0;
			int_part = 0;
		}
		fract_part = (uint32_t)(((uint64_t)scale_micro_units * 0x4000) / 1000000);
	set_scale_reg:
		scale_reg = (sign_part << 15) | (int_part << 14) | fract_part;
	}
	else
	{
		if(scale_micro_units >= 1000000)
		{
			scale_reg = 0;
			scale_micro_units = 1000000;
		}
		if(scale_micro_units <= 0)
		{
			scale_reg = 0;
			scale_micro_units = 0;
		}
		dds[dds->id_no].cached_scale[chan] = scale_micro_units;
		fract_part = (uint32_t)(scale_micro_units);
		scale_reg = 500000 / fract_part;
	}
	dac_stop(dds);
	dac_write(dds, DAC_REG_CHAN_CNTRL_1_IIOCHAN(chan), DAC_DDS_SCALE(scale_reg));
	dac_start_sync(dds, 0);
}

/***************************************************************************//**
 * @brief dds_get_phase
*******************************************************************************/
void dds_get_scale(dds_state *dds, uint32_t chan, int32_t *scale_micro_units)
{
	*scale_micro_units = dds[dds->id_no].cached_scale[chan];
}

/***************************************************************************//**
 * @brief dds_update
*******************************************************************************/
void dds_update(dds_state *dds)
{
	uint32_t chan;

	for(chan = DDS_CHAN_TX1_I_F1; chan <= DDS_CHAN_TX2_Q_F2; chan++)
	{
		dds_set_frequency(dds, chan, dds[dds->id_no].cached_freq[chan]);
		dds_set_phase(dds, chan, dds[dds->id_no].cached_phase[chan]);
		dds_set_scale(dds, chan, dds[dds->id_no].cached_scale[chan]);
	}
}

/***************************************************************************//**
 * @brief dac_datasel
*******************************************************************************/
int32_t dac_datasel(dds_state *dds, int32_t chan, enum dds_data_select sel)
{
	int32_t i;

	if (PCORE_VERSION_MAJOR(dds[dds->id_no].pcore_version) > 7) {
		if (chan < 0) { /* ALL */
			for (i = 0; i < dds[dds->id_no].num_buf_channels; i++) {
				dac_write(dds, DAC_REG_CHAN_CNTRL_7(i), sel);
				dds[dds->id_no].cached_datasel[i] = sel;
			}
		} else {
			dac_write(dds, DAC_REG_CHAN_CNTRL_7(chan), sel);
			dds[dds->id_no].cached_datasel[chan] = sel;
		}
	} else {
		uint32_t reg;

		switch(sel) {
		case DATA_SEL_DDS:
		case DATA_SEL_SED:
		case DATA_SEL_DMA:
			dac_read(dds, DAC_REG_CNTRL_2, &reg);
			reg &= ~DAC_DATA_SEL(~0);
			reg |= DAC_DATA_SEL(sel);
			dac_write(dds, DAC_REG_CNTRL_2, reg);
			break;
		default:
			return -EINVAL;
		}
		for (i = 0; i < dds[dds->id_no].num_buf_channels; i++) {
			dds[dds->id_no].cached_datasel[i] = sel;
		}
	}

	return 0;
}

/***************************************************************************//**
 * @brief dac_get_datasel
*******************************************************************************/
void dac_get_datasel(dds_state *dds, int32_t chan, enum dds_data_select *sel)
{
	*sel = dds[dds->id_no].cached_datasel[chan];
}

/***************************************************************************//**
 * @brief dds_to_signed_mag_fmt
*******************************************************************************/
uint32_t dds_to_signed_mag_fmt(int32_t val, int32_t val2)
{
	uint32_t i;
	uint64_t val64;

	/* format is 1.1.14 (sign, integer and fractional bits) */

	switch (val) {
	case 1:
		i = 0x4000;
		break;
	case -1:
		i = 0xC000;
		break;
	case 0:
		i = 0;
		if (val2 < 0) {
				i = 0x8000;
				val2 *= -1;
		}
		break;
	default:
		/* Invalid Value */
		i = 0;
	}

	val64 = (uint64_t)val2 * 0x4000UL + (1000000UL / 2);
	do_div(&val64, 1000000UL);

	return i | val64;
}

/***************************************************************************//**
 * @brief dds_from_signed_mag_fmt
*******************************************************************************/
void dds_from_signed_mag_fmt(uint32_t val,
							 int32_t *r_val,
							 int32_t *r_val2)
{
	uint64_t val64;
	int32_t sign;

	if (val & 0x8000)
		sign = -1;
	else
		sign = 1;

	if (val & 0x4000)
		*r_val = 1 * sign;
	else
		*r_val = 0;

	val &= ~0xC000;

	val64 = val * 1000000ULL + (0x4000 / 2);
	do_div(&val64, 0x4000);

	if (*r_val == 0)
		*r_val2 = val64 * sign;
	else
		*r_val2 = val64;
}

/***************************************************************************//**
 * @brief dds_set_calib_scale_phase
*******************************************************************************/
int32_t dds_set_calib_scale_phase(dds_state *dds,
								  uint32_t phase,
								  uint32_t chan,
								  int32_t val,
								  int32_t val2)
{
	uint32_t reg;
	uint32_t i;

	if (PCORE_VERSION_MAJOR(dds[dds->id_no].pcore_version) < 8) {
		return -1;
	}

	i = dds_to_signed_mag_fmt(val, val2);

	dac_read(dds, DAC_REG_CHAN_CNTRL_8(chan), &reg);

	if (!((chan + phase) % 2)) {
		reg &= ~DAC_IQCOR_COEFF_1(~0);
		reg |= DAC_IQCOR_COEFF_1(i);
	} else {
		reg &= ~DAC_IQCOR_COEFF_2(~0);
		reg |= DAC_IQCOR_COEFF_2(i);
	}
	dac_write(dds, DAC_REG_CHAN_CNTRL_8(chan), reg);
	dac_write(dds, DAC_REG_CHAN_CNTRL_6(chan), DAC_IQCOR_ENB);

	return 0;
}

/***************************************************************************//**
 * @brief dds_get_calib_scale_phase
*******************************************************************************/
int32_t dds_get_calib_scale_phase(dds_state *dds,
								  uint32_t phase,
								  uint32_t chan,
								  int32_t *val,
								  int32_t *val2)
{
	uint32_t reg;

	if (PCORE_VERSION_MAJOR(dds[dds->id_no].pcore_version) < 8) {
		return -1;
	}

	dac_read(dds, DAC_REG_CHAN_CNTRL_8(chan), &reg);

	/* format is 1.1.14 (sign, integer and fractional bits) */

	if (!((phase + chan) % 2)) {
		reg = DAC_TO_IQCOR_COEFF_1(reg);
	} else {
		reg = DAC_TO_IQCOR_COEFF_2(reg);
	}

	dds_from_signed_mag_fmt(reg, val, val2);

	return 0;
}

/***************************************************************************//**
 * @brief dds_set_calib_scale
*******************************************************************************/
int32_t dds_set_calib_scale(dds_state *dds,
							uint32_t chan,
							int32_t val,
							int32_t val2)
{
	return dds_set_calib_scale_phase(dds, 0, chan, val, val2);
}

/***************************************************************************//**
 * @brief dds_get_calib_scale
*******************************************************************************/
int32_t dds_get_calib_scale(dds_state *dds,
							uint32_t chan,
							int32_t *val,
							int32_t *val2)
{
	return dds_get_calib_scale_phase(dds, 0, chan, val, val2);
}

/***************************************************************************//**
 * @brief dds_set_calib_phase
*******************************************************************************/
int32_t dds_set_calib_phase(dds_state *dds,
							uint32_t chan,
							int32_t val,
							int32_t val2)
{
	return dds_set_calib_scale_phase(dds, 1, chan, val, val2);
}

/***************************************************************************//**
 * @brief dds_get_calib_phase
*******************************************************************************/
int32_t dds_get_calib_phase(dds_state *dds,
							uint32_t chan,
							int32_t *val,
							int32_t *val2)
{
	return dds_get_calib_scale_phase(dds, 1, chan, val, val2);
}
