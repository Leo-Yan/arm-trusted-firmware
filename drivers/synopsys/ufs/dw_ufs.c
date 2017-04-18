/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <debug.h>
#include <dw_ufs.h>
#include <mmio.h>
#include <stdint.h>
#include <string.h>
#include <ufs.h>

static int dwufs_phy_init(ufs_params_t *params)
{
	uintptr_t base;
	unsigned int fsm0, fsm1;
	unsigned int data;
	int result;

	assert((params != NULL) && 		\
	       (params->reg_base != 0));

	base = params->reg_base;

	/* Unipro VS_MPHY disable */
	ufshc_dme_set(VS_MPHY_DISABLE_OFFSET, 0, VS_MPHY_DISABLE_MPHYDIS);
	ufshc_dme_set(PA_HSSERIES_OFFSET, 0, 2);
	/* MPHY CBRATESEL */
	ufshc_dme_set(0x8114, 0, 1);
	/* MPHY CBOVRCTRL2 */
	ufshc_dme_set(0x8121, 0, 0x2d);
	/* MPHY CBOVRCTRL3 */
	ufshc_dme_set(0x8122, 0, 0x1);
	ufshc_dme_set(VS_MPHY_CFG_UPDT_OFFSET, 0, 1);

	/* MPHY RXOVRCTRL4 rx0 */
	ufshc_dme_set(0x800d, 4, 0x58);
	/* MPHY RXOVRCTRL4 rx1 */
	ufshc_dme_set(0x800d, 5, 0x58);
	/* MPHY RXOVRCTRL5 rx0 */
	ufshc_dme_set(0x800e, 4, 0xb);
	/* MPHY RXOVRCTRL5 rx1 */
	ufshc_dme_set(0x800e, 5, 0xb);
	/* MPHY RXSQCONTROL rx0 */
	ufshc_dme_set(0x8009, 4, 0x1);
	/* MPHY RXSQCONTROL rx1 */
	ufshc_dme_set(0x8009, 5, 0x1);
	ufshc_dme_set(VS_MPHY_CFG_UPDT_OFFSET, 0, 1);

	ufshc_dme_set(0x8113, 0, 0x1);
	ufshc_dme_set(VS_MPHY_CFG_UPDT_OFFSET, 0, 1);

	ufshc_dme_set(RX_HS_G3_SYNC_LENGTH_CAP_OFFSET, 4, 0x4a);
	ufshc_dme_set(RX_HS_G3_SYNC_LENGTH_CAP_OFFSET, 5, 0x4a);
	ufshc_dme_set(RX_HS_G2_SYNC_LENGTH_CAP_OFFSET, 4, 0x4a);
	ufshc_dme_set(RX_HS_G2_SYNC_LENGTH_CAP_OFFSET, 5, 0x4a);
	ufshc_dme_set(RX_MIN_ACTIVATETIME_CAP_OFFSET, 4, 0x7);
	ufshc_dme_set(RX_MIN_ACTIVATETIME_CAP_OFFSET, 5, 0x7);
	ufshc_dme_set(TX_HIBERN8TIME_CAP_OFFSET, 0, 0x5);
	ufshc_dme_set(TX_HIBERN8TIME_CAP_OFFSET, 1, 0x5);
	ufshc_dme_set(VS_MPHY_CFG_UPDT_OFFSET, 0, 1);

	result = ufshc_dme_get(VS_MPHY_DISABLE_OFFSET, 0, &data);
	assert((result == 0) && (data == VS_MPHY_DISABLE_MPHYDIS));
	/* enable Unipro VS MPHY */
	ufshc_dme_set(VS_MPHY_DISABLE_OFFSET, 0, 0);

	while (1) {
		result = ufshc_dme_get(TX_FSM_STATE_OFFSET, 0, &fsm0);
		assert(result == 0);
		result = ufshc_dme_get(TX_FSM_STATE_OFFSET, 1, &fsm1);
		assert(result == 0);
		if ((fsm0 == TX_FSM_STATE_HIBERN8) &&
		    (fsm1 == TX_FSM_STATE_HIBERN8))
			break;
	}

	mmio_write_32(base + HCLKDIV, 0xE4);
	mmio_clrbits_32(base + AHIT, 0x3FF);

	ufshc_dme_set(PA_LOCAL_TX_LCC_ENABLE_OFFSET, 0, 0);
	ufshc_dme_set(VS_MK2_EXTN_SUPPORT_OFFSET, 0, 0);

	result = ufshc_dme_get(VS_MK2_EXTN_SUPPORT_OFFSET, 0, &data);
	assert((result == 0) && (data == 0));

	ufshc_dme_set(DL_AFC0_CREDIT_THRESHOLD_OFFSET, 0, 0);
	ufshc_dme_set(DL_TC0_OUT_ACK_THRESHOLD_OFFSET, 0, 0);
	ufshc_dme_set(DL_TC0_TX_FC_THRESHOLD_OFFSET, 0, 9);
	return 0;
}

static int dwufs_phy_set_pwr_mode(ufs_params_t *params)
{
	int result;
	unsigned int data, tx_lanes, rx_lanes;
	uintptr_t base;

	assert((params != NULL) && 		\
	       (params->reg_base != 0));

	base = params->reg_base;

	// PA_Tactive
	result = ufshc_dme_get(0x15A8, 0, &data);
	assert(result == 0);
	if (data < 7) {
		result = ufshc_dme_set(0x15A8, 0, 7);
		assert(result == 0);
	}
	result = ufshc_dme_get(0x1561, 0, &tx_lanes);
	assert(result == 0);
	result = ufshc_dme_get(0x1581, 0, &rx_lanes);
	assert(result == 0);

	// PA TxSkip
	result = ufshc_dme_set(0x155c, 0, 0);
	assert(result == 0);
	// PA TxGear
	result = ufshc_dme_set(0x1568, 0, 3);
	assert(result == 0);
	// PA RxGear
	result = ufshc_dme_set(0x1583, 0, 3);
	assert(result == 0);
	// PA HSSeries
	result = ufshc_dme_set(0x156a, 0, 2);
	assert(result == 0);
	// PA TxTermination
	result = ufshc_dme_set(0x1569, 0, 1);
	assert(result == 0);
	// PA RxTermination
	result = ufshc_dme_set(0x1584, 0, 1);
	assert(result == 0);
	// PA Scrambling
	result = ufshc_dme_set(0x1585, 0, 0);
	assert(result == 0);
	// PA ActiveTxDataLines
	result = ufshc_dme_set(0x1560, 0, tx_lanes);
	assert(result == 0);
	// PA ActiveRxDataLines
	result = ufshc_dme_set(0x1580, 0, rx_lanes);
	assert(result == 0);
	// PA_PWRModeUserData0 = 8191
	result = ufshc_dme_set(0x15b0, 0, 8191);
	assert(result == 0);
	// PA_PWRModeUserData1 = 65535
	result = ufshc_dme_set(0x15b1, 0, 65535);
	assert(result == 0);
	// PA_PWRModeUserData2 = 32767
	result = ufshc_dme_set(0x15b2, 0, 32767);
	assert(result == 0);
	// DME_FC0ProtectionTimeOutVal = 8191
	result = ufshc_dme_set(0xd041, 0, 8191);
	assert(result == 0);
	// DME_TC0ReplayTimeOutVal = 65535
	result = ufshc_dme_set(0xd042, 0, 65535);
	assert(result == 0);
	// DME_AFC0ReqTimeOutVal = 32767
	result = ufshc_dme_set(0xd043, 0, 32767);
	assert(result == 0);
	// PA_PWRModeUserData3 = 8191
	result = ufshc_dme_set(0x15b3, 0, 8191);
	assert(result == 0);
	// PA_PWRModeUserData4 = 65535
	result = ufshc_dme_set(0x15b4, 0, 65535);
	assert(result == 0);
	// PA_PWRModeUserData5 = 32767
	result = ufshc_dme_set(0x15b5, 0, 32767);
	assert(result == 0);
	// DME_FC1ProtectionTimeOutVal = 8191
	result = ufshc_dme_set(0xd044, 0, 8191);
	assert(result == 0);
	// DME_TC1ReplayTimeOutVal = 65535
	result = ufshc_dme_set(0xd045, 0, 65535);
	assert(result == 0);
	// DME_AFC1ReqTimeOutVal = 32767
	result = ufshc_dme_set(0xd046, 0, 32767);
	assert(result == 0);

	result = ufshc_dme_set(0x1571, 0, 0x11);
	assert(result == 0);
	do {
		data = mmio_read_32(base + IS);
	} while ((data & UFS_INT_UPMS) == 0);
	mmio_write_32(base + IS, UFS_INT_UPMS);
	data = mmio_read_32(base + HCS);
	if ((data & HCS_UPMCRS_MASK) == HCS_PWR_LOCAL) {
		INFO("ufs: change power mode success\n");
	} else {
		WARN("ufs: HCS.UPMCRS error, HCS:0x%x\n", data);
	}
	return 0;
}

const ufs_ops_t dw_ufs_ops = {
	.phy_init 		= dwufs_phy_init,
	.phy_set_pwr_mode	= dwufs_phy_set_pwr_mode,
};

int dw_ufs_init(dw_ufs_params_t *params)
{
	ufs_params_t ufs_params;

	memset(&ufs_params, 0, sizeof(ufs_params));
	ufs_params.reg_base = params->reg_base;
	ufs_params.desc_base = params->desc_base;
	ufs_params.desc_size = params->desc_size;
	ufs_params.flags = params->flags;
	ufs_init(&dw_ufs_ops, &ufs_params);
	return 0;
}