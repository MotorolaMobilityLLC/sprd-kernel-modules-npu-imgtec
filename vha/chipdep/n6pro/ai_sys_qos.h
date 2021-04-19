#ifndef __AI_SYS_QOS_H_
#define __AI_SYS_QOS_H_

typedef struct _QOS_REG_STRUCT
{
	const char *reg_name;
	uint32_t base_addr;
	uint32_t mask_value;
	uint32_t set_value;

} QOS_REG_T;

QOS_REG_T nic400_ai_main_mtx_m0_qos_list[] = {
	{ "REG_AI_MTX_REGU_OT_CTRL_EN",        0x0000, 0x00000001, 0x00000001},
	{ "REG_AI_MTX_REGU_OT_CTRL_AW_CFG",    0x0004, 0xffffffff, 0x0a080808},
	{ "REG_AI_MTX_REGU_OT_CTRL_AR_CFG",    0x0008, 0x3f3f3f3f, 0x3f3f3f3f},
	{ "REG_AI_MTX_REGU_OT_CTRL_AX_CFG",    0x000C, 0x3f3fffff, 0x3f3f1010},
	{ "REG_AI_MTX_REGU_LAT_EN",            0x0010, 0x00000003, 0x00000000},
	{ "REG_AI_MTX_REGU_LAT_W_CFG",         0x0014, 0xffffffff, 0x00000000},
	{ "REG_AI_MTX_REGU_LAT_R_CFG",         0x0018, 0xffffffff, 0x00000000},
	{ "REG_AI_MTX_REGU_BW_NRT_EN",         0x0040, 0x00000003, 0x00000000},
	{ "REG_AI_MTX_REGU_BW_NRT_W_CFG_0",    0x0044, 0xffffffff, 0x00000000},
	{ "REG_AI_MTX_REGU_BW_NRT_W_CFG_1",    0x0048, 0x073fffff, 0x00000000},
	{ "REG_AI_MTX_REGU_BW_NRT_R_CFG_0",    0x004C, 0xffffffff, 0x00000000},
	{ "REG_AI_MTX_REGU_BW_NRT_R_CFG_1",    0x0050, 0x073fffff, 0x00000000},
	{ "REG_AI_MTX_REGU_AXQOS_GEN_EN",      0x0060, 0x80000003, 0x00000003},
	{ "REG_AI_MTX_REGU_AXQOS_GEN_CFG",     0x0064, 0x3fff3fff, 0x06660666},
	{ "REG_AI_MTX_REGU_URG_CNT_CFG",       0x0068, 0x00000701, 0x00000001}
};


QOS_REG_T ai_apb_rf_qos_list[] = {
	{ "REG_AI_APB_LPC_MAIN_MTX_M0",        0x0020, 0x00010000, 0x00000000},
	{ "REG_AI_APB_LPC_MAIN_MTX_M0",        0x0020, 0x00010000, 0x00010000},
	{ "REG_AI_APB_LPC_MAIN_MTX_M1",        0x0024, 0x00010000, 0x00000000},
	{ "REG_AI_APB_LPC_MAIN_MTX_M1",        0x0024, 0x00010000, 0x00010000}
};

#endif
