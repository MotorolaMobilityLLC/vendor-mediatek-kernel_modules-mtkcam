#include "mot_imx896_cali.h"

static u8 imx896_spc_data[IMX896_SPC_DATA_LEN+2];
static u8 imx896_spc_data_ready = 0;
#if IMX896_SPC_DEBUG
#ifdef DRV_LOG
#undef DRV_LOG
#define DRV_LOG DRV_LOG_MUST
#endif
static u8 imx896_spc_readback[IMX896_SPC_DATA_LEN+2];
#endif

static u8 imx896_qsc_data[IMX896_QSC_DATA_LEN+2];
static u8 imx896_qsc_data_ready = 0;
#if IMX896_QSC_DEBUG
#ifdef DRV_LOG
#undef DRV_LOG
#define DRV_LOG DRV_LOG_MUST
#endif
static u8 imx896_qsc_readback[IMX896_QSC_DATA_LEN+2];
#endif

static int imx896_crc_reverse_byte(int data)
{
	return ((data * 0x0802LU & 0x22110LU) |
		(data * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
}

static int32_t imx896_check_crc16(struct subdrv_ctx *ctx, uint8_t  *data, uint32_t size, uint32_t ref_crc)
{
	int32_t crc_match = 0;
	uint16_t crc = 0x0000;
	uint16_t crc_reverse = 0x0000;
	uint32_t i, j;

	uint32_t tmp;
	uint32_t tmp_reverse;

	/* Calculate both methods of CRC since integrators differ on
	  * how CRC should be calculated. */
	for (i = 0; i < size; i++) {
		tmp_reverse = imx896_crc_reverse_byte(data[i]);
		tmp = data[i] & 0xff;
		for (j = 0; j < 8; j++) {
			if (((crc & 0x8000) >> 8) ^ (tmp & 0x80))
				crc = (crc << 1) ^ 0x8005;
			else
				crc = crc << 1;
			tmp <<= 1;

			if (((crc_reverse & 0x8000) >> 8) ^ (tmp_reverse & 0x80))
				crc_reverse = (crc_reverse << 1) ^ 0x8005;
			else
				crc_reverse = crc_reverse << 1;

			tmp_reverse <<= 1;
		}
	}

	crc_reverse = (imx896_crc_reverse_byte(crc_reverse) << 8) |
		imx896_crc_reverse_byte(crc_reverse >> 8);

	if (crc == ref_crc || crc_reverse == ref_crc)
		crc_match = 1;

	DRV_LOG(ctx, "ref_crc 0x%x, crc 0x%x, crc_reverse 0x%x, matches? %d\n",
		ref_crc, crc, crc_reverse, crc_match);

	return crc_match;
}

static int mot_imx896_read_spc_data(struct subdrv_ctx *ctx)
{
	int ret = 0;
	u16 ref_crc = 0;
	if (imx896_spc_data_ready) {
		DRV_LOG_MUST(ctx, "spc data is ready.");
		return 0;
	}

	ret = adaptor_i2c_rd_p8(ctx->i2c_client, (IMX896_SPC_EEPROM_ADDR >> 1), IMX896_SPC_DATA_START, imx896_spc_data, IMX896_SPC_DATA_LEN+2) ;
	if (ret < 0) {
		DRV_LOGE(ctx, "Read SPC data failed. ret:%d", ret);
		return -1;
	}

	ref_crc = ((imx896_spc_data[IMX896_SPC_DATA_LEN] << 8) |imx896_spc_data[IMX896_SPC_DATA_LEN+1]);
	if (imx896_check_crc16(ctx, imx896_spc_data, IMX896_SPC_DATA_LEN, ref_crc)) {
		imx896_spc_data_ready = 1;
		DRV_LOG(ctx, "SPC data ready now.");
	} else {
		/*When CRC error, each time camera open will try to read SPC data from EEPROM, maybe retry for several time is better. Currently
		  we don't avoid retry each time camera open to try best to eliminate potential randomly read failure.*/
		DRV_LOGE(ctx, "SPC data CRC error!");
	}

	return 0;
}

static void mot_imx896_apply_spc_data(void *sensor_ctx)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)sensor_ctx;

	if (!imx896_spc_data_ready) {
		mot_imx896_read_spc_data(ctx);
	}

	if (imx896_spc_data_ready) {
		int ret =	adaptor_i2c_wr_seq_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX896_SPC_REG1_START, imx896_spc_data, IMX896_SPC_DATA_LEN/2);
		if (ret < 0) {
			DRV_LOGE(ctx, "Write Left SPC data failed. ret:%d", ret);
			return;
		}

		ret = adaptor_i2c_wr_seq_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX896_SPC_REG2_START,
		                        &imx896_spc_data[IMX896_SPC_DATA_LEN/2], IMX896_SPC_DATA_LEN/2);
		if (ret < 0) {
			DRV_LOGE(ctx, "Read Right SPC data failed. ret:%d", ret);
			return;
		}
#if IMX896_SPC_DEBUG
		{
			int i;
			//u8 spc_en_reg[2];

			ret = adaptor_i2c_rd_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX896_SPC_REG1_START, imx896_spc_readback, IMX896_SPC_DATA_LEN/2) ;
			if (ret < 0) {
				DRV_LOGE(ctx, "L SPC data readback failed. ret:%d", ret);
				return;
			} else {
				for (i=0; i<IMX896_SPC_DATA_LEN/2; i++) {
					if (imx896_spc_data[i] != imx896_spc_readback[i]) {
						DRV_LOGE(ctx, "SPC[%d] E(%02x) != R(%02x)", i, imx896_spc_data[i], imx896_spc_readback[i]);
						return;
					}
				}
				DRV_LOG_MUST(ctx, "L spc readback check pass.");
			}

			ret = adaptor_i2c_rd_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX896_SPC_REG2_START,
			                        &imx896_spc_readback[IMX896_SPC_DATA_LEN/2], IMX896_SPC_DATA_LEN/2) ;
			if (ret < 0) {
				DRV_LOGE(ctx, "R SPC data readback failed. ret:%d", ret);
				return;
			} else {
				for (i=IMX896_SPC_DATA_LEN/2; i<IMX896_SPC_DATA_LEN; i++) {
					if (imx896_spc_data[i] != imx896_spc_readback[i]) {
						DRV_LOGE(ctx, "SPC[%d] E(%02x) != R(%02x)", i, imx896_spc_data[i], imx896_spc_readback[i]);
						return;
					}
				}
				DRV_LOG_MUST(ctx, "R spc readback check pass.");
			}
/*
			ret = adaptor_i2c_rd_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), 0x7BC8, spc_en_reg, 2) ;
			if (ret < 0) {
				DRV_LOGE(ctx, "SPC en reg read failed. ret:%d", ret);
			}
			DRV_LOG(ctx, "SPC en reg:%02x, %02x", spc_en_reg[0], spc_en_reg[1]);
*/
#if IMX896_SPC_DUMP
			for (i=0; i<IMX896_SPC_DATA_LEN; i++) {
				DRV_LOG(ctx, "SPC[%d]\t%02x\t%02x", i, imx896_spc_data[i], imx896_spc_readback[i]);
			}
#endif
		}
#endif
		DRV_LOG_MUST(ctx, "SPC apply done.");
		return;
	}
	DRV_LOGE(ctx, "IMX896 SPC data is NOT ready");
	return;
}

static int mot_imx896_read_qsc_data(struct subdrv_ctx *ctx)
{
	int ret = 0;
	u16 ref_crc = 0;
	if (imx896_qsc_data_ready) {
		DRV_LOG_MUST(ctx, "qsc data is ready.");
		return 0;
	}

	ret = adaptor_i2c_rd_p8(ctx->i2c_client, (IMX896_QSC_EEPROM_ADDR >> 1), IMX896_QSC_DATA_START, imx896_qsc_data, IMX896_QSC_DATA_LEN+2) ;
	if (ret < 0) {
		DRV_LOGE(ctx, "Read QSC data failed. ret:%d", ret);
		return -1;
	}

	ref_crc = ((imx896_qsc_data[IMX896_QSC_DATA_LEN] << 8) |imx896_qsc_data[IMX896_QSC_DATA_LEN+1]);
	if (imx896_check_crc16(ctx, imx896_qsc_data, IMX896_QSC_DATA_LEN, ref_crc)) {
		imx896_qsc_data_ready = 1;
		DRV_LOG(ctx, "QSC data ready now.");
	} else {
		/*When CRC error, each time camera open will try to read QSC data from EEPROM, maybe retry for several time is better. Currently
		  we don't avoid retry each time camera open to try best to eliminate potential randomly read failure.*/
		DRV_LOGE(ctx, "QSC data CRC error!");
	}

	return 0;
}

static void mot_imx896_apply_qsc_data(void *sensor_ctx)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)sensor_ctx;

	if (!imx896_qsc_data_ready) {
		mot_imx896_read_qsc_data(ctx);
	}

	if (imx896_qsc_data_ready) {
		int ret =	adaptor_i2c_wr_seq_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX896_QSC_REG_START, imx896_qsc_data, IMX896_QSC_DATA_LEN);
		if (ret < 0) {
			DRV_LOGE(ctx, "Write Left QSC data failed. ret:%d", ret);
			return;
		}

#if IMX896_QSC_DEBUG
		{
			int i;
			u8 qsc_en_reg[1];

			ret = adaptor_i2c_rd_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), 0x3206, qsc_en_reg, 1) ;
			if (ret < 0) {
				DRV_LOGE(ctx, "QSC en reg read failed. ret:%d", ret);
			}
			DRV_LOG(ctx, "QSC en reg:%02x", qsc_en_reg[0]);

			ret = adaptor_i2c_rd_p8(ctx->i2c_client, (ctx->i2c_write_id>>1), IMX896_QSC_REG_START, imx896_qsc_readback, IMX896_QSC_DATA_LEN) ;
			if (ret < 0) {
				DRV_LOGE(ctx, "L QSC data readback failed. ret:%d", ret);
				return;
			} else {
				for (i=0; i<IMX896_QSC_DATA_LEN; i++) {
					if (imx896_qsc_data[i] != imx896_qsc_readback[i]) {
						DRV_LOGE(ctx, "QSC[%d] E(%02x) != R(%02x)", i, imx896_qsc_data[i], imx896_qsc_readback[i]);
						return;
					}
				}
				DRV_LOG_MUST(ctx, "L qsc readback check pass.");
			}

#if IMX896_QSC_DUMP
			for (i=0; i<IMX896_QSC_DATA_LEN; i++) {
				DRV_LOG(ctx, "QSC[%d]\t%02x\t%02x", i, imx896_qsc_data[i], imx896_qsc_readback[i]);
			}
#endif
		}
#endif
		DRV_LOG_MUST(ctx, "QSC apply done.");
		return;
	}
	DRV_LOGE(ctx, "IMX896 QSC data is NOT ready");
	return;
}

void mot_imx896_apply_qsc_spc_data(void *sensor_ctx)
{
	mot_imx896_apply_spc_data(sensor_ctx);
	mot_imx896_apply_qsc_data(sensor_ctx);
}
