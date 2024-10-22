#include "mot_s5kjns_cali.h"

static u8 s5kjns_hw_ggc_data[S5KJNS_HW_GGC_DATA_LEN+2];
static u16 hw_ggc_data[S5KJNS_HW_GGC_DATA_LEN/2];
static u8 s5kjns_hw_ggc_data_ready = 0;
#if S5KJNS_HW_GGC_DEBUG
#ifdef DRV_LOG
#undef DRV_LOG
#define DRV_LOG DRV_LOG_MUST
#endif
#endif

static int s5kjns_crc_reverse_byte(int data)
{
	return ((data * 0x0802LU & 0x22110LU) |
		(data * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
}

static int32_t s5kjns_check_crc16(struct subdrv_ctx *ctx, uint8_t  *data, uint32_t size, uint32_t ref_crc)
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
		tmp_reverse = s5kjns_crc_reverse_byte(data[i]);
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

	crc_reverse = (s5kjns_crc_reverse_byte(crc_reverse) << 8) |
		s5kjns_crc_reverse_byte(crc_reverse >> 8);

	if (crc == ref_crc || crc_reverse == ref_crc)
		crc_match = 1;

	DRV_LOG(ctx, "ref_crc 0x%x, crc 0x%x, crc_reverse 0x%x, matches? %d\n",
		ref_crc, crc, crc_reverse, crc_match);

	return crc_match;
}

static int mot_s5kjns_read_hw_ggc_data(struct subdrv_ctx *ctx)
{
	int ret = 0;
	u16 ref_crc = 0;
	if (s5kjns_hw_ggc_data_ready) {
		DRV_LOG_MUST(ctx, "hw_ggc data is ready.");
		return 0;
	}

	ret = adaptor_i2c_rd_p8(ctx->i2c_client, (S5KJNS_HW_GGC_EEPROM_ADDR >> 1), S5KJNS_HW_GGC_DATA_START, s5kjns_hw_ggc_data, S5KJNS_HW_GGC_DATA_LEN+2) ;
	if (ret < 0) {
		DRV_LOGE(ctx, "Read HW_GGC data failed. ret:%d", ret);
		return -1;
	}

	ref_crc = ((s5kjns_hw_ggc_data[S5KJNS_HW_GGC_DATA_LEN] << 8) |s5kjns_hw_ggc_data[S5KJNS_HW_GGC_DATA_LEN+1]);
	if (s5kjns_check_crc16(ctx, s5kjns_hw_ggc_data, S5KJNS_HW_GGC_DATA_LEN, ref_crc)) {
		s5kjns_hw_ggc_data_ready = 1;
		DRV_LOG(ctx, "HW_GGC data ready now.");
	} else {
		/*When CRC error, each time camera open will try to read HW_GGC data from EEPROM, maybe retry for several time is better. Currently
		  we don't avoid retry each time camera open to try best to eliminate potential randomly read failure.*/
		DRV_LOGE(ctx, "HW_GGC data CRC error!");
	}

	return 0;
}

static void mot_s5kjns_apply_hw_ggc_data(void *sensor_ctx)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)sensor_ctx;

	if (!s5kjns_hw_ggc_data_ready) {
		mot_s5kjns_read_hw_ggc_data(ctx);
	}

	if (s5kjns_hw_ggc_data_ready) {
		int k =0;
		int ret = adaptor_i2c_wr_u16(ctx->i2c_client, (ctx->i2c_write_id>>1),0x6028,0x2400);
		ret = adaptor_i2c_wr_u16(ctx->i2c_client, (ctx->i2c_write_id>>1),0x602A,0x0CFC);
 		for (k = 0; k < S5KJNS_HW_GGC_DATA_LEN; k += 2)
 		{
 			hw_ggc_data[k/2] = (s5kjns_hw_ggc_data[k] << 8) + s5kjns_hw_ggc_data[k + 1];
#if S5KJNS_HW_GGC_DEBUG
			DRV_LOG_MUST(ctx, "hw_ggc hw_ggc_data[%d]data is 0x%4x.",k/2,hw_ggc_data[k/2]);
#endif
			ret = adaptor_i2c_wr_u16(ctx->i2c_client, (ctx->i2c_write_id>>1),S5KJNS_HW_GGC_REG_START,hw_ggc_data[k/2]);
			if (ret < 0) {
				DRV_LOGE(ctx, "Write Left HW_GGC data failed. ret:%d", ret);
			return;
			}
 		}

		DRV_LOG_MUST(ctx, "HW_GGC apply done.");
		return;
	}
	else
		DRV_LOGE(ctx, "S5KJNS HW_GGC data is NOT ready");
	return;
}

void mot_s5kjns_apply_xtc_data(void *sensor_ctx)
{
	mot_s5kjns_apply_hw_ggc_data(sensor_ctx);
}