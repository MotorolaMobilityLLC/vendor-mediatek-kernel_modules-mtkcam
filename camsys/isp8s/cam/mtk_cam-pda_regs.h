/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 MediaTek Inc.
 */

#ifndef _MTK_CAM_PDA_REGS_H
#define _MTK_CAM_PDA_REGS_H

/* baseaddr 0X3A660400 */

/* following added manually */
#define REG_PDA_PDA_DEBUG_SEL                 (0x288)
#define REG_PDA_PDA_DEBUG_DATA                (0x2a8)

#define REG_E_PDA_OTF_CFG_0                   (0x400)
#define REG_E_PDA_OTF_CFG_1                   (0x404)
#define REG_E_PDA_OTF_CFG_2                   (0x408)
#define REG_E_PDA_OTF_CFG_3                   (0x40c)
#define REG_E_PDA_OTF_CFG_4                   (0x410)
#define REG_E_PDA_OTF_CFG_5                   (0x414)
#define REG_E_PDA_OTF_CFG_6                   (0x418)

#define REG_E_PDA_OTF_CFG_14                  (0x438)
#define REG_E_PDA_OTF_CFG_15                  (0x43c)
#define REG_E_PDA_OTF_CFG_16                  (0x440)
#define REG_E_PDA_OTF_CFG_17                  (0x444)
#define REG_E_PDA_OTF_CFG_18                  (0x448)
#define REG_E_PDA_OTF_CFG_19                  (0x44c)
#define REG_E_PDA_OTF_CFG_20                  (0x450)
#define REG_E_PDA_OTF_CFG_21                  (0x454)
#define REG_E_PDA_OTF_CFG_22                  (0x458)

#define REG_E_PDA_OTF_PDAI_P1_BASE_ADDR       (0x5cc)
#define REG_E_PDA_OTF_PDATI_P1_BASE_ADDR      (0x5d0)
#define REG_E_PDA_OTF_PDAI_P2_BASE_ADDR       (0x5d4)
#define REG_E_PDA_OTF_PDATI_P2_BASE_ADDR      (0x5d8)

#define REG_E_PDA_OTF_PDAI_STRIDE             (0x5dc)
#define REG_E_PDA_OTF_PDAI_P1_CON0            (0x5E0)
#define REG_E_PDA_OTF_PDAI_P1_CON1            (0x5E4)
#define REG_E_PDA_OTF_PDAI_P1_CON2            (0x5E8)
#define REG_E_PDA_OTF_PDAI_P1_CON3            (0x5Ec)
#define REG_E_PDA_OTF_PDAI_P1_CON4            (0x5F0)
#define REG_E_PDA_OTF_PDATI_P1_CON0           (0x5F4)
#define REG_E_PDA_OTF_PDATI_P1_CON1           (0x5F8)
#define REG_E_PDA_OTF_PDATI_P1_CON2           (0x5Fc)
#define REG_E_PDA_OTF_PDATI_P1_CON3           (0x600)
#define REG_E_PDA_OTF_PDATI_P1_CON4           (0x604)
#define REG_E_PDA_OTF_PDAI_P2_CON0            (0x608)
#define REG_E_PDA_OTF_PDAI_P2_CON1            (0x60c)
#define REG_E_PDA_OTF_PDAI_P2_CON2            (0x610)
#define REG_E_PDA_OTF_PDAI_P2_CON3            (0x614)
#define REG_E_PDA_OTF_PDAI_P2_CON4            (0x618)
#define REG_E_PDA_OTF_PDATI_P2_CON0           (0x61c)
#define REG_E_PDA_OTF_PDATI_P2_CON1           (0x620)
#define REG_E_PDA_OTF_PDATI_P2_CON2           (0x624)
#define REG_E_PDA_OTF_PDATI_P2_CON3           (0x628)
#define REG_E_PDA_OTF_PDATI_P2_CON4           (0x62c)

#define REG_E_PDA_OTF_PDAO_P1_BASE_ADDR      (0x630)

#define REG_E_PDA_OTF_PDAO_P1_XSIZE           (0x634)
#define REG_E_PDA_OTF_PDAO_P1_CON0            (0x638)
#define REG_E_PDA_OTF_PDAO_P1_CON1            (0x63c)
#define REG_E_PDA_OTF_PDAO_P1_CON2            (0x640)
#define REG_E_PDA_OTF_PDAO_P1_CON3            (0x644)
#define REG_E_PDA_OTF_PDAO_P1_CON4            (0x648)
#define REG_E_PDA_OTF_PDA_DMA_EN              (0x64c)
#define REG_E_PDA_OTF_PDA_DMA_RST             (0x650)
#define REG_E_PDA_OTF_PDA_DMA_TOP             (0x654)
#define REG_E_PDA_OTF_PDA_SECURE              (0x658)
#define REG_E_PDA_OTF_PDA_DCM_DIS             (0x65c)
#define REG_E_PDA_OTF_PDAI_P1_ERR_STAT        (0x660)
#define REG_E_PDA_OTF_PDATI_P1_ERR_STAT       (0x664)
#define REG_E_PDA_OTF_PDAI_P2_ERR_STAT        (0x668)
#define REG_E_PDA_OTF_PDATI_P2_ERR_STAT       (0x66c)
#define REG_E_PDA_OTF_PDAO_P1_ERR_STAT        (0x670)
#define REG_E_PDA_OTF_PDA_ERR_STAT_EN         (0x674)

/* ERROR Status */
#define REG_E_PDA_OTF_PDA_ERR_STAT            (0x678)
#define PDA_OTF_PDA_DONE_ST                     BIT(0)
#define PDA_OTF_PDA_HANG_ST                     BIT(1)
#define PDA_OTF_PDA_ERR_STAT_RESERVED2          BIT(2)
#define PDA_OTF_PDA_ERR_STAT_RESERVED3          BIT(3)

#define REG_E_PDA_OTF_PDA_TOP_CTL             (0x67c)
#define REG_E_PDA_OTF_PDA_IRQ_TRIG            (0x680)

#define REG_E_PDA_OTF_PDAI_P1_BASE_ADDR_MSB   (0x684)
#define REG_E_PDA_OTF_PDATI_P1_BASE_ADDR_MSB  (0x688)
#define REG_E_PDA_OTF_PDAI_P2_BASE_ADDR_MSB   (0x68c)
#define REG_E_PDA_OTF_PDATI_P2_BASE_ADDR_MSB  (0x690)
#define REG_E_PDA_OTF_PDAO_P1_BASE_ADDR_MSB   (0x694)

#define REG_E_PDA_OTF_PDA_P1_AXSLC            (0x6a0)
#define REG_E_PDA_OTF_PDA_P2_AXSLC            (0x6a4)
#define REG_E_PDA_OTF_PDAO_P1_AXSLC           (0x6a8)

#define REG_E_PDA_OTF_PACK_MODE               (0x6ac)
#define REG_E_PDA_OTF_DILATION_CFG            (0x6b0)

#define REG_E_PDA_OTF_PDA_SECURE_1            (0x6b4)
#define REG_E_PDA_OTF_DCIF_CTL                (0x6b8)

#define REG_E_PDA_OTF_PDA_DCIF_DEBUG_DATA0    (0x6c8)
#define REG_E_PDA_OTF_PDA_DCIF_DEBUG_DATA1    (0x6cc)
#define REG_E_PDA_OTF_PDA_DCIF_DEBUG_DATA2    (0x6d0)
#define REG_E_PDA_OTF_PDA_DCIF_DEBUG_DATA3    (0x6d4)
#define REG_E_PDA_OTF_PDA_DCIF_DEBUG_DATA4    (0x6d8)
#define REG_E_PDA_OTF_PDA_DCIF_DEBUG_DATA5    (0x6dc)
#define REG_E_PDA_OTF_PDA_DCIF_DEBUG_DATA6    (0x6e0)
#define REG_E_PDA_OTF_PDA_DCIF_DEBUG_DATA7    (0x6e4)

#endif	/* _MTK_CAM_PDA_REGS_H */
