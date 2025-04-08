#ifndef __UBOOT__
#include <linux/device.h>
#include <linux/kernel.h>
#include <malloc.h>
#endif
#include <linux/mtd/spinand.h>							  
#include <stdio.h>


#define SPINAND_MFR_issi			0x9D

#define IS37SMW04G8B_STATUS_ECC_1_7_BITFLIPS  (1 << 4)
#define IS37SMW04G8B_STATUS_ECC_8_BITFLIPS    (3 << 4)

#define IS37SMW04G8B_REG_STATUS2           0xf0



#define AM_STATUS_ECC_BITMASK       (7 << 4)  // 0x70, covering ECCS2:ECCS0
#define AM_STATUS_ECC_NONE_DETECTED (0 << 4)  // 0x00
#define AM_STATUS_ECC_CORRECTED     (1 << 4)  // 0x10 (1-3 bit errors corrected)
#define AM_STATUS_ECC_UNCORRECTABLE (2 << 4)  // 0x20 (More than 8 bit errors)
#define AM_STATUS_ECC_REFRESH_REC   (3 << 4)  // 0x30 (4-6 bit errors, refresh recommended)
#define AM_STATUS_ECC_RESERVED_4    (4 << 4)  // 0x40 (Reserved)
#define AM_STATUS_ECC_REFRESH_MAND  (5 << 4)  // 0x50 (7-8 bit errors, refresh mandatory)
#define AM_STATUS_ECC_RESERVED_6    (6 << 4)  // 0x60 (Reserved)
#define AM_STATUS_ECC_INVALID       (7 << 4)  // 0x70 (Invalid state)

static SPINAND_OP_VARIANTS(read_cache_variants,
		SPINAND_PAGE_READ_FROM_CACHE_QUADIO_OP(0, 2, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X4_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_DUALIO_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X2_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(true, 0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(false, 0, 1, NULL, 0));

static SPINAND_OP_VARIANTS(write_cache_variants,
		SPINAND_PROG_LOAD_X4(true, 0, NULL, 0),
		SPINAND_PROG_LOAD(true, 0, NULL, 0));

static SPINAND_OP_VARIANTS(update_cache_variants,
		SPINAND_PROG_LOAD_X4(false, 0, NULL, 0),
		SPINAND_PROG_LOAD(false, 0, NULL, 0));

static int Issi_ooblayout_ecc(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *region)
{
	if (section)
		return -ERANGE;

	region->offset = mtd->oobsize / 2;
	region->length = mtd->oobsize / 2;

	return 0;
}

static int Issi_ooblayout_free(struct mtd_info *mtd, int section,
				   struct mtd_oob_region *region)
{
	if (section)
		return -ERANGE;

	/* Reserve 1 bytes for the BBM. */
	region->offset = 1;
	region->length = (mtd->oobsize / 2) - 1;

	return 0;
}

static const struct mtd_ooblayout_ops Issi_ooblayout = {
	.ecc = Issi_ooblayout_ecc,
	.rfree = Issi_ooblayout_free,
};

static int IS37SMW04G8B_ecc_get_status(struct spinand_device *spinand, uint8_t status)
{
        
	switch (status & AM_STATUS_ECC_BITMASK) {
        case AM_STATUS_ECC_NONE_DETECTED:
            return 0;
    
        case AM_STATUS_ECC_CORRECTED:
    
                return 3;
    
        default:
            return -2;;
        }
    
        return -1;
}

static const struct spinand_info issi_spinand_table[] = {
    SPINAND_INFO("IS37SMW04G8B", 0x35,
             NAND_MEMORG(1, 2048, 128, 64, 2048, 0, 2, 1),
             NAND_ECCREQ(8, 512),
		         SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
             0,
             SPINAND_ECCINFO(&Issi_ooblayout, IS37SMW04G8B_ecc_get_status)),
};

static int issi_spinand_detect(struct spinand_device *spinand)
{
    uint8_t *id = spinand->id.data;
    int ret;

    if (id[2] != SPINAND_MFR_issi)
        return 0;
	
    printf("ISSI: %d : %d/n", id[1], id[2]);
    ret = spinand_match_and_init(spinand, issi_spinand_table,
                    ARRAY_SIZE(issi_spinand_table),
                    &id[1]);
    
    if (ret)
        return ret;

    return 1;
};

static const struct spinand_manufacturer_ops issi_spinand_manuf_ops = {
    .detect = issi_spinand_detect,
};

const struct spinand_manufacturer issi_spinand_manufacturer = {
    .id = SPINAND_MFR_issi,
    .name = "issi",
    .ops = &issi_spinand_manuf_ops,
};

