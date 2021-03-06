#ifndef __CDL_NFC_H__
#define __CDL_NFC_H__

#define COMMAND_REG	0x00
/* The masks reflect the input data to the MAKE_COMMAND macro, rather than
 * the bits in the register itself. These macros are not intended to be
 * used by the user, who should use the MAKE_COMMAND et al macros. */
#define _CMD_SEQ_SHIFT		0
#define _INPUT_SEL_SHIFT	6
#define _DATA_SEL_SHIFT		7
#define _CMD_0_SHIFT		8
#define _CMD_1_3_SHIFT		16
#define _CMD_2_SHIFT		24

#define _CMD_SEQ_MASK		0x3f
#define _INPUT_SEL_MASK		1
#define _DATA_SEL_MASK		1
#define _CMD_MASK		0xff /* for all CMD_foo */

#define MAKE_COMMAND(CMD_SEQ, INPUT_SEL, DATA_SEL, CMD_0, CMD_1_3, CMD_2) \
    ((((CMD_SEQ)    & _CMD_SEQ_MASK)    << _CMD_SEQ_SHIFT)  | \
    (((INPUT_SEL)   & _INPUT_SEL_MASK)  << _INPUT_SEL_SHIFT) | \
    (((DATA_SEL)    & _DATA_SEL_MASK)   << _DATA_SEL_SHIFT) | \
    (((CMD_0)       & _CMD_MASK)	<< _CMD_0_SHIFT)    | \
    (((CMD_1_3)     & _CMD_MASK)	<< _CMD_1_3_SHIFT)  | \
    (((CMD_2)       & _CMD_MASK)	<< _CMD_2_SHIFT))

#define INPUT_SEL_SIU	0
#define INPUT_SEL_DMA	1
#define DATA_SEL_FIFO	0
#define DATA_SEL_DATA_REG	1

#define CONTROL_REG	0x04
#define CONTROL_IO_WIDTH_8	(0 << 12)
#define CONTROL_IO_WIDTH_16	(1 << 12)
#define CONTROL_BLOCK_SIZE_32	(0 << 6)
#define CONTROL_BLOCK_SIZE_64	(1 << 6)
#define CONTROL_BLOCK_SIZE_128	(2 << 6)
#define CONTROL_BLOCK_SIZE_256	(3 << 6)
#define CONTROL_BLOCK_SIZE(SIZE)	((ffs(SIZE) - 6) << 6)
#define CONTROL_ECC_EN		(1 << 5)
#define CONTROL_INT_EN		(1 << 4)
#define CONTROL_ECC_BLOCK_SIZE_256	(0 << 1)
#define CONTROL_ECC_BLOCK_SIZE_512	(1 << 1)
#define CONTROL_ECC_BLOCK_SIZE_1024	(2 << 1)
#define CONTROL_ECC_BLOCK_SIZE(SIZE)	((ffs(SIZE) - 9) << 1)

#define STATUS_REG	0x08
#define STATUS_MEM_ST(DEVICE)	(1 << (DEVICE))
#define STATUS_MEM0_ST		(1 << 0)
#define STATUS_MEM1_ST		(2 << 0)
#define STATUS_CTRL_STAT	(1 << 8)

#define STATUS_MASK_REG		0x0C
#define STATE_MASK_SHIFT	0
#define STATUS_MASK_STATE_MASK(MASK)	(((MASK) & 0xff) << STATE_MASK_SHIFT)
#define ERROR_MASK_SHIFT	8
#define STATUS_MASK_ERROR_MASK(MASK)	(((MASK) & 0xff) << ERROR_MASK_SHIFT)

#define INT_MASK_REG	0x10
#define INT_MASK_ECC_INT0_EN		(1 << 24)
#define INT_MASK_STAT_ERR_INT0_EN	(1 << 16)
#define INT_MASK_MEM0_RDY_INT_EN	(1 << 8)
#define INT_MASK_DMA_INT_EN	(1 << 3)
#define INT_MASK_DATA_REG_EN	(1 << 2)
#define INT_MASK_CMD_END_INT_EN	(1 << 1)

#define INT_STATUS_REG	0x14
#define INT_STATUS_ECC_INT0_FL	(1 << 24)
#define INT_STATUS_STAT_ERR_INT0_FL	(1 << 16)
#define INT_STATUS_MEM0_RDY_INT_FL	(1 << 8)
#define INT_STATUS_DMA_INT_FL		(1 << 3)
#define INT_STATUS_DATA_REG_FL		(1 << 2)
#define INT_STATUS_CMD_END_INT_FL	(1 << 1)
#define INT_STATUS_WAIT_FOR_WRITE	(0 << 0)

#define ECC_CTRL_REG	0x18
#define ECC_CTRL_ECC_CAP_2	(0 << 0)
#define ECC_CTRL_ECC_CAP_4	(1 << 0)
#define ECC_CTRL_ECC_CAP_8	(2 << 0)
#define ECC_CTRL_ECC_CAP_16	(3 << 0)
#define ECC_CTRL_ECC_CAP_24	(4 << 0)
#define ECC_CTRL_ECC_CAP_32	(5 << 0)
#define ECC_CTRL_ECC_CAP(B)	((B) < 24 ? ffs(B) - 2 : (B) / 6)
/* # ECC corrections that are acceptable during read before setting OVER flag */
#define ECC_CTRL_ECC_THRESHOLD(VAL)	(((VAL) & 0x3f) << 8)

#define ECC_OFFSET_REG	0x1C

#define ECC_STAT_REG	0x20
/* Correctable error flag(s) */
#define ECC_STAT_ERROR_0	(1 << 0)
/* Uncorrectable error flag(s) */
#define ECC_STAT_UNC_0	(1 << 8)
/* Acceptable errors level overflow flag(s) */
#define ECC_STAT_OVER_0	(1 << 16)

#define ADDR0_COL_REG	0x24
#define ADDR0_ROW_REG	0x28
#define ADDR1_COL_REG	0x2C
#define ADDR1_ROW_REG	0x30
#define PROTECT_REG	0x34
#define FIFO_DATA_REG	0x38
#define DATA_REG_REG	0x3C
#define DATA_REG_SIZE_REG	0x40
#define DATA_REG_SIZE_DATA_REG_SIZE(SIZE)	(((SIZE) - 1) & 3)

#define DEV0_PTR_REG	0x44
#define DEV1_PTR_REG	0x48
#define DEV2_PTR_REG	0x4C
#define DEV3_PTR_REG	0x50
#define DEV4_PTR_REG	0x54
#define DEV5_PTR_REG	0x58
#define DEV6_PTR_REG	0x5C
#define DEV7_PTR_REG	0x60
#define DMA_ADDR_L_REG	0x64
#define DMA_ADDR_H_REG	0x68
#define DMA_CNT_REG	0x6C

#define DMA_CTRL_REG	0x70
#define DMA_CTRL_DMA_START	(1 << 7) /* start on command */
#define DMA_CTRL_DMA_MODE_SG	(1 << 5) /* scatter/gather mode */
#define DMA_CTRL_DMA_BURST_I_P_4	(0 << 2) /* incr. precise burst */
#define DMA_CTRL_DMA_BURST_S_P_16	(1 << 2) /* stream precise burst */
#define DMA_CTRL_DMA_BURST_SINGLE	(2 << 2) /* single transfer */
#define DMA_CTRL_DMA_BURST_UNSPEC	(3 << 2) /* burst of unspec. length */
#define DMA_CTRL_DMA_BURST_I_P_8	(4 << 2) /* incr. precise burst */
#define DMA_CTRL_DMA_BURST_I_P_16	(5 << 2) /* incr. precise burst */
#define DMA_CTRL_ERR_FLAG	(1 << 1) /* read only */
#define DMA_CTRL_DMA_READY	(1 << 0) /* read only */

#define BBM_CTRL_REG	0x74
#define MEM_CTRL_REG	0x80
#define MEM_CTRL_MEM_CE(CE)	(((CE) & 7) << 0)
#define MEM_CTRL_BANK_SEL(BANK)	(1<<(8+BANK))//(((BANK) & 7) << 16)

#define DATA_SIZE_REG	0x84
#define TIMINGS_ASYN_REG	0x88
#define TIMINGS_SYN_REG	0x8C
#define TIME_SEQ_0_REG	0x90
#define TIME_SEQ_1_REG	0x94
#define TIME_GEN_SEQ_0_REG	0x98
#define TIME_GEN_SEQ_1_REG	0x9C
#define TIME_GEN_SEQ_2_REG	0xA0

#define FIFO_INIT_REG		0xB0
#define FIFO_INIT_FIFO_INIT	1 /* Flush FIFO */
#define FIFO_STATE_REG		0xB4
#define FIFO_STATE_DF_W_EMPTY	(1 << 7)
#define FIFO_STATE_DF_R_FULL	(1 << 6)
#define FIFO_STATE_CF_ACCPT_W	(1 << 5)
#define FIFO_STATE_CF_ACCPT_R	(1 << 4)
#define FIFO_STATE_CF_FULL	(1 << 3)
#define FIFO_STATE_CF_EMPTY	(1 << 2)
#define FIFO_STATE_DF_W_FULL	(1 << 1)
#define FIFO_STATE_DF_R_EMPTY	(1 << 0)

#define GEN_SEQ_CTRL_REG	0xB8    /* aka GENERIC_SEQ_CTRL */
#define _CMD0_EN_SHIFT	0
#define _CMD1_EN_SHIFT	1
#define _CMD2_EN_SHIFT	2
#define _CMD3_EN_SHIFT	3
#define _COL_A0_SHIFT	4
#define _COL_A1_SHIFT	6
#define _ROW_A0_SHIFT	8
#define _ROW_A1_SHIFT	10
#define _DATA_EN_SHIFT	12
#define _DELAY_EN_SHIFT	13
#define _IMD_SEQ_SHIFT	15
#define _CMD3_SHIFT	16

#define _CMD0_EN_MASK	1
#define _CMD1_EN_MASK	1
#define _CMD2_EN_MASK	1
#define _CMD3_EN_MASK	1
#define _COL_A0_MASK	3
#define _COL_A1_MASK	3
#define _ROW_A0_MASK	3
#define _ROW_A1_MASK	3
#define _DATA_EN_MASK	1
#define _DELAY_EN_MASK	3
#define _IMD_SEQ_MASK	1
#define _CMD3_MASK	0xff

/* DELAY_EN field values, non-shifted */
#define _BUSY_NONE	0
#define _BUSY_0		1
#define _BUSY_1		2

/* Slightly confusingly, the DELAYx_EN fields enable BUSY phases. */
#define MAKE_GEN_CMD(CMD0_EN, CMD1_EN, CMD2_EN, CMD3_EN, \
	      COL_A0, ROW_A0, COL_A1, ROW_A1, \
	      DATA_EN, BUSY_EN, IMMEDIATE_SEQ, CMD3) \
     ((((CMD0_EN)    & _CMD0_EN_MASK)    << _CMD0_EN_SHIFT)  | \
      (((CMD1_EN)    & _CMD1_EN_MASK)    << _CMD1_EN_SHIFT)  | \
      (((CMD2_EN)    & _CMD2_EN_MASK)    << _CMD2_EN_SHIFT)  | \
      (((CMD3_EN)    & _CMD3_EN_MASK)    << _CMD3_EN_SHIFT)  | \
      (((COL_A0) & _COL_A0_MASK)     << _COL_A0_SHIFT)   | \
      (((COL_A1) & _COL_A1_MASK)     << _COL_A1_SHIFT)   | \
      (((ROW_A0) & _ROW_A0_MASK)     << _ROW_A0_SHIFT)   | \
      (((ROW_A1) & _ROW_A1_MASK)     << _ROW_A1_SHIFT)   | \
      (((DATA_EN)    & _DATA_EN_MASK)    << _DATA_EN_SHIFT)  | \
      (((BUSY_EN)    & _DELAY_EN_MASK)   << _DELAY_EN_SHIFT) | \
      (((IMMEDIATE_SEQ) & _IMD_SEQ_MASK) << _IMD_SEQ_SHIFT)  | \
      (((CMD3)   & _CMD3_MASK)       << _CMD3_SHIFT))

/* The sequence encodings are not trivial. The ones we use are listed here. */
#define _SEQ_0	0x00 /* send one cmd, then wait for ready */
#define _SEQ_1	0x21 /* send one cmd, one addr, fetch data */
#define _SEQ_4	0x24 /* single cycle write then read */
#define _SEQ_10	0x2A /* read page */
#define _SEQ_12	0x0C /* write page, don't wait for R/B */
#define _SEQ_18	0x32 /* read page using general cycle */
#define _SEQ_19	0x13 /* write page using general cycle */
#define _SEQ_14	0x0E /* 3 address cycles, for block erase */

#define MLUN_REG	0xBC
#define DEV0_SIZE_REG	0xC0
#define DEV1_SIZE_REG	0xC4
#define DEV2_SIZE_REG	0xC8
#define DEV3_SIZE_REG	0xCC
#define DEV4_SIZE_REG	0xD0
#define DEV5_SIZE_REG	0xD4
#define DEV6_SIZE_REG	0xD8
#define DEV7_SIZE_REG	0xDC
#define SS_CCNT0_REG	0xE0
#define SS_CCNT1_REG	0xE4
#define SS_SCNT_REG	0xE8
#define SS_ADDR_DEV_CTRL_REG	0xEC
#define SS_CMD0_REG	0xF0
#define SS_CMD1_REG	0xF4
#define SS_CMD2_REG	0xF8
#define SS_CMD3_REG	0xFC
#define SS_ADDR_REG	0x100
#define SS_MSEL_REG	0x104
#define SS_REQ_REG	0x108
#define SS_BRK_REG	0x10C
#define DMA_TLVL_REG	0x114
#define DMA_TLVL_MAX	0xFF
#define AES_CTRL_REG	0x118
#define AES_DATAW_REG	0x11C
#define AES_SVECT_REG	0x120
#define CMD_MARK_REG	0x124
#define LUN_STATUS_0_REG	0x128
#define LUN_STATUS_1_REG	0x12C
#define TIMINGS_TOGGLE_REG	0x130
#define TIME_GEN_SEQ_3_REG	0x134
#define SQS_DELAY_REG	0x138
#define CNE_MASK_REG	0x13C
#define CNE_VAL_REG	0x140
#define CNA_CTRL_REG	0x144
#define INTERNAL_STATUS_REG	0x148
#define ECC_CNT_REG	0x14C
#define ECC_CNT_ERR_LVL_MASK	0x3F
#define PARAM_REG_REG	0x150

/* NAND flash command generation */

/* NAND flash command codes */
#define NAND_RESET		0xff
#define NAND_READ_STATUS	0x70
#define NAND_READ_ID		0x90
#define NAND_READ_ID_ADDR_STD	0x00  /* address written to ADDR0_COL */
#define NAND_READ_ID_ADDR_ONFI	0x20  /* address written to ADDR0_COL */
#define NAND_PAGE_READ		0x00
#define NAND_PAGE_READ_END	0x30
#define NAND_BLOCK_ERASE	0x60
#define NAND_BLOCK_ERASE_END	0xd0
#define NAND_PAGE_WRITE		0x80
#define NAND_PAGE_WRITE_END	0x10

#define _DONT_CARE 0x00 /* When we don't have anything better to say */

/* Assembled values for putting into COMMAND register */

/* Reset NAND flash */

/* Uses SEQ_0: non-directional sequence, single command, wait for ready */
#define COMMAND_RESET \
	MAKE_COMMAND(_SEQ_0, INPUT_SEL_SIU, DATA_SEL_FIFO, \
		NAND_RESET, _DONT_CARE, _DONT_CARE)

/* Read status */

/* Uses SEQ_4: single command, then read data via DATA_REG */
#define COMMAND_READ_STATUS \
	MAKE_COMMAND(_SEQ_4, INPUT_SEL_SIU, DATA_SEL_DATA_REG, \
		NAND_READ_STATUS, _DONT_CARE, _DONT_CARE)

/* Read ID */

/* Uses SEQ_1: single command, ADDR0_COL, then read data via FIFO */
/* ADDR0_COL is set to NAND_READ_ID_ADDR_STD for non-ONFi, and
 * NAND_READ_ID_ADDR_ONFI for ONFi.
 * The controller reads 5 bytes in the non-ONFi case, and 4 bytes in the
 * ONFi case, so the data reception (DMA or FIFO_REG) needs to be set up
 * accordingly. */
#define COMMAND_READ_ID \
	MAKE_COMMAND(_SEQ_1, INPUT_SEL_DMA, DATA_SEL_FIFO, \
		NAND_READ_ID, _DONT_CARE, _DONT_CARE)

#define COMMAND_READ_ID_SIU_FIFO \
	MAKE_COMMAND(_SEQ_1, INPUT_SEL_SIU, DATA_SEL_FIFO, \
		NAND_READ_ID, _DONT_CARE, _DONT_CARE)

#define COMMAND_READ_ID_SIU_DATA \
	MAKE_COMMAND(_SEQ_1, INPUT_SEL_SIU, DATA_SEL_DATA_REG, \
		NAND_READ_ID, _DONT_CARE, _DONT_CARE)

/* Page read via slave interface (FIFO_DATA register) */

/* Standard 5-cycle read command, with 0x30 end-of-cycle marker */
/* Uses SEQ_10: CMD0 + 5 address cycles + CMD2, read data */
#define COMMAND_READ_PAGE_STD \
	MAKE_COMMAND(_SEQ_10, INPUT_SEL_SIU, DATA_SEL_FIFO, \
		NAND_PAGE_READ, _DONT_CARE, NAND_PAGE_READ_END)

/* 4-cycle read command, together with GEN_SEQ_CTRL_READ_PAGE_4CYCLE */
/* Uses SEQ_18 (generic command sequence, see GEN_SEQ_ECTRL_READ_PAGE_4CYCLE)):
   CMD0 + 2+2 address cycles + CMD2, read data */
#define COMMAND_READ_PAGE_GEN \
	MAKE_COMMAND(_SEQ_10, INPUT_SEL_SIU, DATA_SEL_FIFO, \
		NAND_PAGE_READ, _DONT_CARE, NAND_PAGE_READ_END)

/* Page read via master interface (DMA) */

/* Standard 5-cycle read command, with 0x30 end-of-cycle marker */
/* Uses SEQ_10: CMD0 + 5 address cycles + CMD2, read data */
#define COMMAND_READ_PAGE_DMA_STD \
	MAKE_COMMAND(_SEQ_10, INPUT_SEL_DMA, DATA_SEL_FIFO, \
		NAND_PAGE_READ, _DONT_CARE, NAND_PAGE_READ_END)

/* 4-cycle read command, together with GEN_SEQ_CTRL_READ_PAGE_4CYCLE */
/* Uses SEQ_18 (generic command sequence, see GEN_SEQ_ECTRL_READ_PAGE_4CYCLE)):
   CMD0 + 2+2 address cycles + CMD2, read data */
#define COMMAND_READ_PAGE_DMA_GEN \
	MAKE_COMMAND(_SEQ_18, INPUT_SEL_DMA, DATA_SEL_FIFO, \
		NAND_PAGE_READ, _DONT_CARE, NAND_PAGE_READ_END)

/* Page write via master interface (DMA) */

/* Uses SEQ_12: CMD0 + 5 address cycles + write data + CMD1 + DMA FIFO */
#define COMMAND_WRITE_PAGE_DMA_STD \
	MAKE_COMMAND(_SEQ_12, INPUT_SEL_DMA, DATA_SEL_FIFO, \
		NAND_PAGE_WRITE, NAND_PAGE_WRITE_END, _DONT_CARE)

/* Uses SEQ_19: CMD0 + 4 address cycles + write data + CMD1 */
#define COMMAND_WRITE_PAGE_DMA_GEN \
	MAKE_COMMAND(_SEQ_19, INPUT_SEL_DMA, DATA_SEL_FIFO, \
		NAND_PAGE_WRITE, NAND_PAGE_WRITE_END, _DONT_CARE)

/* Uses SEQ_12: CMD0 + 5 address cycles + write data + CMD1 + SIU FIFO */
#define COMMAND_WRITE_PAGE_FIFO_GEN \
	MAKE_COMMAND(_SEQ_12, INPUT_SEL_SIU, DATA_SEL_FIFO, \
		NAND_PAGE_WRITE, NAND_PAGE_WRITE_END, _DONT_CARE)

/* Block erase */

/* Uses SEQ_14: CMD0 + 3 address cycles + CMD1 */
#define COMMAND_BLOCK_ERASE \
	MAKE_COMMAND(_SEQ_14, INPUT_SEL_SIU, DATA_SEL_FIFO, \
		NAND_BLOCK_ERASE, NAND_BLOCK_ERASE_END, _DONT_CARE)

/* Assembled values for putting into GEN_SEQ_CTRL register */

/* General command sequence specification for 4 cycle PAGE_READ command */
#define GEN_SEQ_CTRL_READ_PAGE_4CYCLE \
	MAKE_GEN_CMD(1, 0, 1, 0,   /* enable command 0 and 2 phases */ \
		2, 2, /* col A0 2 cycles, row A0 2 cycles */ \
		0, 0, /* col A1, row A1 not used */ \
		1,	/* data phase enabled */ \
		_BUSY_0,/* busy0 phase enabled */ \
		0,	/* immediate cmd execution disabled */ \
		_DONT_CARE)/* command 3 code not needed */

/* General command sequence specification for 5 cycle PAGE_READ command */
#define GEN_SEQ_CTRL_READ_PAGE_5CYCLE \
	MAKE_GEN_CMD(1, 0, 1, 0, /* enable command 0 and 2 phases */ \
		2, 3, 	 /* col A0 2 cycles, row A0 3 cycles */ \
		0, 0, 	 /* col A1, row A1 not used */ \
		1,	 /* data phase enabled */ \
		_BUSY_0, /* busy0 phase enabled */ \
		0,	 /* immediate cmd execution disabled */ \
		_DONT_CARE) /* command 3 code not needed */

/* General command sequence specification for 4 cycle PAGE_PROGRAM command */
#define GEN_SEQ_CTRL_WRITE_PAGE_4CYCLE \
	MAKE_GEN_CMD(1, 1, 0, 0,    /* enable command 0 and 1 phases */ \
		2, 2,      /* col A0 2 cycles, row A0 2 cycles */ \
		0, 0,      /* col A1, row A1 not used */ \
		1,	 /* data phase enabled */ \
		_BUSY_1, /* busy1 phase enabled */ \
		0,	 /* immediate cmd execution disabled */ \
		_DONT_CARE)/* command 3 code not needed */

/* General command sequence specification for 5 cycle PAGE_PROGRAM command */
#define GEN_SEQ_CTRL_WRITE_PAGE_5CYCLE \
	MAKE_GEN_CMD(1, 1, 0, 0,    /* enable command 0 and 1 phases */ \
		2, 3,      /* col A0 2 cycles, row A0 3 cycles */ \
		0, 0,      /* col A1, row A1 not used */ \
		1,	 /* data phase enabled */ \
		_BUSY_1,       /* busy1 phase enabled */ \
		0,	 /* immediate cmd execution disabled */ \
		_DONT_CARE)    /* command 3 code not needed */

/* BCH ECC size calculations. Should really go somewhere else? */
/* From "Mr. NAND's Wild Ride: Warning: Suprises Ahead", by Robert Pierce,
 * Denali Software Inc. 2009, table on page 5 */
/* Use 8 bit correction as base. */
#define ECC8_BYTES(BLKSIZE)	(ffs(BLKSIZE) + 3)
/* The following is valid for 4..24 bits of correction. */
#define ECC_BYTES(CAP, BLKSIZE)	((ECC8_BYTES(BLKSIZE) * (CAP) + 7) / 8)

/*
 * Standard NAND flash commands
 */
#define NAND_CMD_READ0		0
#define NAND_CMD_READ1		1
#define NAND_CMD_RNDOUT		5
#define NAND_CMD_PAGEPROG	0x10
#define NAND_CMD_READOOB	0x50
#define NAND_CMD_ERASE1		0x60
#define NAND_CMD_STATUS		0x70
#define NAND_CMD_SEQIN		0x80
#define NAND_CMD_RNDIN		0x85
#define NAND_CMD_READID		0x90
#define NAND_CMD_ERASE2		0xd0
#define NAND_CMD_PARAM		0xec
#define NAND_CMD_GET_FEATURES	0xee
#define NAND_CMD_SET_FEATURES	0xef
#define NAND_CMD_RESET		0xff

#define NAND_CMD_LOCK		0x2a
#define NAND_CMD_UNLOCK1	0x23
#define NAND_CMD_UNLOCK2	0x24

/* Extended commands for large page devices */
#define NAND_CMD_READSTART	0x30
#define NAND_CMD_RNDOUTSTART	0xE0
#define NAND_CMD_CACHEDPROG	0x15

#define NAND_CMD_NONE   -1

/* Read modes */
enum etx_read_mode {
	ETX_READ_STD,
	/* Standard page read with ECC */
	ETX_READ_RAW,
	/* Raw mode read of main area without ECC */
	ETX_READ_OOB,
	/* Read oob only (no ECC) */
	ETX_READ_ALL  /* Read main oob in raw mode (no ECC) */
};

/* Timing parameters, from dt */
struct etx_timings {
	unsigned long time_seq_0;
	unsigned long time_seq_1;
	unsigned long timings_asyn;
	unsigned long time_gen_seq_0;
	unsigned long time_gen_seq_1;
	unsigned long time_gen_seq_2;
	unsigned long time_gen_seq_3;
};

#define	NFC_ECC_NONE	0
#define	NFC_ECC_SOFT	1
#define NFC_ECC_HW	2
#define NFC_ECC_HW_SYNDROME	3
#define	NFC_ECC_HW_OOB_FIRST	4
#define NFC_ECC_SOFT_BCH	5

/* Configuration, from dt */
struct etx_setup {
	int ecc_mode;
	int ecc_blksize;
	int ecc_strength;
	int on_flash_bbt;
	struct etx_timings timings;
};

/* DMA buffer, from both software (buf) and hardware (phys) perspective. */
struct etx_dma {
	unsigned char* buf; /* mapped address */
	unsigned long phys; /* physical address */
	int bytes_left; /* how much data left to read from buffer? */
	unsigned char* ptr; /* work pointer */
};

/* Per-chip controller configuration */
struct etx_config {
	unsigned long mem_ctrl;
	unsigned long control;
	unsigned long ecc_ctrl;
	unsigned long ecc_offset;
	unsigned long mem_status_mask;
};

/* Cache for info that we need to save across calls to etx_nand_command */
struct etx_cmd_cache {
	unsigned int command;
	int page;
	int column;
	int oob_required;
	int write_raw;
};

typedef enum {
	NFC_INTERNAL_DMA = 0,
	NFC_SIU_FIFO
}etx_mode;

struct nfc_ecc_stats {
	unsigned long corrected;
	unsigned long failed;
	unsigned long badblocks;
	unsigned long bbtblocks;
};
struct nfc_mtd_info {
	unsigned long writesize;
	unsigned long oobsize;
	struct nfc_ecc_stats ecc_stats;
};
struct nfc_nand_chip {
	unsigned long long chipsize;
	unsigned char *oob_poi;
	unsigned short ecc_step_ds;
	unsigned int options;
};
struct candence_nfc {
	void* regs;
	unsigned long pclk;
	etx_mode use_mode;
	int cs;

	struct etx_setup setup;
	struct etx_config config;
	struct etx_dma dma;

	struct nfc_mtd_info mtd;
	struct nfc_nand_chip chip;
	struct etx_cmd_cache cmd_cache;

};


#define DMA_BUF_SIZE (8192 + 640) /* main   spare for 8k page flash */

struct nfc_timing {
	unsigned long twhr;
	unsigned long trhw;
	unsigned long tadl;
	unsigned long tccs;
	unsigned long trr;
	unsigned long twb;
	unsigned long t0_d0;
	unsigned long t0_d1;
	unsigned long t0_d2;
	unsigned long t0_d3;
	unsigned long t0_d4;
	unsigned long t0_d5;
	unsigned long t0_d6;
	unsigned long t0_d7;
	unsigned long t0_d8;
	unsigned long t0_d9;
	unsigned long t0_d10;
	unsigned long t0_d11;
	unsigned long t0_d12;
	unsigned long trwh;
	unsigned long trwp;
} ;


#define SP_OPTIONS	NAND_NEED_READRDY
#define SP_OPTIONS16	 NAND_BUSWIDTH_16



/* Driver configuration */

#define ETX_CE_BANK_SEL /* Separate chips connected as separate banks */
#define ETX_RB_WIRED_AND /* Use wired-AND for ready/busy from flash chip */
/* Workarounds for development environment */

#define WORKAROUND_NO_ECC_CNT

/* # bytes into the OOB we put our ECC */
#define ECC_OFFSET  0// 2

#define NAND_OWN_BUFFERS	0x00020000
/* Chip does not allow subpage writes */
#define NAND_NO_SUBPAGE_WRITE	0x00000200
/* Chip has cache program function */
#define NAND_CACHEPRG		0x00000008

/* Status bits */
#define NAND_STATUS_FAIL	0x01
#define NAND_STATUS_FAIL_N1	0x02
#define NAND_STATUS_TRUE_READY	0x20
#define NAND_STATUS_READY	0x40
#define NAND_STATUS_WP		0x80

/* Device supports subpage reads */
#define NAND_SUBPAGE_READ	0x00001000

#define MTD_NORFLASH		3
#define MTD_NANDFLASH		4	/* SLC NAND */
#define MTD_DATAFLASH		6
#define MTD_UBIVOLUME		7
#define MTD_MLCNANDFLASH	8	/* MLC NAND (including TLC) */

/* Device behaves just like nand, but is readonly */
#define NAND_ROM		0x00000800
#define MTD_CAP_ROM		0
#define MTD_WRITEABLE		0x400	/* Device is writeable */
#define MTD_CAP_NANDFLASH	(MTD_WRITEABLE)

/* Non chip related options */
/* This option skips the bbt scan during initialization. */
#define NAND_SKIP_BBTSCAN	0x00010000

#define NAND_HAS_CACHEPROG(chip) ((chip->options & NAND_CACHEPRG))
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#define MAX_CMD_LOOPS 100000


/*apis*/

void etx_update_timing(struct etx_setup *etx_setup,
	struct nfc_timing *time);
void etx_config(struct etx_config* etx_config, void* ref);

int etx_dev_ready(struct candence_nfc* priv);
void etx_nand_command(struct candence_nfc* priv, unsigned int command,
	int column, int page_addr);
unsigned char etx_read_byte(struct candence_nfc* priv);
unsigned short etx_read_word(struct candence_nfc* priv);
void etx_read_buf(struct candence_nfc* priv, unsigned char* buf, int len);

void etx_write_dmabuf(struct candence_nfc* priv,
	const unsigned char* buf, int len);

int etx_read_page_raw(struct candence_nfc* priv, unsigned char* buf,
	int oob_required,int page);
int etx_read_page_hwecc(struct candence_nfc* priv, unsigned char* buf,
	int oob_required, int page);
int etx_write_page_raw(struct candence_nfc* priv, const unsigned char* buf,
	int oob_required);
int etx_write_page_hwecc(struct candence_nfc* priv,
	const unsigned char* buf,int oob_required);

void etx_enable_int(int en);
unsigned long etx_get_int_status(void);
void etx_set_int_mask(unsigned long val);
void etx_clear_int(unsigned long val);

#endif
