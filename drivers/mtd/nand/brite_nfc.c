


#include <common.h>
#include <asm/io.h>
#include <memalign.h>
#include <nand.h>
//#include <asm/arch/clock.h>
//#include <asm/arch/funcmux.h>
//#include <asm/arch-tegra/clk_rst.h>
#include <asm/errno.h>
//#include <asm/gpio.h>
#include <fdtdec.h>
#include <bouncebuf.h>
#include "brite_nfc.h"

DECLARE_GLOBAL_DATA_PTR;


#define NFC_DEBUG 1

#ifdef NFC_DEBUG
#define nfc_debug	debug
#define info	debug
#else
#define nfc_debug(...)
#endif

struct candence_nfc nfc_dev;
struct candence_nfc* p_nfc = &nfc_dev;

static u32 etx_dma_buff[DMA_BUF_SIZE];

const struct etx_timings default_mode0_pll_enabled = {
	0x0d151533,
	0x000b0515,
	0x00000046,
	0x00150000,
	0x00000000,
	0x00000005,
	0x00000015
};

static inline uint32_t etx_read(u32 reg_offset)
{
	return __raw_readl((u32)p_nfc->regs + reg_offset);
}

static inline void etx_write(uint32_t data, uint reg_offset)
{
	__raw_writel(data, (u32)p_nfc->regs + reg_offset);
}

/* Write per-chip specific config to controller */
void etx_config(struct etx_config* etx_config, void* ref)
{
	static void* saved_ref;

	/* To avoid rewriting these unnecessarily every time, we only do
	 * it when the ref has changed, or if ref == NULL (=> force). */
	if (ref) {
		if (ref == saved_ref) {
			return;
		}
		saved_ref = ref; /* only save if non-null */
	}

	etx_write(etx_config->mem_ctrl, MEM_CTRL_REG);
	etx_write(etx_config->control, CONTROL_REG);
	etx_write(etx_config->ecc_ctrl, ECC_CTRL_REG);
	etx_write(etx_config->ecc_offset, ECC_OFFSET_REG);
}
/* Set up CONTROL depending on whether we want ECC or not */
static void etx_setup_control(int enable_ecc)
{
	uint32_t control;

	/* When reading the oob, we never want ECC, when reading the
	 * main area, it depends. */
	control = etx_read(CONTROL_REG) & ~CONTROL_ECC_EN;
	if (enable_ecc) {
		control |= CONTROL_ECC_EN;
	}
	etx_write(control, CONTROL_REG);
}
/* Write timing setup to controller */
static void etx_setup_timing(struct etx_setup* etx_setup)
{
	etx_write(etx_setup->timings.time_seq_0, TIME_SEQ_0_REG);
	etx_write(etx_setup->timings.time_seq_1, TIME_SEQ_1_REG);
	etx_write(etx_setup->timings.timings_asyn, TIMINGS_ASYN_REG);
	etx_write(etx_setup->timings.time_gen_seq_0, TIME_GEN_SEQ_0_REG);
	etx_write(etx_setup->timings.time_gen_seq_1, TIME_GEN_SEQ_1_REG);
	etx_write(etx_setup->timings.time_gen_seq_2, TIME_GEN_SEQ_2_REG);
	etx_write(etx_setup->timings.time_gen_seq_3, TIME_GEN_SEQ_3_REG);
}

/* Set up interrupt, send command, then wait for (any bit of) expected state */
/* Before issuing a command, we should check if the controller is ready.
 * We can't check INT_STATUS_REG.MEM0_RDY_INT_FL as it is not a status bit,
 * it is set on an nfc state transition after the completion of for
 * instance a page program command, so we can use it as a command
 * completed trigger however.
 * (See NFC Design Spec (rev 1.15) figure 35 for illustration.)
 * TODO: However, we could check STATUS.CTRL_STAT, which should always
 * be 0 prior to issuing a command, indicating the controller is not
 * busy. */
static void etx_command_and_wait(uint32_t etx_command, uint32_t int_state)
{
	do{
		/* Clear interrupt status bits */
		etx_write(0, INT_STATUS_REG);//0x14
		/* Send command */
		etx_write( etx_command, COMMAND_REG );//0x00
		nfc_debug("----------------------- etx_command = 0x%x ----------------------- \n",etx_command);
	}while(etx_command != etx_read(COMMAND_REG));

	//todo: wait for done?
	/* Wait for command to complete */
	nfc_debug("Waiting for 0x%08x bit(s) to be set in int_status\n", int_state);
	int cmd_loops = 0;
	uint32_t read_status,read_int_status,dma_status;

	do {
		cmd_loops++;
		read_status = etx_read(STATUS_REG);
		read_int_status = etx_read(INT_STATUS_REG);
		dma_status = etx_read(DMA_CTRL_REG);
		nfc_debug("Wait for command done: 0x%08x/0x%08x/0x%08x (%d)\n",
			read_status,
			read_int_status,
			dma_status,
			cmd_loops);
		if (read_int_status == int_state) {
			break;
		}
	} while (!(read_int_status & int_state) && cmd_loops < MAX_CMD_LOOPS);

	if (cmd_loops >= MAX_CMD_LOOPS)
		info("Int wait for 0x%08x timed out after %d loops: "
			"STATUS = 0x%08x, INT_STATUS=0x%08x, "
			"DMA_CTRL = 0x%08x, command 0x%08x\n",
			cmd_loops,
			int_state,
			read_status,
			read_int_status,
			dma_status,
			etx_command);
	etx_write(0, INT_STATUS_REG);//0x14
}

static void etx_init_siu_fifo(int bytes)
{
	etx_write((bytes + 3) & 0xfffffffc, DATA_SIZE_REG);
}

/* Initialize transfer to or from DMA buffer */
static void etx_init_dmabuf(int bytes)
{
	p_nfc->dma.ptr = p_nfc->dma.buf;
	p_nfc->dma.bytes_left = bytes;
}

/*dma ddr workaroud*/
//#define dma_virt_to_phy(x)	((((unsigned long)(x)>0x80000000) && ((unsigned long)(x)<0xc0000000))? ((unsigned long)(x)-0x80000000):(unsigned long)(x))
#define dma_virt_to_phy(x)	(x)

/* Initialize DMA, wq and interrupt status for upcoming transfer. */
static void ext_init_dma(uint64_t addr, int bytes)
{
	int dma_trig_level;

	/* DMA control */
	/* Start when COMMAND register written, set burst type/size */
	etx_write(DMA_CTRL_DMA_START | DMA_CTRL_DMA_BURST_I_P_4, DMA_CTRL_REG);

	/* DMA address and length */
#ifdef EVATRONIX_DMA64BIT
	/* The manual says this register does not 'occur' (sic) unless
	 * 64 bit DMA support is included. */
	etx_write(addr >> 32, DMA_ADDR_H_REG);
#endif
	etx_write(dma_virt_to_phy(addr), DMA_ADDR_L_REG);
	//etx_write(virt_to_phys(addr), DMA_ADDR_L_REG);

	/* Byte counter */
	/* Round up to nearest 32-bit word */
	etx_write((bytes + 3) & 0xfffffffc, DMA_CNT_REG);

	/* Cap DMA trigger level at FIFO size */
	dma_trig_level = bytes * 8 / 32; /* 32-bit entities */
	if (dma_trig_level > DMA_TLVL_MAX) {
		dma_trig_level = DMA_TLVL_MAX;
	}
	etx_write(dma_trig_level, DMA_TLVL_REG);
}

static u32 etx_read_siu_fifo(struct candence_nfc* priv)
{
	return __raw_readl((u32)priv->regs + FIFO_DATA_REG);
}
static u32 etx_write_siu_fifo(struct candence_nfc* priv, u32 val)
{
	return __raw_writel(val, (u32)priv->regs + FIFO_DATA_REG);
}

static u32 etx_read_fifo(struct candence_nfc* priv)
{
	u32 val = 0;

	if (priv->cmd_cache.command != NAND_CMD_STATUS) {
		val = etx_read_siu_fifo(priv);
	}

	return val;
}
static void etx_read_fifobuf(struct candence_nfc* priv, u32* buf, int len)
{
	nfc_debug("%p, buf %p, len %d\n", priv, buf, len);
	if (len > priv->dma.bytes_left) {
	}
	int i;
	for (i = 0; i < len / 4;) {
		if (!(etx_read(FIFO_STATE_REG) & FIFO_STATE_DF_R_EMPTY)) {
			*buf = etx_read_fifo(priv);
			buf++;
			i++;
		}
	}
}

static void etx_read_mode(struct candence_nfc* priv, int page, int column,
	enum etx_read_mode m)
{
	int size;
	uint32_t command;
	u32	cmd;

	switch (m) {
	case ETX_READ_OOB:
		size = priv->mtd.oobsize;
		break;
	case ETX_READ_ALL:
		size = priv->mtd.oobsize + priv->mtd.writesize;
		break;
	case ETX_READ_STD:
	case ETX_READ_RAW:
		size = priv->mtd.writesize;
		break;
	default:
		size = 0;
		break;
	}

	/* Set up ECC depending on mode */
	etx_setup_control(m == ETX_READ_STD);

	/* Set up DMA and transfer size */
	etx_init_dmabuf(size);
	if (priv->use_mode == NFC_INTERNAL_DMA) {
		ext_init_dma(priv->dma.phys, size);
	}

	etx_write(size, DATA_SIZE_REG);

	/* Set up addresses */
	if (m == ETX_READ_OOB) {
		column += priv->mtd.writesize;
	}
	etx_write(column, ADDR0_COL_REG);
	etx_write(page, ADDR0_ROW_REG);

	/* For devices > 128 MiB we have 5 address cycles and can use a
	 * standard NFC command sequence. For smaller devices we have
	 * 4 address cycles and need to use a Generic Command Sequence. */
	if (priv->chip.chipsize > (128 << 20)) {
		cmd = GEN_SEQ_CTRL_READ_PAGE_5CYCLE;
	} else {
		cmd = GEN_SEQ_CTRL_READ_PAGE_4CYCLE;
	}
	if (priv->use_mode == NFC_INTERNAL_DMA) {
		etx_write(cmd,GEN_SEQ_CTRL_REG);
		etx_write(FIFO_INIT_FIFO_INIT, FIFO_INIT_REG);/* Flush FIFO */
		command = COMMAND_READ_PAGE_DMA_GEN;
	} else if (priv->use_mode == NFC_SIU_FIFO) {
		etx_write(cmd,GEN_SEQ_CTRL_REG);
		etx_write(FIFO_INIT_FIFO_INIT, FIFO_INIT_REG);/* Flush FIFO */
		command = COMMAND_READ_PAGE_GEN;
	}

	if (priv->use_mode == NFC_INTERNAL_DMA) {
		etx_command_and_wait(command, INT_STATUS_DMA_INT_FL);
	} else if (priv->use_mode == NFC_SIU_FIFO) {
		etx_command_and_wait(command, INT_STATUS_CMD_END_INT_FL);
	}
}

static void etx_write_mode(struct candence_nfc* priv, int page, int column,
	int oob, int raw)
{
	int size;
	uint32_t command;
	u32	cmd;
	u8 *write_ptr;

	/* Since the controller handles ECC on its own, raw mode doesn't
	 * come into the size calculations. */
	if (column >= priv->mtd.writesize) {
		/* oob write only */
		size = priv->mtd.oobsize;
		raw = 1;
	} else {
		size = priv->mtd.writesize;
		if (oob) {
			size += priv->mtd.oobsize;
			raw = 1;
		}
	}

	etx_setup_control(!raw);

	if (priv->use_mode == NFC_INTERNAL_DMA) {
		/* Dump selected parts of buffer */
		nfc_debug("Write %d bytes: 0x%08x 0x%08x .. 0x%08x\n",
			size,
			((uint32_t *)(priv->dma.buf))[0],
			((uint32_t *)(priv->dma.buf))[1],
			((uint32_t *)(priv->dma.buf))[size / 4 - 1]);

		/* Set up DMA and transfer size */
		ext_init_dma(priv->dma.phys, size);
	}
	etx_write(size, DATA_SIZE_REG);

	/* Set up addresses */
	etx_write(column, ADDR0_COL_REG);
	etx_write(page, ADDR0_ROW_REG);


	/* For devices > 128 MiB we have 5 address cycles and can use a
	 * standard NFC command sequence. For smaller devices we have
	 * 4 address cycles and need to use a Generic Command Sequence. */
	if (priv->chip.chipsize > (128 << 20)) {
		cmd = GEN_SEQ_CTRL_WRITE_PAGE_5CYCLE;
	} else {
		cmd = GEN_SEQ_CTRL_WRITE_PAGE_4CYCLE;
	}

	if (priv->use_mode == NFC_INTERNAL_DMA) {
		etx_write(cmd,GEN_SEQ_CTRL_REG);
		etx_write(FIFO_INIT_FIFO_INIT, FIFO_INIT_REG);/* Flush FIFO */
		command = COMMAND_WRITE_PAGE_DMA_STD;
	} else if (priv->use_mode == NFC_SIU_FIFO) {
		etx_write(cmd,GEN_SEQ_CTRL_REG);
		etx_write(FIFO_INIT_FIFO_INIT, FIFO_INIT_REG);/* Flush FIFO */
		command = COMMAND_WRITE_PAGE_FIFO_GEN;
	}

	if (priv->use_mode == NFC_INTERNAL_DMA) {
		/* TODO: Use INT_STATUS_MEM0_RDY_INT_FL instead ? */
		etx_command_and_wait(command, INT_STATUS_DMA_INT_FL);

		/* Wait for Ready from device */
		/* TODO: ?needed ?how */
		/* TODO: In the same way as for erase, we could check INT_STATUS_REG.
		 * STAT_ERR_INT0_FL, but nand_base will check the device by reading
		 * error status anyway after the write command.
		*/

		/* clear buffer so it doesn't contain the written data anymore */
		/* TODO: remove this, just useful during development to verify
		 * that a subsequent read just doesn't read what happens to be
		 * lying around in the buffer. 
		*/
		memset((void*)priv->dma.buf, 0, DMA_BUF_SIZE);
	} else if (priv->use_mode == NFC_SIU_FIFO) {
		int i ;
		etx_command_and_wait(command, INT_STATUS_WAIT_FOR_WRITE);
		write_ptr = priv->dma.buf;
		for (i = 0; i < size;) {
			if (!(etx_read(FIFO_STATE_REG) & FIFO_STATE_DF_W_FULL)) {
				etx_write_siu_fifo(priv,
					readl((void*)write_ptr));
				write_ptr += 0x4;
				i = i + 4;
			}
		}
	}
}

/* Block erase */
static void etx_block_erase(int page)
{
	/* Set up addresses */
	etx_write(page, ADDR0_ROW_REG);
	nfc_debug("Erase block containing page %d\n", page);

	/* Send 3 address cycle block erase command */
	etx_command_and_wait(COMMAND_BLOCK_ERASE, INT_STATUS_MEM0_RDY_INT_FL);

	/* TODO: What to do if we get an error bit set here (i.e.INT_STATUS_REG.
	 * STAT_ERR_INT0_FL) ? Normally, error status is checked by nand_base
	 * by doing a status read after the erase command. So we can probably
	 * ignore STAT_ERR_INT0_FL here. If need be, we can save the
	 * status so a subsequent status right might use it for something.
	 * The err bit probably just indicates that the flash didn't pull
	 * R/_B low within tWB. */
}

/* 
Read byte from DMA buffer
Not used directly, only via etx_read_byte 
*/
static uint8_t etx_read_dmabuf_byte(struct candence_nfc* priv)
{
	if (priv->dma.bytes_left) {
		priv->dma.bytes_left--;
		return *priv->dma.ptr++;
	} else {
		return 0; /* no data */
	}
}


unsigned char etx_read_byte(struct candence_nfc* priv)
{
	uint8_t status_value;

	if (priv->cmd_cache.command != NAND_CMD_STATUS) {
		return etx_read_dmabuf_byte(priv);
	}

	nfc_debug("Read status\n");

	/* In order to read status, we need to send a READ_STATUS command
	 * to the NFC first, in order to get the data into the DATA_REG */
	/* Transfer to DATA_REG register */
	etx_write(DATA_REG_SIZE_DATA_REG_SIZE(1), DATA_REG_SIZE_REG);

	/* We want to read all status bits from the device */
	etx_write(STATUS_MASK_STATE_MASK(0xff), STATUS_MASK_REG);
	etx_command_and_wait(COMMAND_READ_STATUS, INT_STATUS_DATA_REG_FL);
	status_value = etx_read(DATA_REG_REG) & 0xff;
	//status_value = 0xC0; /* Bit 7 : No write prot., Bit 6: Device ready */
	nfc_debug("Status 0x%08x\n", status_value);
	return status_value;
}

/* Read block of data from DMA buffer */
static void etx_read_dmabuf(struct candence_nfc* priv, uint8_t* buf, int len)
{
	nfc_debug("%p, buf %p, len %d\n", priv, buf, len);
	if (len > priv->dma.bytes_left) {
		info("Trying to read %d bytes with %d bytes remaining\n",
			 len, priv->dma.bytes_left);
	}
	memcpy(buf, priv->dma.ptr, len);
	priv->dma.ptr += len;
	priv->dma.bytes_left -= len;
}
void etx_read_buf(struct candence_nfc* priv, unsigned char* buf, int len)
{
	if (priv->use_mode == NFC_INTERNAL_DMA)
		etx_read_dmabuf(priv,buf,len);
	else
		etx_read_fifobuf(priv,(u32*)buf,len);
}
/* Write block of data to DMA buffer */
void etx_write_dmabuf(struct candence_nfc* priv, const uint8_t* buf, int len)
{
	nfc_debug("%p, buf %p, len %d\n", priv, buf, len);
	/* TODO: Grab info pointer from mtd instead of using same always ? */
	if (len > priv->dma.bytes_left) {
		info("Trying to write %d bytes with %d bytes remaining\n",
			 len, priv->dma.bytes_left);
	}
	memcpy(priv->dma.ptr, buf, len);
	priv->dma.ptr += len;
	priv->dma.bytes_left -= len;
}

/* Read state of ready pin */
int etx_dev_ready(struct candence_nfc* priv)
{
	struct etx_config* etx_config = &priv->config;

	return !!(etx_read(STATUS_REG) & etx_config->mem_status_mask);
}

/* Do the dirty work for read_page_foo */
static int etx_read_page_mode(struct candence_nfc* priv, uint8_t* buf,
	int oob_required, int page, enum etx_read_mode m)
{
	unsigned int max_bitflips;
	uint32_t ecc_status;
	struct nfc_mtd_info* mtd = &priv->mtd;

	if (page != priv->cmd_cache.page) {
		debug("Warning: Read page has different page number than "
			"READ0: %d vs. %d\n", page, priv->cmd_cache.page);
	}

	if (m == ETX_READ_STD) {
		/* ECC error flags and counters are not cleared automatically
		 * so we do it here.
		 */
		/* Note that the design spec says nothing about having to
		 * zero ECC_STAT (although it explicitly says that ECC_CNT
		 * needs to be zeroed by software), but testing on actual
		 * hardware (RTL at this stage) reveals that this is in fact
		 * the case. 
		 */
		etx_write(0, ECC_STAT_REG);
		etx_write(0, ECC_CNT_REG);
	}

	if (priv->use_mode == NFC_INTERNAL_DMA) {
		etx_read_mode(priv,
			priv->cmd_cache.page,
			priv->cmd_cache.column,
			m);
		/* This is actually etx_read_dmabuf */
		etx_read_dmabuf(priv, buf, mtd->writesize);
	} else if (priv->use_mode == NFC_SIU_FIFO) {
		etx_read_mode(priv,
			priv->cmd_cache.page,
			priv->cmd_cache.column,
			m);

		/* This is actually etx_read_dmabuf */
		etx_read_fifobuf(priv, (u32 *)buf, mtd->writesize);
	}

	if (m == ETX_READ_RAW) {
		return 0;
	}

	/* Get ECC status from controller */
	ecc_status = etx_read(ECC_STAT_REG);
	max_bitflips = etx_read(ECC_CNT_REG) & ECC_CNT_ERR_LVL_MASK;

#ifdef WORKAROUND_NO_ECC_CNT
	/* If we get an ERROR bit set, but ECC_CNT is 0, we assume
	 * a single bit flip has occurred for want of better information. */
	if ((ecc_status & ECC_STAT_ERROR_0) && max_bitflips == 0) {
		max_bitflips = 1;
	}
#endif

	/* The following is actually not really correct, as the stats should
	 * reflect _all_ bitflips, not just the largest one in the latest read.
	 * We could rectify this by reading chip->ecc.bytes at a time,
	 * and accumulating the statistics per read, but at least for now
	 * the additional overhead doesn't seem to warrant the increased
	 * accuracy of the statistics, since the important figure is the
	 * max number of bitflips in a single ECC block returned by this
	 * function. */
	mtd->ecc_stats.corrected += max_bitflips;

	nfc_debug("ECC read status: %s%s%s%s, correction count %d\n",
		ecc_status & ECC_STAT_UNC_0 ? "Uncorrected " : "",
		ecc_status & ECC_STAT_ERROR_0 ? "Corrected " : "",
		ecc_status & ECC_STAT_OVER_0 ? "Over limit " : "",
		ecc_status & (ECC_STAT_UNC_0 | ECC_STAT_ERROR_0 |
		ECC_STAT_OVER_0) ? "" : "ok",
		max_bitflips);

	/* We shouldn't see oob_required for ECC reads. */
	if (oob_required) {
	}

	return max_bitflips;
}

/* Read page with HW ECC */
int etx_read_page_hwecc(struct candence_nfc* priv, uint8_t* buf,
	int oob_required, int page)
{
	nfc_debug("page %d, oobreq %d\n", page, oob_required);
	return etx_read_page_mode(priv, buf, oob_required, page, ETX_READ_STD);
}

/* Read page with no ECC */
int etx_read_page_raw(struct candence_nfc* priv, uint8_t* buf, int oob_required,
	int page)
{
	nfc_debug("page %d, oobreq %d\n", page, oob_required);
	return etx_read_page_mode(priv, buf, oob_required, page, ETX_READ_RAW);
}

/* Write page with HW ECC */
/* This is the only place where we know we'll be writing w/ ECC */
int etx_write_page_hwecc(struct candence_nfc* priv, const uint8_t* buf,
	int oob_required)
{
	struct nfc_mtd_info* mtd = &priv->mtd;
	nfc_debug("oob_required %d\n", oob_required);

	/* The controller can't write data to the oob when ECC is enabled,
	 * so we set oob_required to 0 here and don't process the oob
	 * further even if requested. This could happen for instance if
	 * using nandwrite -o without -n . */
	if (oob_required) {
		debug("Tried to write OOB with ECC!\n");
	}
	priv->cmd_cache.oob_required = 0;
	priv->cmd_cache.write_raw = 0;

	/* A bit silly this, this is actually etx_write_dmabuf */
	etx_write_dmabuf(priv, buf, mtd->writesize);

	return 0;
}

/* Write page with no ECC */
/* This is the only place where we know we won't be writing w/ ECC */
int etx_write_page_raw(struct candence_nfc* priv, const uint8_t* buf,
	int oob_required)
{
	struct nfc_mtd_info* mtd = &priv->mtd;
	nfc_debug("oob_required %d\n", oob_required);

	/* We need this for the upcoming PAGEPROG command */
	priv->cmd_cache.oob_required = oob_required;
	priv->cmd_cache.write_raw = 1;

	/* A bit silly this, this is actually etx_write_dmabuf */
	etx_write_dmabuf(priv, buf, mtd->writesize);

	if (oob_required) {
		etx_write_dmabuf(priv, priv->chip.oob_poi, mtd->oobsize);
	}

	return 0;
}

void etx_nand_command(struct candence_nfc* priv, unsigned int command,
	int column, int page_addr)
{
	/* Save command so that other parts of the API can figure out
	 * what's actually going on. */
	priv->cmd_cache.command = command;

	/* Configure the NFC for the flash chip in question. */
	etx_config(&priv->config, priv);

	/* Some commands we execute immediately, while some need to be
	 * deferred until we have all the data needed, i.e. for page read,
	 * we can't initiate the read until we know if we are going to be
	 * using raw mode or not.
	 */
	switch (command) {
	case NAND_CMD_READ0:
		nfc_debug("READ0 page %d, column %d\n", page_addr, column);
		if (priv->setup.ecc_mode == NFC_ECC_HW) {
			/* We do not yet know if the caller wants to
			   * read the page with or without ECC, so we
			   * just store the page number and main/oob flag
			   * here.
			   * TODO: Since the page number arrives via the
			   * read_page call, we don't really need to
			   * store it.
			*/
			priv->cmd_cache.page = page_addr;
			priv->cmd_cache.column = column;
		} else {
			/* Read the whole page including oob */
			priv->cmd_cache.oob_required = 1;
			etx_read_mode(priv, page_addr, column, ETX_READ_ALL);
		}
		break;
	case NAND_CMD_READOOB:
		nfc_debug("READOOB page %d, column %d\n", page_addr, column);

		if (priv->use_mode == NFC_SIU_FIFO) {
			etx_read_mode(priv, page_addr, column, ETX_READ_OOB);
		} else if (priv->use_mode == NFC_INTERNAL_DMA) {
			/* In contrast to READ0, where nand_base always calls
			   * a read_page_foo function before reading the data,
			   * for READOOB, read_buf is called instead.
			   * We don't want the actual read in read_buf, so
			   * we put it here.
			*/
			etx_read_mode(priv, page_addr, column, ETX_READ_OOB);
		}

		break;
	case NAND_CMD_ERASE1:
		nfc_debug("ERASE1 page %d\n", page_addr);
		/* Just grab page parameter, wait until ERASE2 to do
		 * something.
		*/
		priv->cmd_cache.page = page_addr;
		break;
	case NAND_CMD_ERASE2:
		nfc_debug("ERASE2 page %d, do it\n", priv->cmd_cache.page);
		/* Off we go! */
		etx_block_erase(priv->cmd_cache.page);
		break;
	case NAND_CMD_RESET:
		nfc_debug("chip reset\n");
		etx_command_and_wait(COMMAND_RESET, INT_STATUS_CMD_END_INT_FL);
		break;
	case NAND_CMD_SEQIN:
		nfc_debug("SEQIN column %d, page %d\n", column, page_addr);
		/* Just grab some parameters, then wait until
		 * PAGEPROG to do the actual operation.
		 */
		priv->cmd_cache.page = page_addr;
		priv->cmd_cache.column = column;
		/* Prepare DMA buffer for data. We don't yet know
		 * how much data there is, so set size to max.
		 */
		etx_init_dmabuf(DMA_BUF_SIZE);
		break;
	case NAND_CMD_PAGEPROG:
		/* Used for both main area and oob */
		nfc_debug("PAGEPROG page %d, column %d, w/oob %d, raw %d\n",
			priv->cmd_cache.page,
			priv->cmd_cache.column,
			priv->cmd_cache.oob_required,
			priv->cmd_cache.write_raw);

		etx_write_mode(priv,
			priv->cmd_cache.page,
			priv->cmd_cache.column,
			priv->cmd_cache.oob_required,
			priv->cmd_cache.write_raw);
		break;
	case NAND_CMD_READID:
		nfc_debug("READID (0x%02x)\n", column);

		/* Read specified ID bytes */
		/* 0x00 would be NAND_READ_ID_ADDR_STD
		* 0x20 would be NAND_READ_ID_ADDR_ONFI,
		* but NAND subsystem knows this and sends us the
		* address values directly
		*/
		etx_write(column, ADDR0_COL_REG);

		if (priv->use_mode == NFC_SIU_FIFO) {
			etx_init_siu_fifo(column == NAND_READ_ID_ADDR_STD ? 5 :
				4);
			etx_init_dmabuf(column == NAND_READ_ID_ADDR_STD ? 5 :
				4);
			etx_command_and_wait(COMMAND_READ_ID_SIU_FIFO,
				INT_STATUS_MEM0_RDY_INT_FL);
			int i;
			u32 val;
			for (i=0;i<p_nfc->dma.bytes_left;i+=4) {
				val = etx_read_fifo(priv);
				memcpy(priv->dma.ptr+i,&val,4);
			}
		} else if (priv->use_mode == NFC_INTERNAL_DMA) {
			etx_write(0x3, DATA_REG_SIZE_REG);
			/* Set up expected number of returned bytes */
			etx_init_dmabuf(column == NAND_READ_ID_ADDR_STD ? 5 :
				4);
			ext_init_dma(priv->dma.phys,
				column == NAND_READ_ID_ADDR_STD ? 5 : 4);
			etx_write((p_nfc->dma.bytes_left + 3) & 0xfffffffc,
				DATA_SIZE_REG);
			/* Send read id command */
			etx_command_and_wait(COMMAND_READ_ID,
				INT_STATUS_DMA_INT_FL);
		}
		break;
	case NAND_CMD_STATUS:
		nfc_debug("STATUS, defer to later read byte\n");
		/* Don't do anything now, wait until we need to
		 * actually read status.
		 */
		break;
	default:
		debug("Unhandled command 0x%02x (col %d, page addr %d)\n",
			command,
			column,
			page_addr);
		break;
	}
}

static int etx_init_resources(struct candence_nfc* priv)
{
	if (p_nfc->use_mode == NFC_INTERNAL_DMA) {
		debug("use internal dma mode!\n");
		priv->dma.phys = (u32)etx_dma_buff;
		priv->dma.buf = (void*)etx_dma_buff;

		memset(etx_dma_buff, 0, (u32)DMA_BUF_SIZE);

		if (priv->dma.buf == NULL) {
			debug("dma_alloc_coherent failed!\n");
			return -1;
		}
	} else if (p_nfc->use_mode == NFC_SIU_FIFO) {
		debug("use siu fifo mode!\n");
		priv->dma.phys = (u32)etx_dma_buff;
		priv->dma.buf = (unsigned char*)etx_dma_buff;

		memset(etx_dma_buff, 0, (u32)DMA_BUF_SIZE);

		if (priv->dma.buf == NULL) {
			debug("dma_alloc_coherent failed!\n");
			return -1;
		}
	} else {
		debug("nfc_use_mode error!\n");
		return -1;
	}

	return 0;
}

void etx_update_timing(struct etx_setup* etx_setup, struct nfc_timing* time)
{
	u32 nfc_clk,cycle;
	u32 twhr,trhw,tadl,tccs;
	u32 trr,twb;
	u32 t0_d0,t0_d1,t0_d2,t0_d3;
	u32 t0_d4,t0_d5,t0_d6,t0_d7;
	u32 t0_d8,t0_d9,t0_d10,t0_d11;
	u32 t0_d12;
	u32 trwh,trwp;

	nfc_clk = 50000000;
	cycle = 1000000000 / nfc_clk;

	twhr = time->twhr / cycle;
	trhw = time->trhw / cycle;
	tadl = time->tadl / cycle;
	tccs = time->tccs / cycle;
	trr = time->trr / cycle;
	twb = time->twb / cycle;
	t0_d0 = time->t0_d0 / cycle;
	t0_d1 = time->t0_d1 / cycle;
	t0_d2 = time->t0_d2 / cycle;
	t0_d3 = time->t0_d3 / cycle;
	t0_d4 = time->t0_d4 / cycle;
	t0_d5 = time->t0_d5 / cycle;
	t0_d6 = time->t0_d6 / cycle;
	t0_d7 = time->t0_d7 / cycle;
	t0_d8 = time->t0_d8 / cycle;
	t0_d9 = time->t0_d9 / cycle;
	t0_d10 = time->t0_d10 / cycle;
	t0_d11 = time->t0_d11 / cycle;
	t0_d12 = time->t0_d12 / cycle;
	trwh = time->trwh / cycle;
	trwp = time->trwp / cycle;

	etx_setup->timings.time_seq_0 = ((twhr & 0x3f) << 24) |
		((trhw & 0x3f) << 16) | ((tadl & 0x3f) <<
		8) | ((tccs & 0x3f) << 0); 
	etx_setup->timings.time_seq_1 = ((trr & 0x3f) << 8) |
		((twb & 0x3f) << 0); 
	etx_setup->timings.time_gen_seq_0 = ((t0_d3 & 0x3f) << 24) |
		((t0_d2 & 0x3f) << 16) | ((t0_d1 & 0x3f) <<
		8) | ((t0_d0 & 0x3f) << 0); 
	etx_setup->timings.time_gen_seq_1 = ((t0_d7 & 0x3f) << 24) |
		((t0_d6 & 0x3f) << 16) | ((t0_d5 & 0x3f) <<
		8) | ((t0_d4 & 0x3f) << 0); 
	etx_setup->timings.time_gen_seq_2 = ((t0_d11 & 0x3f) << 24) |
		((t0_d10 & 0x3f) << 16) | ((t0_d9 & 0x3f) <<
		8) | ((t0_d8 & 0x3f) << 0); 
	etx_setup->timings.time_gen_seq_3 = ((t0_d12 & 0x3f) << 24);
	etx_setup->timings.timings_asyn = ((trwh & 0xf) << 4) |
		((trwp & 0xf) << 0); 

	etx_setup_timing(etx_setup);
}

void etx_enable_int(int en)
{
	u32 val;

	val = etx_read(CONTROL_REG);

	val &= ~CONTROL_INT_EN;
	if (en)
		val |= CONTROL_INT_EN;
	etx_write(val,CONTROL_REG);
}
unsigned long etx_get_int_status(void)
{
	return etx_read(INT_STATUS_REG);
}
void etx_set_int_mask(unsigned long val)
{
	etx_write(val,INT_MASK_REG);
}
void etx_clear_int(unsigned long val)
{
	etx_write(~val,INT_STATUS_REG);
}




static int brite_fdt_decode_nand(const void *blob, int node, struct candence_nfc* priv)
{
	priv->regs = (void *)fdtdec_get_addr(blob, node, "reg");
	priv->pclk = fdtdec_get_int(blob, node,"clock-frequency", 0);

	return 0;
}



int etx_init(struct candence_nfc* priv)
{
	int err;
#if 1
	int node;
	node = fdtdec_next_compatible(gd->fdt_blob, 0, COMPAT_CANDENCE_BRITE_NFC);
	if (node < 0)
		return -1;
	if (brite_fdt_decode_nand(gd->fdt_blob, node, priv)) 
	{
		printf("Could not decode nand-flash in device tree\n");
		return -1;
	}

#else

	priv->regs = (void *)BRITE_NFC_BASE;
	priv->pclk = BRITE_NFC_PCLK;

#endif

	priv->use_mode = NFC_INTERNAL_DMA;// NFC_SIU_FIFO;
	priv->cs = 0;

	memset( priv->regs, 0, 0x200 );
	memset( priv->regs, 0, 0x200 );
  
#ifdef ETX_CE_BANK_SEL
	/* Separate chips regarded as different banks. */
	priv->config.mem_ctrl = MEM_CTRL_BANK_SEL(priv->cs);
#else
	/* Separate chips regarded as different chip selects. */
	priv->config.mem_ctrl = MEM_CTRL_MEM_CE(priv->cs);
#endif

#ifdef ETX_RB_WIRED_AND
	/* Ready/busy from nand flash arrives via wired-AND for device 0 */
	priv->config.mem_status_mask = STATUS_MEM0_ST;
#else
	/* Ready/busy from nand flash as separate per-device signals */
	priv->config.mem_status_mask = STATUS_MEM_ST(priv->cs);
#endif


	priv->setup.ecc_mode = NFC_ECC_HW;
	priv->setup.ecc_strength = 8;
	priv->setup.ecc_blksize = 512;
	priv->chip.ecc_step_ds = 512;

	memcpy(&priv->setup.timings,
		&default_mode0_pll_enabled,
		sizeof(priv->setup.timings));

	/* Initialize interrupts and DMA etc. */
	err = etx_init_resources(priv);
	if (err)
		return err;

	etx_setup_timing(&priv->setup);

	return 0;
}


int brite_nand_init(void)
{
	int ret;

	memset(p_nfc, 0, sizeof(struct candence_nfc));
	ret = etx_init(p_nfc);
	if (ret < 0) {
		printf("nand controller init error.\n");
		return -1;
	}

	return 0;
}

struct nand_flash {
	struct mtd_info *mtd;
	struct nand_chip *chip;

	unsigned long long size;
	unsigned long page_size;
	unsigned long page_per_block;
};

struct nand_flash nf;
struct nand_flash* pnf = &nf;
//struct mtd_info nf_mtd;
struct nand_chip nf_chip;


/*wrapper for  nfc controller and mtd*/
static void nfc_cmdfunc(struct mtd_info *mtd, unsigned command, int column,
			int page_addr)
{
	etx_nand_command(p_nfc,command,column,page_addr);
}
static int nfc_dev_ready(struct mtd_info *mtd)
{
	return etx_dev_ready(p_nfc);
}
static uint8_t nfc_read_byte(struct mtd_info *mtd)
{
	return etx_read_byte(p_nfc);
}
static void nfc_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	etx_write_dmabuf(p_nfc,buf,len);
}
static void nfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	etx_read_buf(p_nfc,buf,len);
}

static int nfc_read_page(struct mtd_info *mtd, struct nand_chip *chip,
			uint8_t *buf, int oob_required, int page)
{
	return etx_read_page_hwecc(p_nfc,buf,oob_required,page);
}
static int nfc_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			uint8_t *buf, int oob_required, int page)
{
	return etx_read_page_raw(p_nfc,buf,oob_required,page);
}
static int nfc_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			const uint8_t *buf, int oob_required)
{
	return etx_write_page_hwecc(p_nfc,buf,oob_required);
}
static int nfc_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			const uint8_t *buf, int oob_required)
{
	return etx_write_page_raw(p_nfc,buf,oob_required);
}
void nfc_select_chip(struct mtd_info *mtd, int chip)
{
}

int nf_init(int devnum)
{
	memset(pnf,0,sizeof(struct nand_flash));
	pnf->mtd = &nand_info[devnum];
	//pnf->mtd = &nf_mtd;
	pnf->chip = &nf_chip;

	memset(pnf->mtd,0,sizeof(struct mtd_info));
	memset(pnf->chip,0,sizeof(struct nand_chip));
	pnf->mtd->priv = pnf->chip;

	/* Our interface to the mtd API */
	pnf->chip->cmdfunc = nfc_cmdfunc;
	pnf->chip->dev_ready = nfc_dev_ready;
	pnf->chip->read_byte = nfc_read_byte;
	pnf->chip->read_buf = nfc_read_buf;
	pnf->chip->write_buf = nfc_write_buf;

	pnf->chip->ecc.read_page = nfc_read_page;
	pnf->chip->ecc.read_page_raw = nfc_read_page_raw;
	pnf->chip->ecc.write_page = nfc_write_page;
	pnf->chip->ecc.write_page_raw = nfc_write_page_raw;

	pnf->chip->select_chip = nfc_select_chip;

	pnf->chip->ecc.mode = NAND_ECC_HW;
	pnf->chip->ecc.strength = BRITE_NAND_ECC_STRENGTH;//8;
	pnf->chip->ecc.size = BRITE_NAND_ECC_SIZE;//512;
	return 0;
}


int brite_nf_probe(void)
{
	int ret = 0;
	
	nand_curr_device = 0;
	ret= nf_init(nand_curr_device);

	ret = nand_scan(pnf->mtd,1);

	pnf->page_per_block = 1 << (pnf->chip->phys_erase_shift -
		pnf->chip->page_shift);
	pnf->size = pnf->mtd->size;
	pnf->page_size = pnf->mtd->writesize;

	/* transfer needed info to driver*/
	p_nfc->mtd.writesize = pnf->mtd->writesize;
	p_nfc->mtd.oobsize = pnf->mtd->oobsize;
	memcpy(&p_nfc->mtd.ecc_stats,&pnf->mtd->ecc_stats,
		sizeof(struct mtd_ecc_stats));
	p_nfc->chip.chipsize = pnf->chip->chipsize;
	p_nfc->chip.oob_poi = pnf->chip->oob_poi;
	p_nfc->chip.ecc_step_ds = pnf->chip->ecc_step_ds;
	p_nfc->chip.options = pnf->chip->options;

	/*ECC config*/
	p_nfc->config.control = CONTROL_ECC_BLOCK_SIZE(p_nfc->setup.ecc_blksize) |
               CONTROL_BLOCK_SIZE(pnf->page_per_block);
	if(p_nfc->chip.options & SP_OPTIONS16)
	{
		p_nfc->config.control |= CONTROL_IO_WIDTH_16;
		/* Put ECC bytes into OOB at an offset, to skip bad block marker */
		p_nfc->config.ecc_offset = p_nfc->mtd.writesize/2 + ECC_OFFSET;
	}
	else
	{
		p_nfc->config.control |= CONTROL_IO_WIDTH_8;
		/* Put ECC bytes into OOB at an offset, to skip bad block marker */
		p_nfc->config.ecc_offset = p_nfc->mtd.writesize + ECC_OFFSET;
	}

	/* Set up ECC control and offset of ECC data */
	/* We don't use the threshold capability of the controller, as we
	* let mtd handle that, so set the threshold to same as capability. */
	p_nfc->config.ecc_ctrl = ECC_CTRL_ECC_THRESHOLD(p_nfc->setup.ecc_strength) |
	        ECC_CTRL_ECC_CAP(p_nfc->setup.ecc_strength);

	info("ecc cap=%x\n",ECC_CTRL_ECC_CAP(p_nfc->setup.ecc_strength));
	/* Since we've now completed the configuration, we need to force it to
	* be written to the NFC, else the caching in config_etx will leave
	* the etx_config values written since nand_scan_ident unwritten. */
	etx_config(&p_nfc->config, NULL);
	
	return ret;

}

void board_nand_init()
{
	int ret;
	ret = brite_nand_init();
	if (ret < 0)
		puts("brite_nand_init fail\n");

	ret = brite_nf_probe();
	if (ret)
	{
		puts("brite_nand probe fail\n");
	}

}

