/*
* Copyright (C) 2012-2013 GSI (www.gsi.de)
* Author: Cesar Prados <c.prados@gsi.de>
*
* Released according to the GNU GPL, version 2 or any later version
*
* Driver for VME VME board.
*/
#ifndef __VME_H__
#define __VME_H__

#include <linux/firmware.h>
#include <vmebus.h>
#include <wishbone.h>

#define VME_WB "vme_wb"
#define VME_MAX_DEVICES        32
#define VME_DEFAULT_IDX { [0 ... (VME_MAX_DEVICES-1)] = -1 }
#define VME_IRQ_LEVEL	2
#define VME_VENDOR_ID		0x80031

#define VME_VENDOR_ID_OFFSET	0x24

enum vme_map_win {
	MAP_CR_CSR = 0,	/* CR/CSR */
	MAP_REG		/* A32 space */
};

/* Our device structure */
struct vme_dev {
	int			lun;
	int			slot;
	uint32_t		vmebase;
	int			vector;
	int			level;

	//struct device		*dev;
	char			driver[16];
	char			description[80];
	struct vme_mapping	*map[2];

	/* struct work_struct	work; */
	unsigned long		irqcount;
};


struct vme_wb_dev {
	
	struct device	*vme_dev;
	struct wishbone wb;
	struct vme_dev 	vme_res;
	struct mutex 	mutex;
};

/* Functions and data in vme_wb.c */
extern void vme_setup_csr_fa0(void *base, u32 vme, unsigned vector,
			       unsigned level);
extern int vme_unmap_window(struct vme_wb_dev *vetar, enum vme_map_win map_type);
extern int vme_map_window( struct vme_wb_dev *vetar, enum vme_map_win map_type);


/* VME CSR offsets */
#define FUN0ADER	0x7FF63
#define INT_LEVEL	0x7ff5b
#define INTVECTOR	0x7ff5f
#define WB_32_64	0x7ff33
#define BIT_SET_REG	0x7FFFB
#define BIT_CLR_REG	0x7FFF7
#define WB32		1
#define WB64		0
#define RESET_CORE	0x80
#define ENABLE_CORE	0x10

#endif /* __VME_H__ */
