#define BCM2835_GPIO_H

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include <glib.h>
#include <gio/gio.h>
#include <glib/gstdio.h>
#include <libsoup/soup.h>

#define TYPE_BCM2835_GPIO "bcm2835-gpio"
#define BCM2835_GPIO(obj)      OBJECT_CHECK(bcm2835_gpif_s, (obj), TYPE_BCM2835_GPIO)

#define GPIO_REGISTERS_LEN (0x00A0/4)//por q voy de 4 en 4, por eso son 28hex en total los registros
#define ID(a) (a/sizeof(uint32_t))

#define BCM2835_BAD_REG(addr) \
   return 0;

//#define RPI_IO_BASE	0x3F000000
//#define GPIO_BASE	(RPI_IO_BASE + 0x00200000)

typedef enum {

	GPFSEL0 = 0x0000, //!< GPIO Function Select 0 //0
	GPFSEL1 = 0x0004, //!< GPIO Function Select 1 //1
	GPFSEL2 = 0x0008, //!< GPIO Function Select 2//2
	GPFSEL3 = 0x000c, //!< GPIO Function Select 3 
	GPFSEL4 = 0x0010, //!< GPIO Function Select 4 
	GPFSEL5 = 0x0014, //< GPIO Function Select 5 

	GPSET0 = 0x001c, //!< GPIO Pin Output Set 0 //7
	GPSET1 = 0x0020, //!< GPIO Pin Output Set 1 //8

	GPCLR0 = 0x0028, //!< GPIO Pin Output Clear 0 //10
	GPCLR1 = 0x002c, //!< GPIO Pin Output Clear 1//11

	GPLEV0 = 0x0034, //!< GPIO Pin Level 0 //13
	GPLEV1 = 0x0038, //!< GPIO Pin Level 1 //14

	GPEDS0 = 0x0040, //!< GPIO Pin Event Detect Status 0 //16
	GPEDS1 = 0x0044, //!< GPIO Pin Event Detect Status 1 

	GPREN0 = 0x004c, //!< GPIO Pin Rising Edge Detect Enable 0 //19
	GPREN1 = 0x0050, //!< GPIO Pin Rising Edge Detect Enable 1 

	GPFEN0 = 0x0058, //!< GPIO Pin Falling Edge Detect Enable 0 //22
	GPFEN1 = 0x005c, //!< GPIO Pin Falling Edge Detect Enable 1 

	GPHEN0 = 0x0064, //!< GPIO Pin High Detect Enable 0 
	GPHEN1 = 0x0068, //!< GPIO Pin High Detect Enable 1 
	GPLEN0 = 0x0070, //!< GPIO Pin Low Detect Enable 0 
	GPLEN1 = 0x0074, //!< GPIO Pin Low Detect Enable 1 

	GPAREN0 = 0x007c, //!< GPIO Pin Async. Rising Edge Detect 0 
	GPAREN1 = 0x0080, //!< GPIO Pin Async. Rising Edge Detect 1 

	GPAFEN0 = 0x0088, //!< GPIO Pin Async. Falling Edge Detect 0 
	GPAFEN1 = 0x008c, //!< GPIO Pin Async. Falling Edge Detect 1 

	GPPUD = 0x0094, //!< GPIO Pin Pull-up/down Enable 
	GPPUDCLK0 = 0x0098, //!< GPIO Pin Pull-up/down Enable Clock 0 
	GPPUDCLK1 = 0x009c, //!< GPIO Pin Pull-up/down Enable Clock 1 

}gpio_registers;


typedef struct bcm2835_gpio_s {
	SoupWebsocketConnection* connection; 

	qemu_irq irq[4];

	uint32_t registers[GPIO_REGISTERS_LEN];
} bcm2835_gpio_s;


typedef struct bcm2835_gpif_s {
	SysBusDevice parent_obj;
	  
	MemoryRegion iomem;
	SoupServer *server; 

	struct bcm2835_gpio_s bcm2835_1;
} bcm2835_gpif_s;
