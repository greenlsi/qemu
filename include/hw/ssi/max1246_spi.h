#ifndef MAX1246_SPI_H
#define MAX1246_SPI_H

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include <glib.h>
#include <gio/gio.h>
#include <glib/gstdio.h>
#include <libsoup/soup.h>

#define TYPE_MAX1246_SPI "bcm2835-adc"
#define MAX1246_SPI(obj)      OBJECT_CHECK(max1246_spi_s, (obj), TYPE_MAX1246_SPI)

#define MAX1246_REGISTERS_LEN (0x0018/4)
#define ID(a) (a/sizeof(uint32_t))

#define BCM2835_BAD_REG(addr) \
   return 0;

// Register masks for SPI0_CS
#define CS_LEN_LONG             0x02000000 ///< Enable Long data word in Lossi mode if DMA_LEN is set
#define CS_DMA_LEN              0x01000000 ///< Enable DMA mode in Lossi mode
#define CS_CSPOL2               0x00800000 ///< Chip Select 2 Polarity
#define CS_CSPOL1               0x00400000 ///< Chip Select 1 Polarity
#define CS_CSPOL0               0x00200000 ///< Chip Select 0 Polarity
#define CS_RXF                  0x00100000 ///< RXF - RX FIFO Full
#define CS_RXR                  0x00080000 ///< RXR RX FIFO needs Reading ( full)
#define CS_TXD                  0x00040000 ///< TXD TX FIFO can accept Data
#define CS_RXD                  0x00020000 ///< RXD RX FIFO contains Data
#define CS_DONE                 0x00010000 ///< Done transfer Done
#define CS_TE_EN                0x00008000 ///< Unused
#define CS_LMONO                0x00004000 ///< Unused
#define CS_LEN                  0x00002000 ///< LEN LoSSI enable
#define CS_REN                  0x00001000 ///< REN Read Enable
#define CS_ADCS                 0x00000800 ///< ADCS Automatically Deassert Chip Select
#define CS_INTR                 0x00000400 ///< INTR Interrupt on RXR
#define CS_INTD                 0x00000200 ///< INTD Interrupt on Done
#define CS_DMAEN                0x00000100 ///< DMAEN DMA Enable
#define CS_TA                   0x00000080 ///< Transfer Active
#define CS_CSPOL                0x00000040 ///< Chip Select Polarity
#define CS_CLEAR                0x00000030 ///< Clear FIFO Clear RX and TX
#define CS_CLEAR_RX             0x00000020 ///< Clear FIFO Clear RX 
#define CS_CLEAR_TX             0x00000010 ///< Clear FIFO Clear TX 
#define CS_CPOL                 0x00000008 ///< Clock Polarity
#define CS_CPHA                 0x00000004 ///< Clock Phase
#define CS_CS                   0x00000003 ///< Chip Select

// Register masks for first byte of data_in
#define START                   0x80 // The first logic '1' bit afer CS goes low defines the beginning of the control byte
#define SEL2                    0x40 // Select which of the four channels are used for the conversion
#define SEL1                    0x20 // Select which of the four channels are used for the conversion
#define SEL0                    0x10 // Select which of the four channels are used for the conversion 
#define UNI_BIP                 0x08 // 1 = Unipolar, 0= Bipolar
#define SGL_DIF                 0x04 // 1 = Single ended, 0 =Differential
#define PD1                     0x02 // Selects clock and power-down modes
#define PD0                     0x01 // Selects clock and power-down modes

#define SEL_MASK                0x70
#define PD_MASK                 0x03

#define CH0                     0x10 // Input Data for channel 0
#define CH1                     0x50 // Input Data for channel 1
#define CH2                     0x20 // Input Data for channel 2
#define CH3                     0x60 // Input Data for channel 3
#define DIFF1                   0x10 // Differential input 1
#define DIFF2                   0x20 // Differential input 2
#define DIFF3                   0x50 // Differential input 3
#define DIFF4                   0x60 // Differential input 4


#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

typedef enum {
CS =            0x0000, /*!< SPI Master Control and Status */
FIFO =          0x0004, /*!< SPI Master TX and RX FIFOs */
CLK =           0x0008, /*!< SPI Master Clock Divider */
DLEN =          0x000c, /*!< SPI Master Data Length */
LTOH =          0x0010, /*!< SPI LOSSI mode TOH */
DC =            0x0014, /*!< SPI DMA DREQ Controls */
} MAX1246Registers;


typedef struct max1246_spi_s{
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t data_in;
    uint32_t data_out;
    int index_in;
    int index_out;
    float CHX[4];
    int ch;
    uint32_t registers[MAX1246_REGISTERS_LEN];
    //qemu_irq irq;

    SoupServer *server;
    SoupWebsocketConnection* connection;
} max1246_spi_s;


#endif


