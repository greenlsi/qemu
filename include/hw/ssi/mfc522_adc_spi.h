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

#define TYPE_MFRC522_SPI "mfrc522"
#define MFRC522_SPI(obj)      OBJECT_CHECK(mfrc522_spi_s, (obj), TYPE_MFRC522_SPI)

#define SPI_REGISTERS_LEN (0x0018/4)
#define ID(a) (a/sizeof(uint32_t))


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

typedef enum {
CS            = 0x0000, /*!< SPI Master Control and Status */
FIFO          = 0x0004, /*!< SPI Master TX and RX FIFOs */
CLK           = 0x0008, /*!< SPI Master Clock Divider */
DLEN          = 0x000c, /*!< SPI Master Data Length */
LTOH          = 0x0010, /*!< SPI LOSSI mode TOH */
DC            = 0x0014, /*!< SPI DMA DREQ Controls */
} SPIRegisters;


#define TYPE_MAX1246_SPI "bcm2835-adc"
#define MAX1246_SPI(obj)      OBJECT_CHECK(max1246_spi_s, (obj), TYPE_MAX1246_SPI)

#define BCM2835_BAD_REG(addr) \
   return 0;

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


#define RFID_REGISTERS_LEN 0x40

typedef enum {
  // Page 0: Command and status
  //                    0x00      // reserved for future use
  CommandReg          = 0x01,     // starts and stops command execution
  ComIEnReg           = 0x02,     // enable and disable interrupt request control bits
  DivIEnReg           = 0x03,     // enable and disable interrupt request control bits
  ComIrqReg           = 0x04,     // interrupt request bits
  DivIrqReg           = 0x05,     // interrupt request bits
  ErrorReg            = 0x06,     // error bits showing the error status of the last command executed 
  Status1Reg          = 0x07,     // communication status bits
  Status2Reg          = 0x08,     // receiver and transmitter status bits
  FIFODataReg         = 0x09,     // input and output of 64 byte FIFO buffer
  FIFOLevelReg        = 0x0A,     // number of bytes stored in the FIFO buffer
  WaterLevelReg       = 0x0B,     // level for FIFO underflow and overflow warning
  ControlReg          = 0x0C,     // miscellaneous control registers
  BitFramingReg       = 0x0D,     // adjustments for bit-oriented frames
  CollReg             = 0x0E,     // bit position of the first bit-collision detected on the RF interface
  //                    0x0F      // reserved for future use
  
  // Page 1: Command
  //                    0x10      // reserved for future use
  ModeReg             = 0x11,     // defines general modes for transmitting and receiving 
  TxModeReg           = 0x12,     // defines transmission data rate and framing
  RxModeReg           = 0x13,     // defines reception data rate and framing
  TxControlReg        = 0x14,     // controls the logical behavior of the antenna driver pins TX1 and TX2
  TxASKReg            = 0x15,     // controls the setting of the transmission modulation
  TxSelReg            = 0x16,     // selects the internal sources for the antenna driver
  RxSelReg            = 0x17,     // selects internal receiver settings
  RxThresholdReg      = 0x18,     // selects thresholds for the bit decoder
  DemodReg            = 0x19,     // defines demodulator settings
  //                    0x1A      // reserved for future use
  //                    0x1B      // reserved for future use
  MfTxReg             = 0x1C,     // controls some MIFARE communication transmit parameters
  MfRxReg             = 0x1D,     // controls some MIFARE communication receive parameters
  //                    0x1E      // reserved for future use
  SerialSpeedReg      = 0x1F,     // selects the speed of the serial UART interface
  
  // Page 2: Configuration
  //                    0x20      // reserved for future use
  CRCResultRegH       = 0x21,     // shows the MSB and LSB values of the CRC calculation
  CRCResultRegL       = 0x22,
  //                    0x32      // reserved for future use
  ModWidthReg         = 0x24,     // controls the ModWidth setting?
  //                    0x25      // reserved for future use
  RFCfgReg            = 0x26,     // configures the receiver gain
  GsNReg              = 0x27,     // selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
  CWGsPReg            = 0x28,     // defines the conductance of the p-driver output during periods of no modulation
  ModGsPReg           = 0x29,     // defines the conductance of the p-driver output during periods of modulation
  TModeReg            = 0x2A,     // defines settings for the internal timer
  TPrescalerReg       = 0x2B,     // the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
  TReloadRegH         = 0x2C,     // defines the 16-bit timer reload value
  TReloadRegL         = 0x2D,
  TCounterValueRegH   = 0x2E,     // shows the 16-bit timer value
  TCounterValueRegL   = 0x2F,
  
  // Page 3: Test Registers
  //              0x30      // reserved for future use
  TestSel1Reg         = 0x31,     // general test signal configuration
  TestSel2Reg         = 0x32,     // general test signal configuration
  TestPinEnReg        = 0x33,     // enables pin output driver on pins D1 to D7
  TestPinValueReg     = 0x34,     // defines the values for D1 to D7 when it is used as an I/O bus
  TestBusReg          = 0x35,     // shows the status of the internal test bus
  AutoTestReg         = 0x36,     // controls the digital self test
  VersionReg          = 0x37,     // shows the software version
  AnalogTestReg       = 0x38,     // controls the pins AUX1 and AUX2
  TestDAC1Reg         = 0x39,     // defines the test value for TestDAC1
  TestDAC2Reg         = 0x3A,     // defines the test value for TestDAC2
  TestADCReg          = 0x3B      // shows the value of ADC I and Q channels
  //                    0x3C      // reserved for production tests
  //                    0x3D      // reserved for production tests
  //                    0x3E      // reserved for production tests
  //                    0x3F      // reserved for production 
} MFRC522Registers;

// Commands MFRC522
#define PCD_Idle               0x00   // no action, cancels current command execution
#define PCD_Mem                0x01   // stores 25 bytes into the internal buffer
#define PCD_GenerateRandomID   0x02   // generates a 10-byte random ID number
#define PCD_CalcCRC            0x03   // activates the CRC coprocessor or performs a self test
#define PCD_Transmit           0x04   // transmits data from the FIFO buffer
#define PCD_NoCmdChange        0x07   // no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
#define PCD_Receive            0x08   // activates the receiver circuits
#define PCD_Transceive         0x0C   // transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
#define PCD_MFAuthent          0x0E   // performs the MIFARE standard authentication as a reader
#define PCD_SoftReset          0x0F    // resets the MFRC522

// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
#define PICC_CMD_REQA             0x26   // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
#define PICC_CMD_WUPA             0x52   // Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
#define PICC_CMD_CT               0x88   // Cascade Tag. Not really a command, but used during anti collision.
#define PICC_CMD_SEL_CL1          0x93   // Anti collision/Select, Cascade Level 1
#define PICC_CMD_SEL_CL2          0x95   // Anti collision/Select, Cascade Level 2
#define PICC_CMD_SEL_CL3          0x97   // Anti collision/Select, Cascade Level 3
#define PICC_CMD_HLTA             0x50   // HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
// The commands used for MIFARE Classic (from http://www.nxp.com/documents/data_sheet/MF1S503x.pdf, Section 9)
// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
// The read/write commands can also be used for MIFARE Ultralight.
#define PICC_CMD_MF_AUTH_KEY_A    0x60   // Perform authentication with Key A
#define PICC_CMD_MF_AUTH_KEY_B    0x61   // Perform authentication with Key B
#define PICC_CMD_MF_READ          0x30   // Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
#define PICC_CMD_MF_WRITE         0xA0   // Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
#define PICC_CMD_MF_DECREMENT     0xC0   // Decrements the contents of a block and stores the result in the internal data register.
#define PICC_CMD_MF_INCREMENT     0xC1   // Increments the contents of a block and stores the result in the internal data register.
#define PICC_CMD_MF_RESTORE       0xC2   // Reads the contents of a block into the internal data register.
#define PICC_CMD_MF_TRANSFER      0xB0   // Writes the contents of the internal data register to a block.
// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
#define PICC_CMD_UL_WRITE         0xA2    // Writes one 4 byte page to the PICC.

/*
// State Machine Structs
typedef struct fsm_t fsm_t;

typedef int (*fsm_input_func_t) (fsm_t*);
typedef void (*fsm_output_func_t) (fsm_t*);

typedef struct fsm_trans_t {
  int orig_state;
  fsm_input_func_t in;
  int dest_state;
  fsm_output_func_t out;
} fsm_trans_t;

struct fsm_t {
  int current_state;
  fsm_trans_t* tt;
  void* user_data;
};
*/

typedef struct uid{
    uint8_t    size;         // Number of bytes in the UID. 4, 7 or 10.
    uint8_t    uidByte[10];
    uint8_t    sak;          // The SAK (Select acknowledge) byte returned from the PICC after successful selection.
  } uid;

typedef struct spi_channel_s {
      void (*reset)();
      void (*init)();
      void (*write)(void *opaque, uint64_t value, unsigned size);
      uint64_t (*read)(void *opaque, unsigned size);
      void* opaque;
} spi_channel_s;
 

typedef struct raspi_spi_s{
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t registers_spi[SPI_REGISTERS_LEN];

    spi_channel_s* ch[2];
} raspi_spi_s;

typedef struct mfrc522_spi_s{
    SoupServer *server;
    SoupWebsocketConnection* connection;

    uint8_t data_in[4];
    uint8_t data_out[4];
    int index_in;
    int index_out_w;
    int index_out_r;

    uint8_t registers_rfid[RFID_REGISTERS_LEN];

    int card_present;
    
    uint8_t reg;
    uint8_t rw;
    int finish;
    int read_done;

    uint8_t fifo_rfid [64];
    int i_fifo_rfid_w;
    int i_fifo_rfid_r;

    struct uid uid_1;

    //fsm_t* fsm;

    //qemu_irq irq;

} mfrc522_spi_s;

typedef struct max1246_spi_s{
    SoupServer *server;
    SoupWebsocketConnection* connection;

    uint32_t data_in;
    uint32_t data_out;
    int index_in;
    int index_out;
    float CHX[4];
    int ch;
    //qemu_irq irq;

} max1246_spi_s;


#endif


