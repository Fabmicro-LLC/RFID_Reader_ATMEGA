#ifndef __MFCR522_H__
#define __MFCR522_H__

#include <stdint.h>

/*
	MCFR522 Registers 
*/
#define REG_CMD                 0x01
#define REG_COM_IEN             0x02
#define REG_DIV_IEN             0x03
#define REG_COM_IRQ             0x04
#define REG_DIV_IRQ             0x05
#define REG_ERROR               0x06
#define REG_STATUS1             0x07
#define REG_STATUS2             0x08
#define REG_FIFO_DATA           0x09
#define REG_FIFO_LEVEL          0x0a
#define REG_FIFO_WATER          0x0b
#define REG_CONTROL             0x0c
#define REG_BIT_FRAMING         0x0d
#define REG_COLLISION           0x0e

#define REG_MODE                0x11
#define REG_TX_MODE             0x12
#define REG_RX_MODE             0x13
#define REG_TX_CONTROL          0x14
#define REG_TX_ASK              0x15
#define REG_TX_SEL              0x16
#define REG_RX_SEL              0x17
#define REG_RX_THRESHOLD        0x18

#define REG_CRC_RESULT_H        0x21
#define REG_CRC_RESULT_L        0x22
#define REG_MODE_WIDTH          0x24
#define REG_RF_CFG              0x26
#define REG_GS_N		0x27
#define REG_CW_GS_P		0x28
#define REG_MOD_GS_P		0x29
#define REG_T_MODE              0x2a
#define REG_T_PRESCALER         0x2b
#define REG_T_RELOAD_H          0x2c
#define REG_T_RELOAD_L          0x2d
#define REG_T_COUNTER_H         0x2e
#define REG_T_COUNTER_L         0x2f

/*
	MCFR522 Commands 
*/
#define CMD_IDLE                0x00
#define CMD_MEM                 0x01
#define CMD_GEN_RAND_ID         0x02
#define CMD_CALC_CRC            0x03
#define CMD_TRANSMIT            0x04
#define CMD_NO_CMD_CHANGE       0x07
#define CMD_RECEIVE             0x0a
#define CMD_TRANSCEIVE          0x0c
#define CMD_MF_AUTH             0x0e
#define CMD_SOFT_RESET          0x0f



/* 
	The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
*/
#define PICC_CMD_REQA		0x26 // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
#define PICC_CMD_WUPA		0x52 // Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
#define PICC_CMD_CT		0x88 // Cascade Tag. Not really a command, but used during anti collision.
#define PICC_CMD_SEL_CL1	0x93 // Anti collision/Select, Cascade Level 1
#define PICC_CMD_SEL_CL2	0x95 // Anti collision/Select, Cascade Level 2
#define PICC_CMD_SEL_CL3	0x97 // Anti collision/Select, Cascade Level 3
#define PICC_CMD_HLTA		0x50 // HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
#define PICC_CMD_RATS		0xE0 // Request command for Answer To Reset.
// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
// The read/write commands can also be used for MIFARE Ultralight.
#define PICC_CMD_MF_AUTH_KEY_A	0x60 // Perform authentication with Key A
#define PICC_CMD_MF_AUTH_KEY_B	0x61 // Perform authentication with Key B
#define PICC_CMD_MF_READ	0x30 // Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
#define PICC_CMD_MF_WRITE	0xA0 // Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
#define PICC_CMD_MF_DECREMENT	0xC0 // Decrements the contents of a block and stores the result in the internal data register.
#define PICC_CMD_MF_INCREMENT	0xC1 // Increments the contents of a block and stores the result in the internal data register.
#define PICC_CMD_MF_RESTORE	0xC2 // Reads the contents of a block into the internal data register.
#define PICC_CMD_MF_TRANSFER	0xB0 // Writes the contents of the internal data register to a block.
// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
#define PICC_CMD_UL_WRITE	0xA2 // Writes one 4 byte page to the PICC.

#define	MF_KEY_SIZE		6	/* A Mifare Crypto1 key is 6 bytes */
#define	MF_ACK			0x0a	/* The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK. */

/*
	PICC types we can detect. Remember to update mfrc_GetTypeName() if you add more.
*/
enum {
	PICC_TYPE_UNKNOWN,
	PICC_TYPE_ISO_14443_4,	// PICC compliant with ISO/IEC 14443-4 
	PICC_TYPE_ISO_18092, 	// PICC compliant with ISO/IEC 18092 (NFC)
	PICC_TYPE_MIFARE_MINI,	// MIFARE Classic protocol, 320 bytes
	PICC_TYPE_MIFARE_1K,	// MIFARE Classic protocol, 1KB
	PICC_TYPE_MIFARE_4K,	// MIFARE Classic protocol, 4KB
	PICC_TYPE_MIFARE_UL,	// MIFARE Ultralight or Ultralight C
	PICC_TYPE_MIFARE_PLUS,	// MIFARE Plus
	PICC_TYPE_MIFARE_DESFIRE,	// MIFARE DESFire
	PICC_TYPE_TNP3XXX,	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
	PICC_TYPE_NOT_COMPLETE	= 0xff
};

typedef struct {
        uint8_t size;           /* Number of bytes in the UID. 4, 7 or 10. */
        uint8_t uidByte[10];
        uint8_t sak;            /* The SAK (Select acknowledge) byte returned from the PICC after successful selection. */
} Uid;

typedef struct {
	uint8_t keyByte[MF_KEY_SIZE];
} MIFARE_Key;

extern MIFARE_Key mfrc522_knownKeys[];
extern int MF_NR_KNOWN_KEYS;

int mfrc522_read(uint8_t reg);
int mfrc522_read_array(uint8_t reg, uint8_t *values, uint8_t size, uint8_t rxAlign);
int mfrc522_write(uint8_t reg, uint8_t val);
int mfrc522_write_array(uint8_t reg, uint8_t *data, uint8_t size);
int mfrc522_Init(void);
int mfrc522_PICC_IsNewCardPresent(void);
int8_t mfrc522_PCD_CalculateCRC(uint8_t *data, /* In: Pointer to the data to transfer to the FIFO for CRC calculation. */
                            uint8_t size, /* In: The number of bytes to transfer. */
                            uint8_t *result /* Out: Pointer to result buffer. Result is written to result[0..1], low byte first. */
);
int mfrc522_PCD_CommunicateWithPICC(uint8_t cmd, /* command to execute */
                                   uint8_t waitIRq, /* The bits in the ComIrqReg register that signals successful completion */
                                   uint8_t *sendData, /* Pointer to the data to transfer to the FIFO. */
                                   uint8_t sendLen, /* Number of bytes to transfer to the FIFO. */
                                   uint8_t *backData, /*buffer if data should be read back after executing the command */
                                   uint8_t *backLen, /* Int: Max number of bytes. Out: number of bytes returned */
                                   uint8_t *validBits,  /* In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.*/
                                   uint8_t rxAlign, /* In: Defines the bit position in backData[0] for the first bit received. 
                                                        Default 0. */
                                   uint8_t checkCRC /* If > 0, last two bytes of the response is assumed to be a CRC_A that
                                                        must be validated. */
);
int mfrc522_PICC_Select(Uid *uid, /* Pointer to Uid struct. Normally output, but can also be used to supply a known UID. */
                        uint8_t validBits);
int mfrc522_PICC_REQA_or_WUPA(uint8_t cmd, uint8_t *backBuffer, uint8_t *backSize);
int mfrc522_PICC_GetType(uint8_t sak /* The SAK byte returned from PICC_Select(). */);
const char *mfrc522_PICC_GetTypeName(uint8_t piccType /* One of the PICC_TYPE enums. */);
int mfrc522_PICC_HaltA(void);
void mfrc522_PCD_StopCrypto1(void);
int mfrc522_PCD_Authenticate(uint8_t command, /* PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B */
                        uint8_t blockAddr, /* The block number. See numbering in the comments in the .h file. */
                        MIFARE_Key *key, /* Pointer to the Crypteo1 key to use (6 bytes) */
                        Uid *uid /* Pointer to Uid struct. The first 4 bytes of the UID is used. */
);
int mfrc522_MIFARE_Read(uint8_t blockAddr, /* MIFARE Classic: The block (0-0xff) number.
                                              MIFARE Ultralight: The first page to return data from. */
                        uint8_t *buffer, /* Buffer to store the data to */
                        uint8_t *bufferSize /* Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK. */
);



#endif // __MFCR5222_H__

