#include "mfrc522.h"
#include "main.h"

//#define	MRFC522_DEBUG	1

#ifdef	MRFC522_DEBUG
	#define debug(...) printf(__VA_ARGS__)
#else
	#define	debug(...) {}
#endif

int mfrc522_PICC_GetType(uint8_t sak /* The SAK byte returned from PICC_Select(). */
) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf 
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} 

const char *mfrc522_PICC_GetTypeName(uint8_t piccType /* One of the PICC_TYPE enums. */
) {
	switch (piccType) {
		case PICC_TYPE_ISO_14443_4:	return "PICC compliant with ISO/IEC 14443-4";
		case PICC_TYPE_ISO_18092:	return "PICC compliant with ISO/IEC 18092 (NFC)";
		case PICC_TYPE_MIFARE_MINI:	return "MIFARE Mini, 320 bytes";
		case PICC_TYPE_MIFARE_1K:	return "MIFARE 1KB";
		case PICC_TYPE_MIFARE_4K:	return "MIFARE 4KB";
		case PICC_TYPE_MIFARE_UL:	return "MIFARE Ultralight or Ultralight C";
		case PICC_TYPE_MIFARE_PLUS:	return "MIFARE Plus";
		case PICC_TYPE_MIFARE_DESFIRE:	return "MIFARE DESFire";
		case PICC_TYPE_TNP3XXX:		return "MIFARE TNP3XXX";
		case PICC_TYPE_NOT_COMPLETE:	return "SAK indicates UID is not complete.";
		case PICC_TYPE_UNKNOWN:
		default:			return "Unknown type";
	}
} 

int mfrc522_write(uint8_t reg, uint8_t val)
{
	uint8_t c = 0;

	/* Write reg address byte */
	usart1_putchar(reg & 0x7f, NULL);

	/* Read reg address echoed back */
	int rc = usart1_getchar(&c);	

	/* Check if data's been read and both addresses match, otherwise error state */
	if(rc == 0 || c != (reg & 0x7f)) {
		debug("MFRC522_DEBUG: write addr mismatch: 0x%02x != 0x%02x, rc = %d\r\n", reg & 0x7f, c, rc);
		return -31;
	}

	/* Ok, now write value byte */
	usart1_putchar(val, NULL);

	return 0;
}

int mfrc522_write_array(uint8_t reg, uint8_t *data, uint8_t size)
{
	int i;
	uint8_t error;

	for(i = 0; i < size; i++)
		if (error = mfrc522_write(reg, data[i]))
			return error;

	return 0;
}

int mfrc522_read(uint8_t reg)
{
	uint8_t c = 0;
	int8_t rc;

	usart1_putchar(reg | 0x80, NULL);
	rc = usart1_getchar(&c);	

	return rc > 0 ? c : -1;
}

int mfrc522_read_array(uint8_t reg, uint8_t *values, uint8_t size, uint8_t rxAlign)
{
	int i = 0;
 
	if (rxAlign) {
		/* Only update bit positions rxAlign..7 in values[0]
		   Create bit mask for bit positions rxAlign..7 */
		uint8_t mask = (0xFF << rxAlign) & 0xFF;

		int value = mfrc522_read(reg);

		if(value < 0)
			return -16;

		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		i++;
	}

	/* Read the rest buyes */
	for(; i < size; i++)
		values[i] = mfrc522_read(reg);
	
}

int mfrc522_Init(void)
{
	uint8_t val;
	int rc = 0;

	/* Hardware reset sequence */
	gpio_set_bit(&PORTA, PA0, 0);
	delay_ms(50);
	gpio_set_bit(&PORTA, PA0, 1);
	delay_ms(50);

	/* Read junk data from serial buffer */
	while(usart1_getchar(&val) != 0);

	/* Reset baud rates */
	rc += mfrc522_write(REG_TX_MODE, 0x00);
	rc += mfrc522_write(REG_RX_MODE, 0x00);
	rc += mfrc522_write(REG_RF_CFG, 0x78); /* [6:4] = RxGain[2:0]: 18,23,18,23,33,38,43,48 dB */
	rc += mfrc522_write(REG_GS_N, 0xfc); /* [7:4] = CWGsN[3:0] output conductance, [3:0] = ModGsN[3:0] modulation index */
	rc += mfrc522_write(REG_CW_GS_P, 0x3f); /* [5:0] = CWGsP[5:0] */
//	rc += mfrc522_write(REG_MOD_GS_P, 0x3f); /* [5:0] = ModGsP[5:0] */

	/* Reset Mode Width */
	rc += mfrc522_write(REG_MODE_WIDTH, 0x26);

	/* 
	  When communicating with a PICC we need a timeout if something goes wrong.
	  f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	  TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	*/
	rc += mfrc522_write(REG_T_MODE, 0x80); /* TAuto=1; timer starts automatically at the end of 
					  the transmission in all communication modes at all speeds */

	rc += mfrc522_write(REG_T_PRESCALER, 0xA8); /* TPreScaler = TModeReg[3..0]:TPrescalerReg,
						ie 0x0A8 = 169 => f_timer=40.1kHz, ie a timer period of ~25μs. */

	rc += mfrc522_write(REG_T_RELOAD_H, 0x03); /* Reload timer with 0x3E8 = 1000, ie 25ms before timeout. */
	rc += mfrc522_write(REG_T_RELOAD_L, 0xE8);

	rc += mfrc522_write(REG_TX_ASK, 0x40);	/* Default 0x00. Force a 100 % ASK modulation independent
					   of the ModGsPReg register setting */

	rc += mfrc522_write(REG_MODE, 0x3D);	/* Default 0x3F. Set the preset value for the CRC coprocessor
					for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4) */

	/* Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset) */
	val = mfrc522_read(REG_TX_CONTROL);
	if ((val & 0x3) != 0x03)
		mfrc522_write(REG_TX_CONTROL, val | 0x03);

	return rc;
}


int8_t mfrc522_CalculateCRC(uint8_t *data, /* In: Pointer to the data to transfer to the FIFO for CRC calculation. */
			    uint8_t size, /* In: The number of bytes to transfer. */
			    uint8_t *result /* Out: Pointer to result buffer. Result is written to result[0..1], low byte first. */
) {
	mfrc522_write(REG_CMD, CMD_IDLE); /* Stop any active command. */
	mfrc522_write(REG_DIV_IRQ, 0x04); /* Clear the CRCIRq interrupt request bit */
	mfrc522_write(REG_FIFO_LEVEL, 0x80); /* FlushBuffer = 1, FIFO initialization */
	mfrc522_write_array(REG_FIFO_DATA, data, size);	/* Write data to the FIFO */
	mfrc522_write(REG_CMD, CMD_CALC_CRC); /* Start CRC calculation */
	
	/* Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs. */

	for (uint16_t i = 5000; i > 0; i--) {
		/* DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved */
		uint8_t n = mfrc522_read(REG_DIV_IRQ);
		if (n & 0x04) { /* CRCIRq bit set - calculation done */
			mfrc522_write(REG_CMD, CMD_IDLE); /* Stop calculating CRC for new content in the FIFO. */

			/* Transfer the result from the registers to the result buffer */
			result[0] = mfrc522_read(REG_CRC_RESULT_L);
			result[1] = mfrc522_read(REG_CRC_RESULT_H);
			return 0;
		}
	}

	/* 89ms passed and nothing happend. Communication with the MFRC522 might be down. */
	return -12; // Timeout
}

int mfrc522_CommunicateWithPICC(uint8_t cmd, /* command to execute */
				   uint8_t waitIRq, /* The bits in the ComIrqReg register that signals successful completion */
				   uint8_t *sendData, /* Pointer to the data to transfer to the FIFO. */
				   uint8_t sendLen, /* Number of bytes to transfer to the FIFO. */
				   uint8_t *backData, /*buffer if data should be read back after executing the command */
				   uint8_t *backLen, /* Int: Max number of bytes. Out: number of bytes returned */
				   uint8_t *validBits,	/* In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.*/
				   uint8_t rxAlign, /* In: Defines the bit position in backData[0] for the first bit received. 
							Default 0. */
				   uint8_t checkCRC /* If > 0, last two bytes of the response is assumed to be a CRC_A that
							must be validated. */
) {
	int i, n;

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;
		/* RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0] */
	
	mfrc522_write(REG_CMD, CMD_IDLE); /* Stop any active command */
	mfrc522_write(REG_COM_IRQ, 0x7F); /* Clear all seven interrupt request bits */
	mfrc522_write(REG_FIFO_LEVEL, 0x80); /* FlushBuffer = 1, FIFO initialization */
	mfrc522_write_array(REG_FIFO_DATA, sendData, sendLen); /* Copy sending data to FIFO */
	mfrc522_write(REG_BIT_FRAMING, bitFraming); /* Bit adjustments */
	mfrc522_write(REG_CMD, cmd); /* Execute the command finally */

	if (cmd == CMD_TRANSCEIVE)
		mfrc522_write(REG_BIT_FRAMING, mfrc522_read(REG_BIT_FRAMING) | 0x80);
			/* Set StartSend bit to initiate transmission */
	
	/* 
		Wait for the command to complete.
		In Init() we set the TAuto flag in TModeReg. This means the timer automatically starts
		when the PCD stops transmitting. 

		Each iteration of the do-while-loop takes ~1.97ms.
	*/

	for (i = 20; i > 0; i--) {
		n = mfrc522_read(REG_COM_IRQ);	
			/* ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq */

		if (n < 0) { 
			debug("MFRC522_ERROR: Serial protocol error reading reg 0x%02x\r\n", REG_COM_IRQ);
			return -3;
		}

		if (n & waitIRq) /* One of the interrupts that signal success has been set */
			break;

		if (n & 0x01) /* Timer interrupt - nothing received in 25ms */
			return -4;
	}

	/* ~40ms and nothing happend. Communication with the MFRC522 might be down. */
	if (i == 0) {
		uint16_t t_counter = (mfrc522_read(REG_T_COUNTER_H) << 8) |
				      mfrc522_read(REG_T_COUNTER_L);
		debug("MFRC522_ERROR: Communication timeout, MFRC522 might be down, ComIrqReg: 0x%02x, waitIRq: 0x%02x, Error: 0x%02x, TMode: 0x%02x, TCounter: 0x%04x\r\n", 
			n, waitIRq,
			mfrc522_read(REG_ERROR),
			mfrc522_read(REG_T_MODE),
			t_counter);
		return -5;
	}
	
	/* Stop now if any errors except collisions were detected. */
	int error = mfrc522_read(REG_ERROR); 
			/* ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr */

	if (error < 0) { 
		debug("MFRC522_ERROR: Serial protocol error reading reg 0x%02x\r\n", REG_ERROR);
		return -6;
	}

	if (error)
		debug("MFRC5222_INFO: REG_ERROR = 0x%02X\r\n", error & 0xff);

	if (error & 0x13)  { /* BufferOvfl ParityErr ProtocolErr */
		debug("MFRC522_ERROR: Communication protocol error bits: 0x%02x\r\n", error);
		return -7;
	}
  
	uint8_t _validBits = 0;
	
	/* If the caller wants data back, get it from the MFRC522 */
	if (backData && backLen) {
		n = mfrc522_read(REG_FIFO_LEVEL);	/* Number of bytes in the FIFO */

		if (n < 0) { 
			debug("MFRC522_ERROR: Serial protocol error reading reg 0x%02x\r\n", REG_FIFO_LEVEL);
			return -8;
		}

		if (n > *backLen) {
			debug("MFRC522_ERROR: FIFO len %d > back buffer len %d\r\n", n, *backLen);
			return -9;
		}

		*backLen = n; 

		int error = mfrc522_read_array(REG_FIFO_DATA, backData, n, rxAlign);

		if (error < 0) {
			debug("MFRC522_ERROR: Serial protocol error reading reg 0x%02x\r\n", REG_FIFO_DATA);
			return -10;
		}

		_validBits = mfrc522_read(REG_CONTROL) & 0x07;
				/* RxLastBits[2:0] indicates the number of valid bits in the last received byte.
				   If this value is 000b, the whole byte is valid. */
		if (validBits)
			*validBits = _validBits;
	}
	
	if (error & 0x08)
		return -11; /* Collision occured */
	
	/* Perform CRC_A validation if requested */
	if (backData && backLen && checkCRC) {
		/* In this case a MIFARE Classic NAK is not OK */
		if (*backLen == 1 && _validBits == 4)
			return -12; // STATUS_MIFARE_NACK;

		/* We need at least the CRC_A value and all 8 bits of the last byte must be received. */
		if (*backLen < 2 || _validBits != 0)
			return -13; // STATUS_CRC_WRONG;

		/* Verify CRC_A - do our own calculation and store the control in controlBuffer. */
		uint8_t controlBuffer[2];
		if(mfrc522_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]))
			return -14; // CRC calc error;

		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
			return -15; // STATUS_CRC_MISMATCH;
	}
	
	return 0;
}

int mfrc522_PICC_REQA_or_WUPA(uint8_t cmd, uint8_t *backBuffer, uint8_t *backSize)
{
	uint8_t validBits = 7; /* For REQA and WUPA we need the short frame format - transmit
				only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
				*/
	int error;
	uint8_t sendBuffer[2];

	if (!backBuffer || !backSize || *backSize < 2)
		return -1;

	sendBuffer[0] = cmd;

	mfrc522_write(REG_COLLISION, mfrc522_read(REG_COLLISION) & 0x80); /* clear collision bits */

	error = mfrc522_CommunicateWithPICC(CMD_TRANSCEIVE, 0x30 /*waitIRq = RxIRq | IdleIRq */, 
				sendBuffer, 1, backBuffer, backSize, &validBits, 0 /* rxAlign */, 0 /* checkCRC */);
	if (error) {
		debug("MFRC522: PICC error %d\r\n", error);
		return error;
	}

	if (*backSize != 2 || validBits != 0)
		return -2; /* ATQA must be exactly 16 bits */

	return 0;
}

int mfrc522_PICC_IsNewCardPresent(void)
{
	uint8_t buffer_ATQA[2];
	uint8_t buffer_size = sizeof(buffer_ATQA);
	uint8_t status;

	/* Reset baud rates */
	mfrc522_write(REG_TX_MODE, 0x00);
	mfrc522_write(REG_RX_MODE, 0x00);

	/* Reset Mode Width */
	mfrc522_write(REG_MODE_WIDTH, 0x26);

	return mfrc522_PICC_REQA_or_WUPA(PICC_CMD_REQA, buffer_ATQA, &buffer_size); 
}

int mfrc522_PICC_Select(Uid *uid, /* Pointer to Uid struct. Normally output, but can also be used to supply a known UID. */
			uint8_t validBits /* Number of known UID bits supplied in *uid. Normally 0. 
					     If set you must also supply uid->size. */
) {
	int result;
	uint8_t uidComplete, selectDone, useCascadeTag;
	uint8_t cascadeLevel = 1;
	uint8_t count, index;
	uint8_t uidIndex;	/* First index in uid->uidByte[] that is used in the current Cascade Level. */
	uint8_t currentLevelKnownBits; /* Number of known UID bits in the current Cascade Level. */
	uint8_t buffer[9];	/* SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A */
	uint8_t bufferUsed;	/* Number of bytes used in the buffer, ie number of bytes to transfer to the FIFO. */
	uint8_t rxAlign;	/* Used in BitFramingReg. Defines the bit position for the first bit received. */
	uint8_t txLastBits;	/* Used in BitFramingReg. Number of valid bits in the last transmitted byte. */ 
	uint8_t *responseBuffer, responseLength;
		
	/*
	Description of buffer structure:
	Byte 0: SEL 		Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	Byte 1: NVB		Number of Valid Bits (in complete command, not just the UID): 
				High nibble: complete bytes, Low nibble: Extra bits. 
	Byte 2: UID-data or CT	See explanation below. CT means Cascade Tag.
	Byte 3: UID-data
	Byte 4: UID-data
	Byte 5: UID-data
	Byte 6: BCC		Block Check Character - XOR of bytes 2-5
	Byte 7: CRC_A
	Byte 8: CRC_A
	The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.

	Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	========	=============	=====	=====	=====	=====
	4 bytes		1			uid0	uid1	uid2	uid3
	7 bytes		1			CT		uid0	uid1	uid2
	2			uid3	uid4	uid5	uid6
	10 bytes		1			CT		uid0	uid1	uid2
	2			CT		uid3	uid4	uid5
	3			uid6	uid7	uid8	uid9
	*/
	
	/* Sanity checks */
	if (validBits > 80) 
		return -41;
	
	/* Prepare MFRC522 */
	mfrc522_write(REG_COLLISION, mfrc522_read(REG_COLLISION) & 0x80);
		/* ValuesAfterColl=1 => Bits received after collision are cleared. */
	
	/* Repeat Cascade Level loop until we have a complete UID. */
	uidComplete = 0;
	while (!uidComplete) {
		/* Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2. */
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4; /* When we know that the UID has more than 4 bytes */
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7; /* When we know that the UID has more than 7 bytes */
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = 0; /* Never used in CL3. */
				break;
			
			default:
				return -42;
				break;
		}
		
		/* How many UID bits are known in this Cascade Level? */
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) 
			currentLevelKnownBits = 0;

		/* Copy the known bits from uid->uidByte[] to buffer[] */
		index = 2; /* destination index in buffer[] */
		if (useCascadeTag)
			buffer[index++] = PICC_CMD_CT;

		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0);
			/* The number of bytes needed to represent the known bits for this level. */

		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; 
				/* Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag */

			if (bytesToCopy > maxBytes)
				bytesToCopy = maxBytes;

			for (count = 0; count < bytesToCopy; count++)
				buffer[index++] = uid->uidByte[uidIndex + count];
		}

		/* Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits */
		if (useCascadeTag)
			currentLevelKnownBits += 8;
		
		/* Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations */
		selectDone = 0;
		while (!selectDone) {

			/* Find out how many bits and bytes to send and receive */
			if (currentLevelKnownBits >= 32) { /* All UID bits in this Cascade Level are known. This is a SELECT. */

				buffer[1] = 0x70; /* NVB - Number of Valid Bits: Seven whole bytes */

				/* Calculate BCC - Block Check Character */
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];

				/* Calculate CRC_A */
				result = mfrc522_CalculateCRC(buffer, 7, &buffer[7]);
				if (result < 0)
					return result;

				txLastBits = 0; /* 0 => All 8 bits are valid. */
				bufferUsed = 9;

				/* Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx) */
				responseBuffer	= &buffer[6];
				responseLength	= 3;

			} else { /* This is an ANTICOLLISION. */
				txLastBits = currentLevelKnownBits % 8;
				count = currentLevelKnownBits / 8; /* Number of whole bytes in the UID part. */
				index = 2 + count; /* Number of whole bytes: SEL + NVB + UIDs */
				buffer[1] = (index << 4) + txLastBits;	/* NVB - Number of Valid Bits */
				bufferUsed = index + (txLastBits ? 1 : 0); 

				/* Store response in the unused part of buffer */
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			/* Set bit adjustments */
			rxAlign = txLastBits; 
				/*Having a separate variable is overkill. But it makes the next line easier to read. */

			mfrc522_write(REG_BIT_FRAMING, (rxAlign << 4) + txLastBits);
				/* RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0] */
			
			/* Transmit buffer and receive response. */

			result = mfrc522_CommunicateWithPICC(CMD_TRANSCEIVE, 0x30 /*waitIRq = RxIRq | IdleIRq */, 
				buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0 /* checkCRC */);

			if (result == -11) { /* More than one PICC in the field => collision. */
				uint8_t valueOfCollReg = mfrc522_read(REG_COLLISION);
					/* CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0] */

				if (valueOfCollReg & 0x20) /* CollPosNotValid */
					return -43; /* STATUS_COLLISION: Without a valid collision position we cannot continue */

				uint8_t collisionPos = valueOfCollReg & 0x1F; /* Values 0-31, 0 means bit 32. */
				if (collisionPos == 0)
					collisionPos = 32;

				if (collisionPos <= currentLevelKnownBits) /* No progress - should not happen */ 
					return -44;

				/* Choose the PICC with the bit set. */
				currentLevelKnownBits = collisionPos;
				count = (currentLevelKnownBits - 1) % 8; /* The bit to modify */
				index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); /* First byte is index 0. */
				buffer[index] |= (1 << count);

			} else if (result != 0) {
				return result;

			} else { /* STATUS_OK */
				if (currentLevelKnownBits >= 32) { /* This was a SELECT. */
					selectDone = 1; /* No more anticollision. 
							   We continue below outside the while. */
				} else { /* This was an ANTICOLLISION.
					     We now have all 32 bits of the UID in this Cascade Level */
					currentLevelKnownBits = 32;
					/* Run loop again to do the SELECT. */
				}
			}
		} 
		
		/* We do not check the CBB - it was constructed by us above. */
		
		/* Copy found UID bytes from buffer[] to uid->uidByte[] */
		index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; /* source index in buffer[] */
		bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++)
			uid->uidByte[uidIndex + count] = buffer[index++];
		
		/* Check response SAK (Select Acknowledge) */
		if (responseLength != 3 || txLastBits != 0) /* SAK must be exactly 24 bits (1 byte + CRC_A). */
			return -45;

		/* Verify CRC_A - do our own calculation and store the control in buffer[2..3] 
			- those bytes are not needed anymore. */

		result = mfrc522_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != 0)
			return result;

		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
			return -46; // STATUS_CRC_WRONG;

		if (responseBuffer[0] & 0x04) /* Cascade bit set - UID not complete yes */
			cascadeLevel++;
		else {
			uidComplete = 1;
			uid->sak = responseBuffer[0];
		}
	}
	
	/* Set correct uid->size */
	uid->size = 3 * cascadeLevel + 1;

	return 0;
}

int mfrc522_PICC_HaltA(void)
{
	int result;
	uint8_t buffer[4];
	
	/* Build command buffer */
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;

	/* Calculate CRC_A */
	result = mfrc522_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != 0) 
		return result;
	
	/*
	    Send the command.
	    The standard says:
	    If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	    HLTA command, this response shall be interpreted as 'not acknowledge'.
	    We interpret that this way: Only STATUS_TIMEOUT is a success.
	*/
	result = mfrc522_CommunicateWithPICC(CMD_TRANSCEIVE, 0x30 /*waitIRq = RxIRq | IdleIRq */, 
				buffer, sizeof(buffer), NULL, NULL, NULL, 0 /* rxAlign */, 0 /* checkCRC */);
	if (result == -4) // STATUS_TIMEOUT
		return 0;

	if (result == 0) // That is ironically NOT ok in this case ;-)
		return -31;

	return result;
} 

