/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/
#define F_CPU 90000000UL

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "MPU6050_GlobalHeader.h"     // Header Files for the MPU6050 Project
#include "compileGuard.h"

volatile Uint16 buffer[30];


void initMPU6050_DMP() {

#ifdef MPU6050
	CurrentMsgPtr.MsgStatus = I2C_MSGSTAT_INACTIVE;	// initialize Current message to inactive
	//init mpu6050
	mpu6050_init();
	DSP28x_usDelay(50000);

	//init mpu6050 dmp processor
	mpu6050_dmpInitialize();
	mpu6050_dmpEnable();
	DSP28x_usDelay(10000);

	MPUFlag = 2;// for some reason this conflicts with the dmp init routine if not equal to 0
#endif

}
/*
 * read bytes from chip register
 */
int16 mpu6050_readBytes(Uint16 regAddr, Uint16 length, Uint16 *data) {
	if(length > 0) {
		Uint8 i;
		while (CurrentMsgPtr.MsgStatus != I2C_MSGSTAT_INACTIVE);

		CurrentMsgPtr.MsgStatus		= I2C_MSGSTAT_SEND_NOSTOP;
		CurrentMsgPtr.SlaveAddress	= I2C_SLAVE_ADDR;
		CurrentMsgPtr.NumOfBytes	= length;
		CurrentMsgPtr.MemoryLowAddr	= regAddr;


	    while(I2CA_ReadData(&CurrentMsgPtr) != I2C_SUCCESS);

	    while (CurrentMsgPtr.MsgStatus != I2C_MSGSTAT_INACTIVE);

		for (i = 0; i<length; i++) data[i] = CurrentMsgPtr.MsgBuffer[i];
	}
	return length;
}

/*
 * read 1 byte from chip register
 */
int16 mpu6050_readByte(Uint16 regAddr, Uint16 *data) {
    return mpu6050_readBytes(regAddr, 1, data);
}

/*
 * write bytes to chip register
 */
void mpu6050_writeBytes(Uint16 regAddr, Uint16 length, Uint16* data) {
	if(length > 0) {
		Uint8 i;
		while (CurrentMsgPtr.MsgStatus != I2C_MSGSTAT_INACTIVE);

		CurrentMsgPtr.MsgStatus		= I2C_MSGSTAT_SEND_WITHSTOP;
		CurrentMsgPtr.SlaveAddress	= I2C_SLAVE_ADDR;
		CurrentMsgPtr.NumOfBytes	= length;
		CurrentMsgPtr.MemoryLowAddr	= regAddr;
		for (i = 0; i<length; i++) CurrentMsgPtr.MsgBuffer[i]	 = data[i];

		while (I2CA_WriteData(&CurrentMsgPtr) != I2C_SUCCESS);
		        // end of write section
	}
}

/*
 * write 1 byte to chip register
 */
void mpu6050_writeByte(Uint16 regAddr, Uint16 data) {
    return mpu6050_writeBytes(regAddr, 1, &data);
}

/*
 * read bits from chip register
 */
int16 mpu6050_readBits(Uint16 regAddr, Uint16 bitStart, Uint16 length, Uint16 *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    int16 count = 0;
    if(length > 0) {
		Uint16 b;
		if ((count = mpu6050_readByte(regAddr, &b)) != 0) {
			Uint16 mask = ((1 << length) - 1) << (bitStart - length + 1);
			b &= mask;
			b >>= (bitStart - length + 1);
			*data = b;
		}
    }
    return count;
}

/*
 * read 1 bit from chip register
 */
int16 mpu6050_readBit(Uint16 regAddr, Uint16 bitNum, Uint16 *data) {
    Uint16 b;
    Uint16 count = mpu6050_readByte(regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/*
 * write bit/bits to chip register
 */
void mpu6050_writeBits(Uint16 regAddr, Uint16 bitStart, Uint16 length, Uint16 data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
	if(length > 0) {
		Uint16 b = 0;
		if (mpu6050_readByte(regAddr, &b) != 0) {
			Uint16 mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			mpu6050_writeByte(regAddr, b);
		}
	}
}

/*
 * write one bit to chip register
 */
void mpu6050_writeBit(Uint16 regAddr, Uint16 bitNum, Uint16 data) {
    Uint16 b;
    mpu6050_readByte(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    mpu6050_writeByte(regAddr, b);
}

#if MPU6050_GETATTITUDE == 2
/*
 * write word/words to chip register
 */
void mpu6050_writeWords(Uint16 regAddr, Uint16 length, Uint16* data) {
	if(length > 0)
	{
		Uint16 temp[2] = {0};
		mpu6050_writeBytes(regAddr, 2, temp);
	}
}

/*
 * set a chip memory bank
 */
void mpu6050_setMemoryBank(Uint16 bank, Uint16 prefetchEnabled, Uint16 userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    mpu6050_writeByte(MPU6050_RA_BANK_SEL, bank);
}

/*
 * set memory start address
 */
void mpu6050_setMemoryStartAddress(Uint16 address) {
	mpu6050_writeByte(MPU6050_RA_MEM_START_ADDR, address);
}

/*
 * read a memory block
 */
void mpu6050_readMemoryBlock(Uint16 *data, Uint16 dataSize, Uint16 bank, Uint16 address) {

	Uint16 chunkSize;
    Uint16 i;

	mpu6050_setMemoryBank(bank, 0, 0);
	mpu6050_setMemoryStartAddress(address);

    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        mpu6050_readBytes(MPU6050_RA_MEM_R_W, chunkSize, data + i);

        // increase byte index by [chunkSize]
        i += chunkSize;

        // Uint16 automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu6050_setMemoryBank(bank, 0, 0);
            mpu6050_setMemoryStartAddress(address);
        }
    }
}

/*
 * write a memory block
 */
Uint16 mpu6050_writeMemoryBlock(const Uint16 *data, Uint16 dataSize, Uint16 bank, Uint16 address, Uint16 verify, Uint16 useProgMem) {
    Uint16 chunkSize;
//    Uint16 *verifyBuffer = 0;
    Uint16 *progBuffer = 0;
    Uint16 i;
//    Uint16 j;

	mpu6050_setMemoryBank(bank, 0, 0);
	mpu6050_setMemoryStartAddress(address);

//    if (verify) verifyBuffer = (Uint16 *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
//    if (useProgMem) progBuffer = (Uint16 *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

//        if (useProgMem) {
//            // write the chunk of data as specified
//            for (j = 0; j < chunkSize; j++) progBuffer[j] = data[i + j];
//        } else {
            // write the chunk of data as specified
            progBuffer = (Uint16 *)data + i;
//        }

        mpu6050_writeBytes(MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

        //verify data if needed
//        if (verify && verifyBuffer) {
//        	mpu6050_setMemoryBank(bank, 0, 0);
//            mpu6050_setMemoryStartAddress(address);
//            mpu6050_readBytes(MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
//            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
//                free(verifyBuffer);
//                if (useProgMem) free(progBuffer);
//                return 0; // uh oh.
//            }
//        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // Uint16 automatically wraps to 0 at 256
        address += chunkSize;
        if (address == 256) address = 0;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0)
            {
            	bank++;
            }
            mpu6050_setMemoryBank(bank, 0, 0);
            mpu6050_setMemoryStartAddress(address);
        }
    }
//    if (verify) free(verifyBuffer);
//    if (useProgMem) free(progBuffer);
    return 1;
}

/*
 * write a dmp configuration set
 */
Uint16 mpu6050_writeDMPConfigurationSet(const Uint16 *data, Uint16 dataSize, Uint16 useProgMem) {
    Uint16 *progBuffer = 0;
    Uint16 success, special;
    Uint16 i, j;

    Uint16 bank, offset, length;

    if (useProgMem) {
        progBuffer = (Uint16 *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    }

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]

    for (i = 0; i < dataSize;) {
        if (useProgMem) {
        	bank   = data[i++];
        	offset = data[i++];
        	length = data[i++];
        } else {
            bank   = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write
            if (useProgMem) {
                if (sizeof(progBuffer) < length) progBuffer = (Uint16 *)realloc(progBuffer, length);
                for (j = 0; j < length; j++) progBuffer[j] = data[i + j];
            } else {
                progBuffer = (Uint16 *)data + i;
            }
            success = mpu6050_writeMemoryBlock(progBuffer, length, bank, offset, 1, 0);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (useProgMem) {
                special = *(data + i++);
            } else {
                special = data[i++];
            }
            if (special == 0x01) {
                // enable DMP-related interrupts

            	//mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, 1); //setIntZeroMotionEnabled
            	//mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, 1); //setIntFIFOBufferOverflowEnabled
            	//mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, 1); //setIntDMPEnabled
            	mpu6050_writeByte(MPU6050_RA_INT_ENABLE, 0x32);  // single operation

                success = 1;
            } else {
                // unknown special command
                success = 0;
            }
        }

        if (!success) {
            if (useProgMem) free(progBuffer);
            return 0; // uh oh
        }
    }
    if (useProgMem) free(progBuffer);
    return 1;
}

/*
 * get the fifo count
 */
Uint16 mpu6050_getFIFOCount() {
	mpu6050_readBytes(MPU6050_RA_FIFO_COUNTH, 2, (Uint16 *)buffer);
    return (((Uint16)buffer[0]) << 8) | buffer[1];
}

/*
 * read fifo bytes
 */
void mpu6050_getFIFOBytes(Uint16 *data, Uint16 length) {
	mpu6050_readBytes(MPU6050_RA_FIFO_R_W, length, data);
}

/*
 * get the interrupt status
 */
Uint16 mpu6050_getIntStatus() {
	mpu6050_readByte(MPU6050_RA_INT_STATUS, (Uint16 *)buffer);
    return buffer[0];
}

/*
 * reset fifo
 */
void mpu6050_resetFIFO() {
	mpu6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

/*
 * get gyro offset X
 */
int16 mpu6050_getXGyroOffset() {
	mpu6050_readBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (Uint16 *)buffer);
    return buffer[0];
}

/*
 * set gyro offset X
 */
void mpu6050_setXGyroOffset(int16 offset) {
	mpu6050_writeBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/*
 * get gyro offset Y
 */
int16 mpu6050_getYGyroOffset() {
	mpu6050_readBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (Uint16 *)buffer);
    return buffer[0];
}

/*
 * set gyro offset Y
 */
void mpu6050_setYGyroOffset(int16 offset) {
	mpu6050_writeBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/*
 * get gyro offset Z
 */
int16 mpu6050_getZGyroOffset() {
	mpu6050_readBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (Uint16 *)buffer);
    return buffer[0];
}

/*
 * set gyro offset Z
 */
void mpu6050_setZGyroOffset(int16 offset) {
	mpu6050_writeBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}
#endif

/*
 * set sleep disabled
 */
void mpu6050_setSleepDisabled() {
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
}

/*
 * set sleep enabled
 */
void mpu6050_setSleepEnabled() {
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1);
}


/*
 * test connectino to chip
 */
Uint16 mpu6050_testConnection() {
	mpu6050_readBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, (Uint16 *)buffer);
	if(buffer[0] == 0x34)
		return 1;
	else
		return 0;
}

/*
 * initialize the accel and gyro
 */
void mpu6050_init() {
	
	//allow mpu6050 chip clocks to start up
	DSP28x_usDelay(100000);

	//set sleep disabled
	mpu6050_setSleepDisabled();
	//wake up delay needed sleep disabled
	DSP28x_usDelay(10000);

	//set clock source
	//  it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source)
	//  as the clock reference for improved stability
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	//set DLPF bandwidth to 42Hz
	mpu6050_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
    //set sample rate
	mpu6050_writeByte(MPU6050_RA_SMPLRT_DIV, 4); //1khz / (1 + 4) = 200Hz
	//set gyro range
	mpu6050_writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS);
	//set accel range
	mpu6050_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS);

}


