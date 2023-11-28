/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "debug.h"
#include "ff.h"

#define CS_HIGH() GPIOC->BSHR = GPIO_Pin_3
#define CS_LOW() GPIOC->BCR = GPIO_Pin_3

#define MMC_WP 0
#define MMC_CD 1

#define power_on()
#define power_off()
#define FCLK_SLOW()
#define FCLK_FAST()

static volatile
DSTATUS Stat = STA_NOINIT;  /* Disk status */

static volatile
UINT Timer1, Timer2;        /* 1000Hz decrement timer */

static
UINT CardType;

static BYTE xchg_spi (
    BYTE dat    /* Data to send */
)
{
    uint8_t ret;
    while(SPI1->STATR & SPI_I2S_FLAG_BSY);
    SPI1->DATAR = dat;              /* Start an SPI transaction */
    while (!(SPI1->STATR & SPI_I2S_FLAG_TXE));  /* Wait for end of the transaction */
    asm volatile("nop");
    while(!(SPI1->STATR & SPI_I2S_FLAG_RXNE));
    ret = (BYTE)SPI1->DATAR;       /* Return received byte */
    while(SPI1->STATR & SPI_I2S_FLAG_BSY);
    return ret;
}

static
void xmit_spi_multi (
    const BYTE* buff,   /* Data to be sent */
    UINT cnt            /* Number of bytes to send */
)
{
    do {
        xchg_spi(*buff++);
    } while (cnt -= 1);
}

static
void rcvr_spi_multi (
    BYTE* buff,     /* Buffer to store received data */
    UINT cnt        /* Number of bytes to receive */
)
{
    do {
        *buff++ = xchg_spi(0xff);
    } while (cnt -= 1);
}

static
int wait_ready (void)
{
    BYTE d;

    Timer2 = 500;   /* Wait for ready in timeout of 500ms */
    do {
        d = xchg_spi(0xFF);
    } while ((d != 0xFF) && Timer2);

    return (d == 0xFF) ? 1 : 0;
}

static
void mmc_deselect (void)
{
    CS_HIGH();          /* Set CS# high */
    xchg_spi(0xFF);     /* Dummy clock (force DO hi-z for multiple slave SPI) */
}

static
int mmc_select (void)   /* 1:Successful, 0:Timeout */
{
    CS_LOW();           /* Set CS# low */
    xchg_spi(0xFF);     /* Dummy clock (force DO enabled) */

    if (wait_ready()) return 1; /* Wait for card ready */

    mmc_deselect();
    return 0;   /* Timeout */
}

static
int rcvr_datablock (    /* 1:OK, 0:Failed */
    BYTE *buff,         /* Data buffer to store received data */
    UINT btr            /* Byte count (must be multiple of 4) */
)
{
    BYTE token;


    Timer1 = 100;
    do {                            /* Wait for data packet in timeout of 100ms */
        token = xchg_spi(0xFF);
    } while ((token == 0xFF) && Timer1);

    if(token != 0xFE) return 0;     /* If not valid data token, retutn with error */

    rcvr_spi_multi(buff, btr);      /* Receive the data block into buffer */
    xchg_spi(0xFF);                 /* Discard CRC */
    xchg_spi(0xFF);

    return 1;                       /* Return with success */
}

static
int xmit_datablock (    /* 1:OK, 0:Failed */
    const BYTE *buff,   /* 512 byte data block to be transmitted */
    BYTE token          /* Data token */
)
{
    BYTE resp;


    if (!wait_ready()) return 0;

    xchg_spi(token);        /* Xmit a token */
    if (token != 0xFD) {    /* Not StopTran token */
        xmit_spi_multi(buff, 512);  /* Xmit the data block to the MMC */
        xchg_spi(0xFF);             /* CRC (Dummy) */
        xchg_spi(0xFF);
        resp = xchg_spi(0xFF);      /* Receive a data response */
        if ((resp & 0x1F) != 0x05)
            return 0;    /* If not accepted, return with error */
    }

    return 1;
}

static
BYTE send_cmd (
    BYTE cmd,       /* Command byte */
    DWORD arg       /* Argument */
)
{
    BYTE n, res;


    if (cmd & 0x80) {   /* ACMD<n> is the command sequense of CMD55-CMD<n> */
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }

    /* Select the card and wait for ready except to stop multiple block read */
    if (cmd != CMD12) {
        mmc_deselect();
        if (!mmc_select()) return 0xFF;
    }

    /* Send command packet */
    xchg_spi(0x40 | cmd);           /* Start + Command index */
    xchg_spi((BYTE)(arg >> 24));    /* Argument[31..24] */
    xchg_spi((BYTE)(arg >> 16));    /* Argument[23..16] */
    xchg_spi((BYTE)(arg >> 8));     /* Argument[15..8] */
    xchg_spi((BYTE)arg);            /* Argument[7..0] */
    n = 0x01;                       /* Dummy CRC + Stop */
    if (cmd == CMD0) n = 0x95;      /* Valid CRC for CMD0(0) + Stop */
    if (cmd == CMD8) n = 0x87;      /* Valid CRC for CMD8(0x1AA) + Stop */
    xchg_spi(n);

    /* Receive command response */
    if (cmd == CMD12) xchg_spi(0xFF);   /* Skip a stuff byte on stop to read */
    n = 10;                         /* Wait for a valid response in timeout of 10 attempts */
    do {
        res = xchg_spi(0xFF);
    } while ((res & 0x80) && --n);

    return res;         /* Return with the response value */
}

DSTATUS disk_status (
    BYTE pdrv       /* Physical drive nmuber (0) */
)
{
    if (pdrv != 0) return STA_NOINIT;   /* Supports only single drive */

    return Stat;
}

DSTATUS disk_initialize (
    BYTE pdrv       /* Physical drive nmuber (0) */
)
{
    BYTE n, cmd, ty, ocr[4];


    if (pdrv != 0) return STA_NOINIT;   /* Supports only single drive */
    if (Stat & STA_NODISK) return Stat; /* No card in the socket */

    power_on();                         /* Initialize memory card interface */
    FCLK_SLOW();
    for (n = 10; n; n--) xchg_spi(0xFF);    /* 80 dummy clocks */

    ty = 0;
    if (send_cmd(CMD0, 0) == 1) {           /* Enter Idle state */
        Timer1 = 1000;                      /* Initialization timeout of 1000 msec */
        if (send_cmd(CMD8, 0x1AA) == 1) {   /* SDv2? */
            for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);            /* Get trailing return value of R7 resp */
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {             /* The card can work at vdd range of 2.7-3.6V */
                while (Timer1 && send_cmd(ACMD41, 0x40000000)); /* Wait for leaving idle state (ACMD41 with HCS bit) */
                if (Timer1 && send_cmd(CMD58, 0) == 0) {            /* Check CCS bit in the OCR */
                    for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
                    ty = (ocr[0] & 0x40) ? CT_SD2|CT_BLOCK : CT_SD2;    /* SDv2+ */
                }
            }
        } else {                            /* SDv1 or MMCv3 */
            if (send_cmd(ACMD41, 0) <= 1)   {
                ty = CT_SD1; cmd = ACMD41;  /* SDv1 */
            } else {
                ty = CT_MMC; cmd = CMD1;    /* MMCv3 */
            }
            while (Timer1 && send_cmd(cmd, 0));     /* Wait for leaving idle state */
            if (!Timer1 || send_cmd(CMD16, 512) != 0) ty = 0;   /* Set read/write block length to 512 */
        }
    }
    CardType = ty;
    mmc_deselect();

    if (ty) {       /* Function succeded */
        Stat &= ~STA_NOINIT;    /* Clear STA_NOINIT */
        FCLK_FAST();
    } else {        /* Function failed */
        power_off();    /* Deinitialize interface */
    }

    return Stat;
}

DRESULT disk_read (
    BYTE pdrv,      /* Physical drive nmuber (0) */
    BYTE *buff,     /* Pointer to the data buffer to store read data */
    LBA_t sector,   /* Start sector number (LBA) */
    UINT count      /* Sector count (1..128) */
)
{
    DWORD sect = (DWORD)sector;


    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) sect *= 512;    /* Convert to byte address if needed */

    if (count == 1) {       /* Single block read */
        if ((send_cmd(CMD17, sect) == 0)    /* READ_SINGLE_BLOCK */
            && rcvr_datablock(buff, 512)) {
            count = 0;
        }
    }
    else {              /* Multiple block read */
        if (send_cmd(CMD18, sect) == 0) {   /* READ_MULTIPLE_BLOCK */
            do {
                if (!rcvr_datablock(buff, 512)) break;
                buff += 512;
            } while (--count);
            send_cmd(CMD12, 0);             /* STOP_TRANSMISSION */
        }
    }
    mmc_deselect();

    return count ? RES_ERROR : RES_OK;
}

DRESULT disk_write (
    BYTE pdrv,              /* Physical drive nmuber (0) */
    const BYTE *buff,       /* Pointer to the data to be written */
    LBA_t sector,           /* Start sector number (LBA) */
    UINT count              /* Sector count (1..128) */
)
{
    DWORD sect = (DWORD)sector;


    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;

    if (!(CardType & CT_BLOCK)) sect *= 512;    /* Convert to byte address if needed */

    if (count == 1) {       /* Single block write */
        if ((send_cmd(CMD24, sect) == 0)    /* WRITE_BLOCK */
            && xmit_datablock(buff, 0xFE)) {
            count = 0;
        }
    }
    else {              /* Multiple block write */
        if (CardType & CT_SDC) send_cmd(ACMD23, count);
        if (send_cmd(CMD25, sect) == 0) {   /* WRITE_MULTIPLE_BLOCK */
            do {
                if (!xmit_datablock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            if (!xmit_datablock(0, 0xFD)) count = 1;    /* STOP_TRAN token */
        }
    }
    mmc_deselect();

    return count ? RES_ERROR : RES_OK;
}

DRESULT disk_ioctl (
    BYTE pdrv,      /* Physical drive nmuber (0) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive data block */
)
{
    DRESULT res;
    BYTE n, csd[16], *ptr = buff;
    DWORD csz;


    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    res = RES_ERROR;
    switch (cmd) {
    case CTRL_SYNC :    /* Flush write-back cache, Wait for end of internal process */
        if (mmc_select()) res = RES_OK;
        break;

    case GET_SECTOR_COUNT : /* Get number of sectors on the disk (WORD) */
        if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
            if ((csd[0] >> 6) == 1) {   /* SDv2? */
                csz = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                *(LBA_t*)buff = csz << 10;
            } else {                    /* SDv1 or MMCv3 */
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                csz = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                *(LBA_t*)buff = csz << (n - 9);
            }
            res = RES_OK;
        }
        break;

    case GET_BLOCK_SIZE :   /* Get erase block size in unit of sectors (DWORD) */
        if (CardType & CT_SD2) {    /* SDv2+? */
            if (send_cmd(ACMD13, 0) == 0) {     /* Read SD status */
                xchg_spi(0xFF);
                if (rcvr_datablock(csd, 16)) {              /* Read partial block */
                    for (n = 64 - 16; n; n--) xchg_spi(0xFF);   /* Purge trailing data */
                    *(DWORD*)buff = 16UL << (csd[10] >> 4);
                    res = RES_OK;
                }
            }
        } else {                    /* SDv1 or MMCv3 */
            if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {  /* Read CSD */
                if (CardType & CT_SD1) {    /* SDv1 */
                    *(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
                } else {                    /* MMCv3 */
                    *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
                }
                res = RES_OK;
            }
        }
        break;

    case MMC_GET_TYPE :     /* Get card type flags (1 byte) */
        *ptr = CardType;
        res = RES_OK;
        break;

    case MMC_GET_CSD :  /* Receive CSD as a data block (16 bytes) */
        if ((send_cmd(CMD9, 0) == 0)    /* READ_CSD */
            && rcvr_datablock(buff, 16))
            res = RES_OK;
        break;

    case MMC_GET_CID :  /* Receive CID as a data block (16 bytes) */
        if ((send_cmd(CMD10, 0) == 0)   /* READ_CID */
            && rcvr_datablock(buff, 16))
            res = RES_OK;
        break;

    case MMC_GET_OCR :  /* Receive OCR as an R3 resp (4 bytes) */
        if (send_cmd(CMD58, 0) == 0) {  /* READ_OCR */
            for (n = 0; n < 4; n++)
                *((BYTE*)buff+n) = xchg_spi(0xFF);
            res = RES_OK;
        }
        break;

    case MMC_GET_SDSTAT :   /* Receive SD statsu as a data block (64 bytes) */
        if ((CardType & CT_SD2) && send_cmd(ACMD13, 0) == 0) {  /* SD_STATUS */
            xchg_spi(0xFF);
            if (rcvr_datablock(buff, 64)) res = RES_OK;
        }
        break;

//    case CTRL_POWER_OFF :   /* Power off */
//        power_off();
//        Stat |= STA_NOINIT;
//        res = RES_OK;
//        break;

    default:
        res = RES_PARERR;
    }

    mmc_deselect();

    return res;
}

void disk_timerproc (void)
{
    BYTE s;
    UINT n;


    n = Timer1;                 /* 1000Hz decrement timer with zero stopped */
    if (n) Timer1 = --n;
    n = Timer2;
    if (n) Timer2 = --n;


    /* Update socket status */

    s = Stat;
    if (MMC_WP) {
        s |= STA_PROTECT;
    } else {
        s &= ~STA_PROTECT;
    }
    if (MMC_CD) {
        s &= ~STA_NODISK;
    } else {
        s |= (STA_NODISK | STA_NOINIT);
    }
    Stat = s;
}

DWORD get_fattime (void)
{
    return 0;
}
