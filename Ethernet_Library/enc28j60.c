




#include "enc28j60.h"

uint8_t enc28j60_current_bank = 0;
uint16_t enc28j60_rxrdpt = 0;

#define enc28j60_select()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
#define enc28j60_release()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

uint8_t enc28j60_rxtx(uint8_t data)
{
	//uint8_t spiData;
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	HAL_SPI_Receive(&hspi1,&data, 1, 100);
	return data;
}

#define enc28j60_rx() enc28j60_rxtx(0xff)
#define enc28j60_tx(data) enc28j60_rxtx(data)

// Generic SPI read command
uint8_t enc28j60_read_op(uint8_t cmd, uint8_t adr)
{
	        uint8_t spiData[2];
	        enc28j60_select();
			spiData[0] = (cmd| (adr & ENC28J60_ADDR_MASK));
			HAL_SPI_Transmit(&hspi1, spiData, 1, 100);
			if(adr & 0x80)
			{
				//HAL_SPI_Transmit(&hspi1, spiData, 1, 100);
				HAL_SPI_Receive(&hspi1, &spiData[1], 1, 100);
			}
			HAL_SPI_Receive(&hspi1, &spiData[1], 1, 100);
			enc28j60_release();

			return spiData[1];
}

// Generic SPI write command
void enc28j60_write_op(uint8_t cmd, uint8_t adr, uint8_t data)
{
	uint8_t spiData[2];
	enc28j60_select() ;
	spiData[0] = (cmd| (adr & ENC28J60_ADDR_MASK)); //((oper<<5)&0xE0)|(addr & ADDR_MASK);
	spiData[1] = data;
	HAL_SPI_Transmit(&hspi1, spiData, 2, 100);
	enc28j60_release();
}

// Initiate software reset
void enc28j60_soft_reset()
{
    enc28j60_select() ;
	enc28j60_tx(ENC28J60_SPI_SC);
	enc28j60_release();
	enc28j60_current_bank = 0;
//	HAL_Delay(1); // Wait until device initializes
}


/*
 * Memory access
 */

// Set register bank
void enc28j60_set_bank(uint8_t adr)
{
	uint8_t bank;

	if( (adr & ENC28J60_ADDR_MASK) < ENC28J60_COMMON_CR )
	{
		bank = (adr >> 5) & 0x03; //BSEL1|BSEL0=0x03
		if(bank != enc28j60_current_bank)
		{
			enc28j60_write_op(ENC28J60_SPI_BFC, ECON1, 0x03);
			enc28j60_write_op(ENC28J60_SPI_BFS, ECON1, bank);
			enc28j60_current_bank = bank;
		}
	}
}

// Read register
uint8_t enc28j60_rcr(uint8_t adr)
{
	enc28j60_set_bank(adr);
	return enc28j60_read_op(ENC28J60_SPI_RCR, adr);
}

// Read register pair
uint16_t enc28j60_rcr16(uint8_t adr)
{
	enc28j60_set_bank(adr);
	return enc28j60_read_op(ENC28J60_SPI_RCR, adr) |
		(enc28j60_read_op(ENC28J60_SPI_RCR, adr+1) << 8);
}

// Write register
void enc28j60_wcr(uint8_t adr, uint8_t arg)
{
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_WCR, adr, arg);
}

// Write register pair
void enc28j60_wcr16(uint8_t adr, uint16_t arg)
{
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_WCR, adr, arg);
	enc28j60_write_op(ENC28J60_SPI_WCR, adr+1, arg>>8);
}

// Clear bits in register (reg &= ~mask)
void enc28j60_bfc(uint8_t adr, uint8_t mask)
{
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_BFC, adr, mask);
}

// Set bits in register (reg |= mask)
void enc28j60_bfs(uint8_t adr, uint8_t mask)
{
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_BFS, adr, mask);
}

// Read Rx/Tx buffer (at ERDPT)
void enc28j60_read_buffer(uint8_t *buf, uint16_t len)
{
	uint8_t spiData[2];
	enc28j60_select();
	spiData[0] = ENC28J60_SPI_RBM;
	HAL_SPI_Transmit(&hspi1, spiData, 1, 100);
	while(len--)
		HAL_SPI_Receive(&hspi1,&(*(buf++)), 1, 100);
		//*(buf++) = enc28j60_rx();
	enc28j60_release();
}

// Write Rx/Tx buffer (at EWRPT)
void enc28j60_write_buffer(uint8_t *buf, uint16_t len)
{
	    uint8_t spiData[2];
	    enc28j60_select();
		spiData[0] = ENC28J60_SPI_WBM;
		HAL_SPI_Transmit(&hspi1, spiData, 1, 100);
	    while(len--)
	    HAL_SPI_Transmit(&hspi1, &(*(buf++)), 1, 100);
	    enc28j60_release();
}

// Read PHY register
uint16_t enc28j60_read_phy(uint8_t adr)
{
	enc28j60_wcr(MIREGADR, adr);
	enc28j60_bfs(MICMD, MICMD_MIIRD);
	while(enc28j60_rcr(MISTAT) & MISTAT_BUSY)
		;
	enc28j60_bfc(MICMD, MICMD_MIIRD);
	return enc28j60_rcr16(MIRD);
}

// Write PHY register
void enc28j60_write_phy(uint8_t adr, uint16_t data)
{
	enc28j60_wcr(MIREGADR, adr);
	enc28j60_wcr16(MIWR, data);
	while(enc28j60_rcr(MISTAT) & MISTAT_BUSY);

}


/*
 * Init & packet Rx/Tx
 */

void enc28j60_init(uint8_t *macadr)
{


	// Reset ENC28J60
//	enc28j60_soft_reset();
	enc28j60_write_op(ENC28J60_SPI_SC, 0, ENC28J60_SPI_SC);
	 while(!enc28j60_read_op(ENC28J60_SPI_RCR,ESTAT)&ESTAT_CLKRDY){};
	// Setup Rx/Tx buffer
	enc28j60_wcr16(ERXST, ENC28J60_RXSTART);
	enc28j60_wcr16(ERXRDPT, ENC28J60_RXSTART);
	enc28j60_wcr16(ERXND, ENC28J60_RXEND);
	enc28j60_rxrdpt = ENC28J60_RXSTART;

	// Setup MAC
	enc28j60_wcr(MACON1, MACON1_TXPAUS| // Enable flow control
	MACON1_RXPAUS|MACON1_MARXEN); // Enable MAC Rx
	enc28j60_wcr(MACON2, 0); // Clear reset
	enc28j60_wcr(MACON3, MACON3_PADCFG0| // Enable padding,
	MACON3_TXCRCEN|MACON3_FRMLNEN|MACON3_FULDPX); // Enable crc & frame len chk
	enc28j60_wcr16(MAMXFL, ENC28J60_MAXFRAME);
	enc28j60_wcr(MABBIPG, 0x15); // Set inter-frame gap
	enc28j60_wcr(MAIPGL, 0x12);
	enc28j60_wcr(MAIPGH, 0x0c);
	enc28j60_wcr(MAADR5, macadr[0]); // Set MAC address
	enc28j60_wcr(MAADR4, macadr[1]);
	enc28j60_wcr(MAADR3, macadr[2]);
	enc28j60_wcr(MAADR2, macadr[3]);
	enc28j60_wcr(MAADR1, macadr[4]);
	enc28j60_wcr(MAADR0, macadr[5]);

	// Setup PHY
	enc28j60_write_phy(PHCON1, PHCON1_PDPXMD); // Force full-duplex mode
	enc28j60_write_phy(PHCON2, PHCON2_HDLDIS); // Disable loopback
	enc28j60_write_phy(PHLCON, PHLCON_LACFG2| // Configure LED ctrl
		PHLCON_LBCFG2|PHLCON_LBCFG1|PHLCON_LBCFG0|
		PHLCON_LFRQ0|PHLCON_STRCH);

	// Enable Rx packets
	enc28j60_bfs(ECON1, ECON1_RXEN);
}

void enc28j60_send_packet(uint8_t *data, uint16_t len)
{
	while(enc28j60_rcr(ECON1) & ECON1_TXRTS)
		{
			// TXRTS may not clear - ENC28J60 bug. We must reset
			// transmit logic in cause of Tx error
			if(enc28j60_rcr(EIR) & EIR_TXERIF)
			{
				enc28j60_bfs(ECON1, ECON1_TXRST);
				enc28j60_bfc(ECON1, ECON1_TXRST);
			}
		}

		enc28j60_wcr16(EWRPT, ENC28J60_TXSTART);
		enc28j60_write_buffer((uint8_t*)"\x00", 1);
		enc28j60_write_buffer(data, len);

		enc28j60_wcr16(ETXST, ENC28J60_TXSTART);
		enc28j60_wcr16(ETXND, ENC28J60_TXSTART + len);

		enc28j60_bfs(ECON1, ECON1_TXRTS); // Request packet send


}

uint16_t enc28j60_recv_packet(uint8_t *buf, uint16_t buflen)
{
	uint16_t len = 0, rxlen, status, temp;

	if(enc28j60_rcr(EPKTCNT))
	{
		enc28j60_wcr16(ERDPT, enc28j60_rxrdpt);

		enc28j60_read_buffer((void*)&enc28j60_rxrdpt, sizeof(enc28j60_rxrdpt));
		enc28j60_read_buffer((void*)&rxlen, sizeof(rxlen));
		enc28j60_read_buffer((void*)&status, sizeof(status));

		if(status & 0x80) //success
		{
			len = rxlen - 4; //throw out crc
			if(len > buflen) len = buflen;
			enc28j60_read_buffer(buf, len);	
		}

		// Set Rx read pointer to next packet
		temp = (enc28j60_rxrdpt - 1) & ENC28J60_BUFEND;
		enc28j60_wcr16(ERXRDPT, temp);

		// Decrement packet counter
		enc28j60_bfs(ECON2, ECON2_PKTDEC);
	}

	return len;
}
