/*
 * packet.c
 *
 *  Created on: 27-Dec-2025
 *      Author: debian
 */


#include <stdio.h>
#include <stdint.h>

uint8_t addrMode; // 1
uint8_t shortAddr; //2
uint8_t longAddr; //8
uint8_t sensor; //3
uint8_t BAT; // 3
uint16_t PayLoad; //12
uint8_t status; //1
uint8_t crc; //2

// packet data
uint32_t packet = (uint32_t) 0xFAFBACEA;


int main(void){
	printf("packet content is here : \n");

	// addr Mode
	addrMode = (uint8_t) (packet>>31);
	printf(" address mode of the packet is : %x \n",addrMode);

	// short addr
	shortAddr = (uint8_t) (packet>>29);
	shortAddr &= (3<<0);
	printf(" short address  : %b \n",shortAddr);

	// longAddr
	longAddr = (uint8_t) (packet >> 21);
	printf(" long address : %b \n",longAddr);

	// sensor
	sensor = (uint8_t) (packet >> 18);
	sensor &= (7<<0);
	printf(" sensor address : %b \n",sensor );

	//BAT
	BAT = (uint8_t )(packet >> 15);
	BAT &= (7<<0);
	printf(" BAT address is %b \n",BAT);

	// payload
	PayLoad = (uint16_t)(packet >> 3 );
	PayLoad &= ~(15<<11);
	printf(" payload is %b \n",PayLoad);

	status = (uint8_t)(packet >> 2);
	status &= (1<<0);
	printf("status is %b \n", status );

	// crc
	crc = (uint8_t)(packet>>0);
	crc &= (2<<0);
	printf("crc is %b \n",crc);


	return 0;
}
