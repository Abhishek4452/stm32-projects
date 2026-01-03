/*
 * main.c
 *
 *  Created on: 25-Dec-2025
 *      Author: debian
 *  topic  - clear the bit value form the 4 ,5 and 6th position of an bit
 */


#include <stdint.h>
#include <stdio.h>

int main(){
	int32_t temp1;
	printf("enter a number : \n");
	scanf("%d",&temp1);
	printf("%d method-1\n",temp1&199);// in case of 8 bit number this is more suitable method
	printf("%d method -2 \n",temp1&(~122));

	return 0;
}

