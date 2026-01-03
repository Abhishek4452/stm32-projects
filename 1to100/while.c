/*
 * while.c
 *
 *  Created on: 25-Dec-2025
 *      Author: debian
 */


// writing a code to print the even  number between 0 to 100 ; count how many number did you find

#include <stdio.h>
#include <stdint.h>
int16_t num,count;
int main(void){
	printf("number counting start from here : \n");
	num = 0;
	while (num <= 100){
		if((num&0x01) == 0 ){
			printf("even number : %d \n",num);
		}
		num++;
	}
	return 0;
}
