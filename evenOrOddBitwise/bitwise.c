/*
 * bitwise.c
 *
 *  Created on: 25-Dec-2025
 *      Author: debian
 *
 *  title -- odd and even number
 */


#include <stdio.h>
#include <stdint.h>

int16_t num1,B_output;
int16_t mask = 1; // for temporary storing of value

int main(void){
	printf("enter num for even and odd  : \n");
	scanf("%hd",&num1);

	B_output = num1 & mask;
	if (B_output == 1){
		printf("number is odd ");
	}
	else{
		printf("number is even ");
	}
	return 0;
}
