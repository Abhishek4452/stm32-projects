/*
 * main.c
 *
 *  Created on: 24-Dec-2025
 *      Author: debian
 */

// pointer operation will be performed here

#include <stdio.h>
int main(){
	char variable = 100;
	printf("address of first variable is %u ",variable);
	printf("\n printing the address of the variable : %p ",&variable);


	char* pAddress = &variable;


	char value = *pAddress;
	printf("\n rd data from vlue : %d \n",value);

	*pAddress = 65;
	printf("value of data : %d \n ",variable);


	return 0;
}
