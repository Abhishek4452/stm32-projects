/*
 * tax.c
 *
 *  Created on: 24-Dec-2025
 *      Author: debian
 */


#include <stdio.h>
#include <stdint.h>

int32_t amount;
float tax_amount;

int main(void){
	printf("enter your income :");
	if(scanf("%d",&amount) != 1){
		printf("enter a valid amount");
		return 0;
	}

	if (amount < 9525){
		printf("tax will be zero ");
	}
	else if (9525 <= amount && amount <38700){
		tax_amount = amount*0.12f;
		printf("tax will be %0.2f ",tax_amount);
	}
	else if (38700<= amount && amount <82500){
			tax_amount = amount*0.22f;
			printf("tax will be %0.2f ",tax_amount);
		}
	else{
		tax_amount = (amount*0.32f)+1000.0f;
		printf("tax will be %0.2f",tax_amount);
	}
//	while (getchar != '\n'){
//
//	}
//	getchar();
	return 0;
}
