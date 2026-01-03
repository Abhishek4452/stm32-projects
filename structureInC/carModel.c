/*
 * carModel.c
 *
 *  Created on: 27-Dec-2025
 *      Author: debian
 */


// write a program to create a carModel structure discussed and create 2 variable of carModel type and initialize the variable with the data

#include <stdio.h>
#include <stdint.h>
struct carModel{
	uint16_t CarNumber;
	int32_t CarPrice;
	uint32_t carMaxPrice;
	float carWeight;
};
int main(void){
	struct carModel hunda = {2021,15000,220,1330};
	struct carModel tata = {4031,35000,160,1900.96};
	printf("detail of the car Model is as follows : \n");
	printf(" car number is : %u \n",hunda.CarNumber);
	printf(" car price is : %u \n",hunda.CarPrice);
	printf(" car maxprice is : %u \n",hunda.carMaxPrice);
	printf(" car weight is : %f \n",hunda.carWeight);


	printf("detail of the car Model is as follows : \n");
	printf(" car number is : %u \n",tata.CarNumber);
	printf(" car price is : %u \n",tata.CarPrice);
	printf(" car maxprice is : %u \n",tata.carMaxPrice);
	printf(" car weight is : %f \n",tata.carWeight);
	return 0;
}
