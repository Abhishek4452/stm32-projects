/*
 * charge.c
 *
 *  Created on: 24-Dec-2025
 *      Author: debian
 */


// while creating such file use filename.c

#include <stdio.h>
int main(){
	double charge, chargeE;
	double number;
	printf("enter the total charge : ");
	scanf("%lf",&charge);
	printf("enter the charge of 1 electron : ");
	scanf("%le",&chargeE);
	number = (charge/chargeE)*(-1);
	printf("charge will be %lf",number);
	printf("\ncharge will be in scientific notation : %le",number);
	while (getchar() != '\n'){

	}
	getchar();

	return 0;
}
