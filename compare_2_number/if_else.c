/*
 * if_else.c
 *
 *  Created on: 24-Dec-2025
 *      Author: debian
 */


#include <stdio.h>
#include <stdint.h>

int8_t ab;
int8_t ba;
float num1 , num2;

int main(){

	// taking first input
	printf("enter first number : ");
	if (scanf("%f",&num1)== 0){ // dealing with the error handling if invalid input is enter
		printf("enter a valid number ");
		return 0;
	}

	// taking 2nd input
	printf("enter second number: ");
	if (scanf("%f",&num2)==0){
		printf("enter a valid number ");
		return 0;
	}


	/* changing the format  */
	ab = (int)num1;
	ba = (int)num2;


	if (ab != num1 || ba != num2){
		printf("comparing only integer part only ");
	}

	//comparing the number
	if (ab>ba){
		printf("\ngreater number is %d",ab);
	}else{
		printf("\ngreater number is %d",ba);
	}
	fflush(stdout);
	getchar();
	return 0;
}
