/*
 * main.c
 *
 *  Created on: 24-Dec-2025
 *      Author: debian
 *      this code will run by native compilar
 *      for making such file go to new select c/c++ proj
 *      then make a source file in that you get main.c for writing a code
 */

#include <stdio.h>
int main(){
	printf("hello world \n");// only for verification
	float num1;
	float num2;
	float num3;
	float avg;
	printf("enter first number : ");
	scanf("%f",&num1);
	fflush(stdout); //. will print the output at very fast rate
	printf("enter second number : ");
	scanf("%f",&num2);
	fflush(stdout);
	printf("enter third number : ");
	scanf("%f",&num3);
	fflush(stdout);
	avg = (num1+num2+num3) / 3;
	printf("average will be : %f ",avg);
	fflush(stdout);

	return 0;
}
