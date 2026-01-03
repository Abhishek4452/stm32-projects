/*
 * 25/12/2025
 *
 * using debian 13
 *
 * write a program in which you are setting the 4 and 7 bit of the input value as 1 and print the result
 * 144 = 1 0 0 1 0 0 0 0
 */
#include <stdio.h>
#include <stdint.h>

int main(void){
	int32_t num1;
	printf("enter a number : ");
	scanf("%hd",&num1);
	printf("%hd",num1|144);
	return 0;
}
