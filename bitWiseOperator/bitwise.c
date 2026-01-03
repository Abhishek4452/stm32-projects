/*
 * bitwise.c
 *
 *  Created on: 25-Dec-2025
 *      Author: debian
 */


/* in this we will going to explore the bitwise operator
 *
 */

#include <stdio.h>
#include <stdint.h>

int32_t a = 12;
int32_t b = 13;
int32_t num1 ,num2;
int main(void){
	printf("logical operation will be performerd ");
	printf("%hhd",a&b);
	printf("\n");
	printf("%hhd",a|b);
	printf("\n");
	printf("%hhd",a^b);
	printf("\n");
	printf("%hhd",~b);
	printf("\n");
	printf("%hhd",~a);
	printf("\n taking the input from the user :\n");
	scanf("%d %d",&num1,&num2);
	printf("logical & operation %d \n",num1&num2);
	printf("logical | operation %d \n",num1|num2);
	printf("logical xor ^ operation %d \n",num1^num2);
	printf("logical not ~ operation %d \n",~a);
	printf("logical not ~ operation %d \n",~b);

	/* logical operation will be performerd 12
14
2
-15
-13
 taking the input from the user :
12 15
logical & operation 12
logical | operation 15
logical xor ^ operation 3
logical not ~ operation -13
logical not ~ operation -15
	 *
	 */


	return 0;
}
