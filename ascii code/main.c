/*
 * main.c
 *
 *  Created on: 24-Dec-2025
 *      Author: debian
 */


// ASCII code for c

#include <stdio.h>

int main(){
	char c1,c2,c3,c4,c5,c6;
	printf("enter 6 digit number  :  ");
	scanf("%c %c %c %c %c %c ",&c1,&c2,&c3,&c4,&c5,&c6);
	getchar();
	printf("\nAscii code : %d %d %d %d %d %d",c1,c2,c3,c4,c5,c6);
	printf("enter a number to exit ");
	while(getchar()!= '\n'){

	}
	getchar();

	return 0;
}
