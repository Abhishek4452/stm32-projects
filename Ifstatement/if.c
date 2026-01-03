/*
 * if.c
 *
 *  Created on: 24-Dec-2025
 *      Author: debian
 */


// program for age casting

#include <stdio.h>
#include <stdint.h>
int age;
int main(){
	printf("enter your age : ");
	scanf("%d",&age);

	if (age > 18){
		printf(" YOU CAN GIVE VOTE ");

	}
	else{
		printf("enjoy your life nothing is kept in the voting ");
	}
	return 0;
}
