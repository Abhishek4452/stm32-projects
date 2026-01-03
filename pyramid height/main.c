/*
 * main.c
 *
 *  Created on: 26-Dec-2025
 *      Author: debian
 */


// write  a code in which you take input from the user and print the corresponding height pyramid

#include<stdio.h>
#include <stdint.h>
int16_t height;
int main(){
	printf("enter the height of pyramid : ");
	scanf("%hd ",&height);

	for (int i = 1 ; i<= height ;i++){ // keep track of the height
		for (int j =1;j<=i ;j++ ){
			printf("%d ",j);
		}
		printf("\n");
	}

	return 0;
}
