/*
 * fileHandlingInC.c
 *
 *  Created on: 11-Jan-2026
 *      Author: debian
 */
/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/


#include <stdio.h>
#include <stdint.h>
FILE *fp;
void add(void);
void show(void);
int main(){
	int8_t loop = 1;
	while(loop){
		add();
		show();
	}

	return 0;
}
void add(void){
	int32_t data;
	fp = fopen("HISTORY.txt","a"); // opening in append mode
	printf("ENTER DATA : \n");
	scanf("%d",&data);
	printf("\n");
	fprintf(fp,"%d\n",data);
	fclose(fp); // closing the file
}
void show(void){
    FILE *fp;
    fp = fopen("HISTORY.txt","r");
    char buff[100];
    if(fp == NULL){
        printf("file is EMPTY \n");
    }
    printf(" ****************************** HISTORY ********************************** \n");
    while(fgets(buff ,sizeof(buff),fp) != NULL){
        printf("%s",buff);
    }
    fclose(fp);
}

