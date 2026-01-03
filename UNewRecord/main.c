/*
 * main.c
 *
 *  Created on: 01-Jan-2026
 *      Author: debian
 */


/*
 * write a program to maintain record of students . the program must maintain records of the 10 students and
 * you should give below feature to your program
 * 1. display  all records
 * 2. add nw record
 * 3. delete a record
 * the program also should avoid / alert below situation
 * a. duplication of record
 * b. no space to add a new record
 * c. deleting a unknown record.
 */

#include <stdio.h>
#include <stdint.h>



// record in form of structure
struct student
{
char sName[100];
int16_t sAge;
int16_t sClass;
char sFatherName[100];
int32_t rollNumber;
}student_info_t;

// end of the st record





// function prototype
void display(void);
void delete(void);
void addNew(void);

// variable inside this code
uint32_t UrollNumber; // user input roll number
uint32_t maxRecord = 10;
student_info_t student[maxRecord]; // ARRAYY -- structure record of the student  data in form of array, which store only 10 records


int main(void){
	int8_t appContinue = 1;
while (appContinue){
	uint8_t input;
	printf(" *******************  student record application ******************** \n");
	printf("display record -> press 1 \n");
	printf("add new record -> press 2 \n");
	printf("delete record -> press 3 \n ");
	printf("exit -- > 4 \n");
	scanf("%d \n",&input);

// reading the code
	if (input == 1){
		// displaying the record
		printf("enter the roll number : \n");
		scanf("%d ",&roll_num);
		if (check_roll_numb){
			//
		}else{
			printf("entered roll number is invalid ");
		}
	}
	else if(input == 2){

		printf("adding new record : \n");
	}
	else if (input == 3){
		printf("deleting the record : \n");
		printf("%d ",&roll_num);

	}
	else if (input == 4){
		appContinue = 0; // exiting from the application
	}
	else{
		printf("enter a valid number between 1 and 3  ");
	}
}
	return 0;
}



int check_roll_numb (student_info_t *record, maxRecord){ // 3 varibale urollnumber, record , max record
	int is_exits = 0;
	for (int i = 0; i<maxRecord;i++){
		if(Record[i].rollNumber == urollNumber ){
			is_exits = 1;
			break;
		}
	}
	return is_exits; // return 0 if roll number was not matching
}


void addNew(student_info_t *Record , maxRecord){
	is_exits = 0
	for(int i = 0 ; i<maxRecord;i++){
		if(!Record[i].rollNumber){
			printf("enter the roll number : \n");
			scanf("%d ",&UrollNumber);
			// checking the roll number
			is_exits = check_roll_numb(students,maxRecord);
			if(!is_exits){
				// adding data in the struct array
				Record[i].rollNumber = UrollNumber;
				printf("enter the name of the student : \n");
				scanf("%c",&Record[i].sName);
				printf("enter the class of the student : \n");
				scanf("%d",&Record[i].sClass);
				printf("enter the age  : \n");
				scanf("%d",&Record[i].sAge);
				printf("enter the father name : \n");
				scanf("%c",&Record[i].sFatherName);


			}
			else{
				printf("rollnumber already exits \n");

			}
			break;

		}
		if (i == maxRecord ){
			printf(" no more space to enter the data \n");
			break;
		}
	}


}

//void display(REcordrollNumber,student_info_t *Record){
//	    printf("enter the student name : \n");
//		scanf("%[^\n]s",&Record[i].sName);
//		printf("enter the age : \n");
//		scanf("%[^\n]s",&Record[i].sAge);
//		printf("enter the class : \n");
//		scanf("%[^\n]s",&Record[i].sClass);
//		printf("enter the father name : \n");
//		scanf("%[^\n]s",&Record[i].sFatherName);
//		printf("enter the adhar number : \n");
//		scanf("%[^\n]s",&Record[i].srollNumber);
//}





