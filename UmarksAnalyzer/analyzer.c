	/*
	 * analyzer.c
	 *
	 *  Created on: 02-Jan-2026
	 *      Author: debian
	 */


	/*
	 * problem statement --- Create a C program to store and analyze marks of **5 students**.
	 *
	 */


	#include <stdio.h>
	#include <stdint.h>
    #include <stdlib.h>

	// variables ---
	int8_t option;
	int32_t userRollNumber;
#define MAX_RECORD 2 // use macros
	int32_t count = 0;

/*------------------------------------------------------ structure  ------------------------------------ */
	typedef struct  //structure for storing the data
	{	int32_t rollNumber ;
		int8_t sst; // social science
		int8_t science;
		int8_t math;
		int8_t hindi;
		int8_t english;
		int8_t valid ; // 1- used , 0 - empty
	}student_marks_t;


	// prototypes --
	int showMenu(void);
	void showdetail(int32_t userRollNumber);
	void f_exit(void);
	void add(student_marks_t *aStudent,int32_t userRollNumber); // adding the details
	void delete(int32_t userRollNumber);  /// ------ need to delete
	int find_RollNumber(int32_t userRollNumber);
	void show_RollNumber(void);



	// array
	student_marks_t aStudent[MAX_RECORD] = {0};//storing data for the 5 students

	int main(){
		int8_t choice;
		printf("*** welcome to the marks management system *** \n");
		while(1){
			choice = showMenu();
			switch (choice)
			{
			case 1:
				printf("enter the roll number : \n");
				scanf("%d",&userRollNumber);
				add(aStudent,userRollNumber);
				break;
			case 2:
				printf("enter the roll number to show the detail : \n");
				scanf("%d",&userRollNumber);
				showdetail(userRollNumber);
				break;
			case 3:
				printf("enter roll number to delete detail : \n");
				delete(userRollNumber);
				break;
			case 4:  // done ---- perfect
				f_exit();
				break;
			case 5: // showing the roll NUMBER
				show_RollNumber();
				break;
			default:
				printf(" ERROR !!!!!!   { enter a valid number }  ERROR !!!!!!  \n");
				break;
			}
		}
		return 0;
	}

	// *************************** MENU *****************************//
	int showMenu(void){
		printf("\n ************************** MENU ********************************* \n");
		printf("Add detail for students    --> 1 \n");
		printf("show detail of student     --> 2 \n");
		printf("delete detail of student   --> 3 \n");
		printf("exit from application      --> 4 \n");
		printf("show stored roll Number    --> 5 \n");
		printf("enter number corresponding to that -- \n ");
		scanf("%hhd", &option);
		return option;
	}


	// ******************* *******EXIT ****************************************//
	void f_exit(void){
		printf(" *** thank you for using the application ***");
		//return 0;  // <<< -- why it is so ?
		exit(0);
	}

	//********************** FIND ROLL NUMBER **********************************//
	int find_RollNumber(int32_t userRollNumber){
		for (int i = 0 ; i<MAX_RECORD; i++){
			if (aStudent[i].valid ==1 &&userRollNumber == aStudent[i].rollNumber){
				// printf(" ERROR !!!! { data already exits corresponding to this roll number }\n");
				return i;
			}
		}
		return -1;
	}



	// ***************************** ADD STUDENT **************************************//
	void add(student_marks_t *aStudent,int32_t userRollNumber){
		if (find_RollNumber(userRollNumber) != -1){
			printf("  ERROR !!!! { roll number already exits } !!!!");
			return; // exit(0);
		}
		for (int i = 0; i < MAX_RECORD; i++){
					if (aStudent[i].valid == 0){
						count = i;
						fflush(stdout);
						aStudent[count].rollNumber = userRollNumber;
						printf("social science marks : \n");
						scanf("%hhd",&aStudent[count].sst);
						printf("science marks : \n");
						scanf("%hhd",&aStudent[count].science);
						printf("math marks : \n");
						scanf("%hhd",&aStudent[count].math);
						printf("hindi marks : \n");
						scanf("%hhd",&aStudent[count].hindi);
						printf("English marks : \n");
						scanf("%hhd",&aStudent[count].english);
						aStudent[count].valid = 1;
						return ;

					}
		}
		printf("ERROR : DATA BASE IS FULLL \n");

	}



	// ***************************** SHOW DETAIL **************************************//
	void showdetail(int32_t userRollNumber){
		int32_t index = find_RollNumber(userRollNumber);
		if (index == -1){
			// no such data exits
			printf("no such data exits \n");
			return;
		}


			printf("social science marks :%d \n",aStudent[index].sst);

			printf("science marks        :%d \n",aStudent[index].science);

			printf("math marks           :%d \n",aStudent[index].math);

			printf("hindi marks          :%d \n",aStudent[index].hindi);

			printf("English marks        : %d \n",aStudent[index].english);

			printf(" ***** ******* ******** ******** ******** **** \n");

	}


	/// ***************************** SHOW DETAIL **************************************//
	void delete(int32_t userRollNumber){

		int index = find_RollNumber(userRollNumber);

		if (index == -1){
			printf(" no such record exits corresponding to this roll number \n");
			return;
		}

		aStudent[index].valid = 0;
		printf("**************** RECORD DELETED ***************** \n");
	}


//********************************** showing the student roll number ***************************//
	void show_RollNumber(void){
		//
		// need to be done
		for(int i = 0 ; i< MAX_RECORD;i++){
			if(aStudent[i].valid ==1){
				printf(" ROLL NUMBER ARE AS FOLLOWS : %d \n",aStudent[i].rollNumber);
			}
		}
	}













