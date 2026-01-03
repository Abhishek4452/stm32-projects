/*
 * area.c
 *
 *  Created on: 24-Dec-2025
 *      Author: debian
 */


// different codes for the area calculation
/*
 * t for triangle
 *  z for trapozidal
 *  c for circle
 *  s for square
 *  r for rectangle
 *
 */


// this code can't handle negative value so enter positive value only
#include <stdio.h>
//#include <stdout.h>
float area;
char symbol;
int num_symbol;

float Rect(){
	float height,base;
	printf("enter the height: ");
	scanf("%f",&height);
	printf("enter the base : ");
	scanf("%f",&base);
	area = (height*base);
	return area;
}
float triangle(){
	return Rect()/2;
}
void circle(){
	float radius;
	printf("enter the radius :");
	scanf("%f",&radius);
	area = 3.14*(radius)*radius;
	printf("%f",area);
	//return area;
}
float square(){
	return Rect();
}
void trapezoidal(){
	float base1,base2,height;
	printf("enter base1 :");
	scanf("%f",&base1);
	printf("enter base 2:");
	scanf("%f",&base2);
	printf("enter height:");
	scanf("%f",&height);
	area = ((base1+base2)*height)/2;
	printf("area will be %f ",area);
}


// main body of code
int main(void){
	printf(" enter the special character for the special operation: ");
	printf("\n t for triangle");
	printf("\n z for trapezoidal");
	printf("\n c for circle");
	printf("\n s for square");
	printf("\n r for rectangle\n");

	scanf("%c",&symbol);

	// matching the symbol with char
	if (symbol == 't'){
			num_symbol = 1;}
	else if (symbol == 'z'){
			num_symbol = 2;}
	else if (symbol == 'c'){
			num_symbol = 3;}
	else if (symbol == 's'){
		num_symbol = 4;
	}
	else if (symbol == 'r'){
		num_symbol = 5;
	}
	else{
		printf("program closed !");
		return 0;
	}



	switch (num_symbol)
	{
	case 1:
		area = triangle();
		printf("area will be %f",area);
		break;
	case 2:
			trapezoidal();
			break;
	case 3:
			circle();
			break;
	case 4:
			area = square();
			printf("area will be %f ",area);
			break;
	case 5:
			area = Rect();
			printf("area will be %f ",area);
			break;
	default :
		printf("sorry you have entered wrong character ");
	}







	return 0;
}
