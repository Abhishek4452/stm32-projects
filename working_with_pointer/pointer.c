/*
 * pointer.c
 *
 *  Created on: 24-Dec-2025
 *      Author: debian
 */
#include <stdio.h>
long long int g_data = 0xfEff1234abcd1234;
int main(){
	char* pAddress = (char*)&g_data;
	printf("address of the gdata is %p \n",pAddress);
	printf("address value of the data %p, value is %x",pAddress,*pAddress);

	pAddress = pAddress +1;
	printf("\n address value of the data %p, value is %x",pAddress,*pAddress);

	pAddress = pAddress +5;
	printf("\n address value of the data %p, value is %x",pAddress,*pAddress);


/*
 * OUTPUT --
address of the gdata is 0x55702df62018
address value of the data 0x55702df62018, value is 34

address value of the data 0x55702df62019, value is 12
 address value of the data 0x55702df6201e, value is ffffffff
*/
	printf("\n in case of short \n");
		short* pAddress1 = (short*)&g_data; //short int
		printf("\naddress of the gdata is %p \n",pAddress1);
		printf("address value of the data %p, value is %x",pAddress1,*pAddress1);

		pAddress1 = pAddress1 +1;//pointer will move twice
		printf("\n address value of the data %p, value is %x",pAddress1,*pAddress1);

		pAddress1 = pAddress1+5;
		printf("\n address value of the data %p, value is %x",pAddress1,*pAddress1);
/* OUTPUT
 * address of the gdata is 0x556532dce020
address value of the data 0x556532dce020, value is 1234
 address value of the data 0x556532dce022, value is ffffabcd
 address value of the data 0x556532dce02c, value is 0
 */

	return 0;
}
