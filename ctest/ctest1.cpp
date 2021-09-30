#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "com.h"
char buf[255];
void wifi_print(char * s,double num)
{
	char str[255];
	char n[255];
	sprintf(n, "%.2f",num);
	strcpy(str,s);
	strcat(str, n);
	strcat(buf+strlen(buf), str);
	strcat(buf, ",");
}
float v = 0.1;
void re_command(float *num)
{
	*num = *num + 1;
}
char* str = "abc23";
char* cmd_id = "abc";
main()
{
	re_command(&v);
	char s[strlen(cmd_id)];
	
	strncpy(s,str,strlen(cmd_id));
	printf("%s", s);
	if(strcmp(cmd_id, s) == 0)
	{
		printf("%f", v);
		v = atof(str+strlen(cmd_id));
		printf("%f", v);
	}
		
//	char* da;
//	da = "123456";
//	if (da[0] == '2')
    
}
