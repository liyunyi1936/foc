#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Command.h"
Command command;
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
float v = 0;
float a = 0;
void re_command(float *num)
{
	*num = *num + 1;
}
void doTarget_v(char* cmd) { command.scalar(&v, cmd); }
void doTarget_a(char* cmd) { command.scalar(&a, cmd); }
main()
{
	command.add("T", doTarget_v);
	command.add("A", doTarget_a);
	command.run("A2233");
	printf("%.2f",v);
	printf("%.2f",a);
}
