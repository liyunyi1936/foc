#include "com.h"
void Command::run(char* str){
  for(int i=0; i < call_count; i++){
        if(call_ids[i]){
          call_list[i](&str);
          break;
        }
      }
}
void Command::add(char* id, CommandCallback onCommand){
  call_list[call_count] = onCommand;
  call_ids[call_count] = id;
  call_count++;
}
void Command::scalar(float* value,  char* user_cmd){
  *value = atof(user_cmd);
}
bool Commander::isSentinel(char* ch,char* str)
{
	char s[strlen(ch)];
	strncpy(s,str,strlen(ch));
	if(strcmp(ch, s) == 0)
    	return true;
	else 
  		return false;
}
