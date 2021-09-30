// callback function pointer definiton
typedef void (* CommandCallback)(char*); //!< command callback function pointer
class Command
{
	public:
		void add(char* id , CommandCallback onCommand);
		void run(char* str);
		void Command::scalar(float* value,  char* user_cmd);
		bool Commander::isSentinel(char* ch);
	private:
	    // Subscribed command callback variables
	    CommandCallback call_list[20];//!< array of command callback pointers - 20 is an arbitrary number
	    char* call_ids[20]; //!< added callback commands
	    int call_count = 0;//!< number callbacks that are subscribed
	
}
