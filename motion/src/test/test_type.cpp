#include <iostream>
using namespace std;
 
int main(int argc, char** argv)
{
	for(int i=0; i<argc; i++){
		cout << argv[i] << endl;
	}
	int test_int;
	char test_char1,test_char2,test_char3,test_char4;
	//test_char = 0x80;
	test_int = 0x12345678;
	//test_int = test_char << 24;
	test_char1 = test_int >> 24;
	test_char2 = test_int >> 16;
	test_char3 = test_int >> 8;
	test_char4 = test_int >> 0;
	cout << hex;
	cout << "teset_char1: " <<(int)test_char1 << endl;
	cout << "teset_char2: " <<(int)test_char2 << endl;
	cout << "teset_char3: " <<(int)test_char3 << endl;
	cout << "teset_char4: " <<(int)test_char4 << endl;
	cout << "teset_int: " <<(int)test_int << endl;
	return 0;
}
