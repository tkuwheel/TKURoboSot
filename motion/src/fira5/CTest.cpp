#include "CTest.h"
CTEST::CTEST(int argc, char **argv)
{
	std::cout << *argv <<std::endl;
}
CTEST::~CTEST()
{

}
int CTEST::testFunc()
{
	member = 10;
	std::cout<< member << std::endl;
	return 1;
}
