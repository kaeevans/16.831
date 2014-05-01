#include "testproject.h"

#include <iostream>

using namespace std;

testproject *test = NULL;

void terminateHandlerHelper(int sig)
{
	test->terminateHandler(sig);
}

void preInit()
{
	test = new testproject;

	signal(SIGTERM, terminateHandlerHelper);
	signal(SIGABRT, terminateHandlerHelper);
	signal(SIGINT, terminateHandlerHelper);

}



int main (int argc, char *argv[])
{
	preInit();

	test->runMain(argc, argv);

	delete test;
	return 0;

}




