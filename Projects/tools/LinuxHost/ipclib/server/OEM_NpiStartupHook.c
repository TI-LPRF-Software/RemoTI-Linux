// OEMS SHOULD CHANGE THE FUNCTION BELOW to meet their needs.
// It MUST take the argc & argv parameters passed in from main,
// and it MUST return a bool that is true unless it wants main
// to terminate the process after returning.

#include <stdbool.h>
#include <stdlib.h>

#include "OEM_NpiStartupHook.h"

bool OEM_NpiStartupHook(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	return true;
}
