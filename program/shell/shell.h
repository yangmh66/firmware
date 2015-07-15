#ifndef __SHELL_H__
#define __SHELL_H__

#include "linenoise.h"

#define MAX_CMD_LEN 64 //256 is too big!
#define MAX_PAR_CNT 10

char shell_confirm(char *prompt); //A utility function
void shell_task(void);

#endif
