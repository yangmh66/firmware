#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "usart.h"

#include "linenoise.h"
#include "parser.h"
#include "shell.h"

/* Shell Command handlers */
void shell_unknown_cmd(char parameter[][MAX_CMD_LEN], int par_cnt);
void shell_calibrate(char parameter[][MAX_CMD_LEN], int par_cnt);
void shell_clear(char parameter[][MAX_CMD_LEN], int par_cnt);
void shell_help(char parameter[][MAX_CMD_LEN], int par_cnt);
void shell_license(char parameter[][MAX_CMD_LEN], int par_cnt);

/* The identifier of the command */
enum SHELL_CMD_ID {
	unknown_cmd_ID,
	calibrate_ID,
	clear_ID,
	help_ID,
	license_ID,
	SHELL_CMD_CNT
};

//First string don't need to store anything for unknown commands
command_list shellCmd_list[SHELL_CMD_CNT] = {
	CMD_DEF(unknown_cmd, shell),
	CMD_DEF(calibrate, shell),
	CMD_DEF(clear, shell),
	CMD_DEF(help, shell),
	CMD_DEF(license, shell),
};

char *completion_list[SHELL_CMD_CNT];

/**
  * @brief  Asking user a question and get the result in form of y or n
  * @param  string pointer (prompt or question)
  * @retval char (Y and Y for yes, n and N for no)
  */
char shell_confirm(char *prompt)
{
	char confirm_result;
	while(1) {
		serial1.printf("%s", prompt);
		confirm_result = (char)serial1.getch();
		serial1.printf("%c\n\r", confirm_result);

		if(confirm_result == 'n' || confirm_result == 'N' || confirm_result != 'y' || confirm_result != 'Y')
			return confirm_result;
	}
}

static void shell_linenoise_completion(const char *buf, linenoiseCompletions *lc)
{
	int i; //i = 1 to ignore the "UNKNOWN_COMMAND" string

	for (i = 1; i < SHELL_CMD_CNT; i++) {
		unsigned int j;
		for(j = 0; j < strlen(buf); j++) {
			if(buf[j] != shellCmd_list[i].str[j]) break;

			//Last time
			if(j == (strlen(buf) - 1))
				linenoiseAddCompletion(lc, completion_list, shellCmd_list[i].str);
		}
	}
}

void shell_task(void)
{
	char shell_text[256] = {'\0'};

	/* Clear the screen */
	serial1.printf("\x1b[H\x1b[2J");
	/* Show the prompt messages */
	serial1.printf("UrsusPilot onboard shell\n\r");
	serial1.printf("Please type \"help\" to get more informations\n\r");

	while (1) {
		linenoiseSetCompletionCallback(shell_linenoise_completion);

		command_data shell_cd = {.par_cnt = 0};

		linenoise("shell > ", shell_text);

		if (shell_text[0] == '\0')
			continue;

		commandExec(shell_text, &shell_cd, shellCmd_list, SHELL_CMD_CNT);

		linenoiseHistoryAdd(shell_text);
	}
}
/**** Customize command function ******************************************************/
void shell_unknown_cmd(
	__attribute__((__unused__))char parameter[][MAX_CMD_LEN], 
	__attribute__((__unused__))int par_cnt)
{
	serial1.printf("Command not found\n\r");
}

void shell_clear(
	__attribute__((__unused__))char parameter[][MAX_CMD_LEN],
	__attribute__((__unused__))int par_cnt)
{
	linenoiseClearScreen();
}

void shell_help(
	__attribute__((__unused__))char parameter[][MAX_CMD_LEN],
	__attribute__((__unused__))int par_cnt)
{
	serial1.printf("Support commands:\n\r");
	serial1.printf("clear\t\tClear the screen\n\r");
	serial1.printf("calibrate\tSensor, RC calibration\n\r");
	serial1.printf("help\t\tShow the list of all commands\n\r");
	serial1.printf("license\t\tShow the lincese and authors\n\r");
}

void shell_license(
	__attribute__((__unused__))char parameter[][MAX_CMD_LEN],
	__attribute__((__unused__))int par_cnt)
{
	serial1.printf("Project UrsusPilot\n\r");

	serial1.printf("Copyright (c) 2015 - MIT License\n\r\n\r");

	serial1.printf("QCopterFC\n\r");
	serial1.printf("Wen-Hung Wang <Hom19910422@gmail.com>\n\r\n\r");

	serial1.printf("Linenoise\n\r");
	serial1.printf("Antirez <antirez@gmail.com>\n\r\n\r");

	serial1.printf("Contributors of UrsusPilot\n\r");
	serial1.printf("Da-Feng Huang <fantasyboris@gmail.com>\n\r");
	serial1.printf("Cheng-De Liu <zxc2694zxc2694@gmail.com>\n\r");
	serial1.printf("Cheng-Han Yang <poemofking@gmail.com>\n\r");
	serial1.printf("Shengwen Cheng <shengwen1997.tw@gmail.com>\n\r");
	serial1.printf("Ming <ming6842@hotmail.com>\n\r");
	serial1.printf("Jack Hsu <jackhsu98@gmail.com>\n\r");
}
