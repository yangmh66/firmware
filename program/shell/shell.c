#include <stddef.h>
#include <string.h>
#include <stdlib.h>

/* Shell Command handlers */
void shell_unknown_cmd(char parameter[][MAX_CMD_LEN], int par_cnt);
void shell_clear(char parameter[][MAX_CMD_LEN], int par_cnt);
void shell_help(char parameter[][MAX_CMD_LEN], int par_cnt);
void shell_license(char parameter[][MAX_CMD_LEN], int par_cnt);

/* The identifier of the command */
enum SHELL_CMD_ID {
	unknown_cmd_ID,
	clear_ID,
	help_ID,
	license_ID,
	SHELL_CMD_CNT
};

//First string don't need to store anything for unknown commands
command_list shellCmd_list[SHELL_CMD_CNT] = {
	CMD_DEF(unknown_cmd, shell),
	CMD_DEF(clear, shell),
	CMD_DEF(help, shell),
	CMD_DEF(license, shell),
};

/**** Shell task **********************************************************************/
void shell_linenoise_completion(const char *buf, linenoiseCompletions *lc)
{
	int i; //i = 1 to ignore the "UNKNOWN_COMMAND" string

	for (i = 1; i < SHELL_CMD_CNT; i++) {
		if (buf[0] == shellCmd_list[i].str[0])
			linenoiseAddCompletion(lc, shellCmd_list[i].str);
	}
}

void shell_task()
{
	//Waiting for system finish initialize
	while (system_status != SYSTEM_INITIALIZED);

	/* Clear the screen */
	serial.printf("\x1b[H\x1b[2J");
	/* Show the prompt messages */
	serial.printf("UrsusPilot onboard shell\n\r");
	serial.printf("Please type \"help\" to get more informations\n\r");

	while (1) {
		linenoiseSetCompletionCallback(shell_linenoise_completion);

		command_data shell_cd = {.par_cnt = 0};

		char *shell_str = linenoise("shell > ");

		if (shell_str == NULL)
			continue;

		commandExec(shell_str, &shell_cd, shellCmd_list, SHELL_CMD_CNT);

		linenoiseHistoryAdd(shell_str);
	}
}
/**** Customize command function ******************************************************/
void shell_unknown_cmd(char parameter[][MAX_CMD_LEN], int par_cnt)
{
	serial.printf("Command not found\n\r");
}

void shell_clear(char parameter[][MAX_CMD_LEN], int par_cnt)
{
	linenoiseClearScreen();
}

void shell_help(char parameter[][MAX_CMD_LEN], int par_cnt)
{
	serial.printf("\n\rSupport commands:\n\r");
	serial.printf("clear  \tClear the screen\n\r");
	serial.printf("help \tShow the list of all commands\n\r");
}

void shell_license(char parameter[][MAX_CMD_LEN], int par_cnt)
{
	serial.printf("Project UrsusPilot\n\r");

	serial.printf("Copyright (c) 2015 - MIT License\n\r\n\r");

	serial.printf("QCopterFC\n\r");
	serial.printf("Wen-Hung Wang <Hom19910422@gmail.com>\n\r\n\r");

	serial.printf("Linenoise\n\r");
	serial.printf("Antirez <antirez@gmail.com>\n\r\n\r");

	serial.printf("Contributors of UrsusPilot\n\r");
	serial.printf("Da-Feng Huang <fantasyboris@gmail.com>\n\r");
	serial.printf("Cheng-De Liu <zxc2694zxc2694@gmail.com>\n\r");
	serial.printf("Cheng-Han Yang <poemofking@gmail.com>\n\r");
	serial.printf("Shengwen Cheng <shengwen1997.tw@gmail.com>\n\r");
	serial.printf("Ming <ming6842@hotmail.com>\n\r");
	serial.printf("Jack Hsu <jackhsu98@gmail.com>\n\r");
}

