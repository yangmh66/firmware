#ifndef FILE_USART_H
#define FILE_USART_H
#include <stdint.h>
typedef struct {
        char ch;
} serial_msg;

typedef struct {
	char (*getch)(void);
	int (*receive)(void);
	void (*putch)(char buf);
	void (*putstr)(const char *str);
	int (*printf)(const char *format, ...);
} serial_t;

extern serial_t serial1;
extern serial_t serial2;

void usart_init(void);
void retarget_init(void);
int _write(int fd, char *ptr, int len);
int _read(int fd, char *ptr, int len);
void _ttywrch(int ch);
void usart2_dma_send(uint8_t *s);
void usart2_dma_init(void);

char usart3_read(void);
void usart3_send(char str);
void USART3_IRQHandler(void);
#endif
