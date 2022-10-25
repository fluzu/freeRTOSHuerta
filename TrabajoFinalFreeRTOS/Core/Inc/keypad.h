#ifndef __KEYPAD__
#define __KEYPAD__

#include "stm32f4xx_hal_def.h"

#define		ROW1_PIN		GPIO_PIN_10		//	PB5
#define  	ROW1_PORT		GPIOB

#define		ROW2_PIN		GPIO_PIN_14		//	PB4
#define  	ROW2_PORT		GPIOE

#define		ROW3_PIN		GPIO_PIN_12		//	PB3
#define  	ROW3_PORT		GPIOE

#define		ROW4_PIN		GPIO_PIN_10		//	PB7
#define  	ROW4_PORT		GPIOE

#define		COL1_PIN		GPIO_PIN_8		//	PB11
#define  	COL1_PORT		GPIOE

#define		COL2_PIN		GPIO_PIN_2		//	PB10
#define  	COL2_PORT		GPIOB

#define		COL3_PIN		GPIO_PIN_0		//	PA4
#define  	COL3_PORT		GPIOB

#define		COL4_PIN		GPIO_PIN_4		//	PA5
#define  	COL4_PORT		GPIOC

void keypad_init(void);
char keypad_read(void);
#endif
