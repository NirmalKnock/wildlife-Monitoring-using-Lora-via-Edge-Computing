
#include <stdint.h>
#define GPIOA_Base_ADDR 						(0x40020000)
#define GPIOA_MODER_REGISTER				(0x40020000)
#define GPIOA_OUTPUT_DATA_REGISTER	(0x40020014)
#define RCC													(0x40023800)
#define RCC_GPIOA										(RCC + 0x30)

void delay(void);

void delay(void){
for(uint32_t i = 0; i<500000;i++);
}

int main(void){

	uint32_t *pRCC_clock_Enable  = (uint32_t*)(RCC_GPIOA);
	*pRCC_clock_Enable |= (1 << 0);

	uint32_t *pModer = (uint32_t *)(GPIOA_MODER_REGISTER);
	*pModer &= ~(3 << 10);
	*pModer |= (1 << 10);
	uint32_t *pOutput_Data_reg = (uint32_t*) (GPIOA_OUTPUT_DATA_REGISTER);
	while(1){


	*pOutput_Data_reg |= (1<<5);


	delay();


	*pOutput_Data_reg &= ~(1<<5);
		delay();
	}
}

