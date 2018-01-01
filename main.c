#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "stdio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"

// Macro to use CCM (Core Coupled Memory) in STM32F4
#define CCM_RAM __attribute__((section(".ccmram")))

#define FPU_TASK_STACK_SIZE 256

StackType_t fpuTaskStack[FPU_TASK_STACK_SIZE] CCM_RAM;  // Put task stack in CCM
StaticTask_t fpuTaskBuffer CCM_RAM;  // Put TCB in CCM

void init_USART3(void);
void init_timer(void);
void PositionServoPulse(uint32_t pulse);


__IO uint16_t ServoMinAngle = 20;		// 0
__IO uint16_t ServoMaxAngle = 2500;		// 180

void test_FPU_test(void* p);

int main(void) {
  SystemInit();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  init_USART3();
  init_timer();

  // Create a task
  // Stack and TCB are placed in CCM of STM32F4
  // The CCM block is connected directly to the core, which leads to zero wait states
  xTaskCreateStatic(test_FPU_test, "FPU", FPU_TASK_STACK_SIZE, NULL, 1, fpuTaskStack, &fpuTaskBuffer);

  printf("System Started!\n");
  vTaskStartScheduler();  // should never return

  for (;;);
}

void vApplicationTickHook(void) {
}

/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created.  It is also called by various parts of the
   demo application.  If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  for(;;);
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
   task.  It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()).  If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void) {
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
  (void) pcTaskName;
  (void) pxTask;
  /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for(;;);
}

StaticTask_t xIdleTaskTCB CCM_RAM;
StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE] CCM_RAM;

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
  /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
  state will be stored. */
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

  /* Pass out the array that will be used as the Idle task's stack. */
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;

  /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
  Note that, as the array is necessarily of type StackType_t,
  configMINIMAL_STACK_SIZE is specified in words, not bytes. */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCB CCM_RAM;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH] CCM_RAM;

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void SetServoPosition(uint32_t angle){
	uint32_t range = ServoMaxAngle - ServoMinAngle;
	uint32_t unit = range / 180;

	uint32_t step = unit * angle;

	if(step == 0)
		step = ServoMinAngle;

	if(step > ServoMaxAngle)
		step = ServoMaxAngle;

	PositionServoPulse(step);
}

void PositionServoPulse(uint32_t pulse){
	  TIM_OCInitTypeDef  TIM_OCInitStructure;
	/* Configure timer for PWM - channel 1*/
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = pulse;
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	  //notice the number 1 in TIM_OC1Init
	  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
	while(nCount--)
	{
	}
}

void test_FPU_test(void* p) {
	float ff = 1.0f;
	printf("Start FPU test task.\n");
	for (;;) {
		float s = sinf(ff);
		ff += 1.0f;
		//ff += s;
		printf("sin: %f\r\n", s);
		SetServoPosition(90);
		//SetServoPosition(180);
		//SetServoPosition(0);

		/*
		SetServoPosition(0);
		Delay(50000000);

		SetServoPosition(90);
		Delay(50000000);

		SetServoPosition(180);
		Delay(50000000);
		*/
		// TODO some other test

		vTaskDelay(100);
	}

	vTaskDelete(NULL);
}

void SetupPins(){
	/* Enable TIM4 Clock */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);


	  //Enabled GPIOB we are going to use PB6 which is linked to TIM4_CH1 according to the
	  //documentation
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	  GPIO_InitTypeDef GPIO_InitStructure;
	  //initalise pin 6 B - relating to timer 4 channel 1
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  //link the pins up to the timer - AF stands for alternate function
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
}

void init_timer(void)
{
	uint32_t prescalar = (uint16_t) ( ( (SystemCoreClock / 2) / 1000000) - 1);
	uint32_t period = 20000;
	SetupPins();

	/*Structures used in the configuration*/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* Setup timer defaults */
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalar;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* Configure timer for PWM - channel 1*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//notice the number 1 in TIM_OC1Init
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);


	TIM_ARRPreloadConfig(TIM4, ENABLE);

	/* Start timer*/
	TIM_Cmd(TIM4, ENABLE);
}

/*
 * Configure USART3(PB10, PB11) to redirect printf data to host PC.
 */
void init_USART3(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

  USART_InitStruct.USART_BaudRate = 115200;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART3, &USART_InitStruct);
  USART_Cmd(USART3, ENABLE);
}
