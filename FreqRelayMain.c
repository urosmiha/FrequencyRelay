
#include <stdio.h>
#include<stdlib.h>
#include "system.h"
#include "altera_avalon_pio_regs.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/timers.h"
#include "FreeRTOS/queue.h"
#include "freertos/semphr.h"

#define ReadFreq_Task_P      	(tskIDLE_PRIORITY+4)
#define ManageLoads_Task_P      (tskIDLE_PRIORITY+3)
#define UpdateLoads_Task_P 		(tskIDLE_PRIORITY+2)

TimerHandle_t Timer_Reset;
TimerHandle_t ManageLoadsTimer;

TaskHandle_t H_ManageLoads;

static QueueHandle_t Q_ReadFreqInfo;

// Definition of Semaphore
SemaphoreHandle_t shared_resource_sem;
SemaphoreHandle_t next_signal;

// Defines time and value of the Frequency reading.
typedef struct{
	TickType_t read_time;
	double freq_value;
}FreqInfo;

FreqInfo PrevFreq = {0,0};
FreqInfo CurrentFreq = {0,0};

//---------------------------
// Functions:
void InitiateInterrupts(void);
void InitiateSharedResources(void);
void CreateTasks(void);


// FOR DEBUGGING
int temp = 0;


//------------------------------ START ----------------------------

// Frequency reading ISR
void freq_relay(){

	// Get frequency reading and time it occurred.
	FreqInfo FreqRead = {xTaskGetTickCountFromISR(), (16000/(double)IORD(FREQUENCY_ANALYSER_BASE, 0))};

	//Queue it so Read frequency can read it.
	long feedback = xQueueSendToBackFromISR( Q_ReadFreqInfo, &FreqRead, pdFALSE );

//	puts(feedback ? "" : "FAIL");  // ---- FOR DEBUG
	return;
}

void ReadFreq(void *pvParameters ){

	while(1){

		if(xQueueReceive( Q_ReadFreqInfo, &CurrentFreq, portMAX_DELAY ) == pdTRUE ) {
			// Check if the current freq time is 0 - means that we performed the very first reading and we don't want the rate of change to be drastic since the prev freq is zero. so set it to current freq.
			if(CurrentFreq.read_time == 0) {
				PrevFreq.freq_value = CurrentFreq.freq_value;
			}


			// Do calculations

			// Check for the abnormalities
				// Reset the timer is needed
				// Call the manage loads if the very 1st abnormality is detected

			if(temp == 1) {
				xTaskNotifyGive(H_ManageLoads);	// Notify from the task
				temp = 0;
			}


//			xTimerReset(ManageLoadsTimer, 10);


//			printf("Current : %f @ %d ms \n", CurrentFreq.freq_value, CurrentFreq.read_time);
//			printf("Prev : %f @ %d ms \n", PrevFreq.freq_value, PrevFreq.read_time);

			// Update prev value for the next time
			PrevFreq.read_time = CurrentFreq.read_time;
			PrevFreq.freq_value = CurrentFreq.freq_value;
		}

	}
}

void ManageLoads( void *pvParameters ){
	while(1) {

		ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		printf("Managing Loads ... \n");
	}

}

// Read switches and set LED values.
void UpdateLoads( void *pvParameters ) {
	while(1) {
		
	}
}


void PushButtonISR(){

	unsigned int button = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	printf("Button %d Pressed \n", button);

	if(button == 4) {
		temp = 1;
	} else if(button == 2) {
		temp = 0;
	}

	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); //write 1 to clear all detected falling edges
	return;
}


void vTimerCallback(TimerHandle_t t_timer){ //Timer time up (500 ms)
	printf("500 ms is up \n");
	vTaskNotifyGiveFromISR(H_ManageLoads, pdFALSE);
}


int main()
{
	printf("Hello from Nios \n");

	InitiateInterrupts();
	InitiateSharedResources();
	CreateTasks();

	vTaskStartScheduler();

	while(1){

	}

  return 0;
}

void InitiateInterrupts(void) {
//	--------------------
	// Button Interrupts
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7); //enable interrupt for all three push buttons (Keys 1-3 -> bits 0-2)
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); //write 1 to edge capture to clear pending interrupts
	alt_irq_register(PUSH_BUTTON_IRQ, 0, PushButtonISR);  //register ISR for push button interrupt request
//	-------------------
	// Frequency Relay Interrupt enable
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);
//	------------------

}

void InitiateSharedResources(void) {
//	---------------
	// Initialise the Queues and their size
	Q_ReadFreqInfo = xQueueCreate( 100, sizeof(FreqInfo) );
//	--------------

}

void CreateTasks(void) {
//	-------------
	xTaskCreate( ReadFreq, "0", configMINIMAL_STACK_SIZE, NULL, ReadFreq_Task_P, &Timer_Reset );
	xTaskCreate( ManageLoads, "0", configMINIMAL_STACK_SIZE, NULL, ManageLoads_Task_P, &H_ManageLoads );
	xTaskCreate( UpdateLoads, "0", configMINIMAL_STACK_SIZE, NULL, UpdateLoads_Task_P, &Timer_Reset );

	ManageLoadsTimer = xTimerCreate("ManageLoadsTimer", 5000, pdTRUE, NULL, vTimerCallback);

	if (xTimerStart(ManageLoadsTimer, 0) != pdPASS){
		printf("Cannot start timer");
	}
}




