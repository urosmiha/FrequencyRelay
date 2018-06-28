#include <stdio.h>
#include <stdlib.h>
#include "system.h"
#include "altera_avalon_pio_regs.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/timers.h"
#include "FreeRTOS/queue.h"
#include "freertos/semphr.h"
#include "io.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include <unistd.h>
#include "altera_avalon_pio_regs.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"

// --------------------------
// DEFINE Rate of Change Threshold and Lower Value Threshold
double RoC_Threshold = 10.5;
double Freq_Threshold = 50.4;
// Increment Level
int incrementLvl = 2; // By default
// Increment values.
// Use different threshold increments based on the increment level. (Useful for testing).
#define FREQ_CHANGE_1 0.5
#define ROC_CHANGE_1 0.5
#define FREQ_CHANGE_2 0.05
#define ROC_CHANGE_2 0.05
#define FREQ_CHANGE_3 0.005
#define ROC_CHANGE_3 0.005
// --------------------------
// Priorities:
#define ReadFreq_Task_P      	(tskIDLE_PRIORITY+5)
#define ManageLoads_Task_P      (tskIDLE_PRIORITY+4)
#define UpdateLoads_Task_P 		(tskIDLE_PRIORITY+3)
#define UpdateThresholds_P		(tskIDLE_PRIORITY+2)
#define UpdateScreen_P			(tskIDLE_PRIORITY+1)

// Definition of Timers:
TimerHandle_t Timer_Reset;
TimerHandle_t ManageLoadsTimer;

// Definition of Task Handlers:
TaskHandle_t H_ManageLoads;

// Definition of Queues:
static QueueHandle_t Q_ReadFreqInfo;
static QueueHandle_t Q_KeyboardInput;
static QueueHandle_t Q_VGAUpdateValues;
static QueueHandle_t Q_VGAUpdateTime;

// Definition of Semaphore
SemaphoreHandle_t load_state_sem;
SemaphoreHandle_t reading_state_sem;

// --------------------------------
// Variables:
typedef enum { true, false } bool;

typedef enum { ON, OFF, SHED } loadState;
loadState Loads[5] = {OFF,OFF,OFF,OFF,OFF};

typedef enum { RUN, MAINTENANCE } Mode;
Mode SystemMode = RUN;
Mode PreviousMode = RUN;
typedef enum { STABLE, UNSTABLE } StabilityState;
StabilityState currentReading = STABLE;
StabilityState prevReading = STABLE;
StabilityState SystemState = STABLE;

// Defines time and value of the Frequency reading.
typedef struct{
	TickType_t read_time;
	double freq_value;
}FreqInfo;

FreqInfo PrevFreq = {0,0};
FreqInfo CurrentFreq = {0,0};

TickType_t first_shed_start;	// Indicates the time we obtained the reading.
TickType_t shed_time;			// How long it took to shed.
bool isShed = false;
bool isFirstShed = false;
TickType_t FirstShedTime;

// Stores last 100 frequency readings and RoCs. Only used for displaying graph.
double freq[100], dfreq[100];
int freq_id = 99;

bool isFirstReading = true;
//---------------------------
// Functions:
void InitiateInterrupts(void);
void InitiateSharedResources(void);
void CreateTasks(void);

// FOR DEBUGGING
int temp = 0;

//----------------------------------------
//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 			//minimum frequency to draw

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

//------------------------------ START ----------------------------

// Keyboard ISR called when keyboard button is pressed.
// It sends the ascii value of the pressed button to the queue to be read by the UpdateLoads task.
void ps2_isr (void* context, alt_u32 id)
{
  char ascii;
  int status = 0;
  unsigned char key = 0;
  KB_CODE_TYPE decode_mode;
  status = decode_scancode (context, &decode_mode , &key , &ascii) ;

  if ( status == 0 ) //success
  {
	switch ( decode_mode )
	{
		case KB_ASCII_MAKE_CODE :
			xQueueSendToBackFromISR(Q_KeyboardInput, &key, pdFALSE);
			break ;
		default :
			break ;
	}
  }
}

// Frequency reading ISR
// Send the frequency value and time of the reading to the queue to be read by ReadFreq task.
void freq_relay(){

	// Get frequency reading and time it occurred.
	FreqInfo FreqRead = {xTaskGetTickCountFromISR(), (16000/(double)IORD(FREQUENCY_ANALYSER_BASE, 0))};

	//Queue it so Read frequency can read it.
	if(SystemMode == RUN) {
		xQueueSendToBackFromISR( Q_ReadFreqInfo, &FreqRead, pdFALSE );

		xQueueSendToBackFromISR( Q_VGAUpdateValues, &FreqRead.freq_value, pdFALSE );
	}
//	puts(feedback ? "" : "FAIL");  // ---- FOR DEBUG
	return;
}

// Highest priority.
// Read the frequency from the queue and calculate Rate of Change (RoC).
// Check if either frequency is below the frequency threshold or RoC is above RoC threshold.
// Reset the 500 ms timer if any of above 2 conditions is met.
void ReadFreq(void *pvParameters ){

	double RoC = 0;

	while(1){
		if(xQueueReceive( Q_ReadFreqInfo, &CurrentFreq, portMAX_DELAY ) == pdTRUE ) {
			// Check if the current freq time is 0 - means that we performed the very first reading and we don't want the rate of change to be drastic since the prev freq is zero. so set it to current freq.
			if(CurrentFreq.read_time == 0) {
				PrevFreq.freq_value = CurrentFreq.freq_value;
			}

			prevReading = currentReading;

			RoC = (CurrentFreq.freq_value-PrevFreq.freq_value) * 2.0 * CurrentFreq.freq_value * PrevFreq.freq_value / (CurrentFreq.freq_value+PrevFreq.freq_value);

			// Used only for drawing graph. Assign values in here so that we populate the array faster.
			dfreq[freq_id] = RoC;
			freq[freq_id] = CurrentFreq.freq_value;

			freq_id = ++freq_id % 100; //point to the next data (oldest) to be overwritten

			// Mutex prevents it from overwriting the currentReading value before ManagLoads is done reading it.
			if(xSemaphoreTake(reading_state_sem, 5)) {
				// Check for the abnormalities
				if((CurrentFreq.freq_value < Freq_Threshold) || (fabs(RoC) > RoC_Threshold)) {
					first_shed_start = 	CurrentFreq.read_time;
					currentReading = UNSTABLE;
				}
				else {
					currentReading = STABLE;
				}
				xSemaphoreGive(reading_state_sem);
			}

			// If we read the very first freq change after the system was stable, perform manage loads as soon as possible.
			if((SystemState == STABLE) && (currentReading == UNSTABLE)) {
//				first_shed_start = 	CurrentFreq.read_time;
				isFirstShed = true;
				xTaskNotifyGive(H_ManageLoads);
			}

			// Reset the timer based on the readings state - Every time there is a change in the reading start timing 500 ms again.
			if(currentReading != prevReading) {
				xTimerReset(ManageLoadsTimer, 5);
			}

			// Update prev value for the next time
			PrevFreq.read_time = CurrentFreq.read_time;
			PrevFreq.freq_value = CurrentFreq.freq_value;
		}
	}
}

// Second Highest Priority
// Sets the State of the Load to Shed or Stable based on the reading.
// Executes when notified.
void ManageLoads( void *pvParameters ){
	int i = 0;

	while(1) {

		ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		// Mutex prevents it from writing to Loads at the same time as UpdateLoads.
		// Prevents it from modifying the systemState and Loads while UpdateLoads is reading them.
		if(xSemaphoreTake(load_state_sem, 5)) {
			// This mutex prevents this task from reading the currentReading state while ReadFreq is modifying it.
			if(xSemaphoreTake(reading_state_sem, 5)) {
				// If the reading is unstable (i.e. there was a frequency change) shed the loads.
				if(currentReading == UNSTABLE) {
					// Check which load is ON and SHED it.
					for(i = 0; i < 5; i++) {
						if(Loads[i] == ON) {
							Loads[i] = SHED;
							isShed = true;
							break;
						}
					}
					if(SystemState == STABLE) {
						xTimerReset(ManageLoadsTimer, 5);
					}
				} else {	// If the reading is stable turn SHED loads ON.
					for(i = 4; i >= 0; i--) {
						if(Loads[i] == SHED) {
							Loads[i] = ON;
							break;
						}
					}
				}
				xSemaphoreGive(reading_state_sem);
			}

			// Check the overall system state. (i.e. at least one SHED load = UNSTABLE).
			StabilityState SystemStateTmp = STABLE;
			for(i = 0;  i < 5; i++){
				if(Loads[i] == SHED) {
					SystemStateTmp = UNSTABLE;
					break;
				}
			}

			// If the system is stays stable aftre 500 ms than turn the timer off so we don't waste resources.
			// Timer will get turn back on in the ReadFreq task.
			if(SystemStateTmp == STABLE) {
				xTimerStop(ManageLoadsTimer, 5);
			}

			SystemState = SystemStateTmp;

			xSemaphoreGive(load_state_sem);
		}
	}
}

// Third Highest Priority.
// Read switches and set LED values. Periodic Task - Since we are polling switch values.
void UpdateLoads( void *pvParameters ) {

	TickType_t last_wake_time;

	while(1) {

		unsigned int uSwitches = 0x00;
		unsigned int GreenMask = 0x00;
		unsigned int RedMask = 0x00;
		unsigned int greenLedValue = 0x00;
		unsigned int redLedValue = 0x00;

		// Try to execute every 5 ms.
		last_wake_time = xTaskGetTickCount();
		vTaskDelayUntil(&last_wake_time, 5);

		uSwitches = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;

		// This mutex prevents the task from changing the SystemState and Loads while ManageLoads task is reading it.
		// Also makes sure that this task is not truing to modify the Loads at the same time as ManageLoads
		if(xSemaphoreTake(load_state_sem, 5)) {

			int i = 0;

			// If we go to maintenance mode turn off the timer and set all shed loads to on.
			if((SystemMode == MAINTENANCE) && (PreviousMode == RUN)){
				PreviousMode = MAINTENANCE;
				SystemState = STABLE;
				xTimerStop(ManageLoadsTimer, 5);
				for(i = 0; i < 5; i++) {
					if(Loads[i] == SHED) {
						Loads[i] = ON;
					}
				}
			}

			// Indicate which loads are ON based on the switch readig and the state of the Loads(i.e. is it not SHED).
			for(i = 4;i>=0; i--){
			  if(((uSwitches/(pow(2,i))) >= 1)){
				  if((Loads[i] != SHED) && (SystemState == STABLE)){
					Loads[i] = ON;
				  }
				uSwitches -= pow(2,i);
			  }
			  else {
				  Loads[i] = OFF;
			  }
			}

			// Set up masks for red and green LEDs based on the state of the load.
			for(i = 0; i < 5; i++) {
				if(Loads[i] == SHED) {
					GreenMask += pow(2,i);
				} else if(Loads[i] == ON) {
					RedMask += pow(2,i);
				}
			}
			xSemaphoreGive(load_state_sem);
		}

		greenLedValue = GreenMask;
		redLedValue = RedMask;

		// THIS IS WHERE ACTUAL LOAD SWITCHING HAPPENS (Output to the physical environment)
		// Set LEDs based on the loads state
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLedValue);
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, redLedValue);

		// Get the shed time here - since this is where we are setting the actual output.
		// But only get the time if we have shed the load.
		if(isShed == true) {

			shed_time = xTaskGetTickCount() - first_shed_start;

			xQueueSendToBack( Q_VGAUpdateTime, &shed_time, pdFALSE );

			isShed = false;
		}
	}
}

// Second to last Priority.
// Updates the global variables Freq_Threshold and RoC_Threshold.
// Checks if the reading is UP, DOWN, LEFT or RIGHT arrow.
void UpdateThresholds ( void *pvParameters ) {

	double FREQ_CHANGE = 0.0;
	double ROC_CHANGE = 0.0;

	while(1) {
		unsigned char inputKey;
		if(xQueueReceive( Q_KeyboardInput, &inputKey, portMAX_DELAY ) == pdTRUE ) {

			// Update increment level		// KEY
			if(inputKey == 22) {			//  1
				incrementLvl = 1;
			} else if(inputKey == 30) {		//  2
				incrementLvl = 2;
			} else if(inputKey == 38) {		//  3
				incrementLvl = 3;
			}

			// Since the isr detects bot get press and key release this function will get called twice for one button press.
			// That is why we decide to use 0.5, 0.05, 0.005. So thresholds get incremented by 1, 0.1 and 0.01.
			// Change increments based on the increment level.
			if(incrementLvl == 1) {
				FREQ_CHANGE = FREQ_CHANGE_1;		// +/- 0.5
				ROC_CHANGE = ROC_CHANGE_1;
			} else if(incrementLvl == 2) {
				FREQ_CHANGE = FREQ_CHANGE_2;		// +/- 0.05
				ROC_CHANGE = ROC_CHANGE_2;
			} else {
				FREQ_CHANGE = FREQ_CHANGE_3;		// +/- 0.005
				ROC_CHANGE = ROC_CHANGE_3;
			}

//			Change thresholds using arrows
			if(inputKey == 117) {				// UP - increase frequency threshold
				Freq_Threshold += FREQ_CHANGE;
			}
			else if(inputKey == 114) {			// DOWN - decrease feq threshold
				Freq_Threshold -= FREQ_CHANGE;
				if(Freq_Threshold < 0.0) {
					Freq_Threshold = 0.0;
				}
			}
			else if(inputKey == 107) {			// LEFT - decrease ROC threshold
				RoC_Threshold -= ROC_CHANGE;
				if(RoC_Threshold < 0.0) {
					RoC_Threshold = 0.0;
				}
			}
			else if(inputKey == 116) {			// RIGHT - increase ROC threshold
				RoC_Threshold += ROC_CHANGE;
			}

			// Send a random value to the vga update task queue just to indicate that it should display new thresholds.
			// Update sreen task can just then read global value of RoC and Freq thresholds.
			double tmp = 0.0;
			xQueueSendToBack( Q_VGAUpdateValues, &tmp, pdFALSE );
		}
	}
}

// ----------------- DISPLAY STUFF -------------------
void UpdateScreen(void *pvParameters) {

	TickType_t last_wake_time;

	int lastFiveCount = 0;
	double LastFiveReading[5];

	TickType_t TimeTaken[5] = {0};
	TickType_t avgTime;
	TickType_t maxTime;
	TickType_t minTime;
	int timeCount = 0;
	int timeReadings = 0;

	Line line_freq, line_roc;

//	----------------------------------------
//	---- GRAPH LINES ----------

	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("Cannot find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
//	-------------------------------
//	---- LABELS ------

	//initialize character buffer
	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);

	alt_up_char_buffer_string(char_buf, "Mode: ", 1, 1);
	alt_up_char_buffer_string(char_buf, "System Status: ", 22, 1);
	alt_up_char_buffer_string(char_buf, "Total Run Time: ", 52, 1);

//	-------------------------------------------------------------------
	// Add labels
	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);

//	--------------------------------
	alt_up_char_buffer_string(char_buf, "Frequency Threshold : ", 7, 42);
	alt_up_char_buffer_string(char_buf, "RoC Threshold       : ", 7, 44);
	alt_up_char_buffer_string(char_buf, "_____________________", 7, 46);
	alt_up_char_buffer_string(char_buf, "First Shed Time    : ", 7, 48);
	alt_up_char_buffer_string(char_buf, "Max Reaction Time  : ", 7, 50);
	alt_up_char_buffer_string(char_buf, "Min Reaction Time  : ", 7, 52);
	alt_up_char_buffer_string(char_buf, "Avg Reaction Time  : ", 7, 54);

	alt_up_char_buffer_string(char_buf, "Last 5 Freq Readings", 50, 42);
	alt_up_char_buffer_string(char_buf, "_____________________", 50, 43);

//	----------------------------------------------------------------------

	char temp_buf[6];
	unsigned int run_time;

	while(1) {
		last_wake_time = xTaskGetTickCount();
		vTaskDelayUntil(&last_wake_time, 33);

//		------------------------------------------------------
		// Update system mode
		if(SystemMode == RUN) {
			alt_up_char_buffer_string(char_buf, " RUN        ", 6, 1);
		} else {
			alt_up_char_buffer_string(char_buf, " MAINTENANCE", 6, 1);
		}

		// display system state
		if(SystemState == STABLE) {
			alt_up_char_buffer_string(char_buf, "STABLE  ", 37, 1);
		} else {
			alt_up_char_buffer_string(char_buf, "UNSTABLE", 37, 1);
		}

		// Update run time
		run_time = xTaskGetTickCount();
		sprintf(temp_buf, "%02d:%02d:%02d", (run_time/3600000) % 24, (run_time/60000) % 60, (run_time/1000) % 60);
		alt_up_char_buffer_string(char_buf, temp_buf, 68, 1);

//		--------------------------------------------------------
		// Display Last 5 readings
		double tempVal;
		// Read Queue for new values and update the current values in the array.
		if(xQueueReceive( Q_VGAUpdateValues, &tempVal, portMAX_DELAY ) == pdTRUE ) {

			LastFiveReading[lastFiveCount] = tempVal;

			if(lastFiveCount > 3) {
				lastFiveCount = 0;
			} else {
				lastFiveCount++;
			}

			int i;
			int j = 45;
			// Loop through the array and update the last five readings on the screen.
			for(i = 0; i < 5; i++) {
				sprintf(temp_buf, "%4.4f  Hz", LastFiveReading[i]);
				alt_up_char_buffer_string(char_buf,temp_buf, 55, j);
				j += 2;
			}

			// Display Frequency and RoC thresholds
			sprintf(temp_buf, "%4.4f  Hz", Freq_Threshold);
			alt_up_char_buffer_string(char_buf, temp_buf, 29, 42);
			sprintf(temp_buf, "%4.4f  Hz/s", RoC_Threshold);
			alt_up_char_buffer_string(char_buf, temp_buf, 29, 44);

			//		-----------------------------------------------
			//clear old graph to draw new graph
			alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
			alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);
			j = 0;
			for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
				if (((int)(freq[(freq_id+j)%100]) > MIN_FREQ) && ((int)(freq[(freq_id+j+1)%100]) > MIN_FREQ)){
					//Calculate coordinates of the two data points to draw a line in between
					//Frequency plot
					line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
					line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(freq_id+j)%100] - MIN_FREQ));

					line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
					line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(freq_id+j+1)%100] - MIN_FREQ));

					//Frequency RoC plot
					line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
					line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(freq_id+j)%100]);

					line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
					line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(freq_id+j+1)%100]);

					//Draw
					alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
					alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
				}
			}
		}
//		------------------------------------------------
//		 Display max, min, avg and first load shed time if there are any new values to be displayed.
		TickType_t tempTime;
		if(xQueueReceive( Q_VGAUpdateTime, &tempTime, (TickType_t) 2 ) == pdTRUE ) {

			// Add new time reading to the time array. Update the oldest reading with new reading.
			TimeTaken[timeCount] = tempTime;
			if(timeCount > 3) {
				timeCount = 0;
			} else {
				timeCount++;
			}

			if(timeReadings < 5){
				timeReadings++;
			}

			// If this is the very 1st reading then just assign curent value to min and max.
			if(isFirstReading == true) {
				maxTime = tempTime;
				minTime = tempTime;
				avgTime = tempTime;
				isFirstReading = false;
			} else {
				avgTime = (TimeTaken[0] + TimeTaken[1] + TimeTaken[2] + TimeTaken[3] + TimeTaken[4]) / timeReadings;
				// Update max and min time if new reading is higher or lower.
				if(tempTime > maxTime) {
					maxTime = tempTime;
				}else if(tempTime < minTime) {
					minTime = tempTime;
				}
			}

			if(isFirstShed == true) {
				FirstShedTime = tempTime;
				isFirstShed = false;
			}

			sprintf(temp_buf, " %4d  ms", FirstShedTime);
			alt_up_char_buffer_string(char_buf, temp_buf, 27, 48);

			// Display Average Reaction time
			sprintf(temp_buf, " %4d  ms", avgTime);
			alt_up_char_buffer_string(char_buf, temp_buf, 27, 54);

			// Display max and min time
			sprintf(temp_buf, " %4d  ms", maxTime);
			alt_up_char_buffer_string(char_buf, temp_buf, 27, 50);
			sprintf(temp_buf, " %4d  ms", minTime);
			alt_up_char_buffer_string(char_buf, temp_buf, 27, 52);
		}
	}
}


// ---------------------------------------------------
// Change running mode
void PushButtonISR(){

	unsigned int button = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	if(button == 4) {
		if(SystemMode == RUN) {
			SystemMode = MAINTENANCE;
//			printf("M \n");
		}
		else if(SystemMode == MAINTENANCE) {
//			printf("R \n");
			SystemMode = RUN;
			PreviousMode = RUN;
		}
	}

	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); //write 1 to clear all detected falling edges
	return;
}


void vTimerCallback(TimerHandle_t t_timer){ //Timer time up (500 ms)
//	printf("500 ms is up \n");
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
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4); //enable interrupt for all three push buttons (Keys 1-3 -> bits 0-2)
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x4); //write 1 to edge capture to clear pending interrupts
	alt_irq_register(PUSH_BUTTON_IRQ, 0, PushButtonISR);  //register ISR for push button interrupt request
//	-------------------
	// Frequency Relay Interrupt enable
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);
//	------------------
//	Keyboard
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	if(ps2_device == NULL){
		printf("can't find PS/2 device\n");
	}

	alt_up_ps2_clear_fifo (ps2_device) ;
	alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);
	// register the PS/2 interrupt
	IOWR_8DIRECT(PS2_BASE,4,1);
}

void InitiateSharedResources(void) {
//	---------------
	// Initialise the Queues and their size
	Q_ReadFreqInfo = xQueueCreate( 100, sizeof(FreqInfo) );
	Q_KeyboardInput = xQueueCreate(100, sizeof(unsigned char));
	Q_VGAUpdateValues = xQueueCreate(100, sizeof(double));
	Q_VGAUpdateTime = xQueueCreate(100, sizeof(TickType_t));
//	--------------
	// Initialise Semaphore Mutexes
	load_state_sem = xSemaphoreCreateMutex();
	reading_state_sem = xSemaphoreCreateMutex();
}

void CreateTasks(void) {

	xTaskCreate( ReadFreq, "0", configMINIMAL_STACK_SIZE, NULL, ReadFreq_Task_P, &Timer_Reset );
	xTaskCreate( ManageLoads, "0", configMINIMAL_STACK_SIZE, NULL, ManageLoads_Task_P, &H_ManageLoads );
	xTaskCreate( UpdateLoads, "0", configMINIMAL_STACK_SIZE, NULL, UpdateLoads_Task_P, &Timer_Reset );
	xTaskCreate( UpdateThresholds, "0", configMINIMAL_STACK_SIZE, NULL, UpdateThresholds_P, &Timer_Reset );
	xTaskCreate( UpdateScreen, "0", configMINIMAL_STACK_SIZE, NULL, UpdateScreen_P, &Timer_Reset );

	ManageLoadsTimer = xTimerCreate("ManageLoadsTimer", 500, pdTRUE, NULL, vTimerCallback);

	if (xTimerStart(ManageLoadsTimer, 0) != pdPASS){
		printf("Cannot start timer");
	}
}




