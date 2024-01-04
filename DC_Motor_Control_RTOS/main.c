/*
 * DC_Motor_Control_RTOS.c
 *
 * Created: 04/01/2024 15:30:44
 * Author : Sergio Fink
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Analog pin connected to trimpot
#define TRIMPOT_PIN         PC0

//Speed pin control of motor speed (PWM)
#define MOTOR_SPEED_PIN     PB1


// Define a max ratio of trimpot (0 to 1023 for 10 bits ADC)
#define MAX_TRIMPOT_VALUE   1023

//Variable to store the motor desired speed
volatile uint8_t desiredSpeed = 0;


//Semaphore to guarantee safe access to desiredSpeed variable
SemaphoreHandle_t speedSemaphore;


//Function to configure the DC motor hardware
void setupMotor() {
		//Configure the pin motor as output
	DDRB |= (1 << MOTOR_SPEED_PIN);

	// Configure PWM
	TCCR1A |= (1 << WGM10) | (1 << WGM11) | (1 << COM1A1);
	TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler of 8
	OCR1A = 0; // Initialize with 0 speed
}

// Function to configure the ADC port to read the trimpot values
void setupADC() {
	// Configure the trimpot pin as input
	DDRC &= ~(1 << TRIMPOT_PIN);

	// Configure the ADC
	ADMUX |= (1 << REFS0); // reference voltage AVcc
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //Enable the ADC and configure the preescaler to 128.
}

// Function to read the trimpot value using the ADC
uint16_t readTrimpot() {
	// Select the trimpot channel and start the conversion
	ADMUX = (ADMUX & 0xF0) | (TRIMPOT_PIN & 0x0F);
	ADCSRA |= (1 << ADSC);

	// Wait for the conversion to complete
	while (ADCSRA & (1 << ADSC));

	// Return the converted value
	return ADC;
}

// Task that controls motor speed
void motorControlTask(void *pvParameters) {
	while (1) {
		// Read the trimpot value
		uint16_t trimpotValue = readTrimpot();

		// Map the trimpot value to the range 0 to 100
		desiredSpeed = (uint8_t)((trimpotValue * 100) / MAX_TRIMPOT_VALUE);

		// Update PWM output to control speed
		OCR1A = (desiredSpeed * 255) / 100;

		// Wait a short  time
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

int main(void) {
	// Initialize the semaphore
	speedSemaphore = xSemaphoreCreateMutex();

	// Configure the hardware
	setupMotor();
	setupADC();

	// Create motor control task
	xTaskCreate(motorControlTask, "MotorControl", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// Start the task system
	vTaskStartScheduler();

	return 0;
}