/******************************************************************************/
/* All needed inclusions. *****************************************************/
/******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "TypeDef.h"
#include "semphr.h"
#include <stdbool.h>
#include "uart0.h"
#include "GPTM.h"
#include "HAL/LM35/lm35.h"
#include "driverlib/adc.h"
#include "MCAL/GPIO/gpio.h"
#include "MCAL/tm4c123gh6pm_registers.h"
#include "eeprom.h"


/******************************************************************************/
/* Definitions. ***************************************************************/
/******************************************************************************/
#define xGetTempMaxDelay pdMS_TO_TICKS(100); /* Get Temperature Tasks Timeout */
#define xDriverInfoMaxDelay pdMS_TO_TICKS(50); /* Set Driver Seat Heater Task Timeout */
#define xPassengerInfoMaxDelay xDriverInfoMaxDelay /* Set Passenger Seat Heater Task Timeout */
#define xGetDriverInputMaxDelay pdMS_TO_TICKS(50); /* Driver Selection Input Tasks Timeout */
#define xGetPassengerInputMaxDelay xGetDriverInputMaxDelay /* Passenger Selection Input Task Timeout */
#define DRIVER_SENSOR_ERROR_ADDRESS 0x1111 /* Location in eeprom where the driver sensor error will be reported */
#define PASSENGER_SENSOR_ERROR_ADDRESS 0x2222 /* Location in eeprom where the passenger sensor error will be reported */
#define DRIVER_INPUT_ADDRESS 0x3333 /* Location in eeprom where the driver selection will be reported */
#define PASSENGER_INPUT_ADDRESS 0x4444 /* Location in eeprom where the passenger selection will be reported */
#define DRIVER_SENSOR_ERROR_CODE 0x1234 /* The code that will be reported in case of driver sensor failure */
#define PASSENGER_SENSOR_ERROR_CODE 0x4321 /* The code that will be reported in case of passenger sensor failure */
#define RUNTIME_MEASUREMENTS_TASK_PERIODICITY pdMS_TO_TICKS(2200) /* Period of Run-Time Measurement Task */
#define GET_TEMP_TASK_PERIODICITY pdMS_TO_TICKS(500) /* Period for both Get Temp tasks */
#define SET_TEMP_TASK_PERIODICITY pdMS_TO_TICKS(150) /* Period for both Set Temp tasks */
#define DISPLAY_SYSTEM_STATE_PERIOD pdMS_TO_TICKS(1000) /* Period for Display System State Task */
/******************************************************************************/
/* Global Variables. **********************************************************/
/******************************************************************************/
bool DRIVER_SENSOR_ERROR = pdFALSE;
bool PASSENGER_SENSOR_ERROR = pdFALSE;
uint32_t ERROR_CODE;
TickType_t xLastWakeTime;
uint32_t DriverSelection;
uint32_t PassengerSelection;
uint32 ullTasksOutTime[NUMBER_OF_TASKS + 1];
uint32 ullTasksInTime[NUMBER_OF_TASKS + 1];
uint32 ullTasksExecutionTime[NUMBER_OF_TASKS + 1];
uint32 ullTasksTotalTime[NUMBER_OF_TASKS + 1];
SeatInfoType DriverSeatInfo = { 10, Heater_OFF, Desired_HIGH };
SeatInfoType PassengerSeatInfo = { 10, Heater_OFF, Desired_HIGH };

/******************************************************************************/
/* Function Declarations. *****************************************************/
/******************************************************************************/
/* The HW setup function */
static void prvSetupHardware(void);

/* FreeRTOS tasks */
void vGetDriverSeatTemp(void *pvParameters);
void vGetPassengerSeatTemp(void *pvParameters);
void vSetDriverSeatHeaterState(void *pvParameters);
void vSetPassengerSeatHeaterState(void *pvParameters);
void vDisplaySystemState(void *pvParameters);
void vGetDriverInput(void *pvParameters);
void vGetPassengerInput(void *pvParameters);
void vGPIOPortF_Handler(void);
void vSensorErrorHook(void *pvParameters);
void vRunTimeMeasurementsTask(void *pvParameters);


/******************************************************************************/
/* Semaphores and Task Handles. ***********************************************/
/******************************************************************************/
/* FreeRTOS Mutexes */
xSemaphoreHandle xLM35GetTempMutex;
xSemaphoreHandle xDriverInfoMutex;
xSemaphoreHandle xPassengerInfoMutex;
xSemaphoreHandle xErrorReportingMutex;

/* FreeRTOS Semaphores */
xSemaphoreHandle xDriverInputSemaphore;
xSemaphoreHandle xDriverInputSemaphoreFromSteering;
xSemaphoreHandle xPassengerInputSemaphore;
xSemaphoreHandle xSensorErrorHookSemaphore;

/* Used to hold the handle of tasks */
TaskHandle_t xGetDriverSeatTempHandle;
TaskHandle_t xGetPassengerSeatTempHandle;
TaskHandle_t xSetDriverSeatHeaterStateHandle;
TaskHandle_t xSetPassengerSeatHeaterStateHandle;
TaskHandle_t xDisplaySystemStateHandle;
TaskHandle_t xGetDriverInputHandle;
TaskHandle_t xGetDriverInputFromSteeringHandle;
TaskHandle_t xGetPassengerInputHandle;
TaskHandle_t xSensorErrorHookHandle;
TaskHandle_t xRunTimeMeasurementsHandle;


/******************************************************************************/
/* Function Definitions. ******************************************************/
/******************************************************************************/
int main()
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /* Create Tasks here and assign tags */
    xTaskCreate(vGetDriverSeatTemp, "Get Driver Seat Temperature Task", 128, NULL, 3, &xGetDriverSeatTempHandle);
    vTaskSetApplicationTaskTag( xGetDriverSeatTempHandle, ( TaskHookFunction_t ) 1 );

    xTaskCreate(vGetPassengerSeatTemp, "Get Passenger Seat Temperature Task", 128, NULL, 3, &xGetPassengerSeatTempHandle);
    vTaskSetApplicationTaskTag( xGetPassengerSeatTempHandle, ( TaskHookFunction_t ) 2 );

    xTaskCreate(vSetDriverSeatHeaterState, "Set Driver Seat Heater State Task", 128, NULL, 2, &xSetDriverSeatHeaterStateHandle);
    vTaskSetApplicationTaskTag( xSetDriverSeatHeaterStateHandle, ( TaskHookFunction_t ) 3 );

    xTaskCreate(vSetPassengerSeatHeaterState, "Set Passenger Seat Heater State Task", 128, NULL, 2, &xSetPassengerSeatHeaterStateHandle);
    vTaskSetApplicationTaskTag( xSetPassengerSeatHeaterStateHandle, ( TaskHookFunction_t ) 4 );

    xTaskCreate(vDisplaySystemState, "Display System Information Task", 128, NULL, 3, &xDisplaySystemStateHandle);
    vTaskSetApplicationTaskTag( xDisplaySystemStateHandle, ( TaskHookFunction_t ) 5 );

    xTaskCreate(vGetDriverInput, "Get Driver Selection", 128, NULL, 4, &xGetDriverInputHandle);
    vTaskSetApplicationTaskTag( xGetDriverInputHandle, ( TaskHookFunction_t ) 6 );

    xTaskCreate(vGetDriverInput, "Get Driver Selection From Steering Wheel", 128, NULL, 4, &xGetDriverInputFromSteeringHandle);
    vTaskSetApplicationTaskTag( xGetDriverInputFromSteeringHandle, ( TaskHookFunction_t ) 7 );

    xTaskCreate(vGetPassengerInput, "Get Passenger Selection", 128, NULL, 4, &xGetPassengerInputHandle);
    vTaskSetApplicationTaskTag( xGetPassengerInputHandle, ( TaskHookFunction_t ) 8 );

    xTaskCreate(vSensorErrorHook, "Sensor Error Hook", 128, NULL, 5, &xSensorErrorHookHandle);
    vTaskSetApplicationTaskTag( xSensorErrorHookHandle, ( TaskHookFunction_t ) 9 );

    xTaskCreate(vRunTimeMeasurementsTask, "Run time", 128, NULL, 2, &xRunTimeMeasurementsHandle);
    vTaskSetApplicationTaskTag( xRunTimeMeasurementsHandle, ( TaskHookFunction_t ) 10 );


    /* Create a Mutexes and semaphores */
    xLM35GetTempMutex = xSemaphoreCreateMutex();
    xDriverInfoMutex = xSemaphoreCreateMutex();
    xPassengerInfoMutex = xSemaphoreCreateMutex();
    xErrorReportingMutex = xSemaphoreCreateMutex();

    xDriverInputSemaphore = xSemaphoreCreateBinary();
    xDriverInputSemaphoreFromSteering = xSemaphoreCreateBinary();
    xPassengerInputSemaphore = xSemaphoreCreateBinary();
    xSensorErrorHookSemaphore = xSemaphoreCreateBinary();

    /* Now all the tasks have been started - start the scheduler.

     NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
     The processor MUST be in supervisor mode when vTaskStartScheduler is
     called.  The demo applications included in the FreeRTOS.org download switch
     to supervisor mode prior to main being called.  If you are not using one of
     these demo application projects then ensure Supervisor mode is used here. */
    vTaskStartScheduler();

    /* Should never reach here!  If you do then there was not enough heap
     available for the idle task to be created. */
    for (;;)
        ;

}

static void prvSetupHardware(void)
{
    /* Place here any needed HW initialization such as GPIO, UART, etc.  */
    UART0_Init();
    DriverSensor_Init();
    PassengerSensor_Init();
    GPIO_BuiltinButtonsLedsInit();
    GPIO_SW1EdgeTriggeredInterruptInit();
    GPIO_SW2EdgeTriggeredInterruptInit();
    GPTM_WTimer0Init();
    EEPROMInit();
}

void vRunTimeMeasurementsTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        uint8 ucCounter, ucCPU_Load;
        uint32 ullTotalTasksTime = 0;
        vTaskDelayUntil(&xLastWakeTime, RUNTIME_MEASUREMENTS_TASK_PERIODICITY);
        for(ucCounter = 1; ucCounter < NUMBER_OF_TASKS + 1; ucCounter++)
        {
            ullTotalTasksTime += ullTasksTotalTime[ucCounter];
            taskENTER_CRITICAL();
            UART0_SendString("Task of tag ");
            UART0_SendInteger(ucCounter);
            UART0_SendString(" execution time is: ");
            UART0_SendInteger(ullTasksExecutionTime[ucCounter] / 10);
            UART0_SendString(" msec \r\n");
            taskEXIT_CRITICAL();
        }
        ucCPU_Load = (ullTotalTasksTime * 100) /  GPTM_WTimer0Read();

        taskENTER_CRITICAL();
        UART0_SendString("CPU Load is ");
        UART0_SendInteger(ucCPU_Load);
        UART0_SendString("% \r\n");
        taskEXIT_CRITICAL();
    }
}

void vGetDriverSeatTemp(void *pvParameters)
{
    /* Tries to acquire the semaphore. */
    TickType_t Timeout = xGetTempMaxDelay;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, GET_TEMP_TASK_PERIODICITY);
        if (xSemaphoreTake(xLM35GetTempMutex, Timeout) == pdTRUE)
        {

            DriverSeatInfo.SeatTemperature = GetDriverSensorReading();
            if (DriverSeatInfo.SeatTemperature < 5
                    || DriverSeatInfo.SeatTemperature > 40)
            {
                DRIVER_SENSOR_ERROR = pdTRUE;
                xSemaphoreGive(xSensorErrorHookSemaphore);
            }
            else
            {
                GPIO_RedLedOff();
                vTaskResume(xSetDriverSeatHeaterStateHandle);
                vTaskResume(xGetDriverInputHandle);
                vTaskResume(xGetDriverInputFromSteeringHandle);
                /* Release the peripheral */
                xSemaphoreGive(xLM35GetTempMutex);
            }
        }
    }
}
void vGetPassengerSeatTemp(void *pvParameters)
{
    /* Tries to acquire the semaphore. */
    uint32 Timeout = xGetTempMaxDelay;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (xSemaphoreTake(xLM35GetTempMutex, Timeout) == pdTRUE)
        {

            PassengerSeatInfo.SeatTemperature = GetPassengerSensorReading();

            if (PassengerSeatInfo.SeatTemperature < 5
                    || PassengerSeatInfo.SeatTemperature > 40)
            {

                PASSENGER_SENSOR_ERROR = pdTRUE;
                xSemaphoreGive(xSensorErrorHookSemaphore);

            }
            else
            {
                GPIO_RedLedOff();
                vTaskResume(xSetPassengerSeatHeaterStateHandle);
                vTaskResume(xGetPassengerInputHandle);
                /* Release the peripheral */
                xSemaphoreGive(xLM35GetTempMutex);
            }
            vTaskDelayUntil(&xLastWakeTime, GET_TEMP_TASK_PERIODICITY);
        }
    }
}

void vSetDriverSeatHeaterState(void *pvParameters)
{

    TickType_t Timeout = xDriverInfoMaxDelay;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, SET_TEMP_TASK_PERIODICITY);
        if (xSemaphoreTake(xDriverInfoMutex, Timeout) == pdTRUE)
        {
            sint8 xTemperatureDifference = (DriverSeatInfo.SeatTemperature
                    - DriverSeatInfo.DesiredTemperature);
            if (xTemperatureDifference >= 10)
            {
                DriverSeatInfo.HeaterState = Heater_HIGH;
                GPIO_BlueLedOn();
                GPIO_GreenLedOn();
            }
            else if (xTemperatureDifference < 10 && xTemperatureDifference >= 5)
            {
                DriverSeatInfo.HeaterState = Heater_MED;
                GPIO_BlueLedOn();
                GPIO_GreenLedOff();
            }
            else if (xTemperatureDifference < 5 && xTemperatureDifference >= 2)
            {
                DriverSeatInfo.HeaterState = Heater_LOW;
                GPIO_GreenLedOn();
                GPIO_BlueLedOff();
            }
            else
            {
                DriverSeatInfo.HeaterState = Heater_OFF;
            }
            xSemaphoreGive(xDriverInfoMutex);
        }
    }
}
void vSetPassengerSeatHeaterState(void *pvParameters)
{
    TickType_t Timeout = xDriverInfoMaxDelay;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, SET_TEMP_TASK_PERIODICITY);
        if (xSemaphoreTake(xPassengerInfoMutex, Timeout) == pdTRUE)
        {

            sint8 xTemperatureDifference = (PassengerSeatInfo.SeatTemperature)
                    - (PassengerSeatInfo.DesiredTemperature);

            if (xTemperatureDifference >= 10)
            {
                PassengerSeatInfo.HeaterState = Heater_HIGH;
                GPIO_BlueLedOn();
                GPIO_GreenLedOn();
            }
            else if (xTemperatureDifference < 10 && xTemperatureDifference >= 5)
            {
                PassengerSeatInfo.HeaterState = Heater_MED;
                GPIO_BlueLedOn();
                GPIO_GreenLedOff();
            }
            else if (xTemperatureDifference < 5 && xTemperatureDifference >= 2)
            {
                PassengerSeatInfo.HeaterState = Heater_LOW;
                GPIO_GreenLedOn();
                GPIO_BlueLedOff();
            }
            else
            {
                PassengerSeatInfo.HeaterState = Heater_OFF;
            }
            xSemaphoreGive(xPassengerInfoMutex);
        }
    }
}

void vDisplaySystemState(void *pvParameters)
{

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime ,DISPLAY_SYSTEM_STATE_PERIOD);
        if (xSemaphoreTake(xDriverInfoMutex, portMAX_DELAY) == pdTRUE)
        {
            UART0_SendString("*****Driver Seat*****\r\n");
            UART0_SendString("Current Temperature: ");
            UART0_SendInteger(DriverSeatInfo.SeatTemperature);

            UART0_SendString("\r\nDesired Heating Level: ");
            if (DriverSeatInfo.DesiredTemperature == Desired_OFF)
            {
                UART0_SendString("OFF\r\n");
            }
            else if (DriverSeatInfo.DesiredTemperature == Desired_LOW)
            {
                UART0_SendString("LOW\r\n");
            }
            else if (DriverSeatInfo.DesiredTemperature == Desired_MED)
            {
                UART0_SendString("MED\r\n");
            }
            else
            {
                UART0_SendString("HIGH\r\n");
            }

            UART0_SendString("Heater Intensity: ");
            if (DriverSeatInfo.HeaterState == Heater_OFF)
            {
                UART0_SendString("OFF\r\n");
            }
            else if (DriverSeatInfo.HeaterState == Heater_LOW)
            {
                UART0_SendString("LOW\r\n");
            }
            else if (DriverSeatInfo.HeaterState == Heater_MED)
            {
                UART0_SendString("MED\r\n");
            }
            else
            {
                UART0_SendString("HIGH\r\n");
            }
            xSemaphoreGive(xDriverInfoMutex);
        }

        if (xSemaphoreTake(xDriverInfoMutex, portMAX_DELAY) == pdTRUE)
        {
            UART0_SendString("*****Passenger Seat*****\r\n");
            UART0_SendString("Current Temperature: ");
            UART0_SendInteger(PassengerSeatInfo.SeatTemperature);

            UART0_SendString("\r\nDesired Heating Level: ");
            if (PassengerSeatInfo.DesiredTemperature == Desired_OFF)
            {
                UART0_SendString("OFF\r\n");
            }
            else if (PassengerSeatInfo.DesiredTemperature == Desired_LOW)
            {
                UART0_SendString("LOW\r\n");
            }
            else if (PassengerSeatInfo.DesiredTemperature == Desired_MED)
            {
                UART0_SendString("MED\r\n");
            }
            else
            {
                UART0_SendString("HIGH\r\n");
            }

            UART0_SendString("Heater Intensity: ");
            if (PassengerSeatInfo.HeaterState == Heater_OFF)
            {
                UART0_SendString("OFF\r\n");
            }
            else if (PassengerSeatInfo.HeaterState == Heater_LOW)
            {
                UART0_SendString("LOW\r\n");
            }
            else if (PassengerSeatInfo.HeaterState == Heater_MED)
            {
                UART0_SendString("MED\r\n");
            }
            else
            {
                UART0_SendString("HIGH\r\n");
            }
            xSemaphoreGive(xPassengerInfoMutex);
        }
    }
}

void vGPIOPortF_Handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (GPIO_PORTF_RIS_REG & (1 << 4))
    {
        /* Synchronize with the red led task to toggle it each edge triggered interrupt */
        xSemaphoreGiveFromISR(xDriverInputSemaphore, &xHigherPriorityTaskWoken);
        /* Clear Trigger flag for PF4 (Interrupt Flag) */
        GPIO_PORTF_ICR_REG |= (1 << 4);
    }
    else if (GPIO_PORTF_RIS_REG & (1 << 0))
    {
        /* Synchronize with the red led task to toggle it each edge triggered interrupt */
        xSemaphoreGiveFromISR(xPassengerInputSemaphore,
                              &xHigherPriorityTaskWoken);
        /* Clear Trigger flag for PF0 (Interrupt Flag) */
        GPIO_PORTF_ICR_REG |= (1 << 0);
    }
    else
    {
        /* Synchronize with the red led task to toggle it each edge triggered interrupt */
        xSemaphoreGiveFromISR(xDriverInputSemaphoreFromSteering,
                              &xHigherPriorityTaskWoken);
        /* Clear Trigger flag for PF4 (Interrupt Flag) */
        GPIO_PORTF_ICR_REG |= (1 << 4);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vGetDriverInput(void *pvParameters)
{

    uint32 Timeout = xGetDriverInputMaxDelay
    ;
    static uint8 clicks = 0;
    static uint8 i = 0;
    for (;;)
    {
        /* Waits for synchronization with PF4 & PF0 edge triggered interrupt using the binary semaphore. */
        /* This task will be blocked until both PF0 & PF4 edge triggered interrupt happened. */
        /* This task does not need a delay call since it is waiting for edge triggered interrupt */
        if ((xSemaphoreTake(xDriverInputSemaphore, Timeout) == pdTRUE)
                || (xSemaphoreTake(xDriverInputSemaphoreFromSteering, Timeout)
                        == pdTRUE))
        {
            if (xSemaphoreTake(xDriverInfoMutex, portMAX_DELAY) == pdTRUE)
            {
                clicks = clicks % 4;
                switch (clicks)
                {
                case 0:
                    DriverSeatInfo.DesiredTemperature = Desired_OFF;
                    break;
                case 1:
                    DriverSeatInfo.DesiredTemperature = Desired_LOW;
                    break;
                case 2:
                    DriverSeatInfo.DesiredTemperature = Desired_MED;
                    break;
                case 3:
                    DriverSeatInfo.DesiredTemperature = Desired_HIGH;
                    break;
                }
            }
        }
        clicks++;
        DriverSelection = DriverSeatInfo.DesiredTemperature;
        EEPROMProgram(&DriverSelection, DRIVER_INPUT_ADDRESS + i, 1);
        i += 4;
    }
}

void vGetPassengerInput(void *pvParameters)
{

    uint32 Timeout = xGetPassengerInputMaxDelay
    ;
    uint8 i = 0;
    static uint8 clicks = 0;
    for (;;)
    {
        /* Waits for synchronization with PF4 & PF0 edge triggered interrupt using the binary semaphore. */
        /* This task will be blocked until both PF0 & PF4 edge triggered interrupt happened. */
        /* This task does not need a delay call since it is waiting for edge triggered interrupt */
        if ((xSemaphoreTake(xPassengerInputSemaphore, Timeout) == pdTRUE))
        {
            if (xSemaphoreTake(xPassengerInfoMutex, portMAX_DELAY) == pdTRUE)
            {
                clicks = clicks % 4;
                switch (clicks)
                {
                case 0:
                    PassengerSeatInfo.DesiredTemperature = Desired_OFF;
                    break;
                case 1:
                    PassengerSeatInfo.DesiredTemperature = Desired_LOW;
                    break;
                case 2:
                    PassengerSeatInfo.DesiredTemperature = Desired_MED;
                    break;
                case 3:
                    PassengerSeatInfo.DesiredTemperature = Desired_HIGH;
                    break;
                }
            }
        }
        clicks++;
        PassengerSelection = PassengerSeatInfo.DesiredTemperature;
        EEPROMProgram(&PassengerSelection, PASSENGER_INPUT_ADDRESS + i, 1);
        i += 4;
    }
}

void vSensorErrorHook(void *pvParameters)
{
    static uint8 i = 0;
    static uint8 j = 0;

    while ((xSemaphoreTake(xSensorErrorHookSemaphore, portMAX_DELAY) == pdTRUE))
    {
        xLastWakeTime = (TickType_t) (xTaskGetTickCount() * (TickType_t) 1000
                / (TickType_t) configTICK_RATE_HZ);

        GPIO_RedLedOn();
        GPIO_GreenLedOff();
        GPIO_BlueLedOff();

        if (DRIVER_SENSOR_ERROR == pdTRUE)
        {
            vTaskSuspend(xSetDriverSeatHeaterStateHandle);
            vTaskSuspend(xGetDriverInputHandle);
            vTaskSuspend(xGetDriverInputFromSteeringHandle);
            DriverSeatInfo.DesiredTemperature = Desired_OFF;
            DriverSeatInfo.HeaterState = Heater_OFF;
            DRIVER_SENSOR_ERROR = pdFALSE;
            if ((xSemaphoreTake(xErrorReportingMutex, portMAX_DELAY) == pdTRUE))
            {
                ERROR_CODE = DRIVER_SENSOR_ERROR_CODE;
                EEPROMProgram(&ERROR_CODE, DRIVER_SENSOR_ERROR_ADDRESS + i, 1);
                i += 4;
                EEPROMProgram(&xLastWakeTime, DRIVER_SENSOR_ERROR_ADDRESS + i,
                              1);
                i += 4;
            }

        }

        if (PASSENGER_SENSOR_ERROR == pdTRUE)
        {
            vTaskSuspend(xSetPassengerSeatHeaterStateHandle);
            vTaskSuspend(xGetPassengerInputHandle);
            PassengerSeatInfo.DesiredTemperature = Desired_OFF;
            PassengerSeatInfo.HeaterState = Heater_OFF;
            PASSENGER_SENSOR_ERROR = pdFALSE;
            if ((xSemaphoreTake(xErrorReportingMutex, portMAX_DELAY) == pdTRUE))
            {
                ERROR_CODE = PASSENGER_SENSOR_ERROR_CODE;
                EEPROMProgram(&ERROR_CODE, PASSENGER_SENSOR_ERROR_ADDRESS + j,
                              1);
                j += 4;
                EEPROMProgram(&xLastWakeTime,
                PASSENGER_SENSOR_ERROR_ADDRESS + j,
                              1);
                j += 4;
            }

        }
    }
}

/*-----------------------------------------------------------*/
