/*******************************************
 *  File: main.cpp
 *  Author: Ruslan Gindullin, W2HAT
 *  Date: 2025-02-25
 *  For the HamSCI 2025 Workshop
 *  See header file for details and
 *  user configuration.
*******************************************/

#include "main.h"

/*******************************************
 *      Function definitions
 ******************************************/
 
void arrayInit( uint32_t *freqArray, uint8_t *sequence, uint32_t base, uint32_t offset, uint32_t step, uint8_t len ) {
    for (int i = 0; i < len; i++) {
        freqArray[i] = base + offset + (sequence[i] * step);
    }
}   

void costasLoader(void *pvParameters) {

        // Power up the DDS
        dds.powerUp();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (int i = 0; i < COSTAS_SEQ_LEN; i++) {      // start loading the frequencies
            // Acquire the mutex before accessing the shared resource
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                // Set the frequency
                dds.setFrequency(freqArray[i]);
                // Release the mutex after accessing the shared resource
                xSemaphoreGive(xMutex);
            }
            // Wait for the next clock pulse
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        // Power down the DDS
        dds.setFrequency(0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        dds.powerDown();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Serial.println("Costas Task Deleted");
        costasActive = false;
        digitalWrite(COSTAS_TXRQ_PIN, LOW);
        digitalWrite(RF_TRIG_PIN, LOW);
        TaskHandle_t tempHandle = CostasTaskHandle;
        CostasTaskHandle = NULL;
        vTaskDelete(CostasTaskHandle);
}

void pskLoader(void *pvParameters) {
    uint8_t *msg = (uint8_t *)pvPortMalloc(msgString.length());
    msgString.getBytes(msg, msgString.length());
    // Power up the DDS
    dds.powerUp();
    // Wait for the first clock pulse
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Set the frequency
    dds.setFrequency(BASEBAND_FREQ);
    for(int i = 0; i < 8; i++) { // transmit a preamble
        // Wait for the next clock pulse
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    for(int i = 0; i < sizeof(BEACON_ID_MSG); i++) {
        uint8_t c = msg[i];
        for(int j = 0; j < 8; j++) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                // Set the phase
                dds.setPhase (15*( (c >> j) & 1) );
                // Release the mutex after accessing the shared resource
                xSemaphoreGive(xMutex);
            }
            // Wait for the next clock pulse
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }
    // load empty word
    dds.setFrequency(0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Power down the DDS
    dds.powerDown();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    digitalWrite(PSK_TXRQ_PIN, LOW);
    digitalWrite(RF_TRIG_PIN, LOW);
    Serial.println("PSK Task Deleted");
    if(msg != NULL)  // Free the memory allocated for the message
    vPortFree(msg);
    pskActive = false;
    TaskHandle_t tempHandle = PskTaskHandle;
    PskTaskHandle = NULL;
    vTaskDelete(PskTaskHandle);
}


void modeMaster(void *pvParameters) {
    Message msg;
    while (true) {
        if (xQueueReceive(xQueue, &msg, portMAX_DELAY) && !costasActive && !pskActive) {
            switch (msg.type) {
                case COSTAS_TRIGGER:
                    // Handle Costas trigger
                    costasActive = true;
                    pskActive = false;
                    digitalWrite(COSTAS_TXRQ_PIN, HIGH);
                    digitalWrite(RF_TRIG_PIN, HIGH);
                    if (xTaskCreate(
                        costasLoader,               // Function to implement the task
                        "CostasLoader",             // Name of the task
                        2048,                       // Stack size in words
                        NULL,                       // Task input parameter
                        3,                          // Priority of the task
                        &CostasTaskHandle           // Task handle
                    ) != pdPASS) {
                        Serial.println("Failed to create CostasLoader task");
                        Serial.print("Stack size watermark: ");
                        Serial.println(uxTaskGetStackHighWaterMark(NULL));
                        Serial.print("Free heap: ");
                        Serial.println(xPortGetFreeHeapSize());
                        vTaskDelete(CostasTaskHandle);
                        costasActive = false;
                        CostasTaskHandle = NULL;
                        vTaskDelay(xDelay*1000);
                    } else {
                        Serial.println("Costas Triggered");
                    }
                    break;
                case PSK_TRIGGER:
                    // Handle PSK trigger
                    costasActive = false;
                    pskActive = true;
                    digitalWrite(PSK_TXRQ_PIN, HIGH);
                    digitalWrite(RF_TRIG_PIN, HIGH);
                    if (xTaskCreate(
                        pskLoader,                  // Function to implement the task
                        "PskLoader",                // Name of the task
                        2048,                       // Stack size in words
                        NULL,                       // Task input parameter
                        3,                          // Priority of the task
                        &PskTaskHandle              // Task handle
                    ) != pdPASS) {
                        Serial.println("Failed to create PskLoader task");
                        Serial.print("Stack size watermark: ");
                        Serial.println(uxTaskGetStackHighWaterMark(NULL));
                        Serial.print("Free heap: ");
                        Serial.println(xPortGetFreeHeapSize());
                        vTaskDelete(PskTaskHandle);
                        PskTaskHandle = NULL;
                        pskActive = false;
                        vTaskDelay(xDelay*1000);
                    } else {
                        Serial.println("PSK Triggered");
                    }
                    break;
            }
        }
    }
}

/*******************************************
 *  Interrupt Service Routines
 *******************************************/

void IRAM_ATTR onCostasTrigger() {
    if(!costasActive){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        Message msg = { COSTAS_TRIGGER };
        xQueueSendFromISR(xQueue, &msg, &xHigherPriorityTaskWoken);
        #ifdef DEBUG
        Serial.println("Costas TriggeredISR");
        #endif
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

void IRAM_ATTR onPskTrigger() {
    if(!pskActive){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        Message msg = { PSK_TRIGGER };
        xQueueSendFromISR(xQueue, &msg, &xHigherPriorityTaskWoken);
        #ifdef DEBUG
        Serial.println("PSK TriggeredISR");
        #endif
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

void IRAM_ATTR onCostasClock() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (CostasTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(CostasTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

void IRAM_ATTR onPskClock() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (PskTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(PskTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

/*******************************************
 *  Setup and loop
 ******************************************/

// Setup function
void setup() {
    // Initialize the serial port for debugging
    Serial.begin(115200);
    #ifdef DEBUG
    Serial.println("Waiting for user input...");
    while (Serial.available() == 0) {
        delay(100); // Wait 100ms
    }
    Serial.read(); // Consume the user input
    Serial.println("User input received. Continuing setup.");
    #endif
    // Create the queue for messages
    xQueue = xQueueCreate(10, sizeof(Message));
    // Initialize the pins that are not handled by the library
    pinMode(COSTAS_TRIG_PIN, INPUT_PULLUP);
    pinMode(COSTAS_CLK_PIN, INPUT);
    pinMode(FQ_UD_PIN, INPUT_PULLUP);
    pinMode(PSK_CLK_PIN, INPUT);
    pinMode(PSK_TRIG_PIN, INPUT_PULLUP);
    pinMode(PSK_TXRQ_PIN, OUTPUT);
    pinMode(COSTAS_TXRQ_PIN, OUTPUT);
    pinMode(RF_TRIG_PIN, OUTPUT);
    #ifdef DEBUG
    Serial.println("Pins initialized");
    #endif
    // Attach the interrupts to the pins
    // The interrupts are triggered on the rising edge
    attachInterrupt(digitalPinToInterrupt(COSTAS_TRIG_PIN), onCostasTrigger, RISING);
    attachInterrupt(digitalPinToInterrupt(PSK_TRIG_PIN), onPskTrigger, RISING);
    attachInterrupt(digitalPinToInterrupt(COSTAS_CLK_PIN), onCostasClock, RISING);
    attachInterrupt(digitalPinToInterrupt(PSK_CLK_PIN), onPskClock, RISING);

    #ifdef DEBUG
    Serial.println("Interrupts attached");
    #endif
    // Initialize the DDS object
    dds.begin(); 
    // Power up the DDS
    dds.powerUp();
    #ifdef DEBUG
    Serial.println("DDS powered up");
    #endif
    // Initialize the frequency array
    arrayInit(freqArray, sequence, BASEBAND_FREQ, FREQ_OFFSET, FREQ_STEP, COSTAS_SEQ_LEN);
    // Create the mode manager task
    xTaskCreate(
        modeMaster,                 // Function to implement the task
        "ModeMaster",               // Name of the task
        2048,                       // Stack size in words
        NULL,                       // Task input parameter
        1,                          // Priority of the task
        &ModeManagerTaskHandle      // Task handle
    );
    #ifdef DEBUG
    Serial.println("Mode manager task created");
    #endif
    // Create the mutex for the AD9850 driver
    xMutex = xSemaphoreCreateMutex();
    // Check if the mutex was created successfully
    if (xMutex == NULL) {
        Serial.println("Failed to create mutex! Exiting...");
        ESP.restart();
    }
    #ifdef DEBUG
    Serial.println("Mutex created successfully");
    #endif
}
// Loop function
void loop() {
    // Do nothing
    // The tasks are running in the background
    vTaskDelay(xDelay); 
}