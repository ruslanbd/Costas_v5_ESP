#include "Arduino.h"
#include "AD985X_mod.h"
#include <pins.h>
// Constants and globals
#define COSTAS_SEQ_LEN 7
#define AD9850_WORD_LEN 40
#define DDS_DATA_PIN D11
#define DDS_CLK_PIN D12
#define COSTAS_TXRQ_PIN A5
#define PSK_TXRQ_PIN A4
#define PSK_CLK_PIN D2
#define PSK_TRIG_PIN A2
#define BEACON_ID_MSG "W2HAT COSTAS ARRAY BEACON" // Beacon ID message
#define COSTAS_TRIG_PIN A3
#define COSTAS_CLK_PIN D4
#define FQ_UD_PIN D6
#define DDS_RST D7
#define COSTAS_SEQUENCE = { 3, 1, 4, 0, 6, 5, 2 }
// Constants for frequency calculation
#define BASEBAND_FREQ 28250000ULL  // 28.25 MHz for compliance with the FCC part 97.203 on unattended beacon transmissions.
#define FREQ_OFFSET 1000ULL       // 1000 Hz offset (for good USB reception)
#define FREQ_STEP 100ULL          // 100 Hz step

enum Mode {
    COSTAS,
    PSK
}; 

// SPI object
//SPIClass *spi = new SPIClass(FSPI);

// DDS object
AD9850 dds(99, DDS_RST, 98, DDS_DATA_PIN, DDS_CLK_PIN);

uint8_t sequence [ COSTAS_SEQ_LEN ] = { 3, 1, 4, 0, 6, 5, 2 };
uint32_t freqArray [ COSTAS_SEQ_LEN ];
bool costasActive = false;
bool pskActive = false;
String msgString = "W2HAT COSTAS ARRAY BEACON"; // Beacon ID message


SemaphoreHandle_t xMutex;

TaskHandle_t ModeManagerTaskHandle = NULL;
TaskHandle_t CostasTaskHandle = NULL;
TaskHandle_t PskTaskHandle = NULL;
QueueHandle_t xQueue;

const TickType_t xDelay = 1 / portTICK_PERIOD_MS;


enum MessageType { COSTAS_TRIGGER, PSK_TRIGGER};
struct Message {
    MessageType type;
};

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
                Serial.println(dds.getFactor());
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
                        Serial.print("Stack size watermark: ");
                        Serial.println(uxTaskGetStackHighWaterMark(NULL));
                        Serial.print("Free heap: ");
                        Serial.println(xPortGetFreeHeapSize());
                        /*
                        Serial.print("SPI MOSI: ");
                        Serial.println(MOSI);
                        Serial.print("SPI MISO: ");
                        Serial.println(MISO);
                        Serial.print("SPI SCK: ");
                        Serial.println(SCK);
                        Serial.print("SPI SS: ");
                        Serial.println(SS);*/
                    }
                    break;
                case PSK_TRIGGER:
                    // Handle PSK trigger
                    costasActive = false;
                    pskActive = true;
                    digitalWrite(PSK_TXRQ_PIN, HIGH);
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
                        Serial.print("Stack size watermark: ");
                        Serial.println(uxTaskGetStackHighWaterMark(NULL));
                        Serial.print("Free heap: ");
                        Serial.println(xPortGetFreeHeapSize());
                    }
                    break;
            }
        }
    }
}



void IRAM_ATTR onCostasTrigger() {
    if(!costasActive){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        Message msg = { COSTAS_TRIGGER };
        xQueueSendFromISR(xQueue, &msg, &xHigherPriorityTaskWoken);
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

void setup() {
    Serial.begin(115200);
    xQueue = xQueueCreate(10, sizeof(Message));
    pinMode(COSTAS_TRIG_PIN, INPUT);
    pinMode(COSTAS_CLK_PIN, INPUT);
    pinMode(FQ_UD_PIN, INPUT);
    pinMode(PSK_CLK_PIN, INPUT);
    pinMode(PSK_TRIG_PIN, INPUT);
    pinMode(PSK_TXRQ_PIN, OUTPUT);
    pinMode(COSTAS_TXRQ_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(COSTAS_TRIG_PIN), onCostasTrigger, RISING);
    attachInterrupt(digitalPinToInterrupt(PSK_TRIG_PIN), onPskTrigger, RISING);
    attachInterrupt(digitalPinToInterrupt(COSTAS_CLK_PIN), onCostasClock, RISING);
    attachInterrupt(digitalPinToInterrupt(PSK_CLK_PIN), onPskClock, RISING);
    //SPI.begin();
    dds.begin();
    //dds.setSPIspeed(40000); 
    dds.powerUp();
    arrayInit(freqArray, sequence, BASEBAND_FREQ, FREQ_OFFSET, FREQ_STEP, COSTAS_SEQ_LEN);
    xTaskCreate(
        modeMaster,                 // Function to implement the task
        "ModeMaster",               // Name of the task
        2048,                       // Stack size in words
        NULL,                       // Task input parameter
        1,                          // Priority of the task
        &ModeManagerTaskHandle      // Task handle
    );
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        Serial.println("Failed to create mutex");
    }
}
void loop() {
    vTaskDelay(xDelay);
}