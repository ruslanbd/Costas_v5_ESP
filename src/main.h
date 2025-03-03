////////////////////////////////////////////////////////////////
//              W2HAT Costas Array Beacon Driver
////////////////////////////////////////////////////////////////
//  Board: Adafruit Metro ESP32-S3
//  Core: ESP32-S3
//  Clock: 160 MHz
//  
//  IDE: PlatformIO (Using the Espressif ESP32-S3 Box 
//  board definition, since PlatformIO is very adamant 
//  about flashing the micropython  interpreter 
//  with the native board definition.)
//  
//  Framework: Arduino
//  Firmware version: 5.0
////////////////////////////////////////////////////////////////
//                      Description:
////////////////////////////////////////////////////////////////
//  This is a DDS Driver for the W2HAT Costas Array beacon.
//
//  The AD9850 is loaded by an ESP32-S3 MCU, which is a 
//  helper core to the onboard GW2AR FPGA. It also provides
//  a serial interface to the GPSDO unit for timestamping (TODO!).
//  The timestamps are transmitted the next second after the
//  beacon is triggered. (TODO!) Timestamps are BPSK modulated.
//   
//  (Currently implemented a simple BPSK beacon ID once per 10
//  minutes, as per the FCC regulations.)
//
//  The DDS frequency output is triggered by an FPGA which is 
//  programmed to provide precise timing using an external
//  GPSDO.
//
//  This code uses a FreeRTOS task to load the DDS with an
//  array of frequencies. Or phases, depending on the mode.
//  When COSTAS_TRIG is asserted, the program enters the 
//  Costas mode, and the DDS is loaded with an array of
//  frequencies. The DDS is loaded with a new frequency
//  on every positive edge of the COSTAS_CLK pin. When
//  PSK_TRIG is asserted, the program enters the PSK mode, and
//  the DDS is loaded with a new phase on every positive
//  edge of the PSK_CLK pin. 
//  
//  The DDS is powered down when not in use, and the frequency
//  is set to zero. The DDS is powered up when the program
//  enters the Costas or PSK mode.
//
//  I slightly modified Rob Tilaart's AD985X library to
//  use a lower SPI speed, as the DDS is not very fast.
//  I couldn't get it to work with ESP32-S3's HW SPI, so
//  I used the library's SW SPI implementation.
//  
//  The Costas array and its parameters, as well as pin 
//  definitions and PSK message, are defined in the 
//  user configuration section.
////////////////////////////////////////////////////////////////
// Author: Ruslan Gindullin, W2HAT
// Date: 2025-02-25
////////////////////////////////////////////////////////////////
//              For the HamSCI 2025 Workshop
////////////////////////////////////////////////////////////////

#include "pins.h"
#include "AD985X_mod.h"
#include "Arduino.h"

/**************************************************************
   User configuration. TODO: move to an SD card and implement
                a configuration file reader.
**************************************************************/
/***************************
   Costas array parameters
***************************/

    // Length of the Costas array
    #define COSTAS_SEQ_LEN 7                          
    // Costas sequence
    #define COSTAS_SEQUENCE { 3, 1, 4, 0, 6, 5, 2 } 
    // Baseband frequency
    #define BASEBAND_FREQ 28260000ULL  // 28.25 MHz for compliance with the FCC part 97.203 on unattended beacon operation.
    // Frequency offset (frequency of 0 in the sequence)
    #define FREQ_OFFSET 1000ULL        // 1000 Hz offset (for good USB reception if tuned to 28.25 MHz)
    // Frequency step
    #define FREQ_STEP 100ULL           // 100 Hz step (sounds good haha)

/***************************
      Pin definitions
***************************/

    // Data pin for the DDS. Connect with the AD9850 PCB module's serial data pin (or D0 on the bare chip)
    #define DDS_DATA_PIN A2     

    // Clock pin for the DDS. Connect with the AD9850 module's clock pin
    #define DDS_CLK_PIN A4      

    // TX request pin for Costas mode. Enables FQ_UD clock from the FPGA. Connect with the corresponding pin on the FPGA.
    #define COSTAS_TXRQ_PIN D2   

    // TX request pin for PSK mode. Enables FQ_UD clock from the FPGA. Connect with the corresponding pin on the FPGA.
    #define PSK_TXRQ_PIN D3     

    // MCU clock pin for PSK mode. Triggers individual word loading. Connect with the corresponding pin on the FPGA.
    #define PSK_CLK_PIN D4        

    // MCU trigger pin for PSK mode. Triggers the whole message transmission. Connect with the corresponding pin on the FPGA.
    #define PSK_TRIG_PIN D5         

    // MCU trigger pin for Costas mode. Triggers the whole array transmission. Connect with the corresponding pin on the FPGA.
    #define COSTAS_TRIG_PIN D6      

    // MCU clock pin for Costas mode. Triggers individual word loading. Connect with the corresponding pin on the FPGA.
    #define COSTAS_CLK_PIN D7     

    // FQ_UD signal pin. Driven by the FPGA. Connect with both the AD9850 module and the FPGA
    #define FQ_UD_PIN A3        

    // Reset pin for the DDS. Connect with the AD9850 module's reset pin. Currently not used.
    #define DDS_RST A1             

/*************************** 
    Beacon ID message
***************************/

    // Message to be transmitted in PSK mode. Any size.
    #define BEACON_ID_MSG "W2HAT COSTAS ARRAY BEACON" 

/**************************************************************
                End of user configuration
**************************************************************/

// enum for mode of operation
enum Mode {
    COSTAS,
    PSK
}; 

// SPI object
//SPIClass *spi = new SPIClass(FSPI);               // SPI object

// DDS object
AD9850 dds(99, DDS_RST, 98, DDS_DATA_PIN, DDS_CLK_PIN); // DDS object: (ss, reset, FQ_UD, data, clock). 
// 99 and 98 are dummy pins since we only have one DDS and we use an fpga to update the frequency

   
/*************************** 
        Variables
***************************/ 
// Costas sequence array
uint8_t sequence [ COSTAS_SEQ_LEN ] = COSTAS_SEQUENCE; 
// Array with frequencies derived from the sequence and parameters
uint32_t freqArray [ COSTAS_SEQ_LEN ];                 
// Flag for sequence control
bool costasActive = false;                             
// Flag for message control
bool pskActive = false;    
// String that holds the message
String msgString = BEACON_ID_MSG; 

// Mutex for the AD9850 driver
// This is to prevent the task from being preempted while accessing the shared resource
SemaphoreHandle_t xMutex;

/*************************** 
        Task handles
***************************/ 
// Mode manager handle. This is the main task that controls the mode of operation
TaskHandle_t ModeManagerTaskHandle = NULL;
// Costas task handle. This is the task that loads the DDS with the Costas sequence
TaskHandle_t CostasTaskHandle = NULL;
// PSK task handle. This is the task that loads the DDS with the PSK message
TaskHandle_t PskTaskHandle = NULL;
// Queue for messages to the mode manager from the interrupts
QueueHandle_t xQueue;

// Tick for 1ms delay
const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

// enum type for messages to the mode manager
enum MessageType { COSTAS_TRIGGER, PSK_TRIGGER};
// Structure for messages to the mode manager
struct Message {
    MessageType type;
};

/*******************************************
 *  Function prototypes
 *******************************************/

 // Function to initialize the frequency array
// This is a simple function that takes the sequence, base frequency, offset, 
// step and length of the array and fills the array with the frequencies.
// The frequencies are calculated as base + offset + (MemberOfArray * step)

void arrayInit( uint32_t *freqArray, uint8_t *sequence, uint32_t base, uint32_t offset, uint32_t step, uint8_t len );

// Function to load the Costas sequence into the DDS
// This is a task that is created when the Costas trigger is received
// The task is deleted when the sequence is loaded
// The task is created with a higher priority than the mode manager
// The task is notified by an interrupt on the clock pin

void costasLoader(void *pvParameters);

// Function to load the PSK message into the DDS
// This is a task that is created when the PSK trigger is received
// The task is deleted when the message is loaded
// The task is created with a higher priority than the mode manager
// The task is notified by an interrupt on the clock pin
// The message is loaded bit by bit, with a preamble of 8 bits

void pskLoader(void *pvParameters);

// Function to manage the mode of operation
// This is the main task that controls the mode of operation
// The task is created with a lower priority than the Costas and PSK tasks
// It runs indefinitely.
// It waits for a message from the queue and then creates the corresponding task
// Messages are sent from both trigger interrupts.

void modeMaster(void *pvParameters);

/*******************************************
 *  Interrupt Service Routine prototypes
 *******************************************/

// Interrupt service routine for Costas trigger
// It handles the event when the FPGA triggers the Costas sequence
// It sends a message to the mode manager task

void IRAM_ATTR onCostasTrigger();

// Interrupt service routine for PSK trigger
// It handles the event when the FPGA triggers the PSK message
// It sends a message to the mode manager task

void IRAM_ATTR onPskTrigger();

 // Interrupt service routine for Costas clock
 // It handles the event when the FPGA triggers the Costas clock
 // It notifies the Costas task to load the next frequency

void IRAM_ATTR onCostasClock();

 // Interrupt service routine for PSK clock
 // It handles the event when the FPGA triggers the PSK clock
 // It notifies the PSK task to load the next phase


void IRAM_ATTR onPskClock();


/*******************************************
 *  Setup and loop prototypes
 ******************************************/

void setup();
void loop();
