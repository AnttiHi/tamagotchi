/* C Standard library */
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
 
/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
 
/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/UART.h>
 
/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"
 
// RTOS:n globaalit muuttujat pinnien käyttöön
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;
 
// Pinnien alustukset, molemmille pinneille oma konfiguraatio
// Vakio BOARD_BUTTON_0 vastaa toista painonappia
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};
 
// Vakio Board_LED0 vastaa toista lediä
PIN_Config ledConfig[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};
 
// Alusta MPU:n virtapinni
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
 
// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};
 
/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
 
//Syötteen tunnistuksen hienosäätö
#define COMMAND_DELAY 1000000
#define MOTION_THRESHOLD 180
 
// Tilakoneen esittely
enum state { WAITING=1, COMMAND_DETECTED };
enum state programState = WAITING;
 
// Valoisuuden globaali muuttuja
//double ambientLight = -1000.0;
 
// Liikekomennon globaali muuttuja
int commandCode = 0;
 
void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
 
    // Vaihdetaan led-pinnin tilaa negaatiolla
    uint_t pinValue = PIN_getOutputValue( Board_LED1 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED1, pinValue );
}
char uartBuffer[80];
 
// Käsittelijäfunktio
static void uartFxn(UART_Handle handle, void *rxBuf, size_t len) {
   char testijono[80];
   // Nyt meillä on siis haluttu määrä merkkejä käytettävissä
   // rxBuf-taulukossa, pituus len, jota voimme käsitellä halutusti
   // Tässä ne annetaan argumentiksi toiselle funktiolle (esimerkin vuoksi)
   sprintf(testijono, rxBuf);
   System_printf("%s\n", testijono);
   System_flush();
 
   // Käsittelijän viimeisenä asiana siirrytään odottamaan uutta keskeytystä..
   UART_read(handle, rxBuf, 1);
}
 
/* Task Functions */
Void uartTask(UArg arg0, UArg arg1) {
 
    // UART-kirjaston asetukset
    UART_Handle uart;
    UART_Params uartParams;
 
    // Alustetaan sarjaliikenne
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_ON;
    uartParams.readMode= UART_MODE_CALLBACK;
    uartParams.readCallback = &uartFxn;
    uartParams.baudRate = 9600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1
 
    // Avataan yhteys laitteen sarjaporttiin vakiossa Board_UART0
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        System_abort("Error opening the UART");
    }
 
    UART_read(uart, uartBuffer, 1);
 
    while (1) {
 
        if(programState == COMMAND_DETECTED){
            char merkkijono[64];
            int index = 0;
            index += sprintf(merkkijono + index, "id:59,");
 
            if (commandCode == 0) {
                index += sprintf(merkkijono + index, "EAT:");
            }
            else if (commandCode == 1) {
                index += sprintf(merkkijono + index, "EXERCISE:");
            }
            else if (commandCode == 2) {
                index += sprintf(merkkijono + index, "PET:");
            }
            index += sprintf(merkkijono + index,"%d\n\r\0",1);
            System_printf(merkkijono);
            System_flush();
 
            programState = WAITING;
 
            // JTKJ: Teht�v� 4. L�het� sama merkkijono UARTilla
            // JTKJ: Exercise 4. Send the same sensor data string with UART
 
            UART_write(uart, merkkijono, strlen(merkkijono));
 
            // Just for sanity check for exercise, you can comment this out
            //System_printf("uartTask\n");
            //System_flush();
        }
        // Once per second, you can modify this
        Task_sleep(100000 / Clock_tickPeriod);
    }
}
 
Void sensorTaskFxn(UArg arg0, UArg arg1) {
 
    //double lightValue;
    float ax, ay, az, gx, gy, gz;
 
    // RTOS:n i2c-muuttujat ja alustus
    //I2C_Handle      i2c;
    //I2C_Params      i2cParams;
    I2C_Handle      i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params      i2cMPUParams;
 
    // Alustetaan i2c-väylät
    //I2C_Params_init(&i2cParams);
    //i2cParams.bitRate = I2C_400kHz;
    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;
 
    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
 
    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();
 
 
    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
        System_flush();
    }
 
    mpu9250_setup(&i2cMPU);
 
    /*
    // Avataan yhteys
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        System_abort("Error Initializing I2C\n");
    }
 
    opt3001_setup(&i2c);
*/
 
    Task_sleep(100000 / Clock_tickPeriod);
 
    while (1) {
        if(programState == WAITING){
            /*
            // Muunnetaan 2-tavuinen data rxBuffer:ssa
            lightValue = opt3001_get_data(&i2c);
 
            //if sensor data not ready, restart the loop
            if(lightValue < 0){
                Task_sleep(1000000 / Clock_tickPeriod);
                continue;
            }
 
            // Lämpötila-arvo tiedoksi konsoli-ikkunaan
            char merkkijono[64];
            sprintf(merkkijono,"\nValue: %f",lightValue);
            System_printf(merkkijono);
            System_flush();
 
            ambientLight = lightValue;
            programState = DATA_READY;
            */
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
 
            //test for motion commands
            if(gx > MOTION_THRESHOLD){
                System_printf("X-command detected\n");
                System_flush();
                commandCode = 0;
                programState = COMMAND_DETECTED;
                Task_sleep(COMMAND_DELAY / Clock_tickPeriod);
                continue;
            }
            if(gy > MOTION_THRESHOLD){
                System_printf("Y-command detected\n");
                System_flush();
                commandCode = 1;
                programState = COMMAND_DETECTED;
                Task_sleep(COMMAND_DELAY / Clock_tickPeriod);
                continue;
            }
            if(gz > MOTION_THRESHOLD){
                System_printf("Z-command detected\n");
                System_flush();
                commandCode = 2;
                programState = COMMAND_DETECTED;
                Task_sleep(COMMAND_DELAY / Clock_tickPeriod);
                continue;
            }
 
 
            //char merkkijono[64];
            //sprintf(merkkijono,"\nValue: %02.4f, %02.4f, %02.4f",gx, gy, gz);
            //System_printf(merkkijono);
 
        }
        // Once per second, you can modify this
        Task_sleep(10000 / Clock_tickPeriod);
 
    }
}
 
Int main(void) {
 
    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
 
    // Initialize board
    Board_initGeneral();
 
 
    Board_initI2C();
    //Initialize UART
    Board_initUART();
 
    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }
 
    // Otetaan pinnit käyttöön ohjelmassa
    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
        System_abort("Error initializing button pins\n");
    }
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
        System_abort("Error initializing LED pins\n");
    }
 
    // Asetetaan painonappi-pinnille keskeytyksen käsittelijäksi
    // funktio buttonFxn
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
        System_abort("Error registering button callback function");
    }
 
    /* Task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
 
    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTask, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
 
    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();
 
    /* Start BIOS */
    BIOS_start();
 
    return (0);
}
 