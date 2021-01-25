


#include <string.h>

/* XDCtools Header files */
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>
#include <time.h>

#include <unistd.h>
#include "Board.h"
#include <sys/socket.h>

#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include <ti/drivers/GPIO.h>


#define TASKSTACKSIZE     4096
#define NTP_IP            "132.163.97.5"
#define SOCKETTEST_IP     "192.168.1.30"
#define LEDS GPIO_PIN_1

char buffer[80];

extern Swi_Handle swi0;
extern Mailbox_Handle mailbox0;
extern Mailbox_Handle mailbox2;
extern Event_Handle event0;


// ADC values will be put here
uint32_t ADCValues[2];

Void timerHWI(UArg arg1)
{
    //
    // Just trigger the ADC conversion for sequence 3. The rest will be done in SWI
    ADCProcessorTrigger(ADC0_BASE, 3);

    // post the SWI for the rest of ADC data conversion and buffering
    Swi_post(swi0);
}

Void ADCSwi(UArg arg1, UArg arg2)
{
    static uint32_t PE3_value;

    // Wait for conversion to be completed for sequence 3
    while(!ADCIntStatus(ADC0_BASE, 3, false));


    // Clear the ADC interrupt flag for sequence 3
    ADCIntClear(ADC0_BASE, 3);


    // Read ADC Value from sequence 3
    ADCSequenceDataGet(ADC0_BASE, 3, ADCValues);


    // Port E Pin 3 is the AIN0 pin. Therefore connect PE3 pin to the line that you want to
    // acquire. +3.3V --> 4095 and 0V --> 0
    PE3_value = ADCValues[0]; // PE3 : Port E pin 3

    // send the ADC PE3 values to the taskLEDWrite
    Mailbox_post(mailbox0, &PE3_value, BIOS_NO_WAIT);
}

/*
 *  ======== taskLEDWrite for writing UVC LED ========
 */
Void taskLEDWrite(UArg arg1, UArg arg2)
{

    static uint32_t pe3_val,tot;
    int iter=0;
    while(1) {
        int x=0,i=0;
        tot=0;
        for(i=0; i<1;i++){// get two ADC value and take average
            // wait the ADC PE3 values from swi
            Mailbox_pend(mailbox0, &pe3_val, BIOS_WAIT_FOREVER);
            x +=pe3_val;
            System_printf("adc value : %d\n",x);
            System_flush();

        }
        tot =  x/2;
        // detector is LED variable
        int detector = 0;

        if(tot < 10){
            // tot => adc value average is less then 10
            detector = 1;
            // PF1--> 1 LED is ON
            GPIOPinWrite(GPIO_PORTF_BASE, LEDS, 2);
        }

        else {
             //otherwise PF1--> 0 LED is OFF
            GPIOPinWrite(GPIO_PORTF_BASE, LEDS, 0);

        }

        // send the detector values to the taskHercules
        Mailbox_post(mailbox2, &detector, BIOS_NO_WAIT);

        iter++;
        if(iter == 20) {
        BIOS_exit(-1);  // quit from TI-RTOS
        }
    }
}
/*
 *  ======== functionTimer for receiving data from NTP server ========
 */
Void functionTimer(UArg arg1, UArg arg2) // getting TIME values
{

    while(1){
        char data2[4];
        int fd, connStat;
        struct sockaddr_in serverAddr;
        // wait the event from taskHercules
        Event_pend(event0, Event_Id_00, Event_Id_NONE, BIOS_WAIT_FOREVER);

        fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); //creating socket
        if (fd == -1) {
           System_printf("Socket not created");
           BIOS_exit(-1);
        }

        memset(&serverAddr, 0, sizeof(serverAddr));  // clear serverAddr structure
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(37);     // convert port # to network order
        inet_pton(AF_INET, "132.163.97.5", &(serverAddr.sin_addr));
        // connecting to NTP server
        connStat = connect(fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        if(connStat < 0) {
           System_printf("sendData2Server::Error while connecting to server\n");
           if(fd>0) close(fd);
           BIOS_exit(-1);

        }

        // recv 4 byte data from NTP SERVER
        recv(fd, data2, 4, 0);
        if(fd>0) close(fd);

        System_flush();
        //conversion data2 to second
        unsigned long int seconds= data2[0]*16777216 +  data2[1]*65536 + data2[2]*256 + data2[3];
        seconds+=10800; // for gmt+3
        time_t rawtime = seconds;
        struct tm  ts;
        // Format time, "yyyy-mm-dd hh:mm:ss"
         ts = *localtime(&rawtime);
         strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &ts);
         System_printf("time : %s\n",buffer);
         System_flush();
    }
}
/*
 *  ======== taskHercules for sending data to server ========
 */
Void taskHercules(UArg arg1, UArg arg2)
{
    while(1){

        int sockfd1;
        struct sockaddr_in serverAddr;  //take server address
        int data;
        // wait the detector values from taskLEDWrite
        Mailbox_pend(mailbox2, &data, BIOS_WAIT_FOREVER);

        char message[40];
        sockfd1 = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); //creating socket TCP
        if (sockfd1 == -1) {  //errorchecking
            System_printf("Socket not created BB");
            BIOS_exit(-1);
        }
        memset(&serverAddr, 0, sizeof(serverAddr));  /* clear serverAddr structure */
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(5011);     /* convert port # to network order */
        inet_pton(AF_INET, SOCKETTEST_IP, &(serverAddr.sin_addr));
        // connecting to server
        int connStat = connect(sockfd1, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        if(connStat < 0) {
            System_printf("Error while connecting to server\n");
            if (sockfd1 > 0)
                close(sockfd1);
            BIOS_exit(-1);
        }


        // post the event to the functionTimer
        Event_post(event0, Event_Id_00);

        if(data == 0){

            // detector --> 0 led is off
            strcat(message,buffer);
            strcat(message," Moving people are detected.UVC LED is off\n");

        }
        else{
            //otherwise --> les is on
            strcat(message,buffer);
            strcat(message," There is no people. UVC LEDS are ON");
            strcat(message,"\n");
        }
        //send data to server and listen to 5011 port from hercules, then see your message
        int numSend = send(sockfd1, message, strlen(message), 0);  /* send data to the server*/

        if(numSend < 0) {
            System_printf("Error while sending data to server\n");
            if (sockfd1 > 0) close(sockfd1);
            BIOS_exit(-1);
        }
        if(sockfd1>0) close(sockfd1);//close socket
    }
}
void initialize_ADC()
{
    // enable ADC and Port E
    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlDelay(10);

    // Select the analog ADC function for Port E pin 3 (PE3)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // enable Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // configure PF1 GPIO output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LEDS);

    // configure sequence 3
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // every step, only PE3 will be acquired
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    ADCIntClear(ADC0_BASE, 3);

}


void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}
/*
 *  ======== creating task with netIPAddrHook ========
 */

void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
        // when connecting network, task is occur
       static Task_Handle taskHandle1,taskHandle2;
       Task_Params taskParams;
       Error_Block eb;
       if (fAdd && !taskHandle1 && !taskHandle2) {
           Error_init(&eb);
       }
       // creating taskHercules for sending data to server
       Task_Params_init(&taskParams);
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.priority = 1;
       taskHandle1 = Task_create((Task_FuncPtr)taskHercules, &taskParams, &eb);
       //creating function Timer for receiving data from NTP server
       Task_Params_init(&taskParams);
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.priority = 1;
       taskHandle2 = Task_create((Task_FuncPtr)functionTimer, &taskParams, &eb);

       if (taskHandle1 == NULL) {
           printError("netIPAddrHook: Failed to create HTTP and Socket Tasks\n", -1);
       }
       System_printf("netIPAddrHook end\n");
       System_flush();

}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();

    System_printf("Starting the HTTP GET example\nSystem provider is set to "
            "SysMin. Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();
    initialize_ADC();

    BIOS_start();

    return (0);
}
