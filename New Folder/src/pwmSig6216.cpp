/*********************************************************************
*
* ANSI C Example program:
*    genDigPulseTrainCont.c
*
* Example Category:
*    CO
*
* Description:
*    This example demonstrates how to generate a continuous digital
*    pulse train from a Counter Output Channel. The Frequency, and
*    Duty Cycle are all configurable.
*
* Instructions for Running:
*    1. Select the Physical Channel which corresponds to the counter
*       you want to output your signal to on the DAQ device.
*    2. Enter the Frequency and Duty Cycle to define the pulse
*       parameters.
*       Additionally, you can set the Initial Delay (in seconds)
*       which will delay the beginning of the pulse train from the
*       start call; this is currently set to 0.0 in the code.
*    Note: Use the buffPeriodFinite example to verify you are
*          outputting the pulse train on the DAQ device.
*
* Steps:
*    1. Create a task.
*    2. Create a Counter Output channel to produce a Pulse in terms
*       of Frequency.
*    3. Call the timing function to configure the duration of the
*       pulse generation.
*    4. Call the Start function to arm the counter and begin the
*       pulse train generation.
*    5. For continuous generation, the counter will continually
*       generate the pulse train until told to stop. This will happen
*       after 10 seconds by default.
*    6. Call the Clear Task function to clear the Task.
*    7. Display an error if any.
*
* I/O Connections Overview:
*    The counter will output the pulse train on the output terminal
*    of the counter specified in the Physical Channel I/O control.
*
*    In this example the output will be sent to the default output
*    terminal on ctr0.
*
*    For more information on the default counter input and output
*    terminals for your device, open the NI-DAQmxBase Help, and refer
*    to Counter Signal Connections found under the Device Considerations
*    book in the table of contents.
*
* Recommended Use:
*    Call Configure and Start functions.
*    Call Write function in a loop.
*    Call Stop function at the end.
*
*********************************************************************/

#include <NIDAQmxBase.h>
#include "ros/ros.h"
#include <signal.h>
#include "ros/console.h"
#include "nidaq/analogInput.h"
#include "nidaq/analogOutput.h"
#include "nidaq/fsrInput.h"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include "NIDAQmxBase.h"
#include <math.h>
#include <pthread.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }
#define constDuty 0.99

using namespace ros; 
using namespace std;

static int gRunning=0;

float64 msgCoef = 1;

void my_handler(int s){
    printf("Caught signal %d\n",s);
    exit(1); 
}

void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%f]", msg->data);
  msgCoef = msg->data;
}

void *call_from_thread(void *){
	//launch a new thread, to handle arriving messages
    
    NodeHandle n;
    Subscriber sub = n.subscribe("Total_load", 1, chatterCallback);

    spin();	//Runs indefinetely, without interruption. Has to be placed somewhere outside (?)
}

int main(int argc, char *argv[])
{ 
	init(argc, argv, "pwmSig6216");

	pthread_t t;

	pthread_create(&t, NULL, call_from_thread, NULL);

    // Task parameters
    int32       error = 0;
    int32 		counter = 0;
    TaskHandle  taskHandle = 0;
    char        errBuff[2048]={'\0'};
    time_t      startTime;

    // Channel parameters
    char        chan[] = "Dev1/ctr0";
    float64     freq = 30;
    int32 		repFreq = freq;
    float64     duty = constDuty;
    float64     delay = 0.0;

    // Data write parameters
    float64     data = 0.0;
    float64     timeout = 10.0;
    bool32      done = 0;
	
	//ROS_INFO("Task init");
    DAQmxErrChk (DAQmxBaseCreateTask("",&taskHandle));
    DAQmxErrChk (DAQmxBaseCreateCOPulseChanFreq(taskHandle,chan,"",DAQmx_Val_Hz,DAQmx_Val_Low,delay,freq,duty));
    DAQmxErrChk (DAQmxBaseCfgImplicitTiming(taskHandle,DAQmx_Val_ContSamps,1000));
    DAQmxErrChk (DAQmxBaseStartTask(taskHandle));

    gRunning = 1;

    while( gRunning && !done ) {
    	counter++;
    	//ROS_INFO("CounterIncrease");
    	if(counter >= repFreq){ 	//recreate task with some frequency
    		counter = 0;
    		DAQmxErrChk (DAQmxBaseStopTask(taskHandle));
    		DAQmxErrChk (DAQmxBaseClearTask(taskHandle));
    		DAQmxErrChk (DAQmxBaseCreateTask("",&taskHandle));
    		duty = constDuty * msgCoef + (1 - msgCoef) * 0.3;
    		if(duty > 0.99){
    			duty = 0.99;
    		}
    		//ROS_INFO("Duty: [%f]", duty);
		    DAQmxErrChk (DAQmxBaseCreateCOPulseChanFreq(taskHandle,chan,"",DAQmx_Val_Hz,DAQmx_Val_Low,delay,freq,duty));
		    DAQmxErrChk (DAQmxBaseCfgImplicitTiming(taskHandle,DAQmx_Val_ContSamps,1000));
		    DAQmxErrChk (DAQmxBaseStartTask(taskHandle));
		}
        DAQmxErrChk (DAQmxBaseIsTaskDone(taskHandle,&done));
		signal(SIGINT, my_handler);
        if( !done )
            usleep(100000);	//0.1 s.
    }

Error:
    if( DAQmxFailed(error) )
        DAQmxBaseGetExtendedErrorInfo(errBuff,2048);
    if( taskHandle!=0 ) {
        DAQmxBaseStopTask(taskHandle);
        DAQmxBaseClearTask(taskHandle);
    }
    if( DAQmxFailed(error) )
		printf ("DAQmxBase Error %ld: %s\n", error, errBuff);

    pthread_join(t, NULL);
    return 0;
}
