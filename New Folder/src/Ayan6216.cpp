/*********************************************************************
*
* ANSI C Example program:
*    contAcquireNChan.c
*
* Example Category:
*    AI
*
* Description:
*    This example demonstrates how to continuously acquire data on
*    multiple channels using the DAQ device's internal clock.
*
* Instructions for Running:
*    1. Select the physical channels to correspond to where your
*       signals are input on the DAQ device.
*    2. Enter the minimum and maximum voltage range.
*    Note: For better accuracy try to match the input range to the
*          expected voltage level of the measured signal.
*    3. Set the rate of the acquisition. Also set the Samples to Read
*       control. This will determine how many samples are read each
*       time the while loop iterates. This also determines how many
*       points are plotted on the graph each iteration.
*    Note: The rate should be at least twice as fast as the maximum
*          frequency component of the signal being acquired.
*
* Steps:
*    1. Create a task.
*    2. Create an analog input voltage channel.
*    3. Set the rate for the sample clock. Additionally, define the
*       sample mode to be continuous.
*    4. Call the Start function to start the acquistion.
*    5. Read the data in a loop until 10 seconds or an
*       error occurs.
*    6. Call the Clear Task function to clear the task.
*    7. Display an error if any.
*
* I/O Connections Overview:
*    Make sure your signal input terminal matches the Physical
*    Channel I/O control. Also, make sure your analog trigger
*    terminal matches the Trigger Source Control. For further
*    connection information, refer to your hardware reference manual.
*
* Recommended Use:
*    1. Call Configure and Start functions.
*    2. Call Read function in a loop.
*    3. Call Stop function at the end.
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

#define PI	3.1415926535
#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }

using namespace ros;

void my_handler(int s){
    printf("Caught signal %d\n",s);
    exit(1); 
}

int main(int argc, char *argv[])
{ 
    const double common_rate = 5000;		//80Hz

    init(argc, argv, "Ayan6216");
    NodeHandle n;

    Publisher nidaq_pub = n.advertise <nidaq::fsrInput> ("Ayan6216", 0);    
    Rate loop_rate(10);

    // Task parameters
    TaskHandle  taskHandleAI = 0;
    int32       error = 0;
    char        errBuff[2048]={'\0'};
    int32       i,j;
    bool32      done=0;

    // Channel parameters
    char        chanAI[] = "Dev2/ai0";	//get 0-15
    float64     maxAI = 10.0;
    float64     minAI = -10.0;
    
    // Timing parameters
    char        clockSource[] = "OnboardClock";
    uInt64      samplesPerChanAI = 1;

    // Data read parameters
    #define     bufferSize (uInt32)1
    float64     dataAI[bufferSize];	//data read on AI, and published.
    int32       pointsToRead = bufferSize;
    int32       pointsRead;
    float64     timeout = 1.0;
    int32       totalRead = 0;
    int32       pointsWrittenAO;

    ROS_INFO("NIDAQmx Base node started");
    DAQmxErrChk(DAQmxBaseCreateTask("", &taskHandleAI));

    DAQmxErrChk (DAQmxBaseCreateAIVoltageChan(taskHandleAI, chanAI, "", DAQmx_Val_RSE, minAI, maxAI, DAQmx_Val_Volts, NULL));
   
    //DAQmxErrChk (DAQmxBaseCfgSampClkTiming(taskHandleAI, clockSource, common_rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChanAI));
    DAQmxErrChk (DAQmxBaseStartTask(taskHandleAI));
    ROS_INFO("NIDAQmx AI");

    while(!done) {
	//stop AO in here, just to relaunch it with new data
	//DAQmxErrChk(DAQmxBaseStopTask(taskHandleHAO));
	//DAQmxErrChk (DAQmxBaseWriteAnalogF64(taskHandleHAO, samplesPerChanHAO, 0, timeout, DAQmx_Val_GroupByChannel, data, &pointsWrittenHAO, NULL));
	//DAQmxErrChk (DAQmxBaseStartTask(taskHandleHAO));

	DAQmxBaseIsTaskDone(taskHandleAI, &done);
	signal(SIGINT, my_handler);

	ROS_INFO("Still running");
        DAQmxErrChk (DAQmxBaseReadAnalogF64(taskHandleAI, pointsToRead, timeout, DAQmx_Val_GroupByScanNumber, dataAI, bufferSize, &pointsRead, NULL));
        totalRead += pointsRead;

        nidaq::fsrInput msg;
	msg.header.stamp = Time::now();

	msg.i0 = dataAI[0];

	printf("%f \n", dataAI[0]);

	totalRead += pointsRead;
		
	nidaq_pub.publish(msg);
	spinOnce();
	loop_rate.sleep();
    }
    
Error:
    if( DAQmxFailed(error) ){
        DAQmxBaseGetExtendedErrorInfo(errBuff,2048);
    }
    if(taskHandleAI != 0) {
        DAQmxBaseStopTask (taskHandleAI);
        DAQmxBaseClearTask (taskHandleAI);
    }
    if( DAQmxFailed(error) )
		ROS_INFO ("DAQmxBase Error %ld: %s\n", error, errBuff);
    return 0;
}
