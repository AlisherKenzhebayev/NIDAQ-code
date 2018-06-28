/*********************************************************************
*
* ANSI C Example program:
*    genVoltage.c
*
* Example Category:
*    AO
*
* Description:
*    This example demonstrates how to output a single Voltage Update
*    (Sample) to an Analog Output Channel.
*
* Instructions for Running:
*    1. Select the Physical Channel to correspond to where your
*       signal is output on the DAQ device.
*    2. Enter the Minimum and Maximum Voltage Ranges.
*    Note: Use the acquire1Scan example to verify you are
*          generating the correct output on the DAQ device.
*
* Steps:
*    1. Create a task.
*    2. Create an Analog Output Voltage Channel.
*    3. Use the Write function to Output 1 Sample to 1 Channel on the
*       Data Acquisition Card.
*    4. Display an error if any.
*
* I/O Connections Overview:
*    Make sure your signal output terminal matches the Physical
*    Channel I/O Control. For further connection information, refer
*    to your hardware reference manual.
*
* Recommended Use:
*    1. Call the Write function.
*
*********************************************************************/

#include <NIDAQmxBase.h>
#include <stdio.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "nidaq/analogOutput.h"
#include <signal.h>
#include <stdlib.h>

#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }

using namespace ros;

static int gRunning = 0;

void my_handler(int s){
    printf("Caught signal %d\n",s);
    exit(1); 
}

int main(int argc, char *argv[])
{
    const double common_sampling_rate = 2000.0;  //drastic freq change (/1000) -> Hz
    
    init(argc, argv, "VoltGen6221_test");

    NodeHandle n;

    Publisher nidaq_pub = n.advertise <nidaq::analogOutput> ("VoltGen6221_test", 0);

    // Task parameters
    int32       error = 0;
    TaskHandle  taskHandle = 0;
    char        errBuff[2048]={'\0'};
    bool32	done = 0;

    // Channel parameters
    char        chan[] = "Dev2/ao0";
    float64     min = 0.0;
    float64     max = 5.0;

    // Timing parameters
    #define     bufferSize 1		//affects frequency
    char	clockSource[] = "OnboardClock";
    uInt64      samplesPerChan = bufferSize;
    float64     sampleRate = common_sampling_rate;
    time_t 	startTime;

    // Data write parameters
    float64     data = 5.0;
    float64	data2 = 3.0;
    int32       pointsWritten;
    float64     timeout = 10.0;
    int32 	totalWritten = 0;

    ROS_INFO("NIDAQmx Base Voltage Generation started");
    DAQmxErrChk (DAQmxBaseCreateTask("",&taskHandle));
    DAQmxErrChk (DAQmxBaseCreateAOVoltageChan(taskHandle,chan,clockSource,min,max, DAQmx_Val_Volts ,NULL));
    DAQmxErrChk (DAQmxBaseCfgSampClkTiming(taskHandle, clockSource, sampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChan));

    DAQmxErrChk (DAQmxBaseWriteAnalogF64(taskHandle,samplesPerChan,0,timeout,DAQmx_Val_GroupByChannel,&data,&pointsWritten,NULL));

    DAQmxErrChk (DAQmxBaseStartTask(taskHandle));

    gRunning = 1;	
    ROS_INFO("Loop will quit after 10s");
    startTime = time(NULL);	    
    while(gRunning && !done && time(NULL) < startTime+10) {
//	signal(SIGINT, my_handler);

        DAQmxErrChk (DAQmxBaseIsTaskDone(taskHandle, &done));
	if( !done )
            usleep(2000);	//500 Hz <-> 10^6/x Hz

	nidaq::analogOutput msg;
	msg.header.stamp = Time::now();

	msg.o0 = data;
	totalWritten += pointsWritten;	

	nidaq_pub.publish(msg);
	spinOnce();
    }

    ROS_INFO("Trying to recreate the task with other parameters");
        
    DAQmxErrChk(DAQmxBaseStopTask(taskHandle));
    
    DAQmxErrChk (DAQmxBaseWriteAnalogF64(taskHandle,samplesPerChan,0,timeout,DAQmx_Val_GroupByChannel,&data2,&pointsWritten,NULL));
    DAQmxErrChk (DAQmxBaseStartTask(taskHandle));

    gRunning = 1;	
    ROS_INFO("Loop will quit after 10s");
    startTime = time(NULL);	    
    while(gRunning && !done && time(NULL) < startTime+10) {
//	signal(SIGINT, my_handler);

        DAQmxErrChk (DAQmxBaseIsTaskDone(taskHandle, &done));
	if( !done )
            usleep(2000);	//500 Hz <-> 10^6/x Hz

	nidaq::analogOutput msg;
	msg.header.stamp = Time::now();

	msg.o0 = data;
	totalWritten += pointsWritten;	

	nidaq_pub.publish(msg);
	spinOnce();
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
    return 0;
}
