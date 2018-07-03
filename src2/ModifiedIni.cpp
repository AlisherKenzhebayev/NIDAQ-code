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
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include "NIDAQmxBase.h"
#include <math.h>

#define PI	3.1415926535
#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }

using namespace ros;

    static TaskHandle  taskHandleAI = 0;
    static TaskHandle  taskHandleAO = 0;
    static TaskHandle  taskHandleHAO = 0;

void my_handler(int s){
    printf("Caught signal %d\n",s);
    exit(1); 
}

/*********************************************************************
*    Recalculates the number in max_analog_value to enable simple 
*    self-adjustment 
*********************************************************************/
//void my_callback(const nidaq::analogInput& msg)
//{
//    ROS_INFO("I heard on node A0: [%f]", msg.a0);
//    if(msg.a0 > max_analog_value)
//        max_analog_value = msg.a0;
//    output_sampling_rate = (msg.a0 / max_analog_value) * max_sampling_rate;
//}


//THREADS? -> different frequencies

int main(int argc, char *argv[])
{ 
    float64 MINi = 10;
    float64 MAXi = 0;

    float64 common_rate = 5000;		//80Hz
    float64 acqui_rate = 10000;		//1Hz
    float64 wave_rate = 200000;

    init(argc, argv, "ModifiedIni");
    NodeHandle n;

    Publisher nidaq_pub = n.advertise <nidaq::analogInput> ("ModifiedIni", 1);    
    Rate loop_rate(10000);

    // Task parameters
    int32       error = 0;
    char        errBuff[2048]={'\0'};
    int32       i,j;
    bool32      done=0;

    // Channel parameters
    char        chanAI[] = "Dev1/ai0:15";	//get 0-15
    char        chanAO[] = "Dev1/ao1";		//gen 5V
    char	chanHAO[] = "Dev1/ao0";		//gen sin wave
    float64     maxAI = 10.0;
    float64     minAI = -10.0;
    float64     maxAO = 5.0;
    float64     minAO = -5.0;

    // Timing parameters
    #define     bufferSize (uInt32)512
    char        clockSource[] = "OnboardClock";
    uInt64      samplesPerChanAI = 1;
    uInt64      samplesPerChanAO = 1;
    uInt64      samplesPerChanHAO = bufferSize;

    uInt64 	freqUpd = 10;
    uInt64 	counter = 0;

    // Data read parameters
    #define     bufferSize16 (uInt32)16
    float64     dataAI[bufferSize16];	//data read on AI, and published.
    float64	data[bufferSize];	//sine wave with samplesPerChanHAO num of samples
    float64 	dataAO = 5;		//5V
    int32       pointsToRead = bufferSize16;
    int32       pointsRead;
    float64     timeout = 0.1;
    int32       totalRead = 0;
    int32       pointsWrittenAO;
    int32       pointsWrittenHAO;
    float64     timeoutAO = 0.1;

    for(int i=0; i<bufferSize; i++){
        data[i] = 2.5*sin((double)i*2.0*PI/(double)bufferSize);
    }

    ROS_INFO("NIDAQmx Base node started");
    DAQmxErrChk(DAQmxBaseCreateTask("", &taskHandleAI));
    DAQmxErrChk(DAQmxBaseCreateTask("", &taskHandleAO));	 
    DAQmxErrChk(DAQmxBaseCreateTask("", &taskHandleHAO));

    DAQmxErrChk (DAQmxBaseCreateAIVoltageChan(taskHandleAI, chanAI, "", DAQmx_Val_RSE, minAI, maxAI, DAQmx_Val_Volts, NULL));
    DAQmxErrChk (DAQmxBaseCreateAOVoltageChan(taskHandleAO, chanAO, "", 0, maxAO, DAQmx_Val_Volts, NULL));
    DAQmxErrChk (DAQmxBaseCreateAOVoltageChan(taskHandleHAO, chanHAO, "", minAO, maxAO, DAQmx_Val_Volts, NULL));

//start 5V
    DAQmxErrChk (DAQmxBaseCfgSampClkTiming(taskHandleAO, clockSource, 2000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChanAO));
    DAQmxErrChk (DAQmxBaseWriteAnalogF64(taskHandleAO, samplesPerChanAO, 0, timeoutAO, DAQmx_Val_GroupByChannel, &dataAO, &pointsWrittenAO,NULL));
    DAQmxErrChk (DAQmxBaseStartTask(taskHandleAO));
    ROS_INFO("NIDAQmx 5V");
	
//Wave Sine
    DAQmxErrChk (DAQmxBaseCfgSampClkTiming(taskHandleHAO, clockSource, wave_rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChanHAO));
    DAQmxErrChk (DAQmxBaseWriteAnalogF64(taskHandleHAO, samplesPerChanHAO, 0, timeout, DAQmx_Val_GroupByChannel, data, &pointsWrittenHAO, NULL));
    DAQmxErrChk (DAQmxBaseStartTask(taskHandleHAO));
    ROS_INFO("NIDAQmx Sine");

    //DAQmxErrChk (DAQmxBaseCfgSampClkTiming(taskHandleAI, clockSource, acqui_rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChanAI));
    DAQmxErrChk (DAQmxBaseStartTask(taskHandleAI));
    ROS_INFO("NIDAQmx AI");

    while(!done) {
	counter ++;

	if(counter >= freqUpd){
	    counter = 0;
	    //stop AO in here, just to relaunch it with new data
	    DAQmxErrChk(DAQmxBaseStopTask(taskHandleHAO));
	    DAQmxErrChk (DAQmxBaseWriteAnalogF64(taskHandleHAO, samplesPerChanHAO, 0, timeout, DAQmx_Val_GroupByChannel, data, &pointsWrittenHAO, NULL));
	    DAQmxErrChk (DAQmxBaseStartTask(taskHandleHAO));
	}

	signal(SIGINT, my_handler);
	DAQmxErrChk (DAQmxBaseIsTaskDone(taskHandleAO, &done));

	ROS_INFO("Still running");
        DAQmxErrChk (DAQmxBaseReadAnalogF64(taskHandleAI, pointsToRead, timeout, DAQmx_Val_GroupByScanNumber, dataAI, bufferSize, &pointsRead, NULL));
        totalRead += pointsRead;

        nidaq::analogInput msg;
	msg.header.stamp = Time::now();

	msg.a0 = dataAI[0];
	msg.a1 = dataAI[1];
	msg.a2 = dataAI[2];
	msg.a3 = dataAI[3];
	msg.a4 = dataAI[4];
	msg.a5 = dataAI[5];
	msg.a6 = dataAI[6];
	msg.a7 = dataAI[7];
	msg.a8 = dataAI[8];
	msg.a9 = dataAI[9];
	msg.a10 = dataAI[10];
	msg.a11 = dataAI[11];
	msg.a12 = dataAI[12];
	msg.a13 = dataAI[13];
	msg.a14 = dataAI[14];
	msg.a15 = dataAI[15];

	if(dataAI[0] > MAXi)
	    MAXi = dataAI[0];

	if(dataAI[0] < MINi)
	    MINi = dataAI[0];

	//wave_rate is dependent on a0;
	//wave_rate = (dataAI[0]/MAXi) * common_rate;

    for(int i=0; i<bufferSize; i++){
	if(MINi >= MAXi){
		data[i] = 0;	//bug fix from min being too big and max too small	
	}else{
	        data[i] = 2.5*((dataAI[0]-MINi)/(MAXi - MINi))*sin((double)i*2.0*PI/(double)bufferSize);
	}
	printf("%f ", data[i]);
    }
	printf("\n");
	printf("MIN %fMAX %f\n\n", MINi, MAXi);

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
        DAQmxBaseStopTask (taskHandleAO);
        DAQmxBaseClearTask (taskHandleAO);
        DAQmxBaseStopTask (taskHandleAI);
        DAQmxBaseClearTask (taskHandleAI);
        DAQmxBaseStopTask (taskHandleHAO);
        DAQmxBaseClearTask (taskHandleHAO);
    }
    if( DAQmxFailed(error) )
		printf ("DAQmxBase Error %ld: %s\n", error, errBuff);
    return 0;
}
