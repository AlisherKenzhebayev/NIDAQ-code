/*********************************************************************
*
* ANSI C Example program:
*    contGeneration.c
*
* Example Category:
*    AO
*
* Description:
*    This example demonstrates how to output a continuous periodic
*    waveform using an internal sample clock.
*
* Instructions for Running:
*    1. Select the Physical Channel to correspond to where your signal
*       is output on the DAQ device.
*    2. Enter the Minimum and Maximum Voltage Ranges.
*    3. Enter the desired rate for the generation. The onboard sample
*       clock will operate at this rate.
*    4. Select the desired waveform type.
*    5. The rest of the parameters in the Waveform Information section
*       will affect the way the waveform is created, before it's sent
*       to the analog output of the board. Select the number of samples
*       per cycle and the total number of cycles to be used as waveform
*       data.
*
* Steps:
*    1. Create a task.
*    2. Create an Analog Output Voltage channel.
*    3. Define the update Rate for the Voltage generation. Additionally,
*       define the sample mode to be continuous.
*    4. Write the waveform to the output buffer.
*    5. Call the Start function.
*    6. Loop continuously for 10 seconds. Check for errors every
*       100 ms using the IsTaskDone function.
*    7. Call the Clear Task function to clear the Task.
*    8. Display an error if any.
*
* I/O Connections Overview:
*    Make sure your signal output terminal matches the Physical Channel
*    I/O Control. For further connection information, refer to your
*    hardware reference manual.
*
* Recommended use:
*    Call Configure, Write and Start functions.
*    Call IsDone function in a loop.
*    Call Stop function at the end.
*
*********************************************************************/
#include <NIDAQmxBase.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "nidaq/analogOutput.h"
#include "NIDAQmxBase.h"
#include <signal.h>
#include <stdlib.h>


#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }
#define PI	3.1415926535

using namespace ros;

static int gRunning=0;

void my_handler(int s){
           printf("Caught signal %d\n",s);
           exit(1); 

}

int main(int argc, char *argv[])
{
    const double common_sampling_rate = 82000.0;  //drastic freq change (/1000) -> Hz
    init(argc, argv, "nidaqPublOutputNI6221");

    NodeHandle n;

    Publisher nidaq_pub = n.advertise <nidaq::analogOutput> ("nidaqOutput6221", 0);

    // Task parameters
    int32       error = 0;
    TaskHandle  taskHandle = 0;
    int32       i = 0;
    char        errBuff[2048]={'\0'};
    bool32      done = 0;
    time_t      startTime;
 
    // Channel parameters
    char        chan[] = "Dev1/ao0";
    float64     min = -2.5;
    float64     max = 2.5;

    // Timing parameters
    #define     bufferSize 512		//affects frequency
    char	clockSource[] = "OnboardClock";
    uInt64      samplesPerChan = bufferSize;
    float64     sampleRate = common_sampling_rate;

    // Data write parameters
    float64     data[bufferSize];
    int32       pointsWritten;
    float64     timeout = 0.5;		//affects frequency
    int32 	totalWritten = 0;	

    for(;i<bufferSize; i++){	//basically, generates a sine wave with a number of "bufferSize" points on it and repeats it.
        data[i] = 2.35*sin((double)i*2.0*PI/(double)bufferSize);
    }

    ROS_INFO("NIDAQmx Base node started");	
    DAQmxErrChk (DAQmxBaseCreateTask("",&taskHandle));
    DAQmxErrChk (DAQmxBaseCreateAOVoltageChan(taskHandle, chan, "", min, max, DAQmx_Val_Volts, NULL));
    DAQmxErrChk (DAQmxBaseCfgSampClkTiming(taskHandle, clockSource, sampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChan));

    DAQmxErrChk (DAQmxBaseWriteAnalogF64(taskHandle,samplesPerChan,0,timeout,DAQmx_Val_GroupByChannel,data,&pointsWritten,NULL));

    DAQmxErrChk (DAQmxBaseStartTask(taskHandle));

    i = 0;
    gRunning = 1;	
    startTime = time(NULL);
    ROS_INFO("Loop will quit after pressing Ctrl+C");	
    while(gRunning && !done) {
	signal(SIGINT, my_handler);


        DAQmxErrChk (DAQmxBaseIsTaskDone(taskHandle, &done));
	if( !done )
            usleep(2000);	//500 Hz <-> 10^6/x Hz

	nidaq::analogOutput msg;
	msg.header.stamp = Time::now();

	msg.o0 = data[i];
	totalWritten += pointsWritten;	

	nidaq_pub.publish(msg);
	spinOnce();

	i=(i + 1) % bufferSize;
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
