#include "ros/ros.h"
#include "ros/console.h"
#include "nidaq/analogInput.h"
#include "NIDAQmxBase.h"
#include <stdio.h>
#include <time.h>

#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }

using namespace ros;

int main (int argc, char **argv){
	const double common_sampling_rate = 5;	//10.0
        init(argc, argv, "nidaqAnalog6221");

        NodeHandle n;

        Publisher nidaq_pub = n.advertise <nidaq::analogInput> ("nidaqAnalog6221", 1000);	//0
        Rate loop_rate(common_sampling_rate);

	// Task parameters
	int32		error = 0;
	TaskHandle	taskHandle = 0;
	char		errBuff[2048] = { '\0' };
	int32		i, j;
	time_t		startTime;
	bool32		done = 0;

	//Channel parameters
	char		chan[] = "Dev1/ai0:15";
	float64		min = -10.0;
	float64		max = 10.0;

	//Timing parameters
	char		clockSource[] = "OnboardClock";
	uInt64		samplesPerChan = 1;	//1
	float64		sampleRate = common_sampling_rate;

	//Data read parameters
	#define		bufferSize (uInt32)16		//16
	float64		data[bufferSize];
	int32		pointsToRead = bufferSize;
	int32		pointsRead;
	float64 	timeout = 1.0;
	int32 		totalRead = 0;

	ROS_INFO("NIDAQmx Base node started");	
	DAQmxErrChk(DAQmxBaseCreateTask("", &taskHandle));
	DAQmxErrChk(DAQmxBaseCreateAIVoltageChan(taskHandle, chan, "", DAQmx_Val_RSE, min, max, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxBaseCfgSampClkTiming(taskHandle, clockSource, sampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChan));
	DAQmxErrChk (DAQmxBaseCfgInputBuffer(taskHandle,2000));	//wasnt here
	DAQmxErrChk(DAQmxBaseStartTask(taskHandle));	
        
	while(ok()){
           	DAQmxErrChk(DAQmxBaseReadAnalogF64(taskHandle, pointsToRead, timeout, DAQmx_Val_GroupByScanNumber, data, bufferSize, &pointsRead, NULL)); 
		nidaq::analogInput msg;
		msg.header.stamp = Time::now();

		msg.a0 = data[0];
		msg.a1 = data[1];
		msg.a2 = data[2];
		msg.a3 = data[3];
		msg.a4 = data[4];
		msg.a5 = data[5];
		msg.a6 = data[6];
		msg.a7 = data[7];
		msg.a8 = data[8];
		msg.a9 = data[9];
		msg.a10 = data[10];
		msg.a11 = data[11];
		msg.a12 = data[12];
		msg.a13 = data[13];
		msg.a14 = data[14];
		msg.a15 = data[15];

		totalRead += pointsRead;
		
		nidaq_pub.publish(msg);
		spinOnce();
		loop_rate.sleep();
	}

	Error:
		if (DAQmxFailed(error))
			DAQmxBaseGetExtendedErrorInfo(errBuff, 2048);
		if (taskHandle != 0)
		{
			DAQmxBaseStopTask(taskHandle);
			DAQmxBaseClearTask(taskHandle);
		}
		if (DAQmxFailed(error))
			printf("DAQmxBase Error %ld: %s\n", error, errBuff);
	return 0;
}
