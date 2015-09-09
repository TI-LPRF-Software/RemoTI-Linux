/**************************************************************************************************
 Filename:       lnxsample_main_target.cpp

 Description:    Linux Host application

 Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/


 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.

 Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the
 distribution.

 Neither the name of Texas Instruments Incorporated nor the names of
 its contributors may be used to endorse or promote products derived
 from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 **************************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>

#include <sys/time.h>

#include "RTI_Testapp.h"
#include "lprfLogging.h"

// Application test mode state variable
static uint8 appTestModeState;

// Application test mode state variable
uint8 appTestPairState = APP_TEST_PAIR_STATE_INIT;
struct {
	int successfulPairings;
	int unSuccessfulPairings;
} appTestPairStats = { 0, 0 };

// Application test mode run count variable
static struct {
	int16 currentCount;
	int16 goal;
} appTestModeRunCount[RCN_CAP_PAIR_TABLE_SIZE];

// Structure array for all test mode parameters associated with a pairing reference
static rrtTestReq_t appTestModeParameters[RCN_CAP_PAIR_TABLE_SIZE];

// Array to count received packets
uint16 appTestModeReceivedPacketCnt[RCN_CAP_PAIR_TABLE_SIZE];
// Array to count accumulatively received packets
uint32 appTestModeReceivedPacketCntAcc[RCN_CAP_PAIR_TABLE_SIZE];
// Array to count accumulated bins
uint32 appTestModeReceivedBinAcc[RCN_CAP_PAIR_TABLE_SIZE][29];
// Variable to keep track of what version the different pairing references support
uint8 appTestModeVersion[RCN_CAP_PAIR_TABLE_SIZE];
// Hold off new attempt of the synchronous when packets are not coming as frequent as expected.
uint8 appTestModeHoldOffSynchronousTest = 1;

// Array to count cumulative payload
uint32 appTestModeReceivedcumulativePayload[RCN_CAP_PAIR_TABLE_SIZE];
// Array to count cumulative Latency
uint32 appTestModeReceivedcumulativeLatency[RCN_CAP_PAIR_TABLE_SIZE];
float appTestModeReceivedaverageLatency[RCN_CAP_PAIR_TABLE_SIZE];
uint16 appTestModeReceivedminLatency[RCN_CAP_PAIR_TABLE_SIZE];
uint16 appTestModeReceivedmaxLatency[RCN_CAP_PAIR_TABLE_SIZE];
struct timeval appTestModeReceivedStartTime[RCN_CAP_PAIR_TABLE_SIZE];
float appTestModeReceivedThroughput[RCN_CAP_PAIR_TABLE_SIZE];
FILE* appTestModeReceivedFilep[RCN_CAP_PAIR_TABLE_SIZE] = { NULL };
int appTestModeReceivedcumulativeRSSI[RCN_CAP_PAIR_TABLE_SIZE];
float appTestModeReceivedaverageRSSI[RCN_CAP_PAIR_TABLE_SIZE];
int appTestModeReceivedminRSSI[RCN_CAP_PAIR_TABLE_SIZE];
int appTestModeReceivedmaxRSSI[RCN_CAP_PAIR_TABLE_SIZE];

// Pairing reference of destination during test
testModeDest_t testModeDest = { 0x00, { 0x00, RTI_INVALID_PAIRING_REF,
		RTI_INVALID_PAIRING_REF, RTI_INVALID_PAIRING_REF,
		RTI_INVALID_PAIRING_REF, RTI_INVALID_PAIRING_REF,
		RTI_INVALID_PAIRING_REF, RTI_INVALID_PAIRING_REF,
		RTI_INVALID_PAIRING_REF, RTI_INVALID_PAIRING_REF }, 0x01 };

static const char * const testMode_testType_list[RSA_TEST_PER + 1] =
{ [0 ... 2] = NULL,
		NAME_ELEMENT(RSA_TEST_TERMINATE),
		NAME_ELEMENT(RSA_TEST_LATENCY),
		NAME_ELEMENT(RSA_TEST_PER)
};

const char * const testMode_startCondition_list[RSA_TEST_START_ON_BUTTON + 1] =
{ [0 ... 1] = NULL, NAME_ELEMENT(RSA_TEST_START_IMMEDIATE),
		NAME_ELEMENT(RSA_TEST_START_ON_BUTTON)
};

const char * const testMode_txPower_list[28] =
{ " \0407\tdBm",
		" \0407\tdBm",
		" \0407\tdBm",
		" \0404.5\tdBm",
		" \0403\tdBm",
		" \0403\tdBm",
		" \0401.7\tdBm",
		" \0400.3\tdBm",
		" -1\tdBm",
		" -1\tdBm",
		" -2.8\tdBm",
		" -2.8\tdBm",
		" -4.1\tdBm",
		" -5.9\tdBm",
		" -5.9\tdBm",
		" -7.7\tdBm",
		" -7.7\tdBm",
		" -9.9\tdBm",
		" -9.9\tdBm",
		" -9.9\tdBm",
		"-12.8\tdBm",
		"-12.8\tdBm",
		"-14.9\tdBm",
		"-14.9\tdBm",
		"-16.6\tdBm",
		"-16.6\tdBm",
		"-18.7\tdBm",
		"-18.7\tdBm"
};

#define LQI_TO_DBM_CONVERSION(LQI)			((LQI - 0xFF) >> 1)

struct timeval curTime, prevTimeSend;

// Random timeout for SREQ test
uint16 sreqRandomTimeout = 0;

//SAve Test to file:
bool appTestFileSave = FALSE;

void appTestModeSendTestSettings(uint8 destIdx);
void appTestModeProcessReport(uint8 srcIndex, uint8* pData, uint8 len);

void appTestModeProcessKey(char* strIn);
void appTestModeProcessKeyForCFGDestinationIndex(char* keyStr);
void appTestModeProcessKeyForCFGStartCondition(char key);
void appTestModeProcessKeyForCFGTxPower(char key);
void appTestModeProcessKeyForCFGPayload(char* strIn);
//void appTestModeProcessKeyForSetVersion(char key);
void appTestModeProcessKeyForNumOfRuns(char* strIn);
void appTestModeProcessKeyForNumOfPackets(char* strIn);
void appTestModeProcessKeyForTxOptions(char* strIn);

void DispTestModeMenuInit(uint8 destIdx);
void DispTestModeCurrentCfg(rrtTestReq_t testReq, uint8 destIdx, uint8 appTestModeVersion, uint8 numOfCycles, bool boolFileSave );
void DispTestModeCFGDestinationIndexMenuInit ( void );
void DispTestModeCFGStartConditionMenuInit (void );
void DispTestModeCFGTxPowerMenuInit( void );
void DispTestModeCFGPayloadSizeMenuInit ( void );
void DispTestModeCFGTxOptions ( void );
void DispTestModeSetNumOfRunsMenuInit ( void );
void DispTestModeCFGNumOfPacketsMenuInit ();
void DispTestModeCFGMaxAdditionalDelayMenuInit (uint16 appTestModeVersion);
void DispTestModeReport(uint8 srcIdx,
		int16 runNumber,
		void* bin,
		int8 txPower,
		float PER,
		uint32 numOfSentPackets,
		uint32 numReceivedPackets,
		uint8 appTestModeVersion);

static rtiReturnFromTestmodeModuleCbackFn_t rtiTestmodeCb = NULL;

/**************************************************************************************************
 *
 * @fn      RTI_TestModeInit
 *
 **************************************************************************************************/

void RTI_TestModeInit(rtiReturnFromTestmodeModuleCbackFn_t cback)
{
	rtiTestmodeCb = cback;
	// Initialize Test Parameters structure
	uint8 i;
	for (i = 0; i < RCN_CAP_PAIR_TABLE_SIZE; i++) {
		appTestModeParameters[i].numPackets = 1000;
		appTestModeParameters[i].startCondition = RSA_TEST_START_IMMEDIATE;
		appTestModeParameters[i].testType = RSA_TEST_LATENCY;
		appTestModeParameters[i].txOptions = RTI_TX_OPTION_ACKNOWLEDGED
				| RTI_TX_OPTION_VENDOR_SPECIFIC | RTI_TX_OPTION_SECURITY;
		appTestModeParameters[i].maxBackoffDuration = 0; // This field is not supported by old versions of Test Mode
		appTestModeParameters[i].userDataSize = 6;
		appTestModeParameters[i].txPower = 7;// This field is not supported by old versions of Test Mode
		appTestModeRunCount[i].goal = 1;

		appTestModeVersion[i] = RSA_TEST_MODE_VERSION_1_3_1;
	}
}

void RTI_EnterTestMode(void)
{
	DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
}

/**************************************************************************************************
 *
 * @fn      RTI_TestModeSendDataCnf_Ndata
 *
 **************************************************************************************************/

uint8 RTI_TestModeSendDataCnf_Ndata(rStatus_t status)
{
	uint8 res = TRUE;

	LOG_DEBUG("RTI_TestModeSendDataCnf_Ndata, status: 0x%.2X\n", status);

	return res;
}

/**************************************************************************************************
 *
 * @fn      RTI_TestModeSendDataCnf
 *
 **************************************************************************************************/

void RTI_TestModeSendDataCnf(rStatus_t status)
{
	LOG_DEBUG("RTI_TestModeSendDataCnf, status: 0x%.2X\n", status);

	if (appTestModeState == APP_TEST_MODE_STATE_LAUNCH)
	{
		if (status != RTI_SUCCESS)
		{
			LOG_WARN("[RTI TEST] Failed to send test parameters (0x%.2X)\n", status);
			if (status == RTI_ERROR_MAC_NO_ACK)
			{
				LOG_INFO("Are you sure the Controller is in test mode?\n");
			}
			else if (status == RTI_ERROR_INVALID_PARAMETER)
			{
				LOG_ERROR("You selected an invalid destination index. Check the index of the Controller you want to test\n");
			}
			// Allow test to be aborted.
			appTestModeReceivedPacketCnt[testModeDest.idxArr[testModeDest.currentIdx]] = 0;
			// Test aborted
			LOG_WARN("--- Test aborted ---\n");
			// Return to INIT state
			appTestModeState = APP_TEST_MODE_STATE_INIT;
		}
	}
}

void TestappOpenReportFile(uint8 srcIndex)
{
	char suffix[256] = "RTI_test_report_";
	time_t currentTime;
	struct tm *local;
	char filename1[256];
	FILE* fp;

	//First Create filename
	time(&currentTime);
	local = localtime(&currentTime);
	//filename1 =  ctime(&currentTime);
	strftime(filename1, 255, "Test_report_%Y_%b_%d_%Hh%M_%S", local);
	sprintf(suffix, "_idx%d", srcIndex);
	strcat(filename1, suffix);

	//Open the file
	fp = fopen(filename1, "w");

	//Set the file descriptor to the global variable
	if (fp == NULL) {
		LOG_ERROR("can't create test file");
		exit(-1);
	}

	appTestModeReceivedFilep[srcIndex] = fp;

	//Write Test condition to the file
	fprintf(fp, "%s \n", filename1);
	fprintf(fp, "------------------------------------------------------\n");
	fprintf(fp, "- Current Test Mode Configuration:\n");
	fprintf(fp, "- \t Current destination index: 0x%.2X\n", srcIndex);
	fprintf(fp, "------------------------------------------------------\n");
	fprintf(fp, "- \t Start condition: \t %s [0x%.2X]\n",
			testMode_startCondition_list[appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].startCondition],
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].startCondition);
	fprintf(fp, "- \t Tx Options: \t \t \t 0x%.2X\n",
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txOptions);

	{
		fprintf(fp, "- \t Tx Power: \t \t \t%s\n",
				testMode_txPower_list[(-(appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower
						- 7))]);
	}
	fprintf(fp, "- \t Payload size: \t \t \t %.2d \tbytes\n",
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].userDataSize);
	fprintf(fp, "- \t Number of packets: \t  %4s \040%.5d \n", "",
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].numPackets);
	fprintf(fp, "- \t Max delay between packets:  %2s %.5d \tms\n", "",
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].maxBackoffDuration);

	fprintf(fp, "------------------------------------------------------\n");

//	fclose(fp);

}

/**************************************************************************************************
 *
 * @fn      RTI_TestModeReceiveDataInd
 *
 * @brief   RTI receive data indication callback asynchronously initiated by
 *          another node. The client is expected to complete this function.
 *
 * input parameters
 *
 * @param   srcIndex:  Pairing table index.
 * @param   profileId: Profile identifier.
 * @param   vendorId:  Vendor identifier.
 * @param   rxLQI:     Link Quality Indication.
 * @param   rxFlags:   Receive flags.
 * @param   len:       Number of bytes to send.
 * @param   *pData:    Pointer to data to be sent.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void RTI_TestModeReceiveDataInd(uint8 srcIndex, uint8 profileId,
		uint16 vendorId, uint8 rxLQI, uint8 rxFlags, uint8 len, uint8 *pData)
{
//	int error = FALSE;

	if (
			//				(vendorId == RTI_VENDOR_TEXAS_INSTRUMENTS) &&
			(len > 1))
	{
		if (pData[0] == RTI_PROTOCOL_TEST)
		{
			if (pData[1] == RTI_CMD_TEST_DATA)
			{
				//static float averageLatency = 0;
				uint16 latencyTime = 0;
				static struct timeval currentTime;
				long int diffPrev;
				uint16 pckSeqNumber = 0;
				static uint32 pckErrors = 0;
				static uint16 lastPckSeqNumber = 0;
				if (testModeDest.size == 1)
				{
					//pckSeqNumber = pData[3];
					//pckSeqNumber += (uint16)((pData[4] << 8) & 0xFF00);
					if (len > 3)
					{
						pckSeqNumber = (uint16) (((pData[3] << 8) & 0xFF00) + pData[2]);
					}
					else
					{
						pckSeqNumber = 0;
					}

					if (appTestModeReceivedPacketCnt[srcIndex] == 0)
					{
						// First Packet of the test, reset stats
						pckErrors = 0;
						gettimeofday(&appTestModeReceivedStartTime[srcIndex], NULL);
						appTestModeReceivedcumulativePayload[srcIndex] = 0;
						appTestModeReceivedThroughput[srcIndex] = 0;
						appTestModeReceivedcumulativeLatency[srcIndex] = 0;
						appTestModeReceivedaverageLatency[srcIndex] = 0;
						appTestModeReceivedmaxLatency[srcIndex] = 0;
						appTestModeReceivedminLatency[srcIndex] = 1000;
						appTestModeReceivedcumulativeRSSI[srcIndex] = 0;
						appTestModeReceivedaverageRSSI[srcIndex] = 0;
						appTestModeReceivedmaxRSSI[srcIndex] = -128;
						appTestModeReceivedminRSSI[srcIndex] = 0;
						if (appTestFileSave)
						{
							if (appTestModeReceivedFilep[srcIndex])
							{
								fclose(appTestModeReceivedFilep[srcIndex]);
							}
							TestappOpenReportFile(srcIndex);
						}
						//LOG_INFO("First test packet @ %d \n", appTestModeReceivedStartTime[srcIndex].tv_sec);
					}
					else if (pckSeqNumber && ((lastPckSeqNumber + 1) != pckSeqNumber))
					{
						pckErrors += (pckSeqNumber - (lastPckSeqNumber + 1));
						LOG_INFO("\npckErrors: %.4d (last seqNum: %.4d)\n", pckErrors, lastPckSeqNumber);
					}
					lastPckSeqNumber = pckSeqNumber;
				}
				if (len > 5)
				{
					latencyTime = (uint16) (((pData[5] << 8) & 0xFF00) + pData[4]);
				}
				else
				{
					latencyTime = 0;
				}

				if (LQI_TO_DBM_CONVERSION(rxLQI) < appTestModeReceivedminRSSI[srcIndex])
				{
					appTestModeReceivedminRSSI[srcIndex] = LQI_TO_DBM_CONVERSION(rxLQI);
				}
				if (LQI_TO_DBM_CONVERSION(rxLQI) > appTestModeReceivedmaxRSSI[srcIndex])
				{
					appTestModeReceivedmaxRSSI[srcIndex] = LQI_TO_DBM_CONVERSION(rxLQI);
				}

				appTestModeReceivedPacketCnt[srcIndex]++;
				appTestModeReceivedcumulativePayload[srcIndex] += len;
				gettimeofday(&currentTime, NULL);
				if (currentTime.tv_usec >= appTestModeReceivedStartTime[srcIndex].tv_usec)
				{
					diffPrev = currentTime.tv_usec - appTestModeReceivedStartTime[srcIndex].tv_usec;
					diffPrev += ((currentTime.tv_sec - appTestModeReceivedStartTime[srcIndex].tv_sec) * 1000000);
				}
				else
				{
					diffPrev = (currentTime.tv_usec + 1000000) - appTestModeReceivedStartTime[srcIndex].tv_usec;
					diffPrev += ((currentTime.tv_sec - appTestModeReceivedStartTime[srcIndex].tv_sec - 1) * 1000000);
				}
				appTestModeReceivedThroughput[srcIndex] = (float) ((float) (appTestModeReceivedcumulativePayload[srcIndex]) * 8 * 1000 * 1000 / diffPrev);
				//int i;

				if (latencyTime)
				{
					if (latencyTime < appTestModeReceivedminLatency[srcIndex])
					{
						appTestModeReceivedminLatency[srcIndex] = latencyTime;
					}
					if (latencyTime > appTestModeReceivedmaxLatency[srcIndex])
					{
						appTestModeReceivedmaxLatency[srcIndex] = latencyTime;
					}
					appTestModeReceivedcumulativeLatency[srcIndex] += latencyTime;
					if (appTestModeReceivedPacketCnt[srcIndex] > 1)
					{
						appTestModeReceivedaverageLatency[srcIndex] = (float) ((float) appTestModeReceivedcumulativeLatency[srcIndex]) / (appTestModeReceivedPacketCnt[srcIndex] - 1);
					}
				}

				appTestModeReceivedcumulativeRSSI[srcIndex] += LQI_TO_DBM_CONVERSION(rxLQI);
				if (appTestModeReceivedPacketCnt[srcIndex] > 1)
				{
					appTestModeReceivedaverageRSSI[srcIndex] = (float) ((float) appTestModeReceivedcumulativeRSSI[srcIndex]) / (appTestModeReceivedPacketCnt[srcIndex] - 1);
				}

				LOG_INFO("Received test packet: [Idx 0x%.2X = %.5d (seq: %d, RSSI: %ddBm (ave:%.2fdBm); during %.2fms, Prev Packet Latency:%dms, ave Latency: %.2fms, min: %dms, max: %dms]\r",
							//								testModeDest.idxArr[i],
							srcIndex,
							appTestModeReceivedPacketCnt[testModeDest.idxArr[0]],
							pckSeqNumber, LQI_TO_DBM_CONVERSION(rxLQI),
							appTestModeReceivedaverageRSSI[srcIndex],
							(float) diffPrev / 1000,
							latencyTime,
							appTestModeReceivedaverageLatency[srcIndex],
							appTestModeReceivedminLatency[srcIndex],
							appTestModeReceivedmaxLatency[srcIndex]);

				//Save Data to file;
				if (appTestFileSave
						&& (appTestModeReceivedFilep[srcIndex])) {
					uint8 i;
					fprintf(appTestModeReceivedFilep[srcIndex],
							"%d;%d;%d;%d;", (int) diffPrev,
							appTestModeReceivedPacketCnt[srcIndex], LQI_TO_DBM_CONVERSION(rxLQI),
							len);

					for (i = 0; i < len; i++)
						fprintf(appTestModeReceivedFilep[srcIndex], "%d;",
								pData[i]);
					fprintf(appTestModeReceivedFilep[srcIndex], "\n");
				}
			}
			else if (pData[1] == RTI_CMD_TEST_REPORT)
			{
				LOG_INFO("\n");
				appTestModeProcessReport(srcIndex, pData, len);
			}
			else if (pData[1] == RTI_CMD_TEST_PARAMETERS)
			{
				LOG_WARN("Received unsupported test data\n");
			}
		}
		else
		{
			LOG_WARN("1 Received %d bytes of other than test data in TestMode state\n", len);
		}
	}
	else
	{
		LOG_WARN("2 Received %d bytes of other than test data in TestMode state\n", len);
	}

//	//For Debug Purpose!!!
//	if (error)
//		exit(-1);
}

/**************************************************************************************************
 * @fn          appTestModeProcessReport
 *
 * @brief       This function parses the incoming report, and calls a function to display the info
 *
 * input parameters
 *
 * @param srcIdx - Index of source for this report
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeProcessReport(uint8 srcIdx, uint8* pData, uint8 len)
{
	// Parse the incoming report and do the math for presentation
	float PER = 0;
	uint8 i;
	int txPower;
	uint32 numOfSentPackets = 0;

	// First verify that the length of the report is as expected,
	// to avoid segmentation fault
	if (len == 63)
	{
		// TODO: For cycled tx power read out tx power index from the last bin
		txPower = appTestModeParameters[srcIdx].txPower;
		// Calculate the number of sent packets:
		for (i = 0; i < 25; i += 3)
		{
			appTestModeReceivedBinAcc[srcIdx][i] += ((rrtTestReportRsp_t *) (&pData[2]))->latency.bin[i];
			numOfSentPackets += ((rrtTestReportRsp_t *) (&pData[2]))->latency.bin[i];
		}
		// Avoid division by zero.
		if (appTestModeReceivedPacketCnt[srcIdx] > 0)
		{
			PER = (1 - (numOfSentPackets / appTestModeReceivedPacketCnt[srcIdx])) * 100;
		}
		else
		{
			PER = -1;
		}
		// The 4th last bin contains the number of packets that were not sent. The following
		// test makes sure that the number of packets processed by the Controller
		// is the same as that which was set up in the test.
		if ((numOfSentPackets + ((rrtTestReportRsp_t *) (&pData[2]))->latency.bin[25]) != appTestModeReceivedPacketCnt[srcIdx])
		{
			// The number of packet does not add up and we cannot trust PER
			PER = -1;
		}

		// Present Report
		DispTestModeReport(srcIdx, appTestModeRunCount[srcIdx].currentCount,
				((rrtTestReportRsp_t *) (&pData[2]))->latency.bin, txPower, PER,
				numOfSentPackets, appTestModeReceivedPacketCnt[srcIdx],
				appTestModeVersion[srcIdx]);
	}
	else
	{
		LOG_ERROR("Invalid Report Length (%d != 63)\n", len);
	}

	printf("\n**************************************************\n");
	printf("*\t    Current test counters\n");
	printf("*\t\t      run count: \t\t\t\t %d (/%d)\n", appTestModeRunCount[srcIdx].currentCount, appTestModeRunCount[srcIdx].goal);
	printf("*\t\t      received packet count: \t\t\t %d \n", appTestModeReceivedPacketCnt[srcIdx]);
	printf("*\t\t      received packet count accumulated: \t %d \n", appTestModeReceivedPacketCntAcc[srcIdx]);
//	printf("*\t\t      received packet bin[0] accumulated: \t %d \n",appTestModeReceivedBinAcc[srcIdx][0]);
	printf("*\t\t      Payload Accumulated: \t %d \n", appTestModeReceivedcumulativePayload[srcIdx]);
//	printf("*\t\t      Latency Accumulated: \t %d \n",appTestModeReceivedcumulativeLatency[srcIdx];
	printf("*\t\t      Average Latency \t %.2fms \n", appTestModeReceivedaverageLatency[srcIdx]);
	printf("*\t\t      Min Latency: \t %dms \n", appTestModeReceivedminLatency[srcIdx]);
	printf("*\t\t      Max Latency: \t %dms \n", appTestModeReceivedmaxLatency[srcIdx]);
	printf("*\t\t      Average RSSI \t %.2fdBm \n", appTestModeReceivedaverageRSSI[srcIdx]);
	printf("*\t\t      Min RSSI: \t %ddBm \n", appTestModeReceivedminRSSI[srcIdx]);
	printf("*\t\t      Max RSSI: \t %ddBm \n", appTestModeReceivedmaxRSSI[srcIdx]);
	printf("*\t\t      Throughput: \t %.2fbps \n", appTestModeReceivedThroughput[srcIdx]);

	if (appTestFileSave && (appTestModeReceivedFilep[srcIdx]))
	{
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t    Current test counters\n");
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      run count: \t\t\t\t %d (/%d)\n",
				appTestModeRunCount[srcIdx].currentCount,
				appTestModeRunCount[srcIdx].goal);
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      received packet count: \t\t\t %d \n", appTestModeReceivedPacketCnt[srcIdx]);
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      received packet count accumulated: \t %d \n", appTestModeReceivedPacketCntAcc[srcIdx]);
//		fprintf( appTestModeReceivedFilep[srcIdx],"*\t\t      received packet bin[0] accumulated: \t %d \n",appTestModeReceivedBinAcc[srcIdx][0]);
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Payload Accumulated: \t %d \n", appTestModeReceivedcumulativePayload[srcIdx]);
//		fprintf( appTestModeReceivedFilep[srcIdx],"*\t\t      Latency Accumulated: \t %d \n",appTestModeReceivedcumulativeLatency[srcIdx];
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Average Latency \t %.2fms \n", appTestModeReceivedaverageLatency[srcIdx]);
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Min Latency: \t %dms \n", 				appTestModeReceivedminLatency[srcIdx]);
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Max Latency: \t %dms \n", appTestModeReceivedmaxLatency[srcIdx]);
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Average RSSI \t %.2fdBm \n", appTestModeReceivedaverageRSSI[srcIdx]);
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Min RSSI: \t %ddBm \n", appTestModeReceivedminRSSI[srcIdx]);
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Max RSSI: \t %ddBm \n", appTestModeReceivedmaxRSSI[srcIdx]);
		fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Throughput: \t %.2fbps \n", appTestModeReceivedThroughput[srcIdx]);
	}

	// Add current count to the accumulated counter
	appTestModeReceivedPacketCntAcc[srcIdx] +=
			appTestModeReceivedPacketCnt[srcIdx];

	// Test Report processed; clear buffers
	appTestModeReceivedPacketCnt[srcIdx] = 0;

	// Increment current count
	appTestModeRunCount[srcIdx].currentCount++;

	// If the wanted number of runs is reached, then return to Init, otherwise
	// prepare for next run
	if ((appTestModeRunCount[srcIdx].currentCount >= appTestModeRunCount[srcIdx].goal)
			&& (appTestModeRunCount[srcIdx].goal > 1))
	{
		// Process accumulated test
		// TODO: For cycled tx power read out tx power index from the last bin
		txPower = appTestModeParameters[srcIdx].txPower;

		numOfSentPackets = 0;

		// Calculate the number of sent packets:
		for (i = 0; i < 25; i += 3)
		{
			numOfSentPackets += appTestModeReceivedBinAcc[srcIdx][i];
		}
		PER = (1 - (numOfSentPackets / appTestModeReceivedPacketCntAcc[srcIdx])) * 100;
		// The 4th last bin contains the number of packets that were not sent. The following
		// test makes sure that the number of packets processed by the Controller
		// is the same as that which was set up in the test.
		if ((numOfSentPackets + appTestModeReceivedBinAcc[srcIdx][25])
				!= appTestModeReceivedPacketCntAcc[srcIdx]) {
			// The number of packet does not add up and we cannot trust PER
			PER = -1;
		}
		// Present Report
		DispTestModeReport(srcIdx, -1, appTestModeReceivedBinAcc[srcIdx],
				txPower, PER, numOfSentPackets,
				appTestModeReceivedPacketCntAcc[srcIdx],
				appTestModeVersion[srcIdx]);

		printf("\n**************************************************\n");
		printf("*\t    Current test counters\n");
		printf("*\t\t      run count: \t\t\t\t %d (/%d)\n", appTestModeRunCount[srcIdx].currentCount, appTestModeRunCount[srcIdx].goal);
		printf("*\t\t      received packet count: \t\t\t %d \n", appTestModeReceivedPacketCnt[srcIdx]);
		printf("*\t\t      received packet count accumulated: \t %d \n", appTestModeReceivedPacketCntAcc[srcIdx]);
		//	printf("*\t\t      received packet bin[0] accumulated: \t %d \n",appTestModeReceivedBinAcc[srcIdx][0]);
		printf("*\t\t      Payload Accumulated: \t %d \n", appTestModeReceivedcumulativePayload[srcIdx]);
		//	printf("*\t\t      Latency Accumulated: \t %d \n",appTestModeReceivedcumulativeLatency[srcIdx];
		printf("*\t\t      Average Latency \t %.2fms \n", appTestModeReceivedaverageLatency[srcIdx]);
		printf("*\t\t      Min Latency: \t %dms \n", appTestModeReceivedminLatency[srcIdx]);
		printf("*\t\t      Max Latency: \t %dms \n", appTestModeReceivedmaxLatency[srcIdx]);
		printf("*\t\t      Average RSSI \t %.2fdBm \n", appTestModeReceivedaverageRSSI[srcIdx]);
		printf("*\t\t      Min RSSI: \t %ddBm \n", appTestModeReceivedminRSSI[srcIdx]);
		printf("*\t\t      Max RSSI: \t %ddBm \n", appTestModeReceivedmaxRSSI[srcIdx]);
		printf("*\t\t      Throughput: \t %.2fbps \n", appTestModeReceivedThroughput[srcIdx]);
		if (appTestFileSave && (appTestModeReceivedFilep[srcIdx]))
		{
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t    Current test counters\n");
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      run count: \t\t\t\t %d (/%d)\n", appTestModeRunCount[srcIdx].currentCount, appTestModeRunCount[srcIdx].goal);
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      received packet count: \t\t\t %d \n", appTestModeReceivedPacketCnt[srcIdx]);
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      received packet count accumulated: \t %d \n", appTestModeReceivedPacketCntAcc[srcIdx]);
			//		fprintf( appTestModeReceivedFilep[srcIdx],"*\t\t      received packet bin[0] accumulated: \t %d \n",appTestModeReceivedBinAcc[srcIdx][0]);
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Payload Accumulated: \t %d \n", appTestModeReceivedcumulativePayload[srcIdx]);
			//		fprintf( appTestModeReceivedFilep[srcIdx],"*\t\t      Latency Accumulated: \t %d \n",appTestModeReceivedcumulativeLatency[srcIdx];
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Average Latency \t %.2fms \n", appTestModeReceivedaverageLatency[srcIdx]);
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Min Latency: \t %dms \n", appTestModeReceivedminLatency[srcIdx]);
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Max Latency: \t %dms \n", appTestModeReceivedmaxLatency[srcIdx]);
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Average RSSI \t %.2fdBm \n", appTestModeReceivedaverageRSSI[srcIdx]);
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Min RSSI: \t %ddBm \n", appTestModeReceivedminRSSI[srcIdx]);
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Max RSSI: \t %ddBm \n", appTestModeReceivedmaxRSSI[srcIdx]);
			fprintf(appTestModeReceivedFilep[srcIdx], "*\t\t      Throughput: \t %.2fbps \n", appTestModeReceivedThroughput[srcIdx]);
		}

		// If there are more destination indices to send to, do it now
		uint8 cycledIndex = testModeDest.currentIdx;
		testModeDest.currentIdx++;
		if (testModeDest.currentIdx >= testModeDest.size)
		{
			testModeDest.currentIdx = 0;
		}
		while (appTestModeRunCount[testModeDest.idxArr[testModeDest.currentIdx]].currentCount
				>= appTestModeRunCount[testModeDest.idxArr[testModeDest.currentIdx]].goal)
		{
			testModeDest.currentIdx++;
			if (testModeDest.currentIdx >= testModeDest.size)
			{
				testModeDest.currentIdx = 0;
			}
			if (cycledIndex == testModeDest.currentIdx)
			{
				break;
			}
		}

		if (cycledIndex != testModeDest.currentIdx)
		{
			// Continue with the other indices
			appTestModeSendTestSettings(testModeDest.idxArr[testModeDest.currentIdx]);
			LOG_INFO("Processed 0x%.2X, going to 0x%.2X\n", srcIdx, testModeDest.idxArr[testModeDest.currentIdx]);
		}
		else
		{
			for (testModeDest.currentIdx = 0; testModeDest.currentIdx < testModeDest.size; testModeDest.currentIdx++)
			{
				// Clear current count
				appTestModeRunCount[testModeDest.idxArr[testModeDest.currentIdx]].currentCount = 0;
				// Clear accumulated buffers
				appTestModeReceivedPacketCntAcc[testModeDest.idxArr[testModeDest.currentIdx]] = 0;
				memset(
						appTestModeReceivedBinAcc[testModeDest.idxArr[testModeDest.currentIdx]],
						0,
						sizeof(appTestModeReceivedBinAcc[testModeDest.idxArr[testModeDest.currentIdx]]));
			}
			testModeDest.currentIdx = 0;

			// Return to INIT state
			appTestModeState = APP_TEST_MODE_STATE_INIT;
			LOG_INFO(
					"Finished with cycledIndex 0x%.2X, going to 0x%.2X (both index of index)\n",
					cycledIndex, testModeDest.currentIdx);
		}
	}
	else if (appTestModeRunCount[srcIdx].goal > 1)
	{
		// If there are more destination indices to send to, do it now
		uint8 cycledIndex = testModeDest.currentIdx;
		testModeDest.currentIdx++;
		if (testModeDest.currentIdx >= testModeDest.size)
		{
			testModeDest.currentIdx = 0;
		}
		while (appTestModeRunCount[testModeDest.idxArr[testModeDest.currentIdx]].currentCount
				>= appTestModeRunCount[testModeDest.idxArr[testModeDest.currentIdx]].goal)
		{
			testModeDest.currentIdx++;
			if (testModeDest.currentIdx >= testModeDest.size)
			{
				testModeDest.currentIdx = 0;
			}
			if (cycledIndex == testModeDest.currentIdx)
			{
				break;
			}
		}

		LOG_INFO("Processed 0x%.2X, going to 0x%.2X\n", srcIdx, testModeDest.idxArr[testModeDest.currentIdx]);
		appTestModeSendTestSettings(testModeDest.idxArr[testModeDest.currentIdx]);

//		// Send test settings to trigger next test. The settings are already prepared
//		appTestModeSendTestSettings(srcIdx);
	}
	else
	{
		// We are not cycling through a number of tests,
		// hence clear remaining buffers, and return to init state

		// Clear current count
		appTestModeRunCount[srcIdx].currentCount = 0;
		// Clear accumulated buffers
		appTestModeReceivedPacketCntAcc[srcIdx] = 0;
		memset(appTestModeReceivedBinAcc[srcIdx], 0,
				sizeof(appTestModeReceivedBinAcc[srcIdx]));
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		if (appTestFileSave)
		{
			if (appTestModeReceivedFilep[srcIdx])
			{
				fclose(appTestModeReceivedFilep[srcIdx]);
			}
			appTestModeReceivedFilep[srcIdx] = NULL;
		}
	}
}

/**************************************************************************************************
 * @fn          appTestModeProcessKey
 *
 * @brief       This function executes the Test Mode
 *
 * input parameters
 *
 * @param key - key from key scanner to be processed
 * @param strIn - key from key scanner to be processed
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeProcessKey(char* strIn) {
	int i;
	if (appTestModeState == APP_TEST_MODE_STATE_INIT)
	{
		switch (strIn[0])
		{
		case 'r':
			if (rtiTestmodeCb != NULL)
			{
				rtiTestmodeCb();
			}
			return;
		case 'm':
			// Display Test Mode menu
			DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
			return;
		case 's':
			testModeDest.currentIdx = 0;
			LOG_INFO("Test starting; sending first set of test parameters to 0x%.2X\n", testModeDest.idxArr[testModeDest.currentIdx]);
			// Go to Launch state to send test parameters
			appTestModeState = APP_TEST_MODE_STATE_LAUNCH;
			// We're Target so send test settings to Controller
			appTestModeSendTestSettings(testModeDest.idxArr[testModeDest.currentIdx]);
			break;
		case '1':
			// Go to configuration state (of destination index) to allow setting start condition
			appTestModeState = APP_TEST_MODE_STATE_CFG_DEST_IDX;
			DispTestModeCFGDestinationIndexMenuInit();
			break;
		case '2':
			// Go to configuration state (of start condition) to allow setting start condition
			appTestModeState = APP_TEST_MODE_STATE_CFG_START_CONDITION;
			DispTestModeCFGStartConditionMenuInit();
			break;
		case '3':
			// Go to configuration state (of Tx power) to allow setting start condition
			appTestModeState = APP_TEST_MODE_STATE_CFG_TX_POWER;
			DispTestModeCFGTxPowerMenuInit();
			break;
		case '4':
			// Go to configuration state (of payload size) to allow setting start condition
			appTestModeState = APP_TEST_MODE_STATE_CFG_PAYLOAD;
			DispTestModeCFGPayloadSizeMenuInit();
			break;
		case '5':
			// Go to configuration state (of number of packet) to allow setting start condition
			appTestModeState = APP_TEST_MODE_STATE_CFG_NUM_OF_PACKETS;
			DispTestModeCFGNumOfPacketsMenuInit();
			break;
		case '6':
			// Go to configuration state (of maximum random additional delay between packets) to allow setting start condition
			appTestModeState = APP_TEST_MODE_STATE_CFG_MAX_DELAY;
			DispTestModeCFGMaxAdditionalDelayMenuInit(
					appTestModeVersion[testModeDest.idxArr[testModeDest.currentIdx]]);
			break;
		case '7':
			// Go to number of run state
			appTestModeState = APP_TEST_MODE_STATE_SET_NUM_OF_RUNS;
			DispTestModeSetNumOfRunsMenuInit();
			break;
		case '8':
			// Load Predefined Throughput Test
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].startCondition = RSA_TEST_START_IMMEDIATE;
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower = 7;
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].userDataSize = 100;
			appTestModeVersion[testModeDest.idxArr[testModeDest.currentIdx]] = RSA_TEST_MODE_VERSION_1_3_1;
			for (i = 0; i < testModeDest.size; i++)
			{
				appTestModeRunCount[testModeDest.idxArr[i]].goal = 1;
			}
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].numPackets = 1000;
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].maxBackoffDuration = 0;

			for (i = 0; i < testModeDest.size; i++)
			{
				DispTestModeCurrentCfg(
						appTestModeParameters[testModeDest.idxArr[i]],
						testModeDest.idxArr[i],
						appTestModeVersion[testModeDest.idxArr[i]],
						appTestModeRunCount[testModeDest.idxArr[i]].goal,
						appTestFileSave);
			}
			break;
		case '9':
			// Load Predefined Latency Test
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].startCondition = RSA_TEST_START_IMMEDIATE;
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower = 7;
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].userDataSize = 6;
			appTestModeVersion[testModeDest.idxArr[testModeDest.currentIdx]] = RSA_TEST_MODE_VERSION_1_3_1;
			for (i = 0; i < testModeDest.size; i++)
			{
				appTestModeRunCount[testModeDest.idxArr[i]].goal = 1;
			}
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].numPackets = 1000;
			appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].maxBackoffDuration = 50;

			for (i = 0; i < testModeDest.size; i++)
			{
				DispTestModeCurrentCfg(
						appTestModeParameters[testModeDest.idxArr[i]],
						testModeDest.idxArr[i],
						appTestModeVersion[testModeDest.idxArr[i]],
						appTestModeRunCount[testModeDest.idxArr[i]].goal,
						appTestFileSave);
			}

			appTestModeParameters[i].txOptions = RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_VENDOR_SPECIFIC | RTI_TX_OPTION_SECURITY;
			break;
		case 'o':
			// Change Tx options
			LOG_INFO("Test saving behaviour toggle.\n");
			appTestModeState = APP_TEST_MODE_STATE_TX_OPTIONS;
			DispTestModeCFGTxOptions();
			break;
		case 'f':
			// Toggle Test Saving  to file
			LOG_INFO("Test saving behaviour toggle.\n");
			appTestFileSave = !appTestFileSave;
			break;
		case 'l':
			for (i = 0; i < testModeDest.size; i++)
			{
				DispTestModeCurrentCfg(
						appTestModeParameters[testModeDest.idxArr[i]],
						testModeDest.idxArr[i],
						appTestModeVersion[testModeDest.idxArr[i]],
						appTestModeRunCount[testModeDest.idxArr[i]].goal,
						appTestFileSave);
			}
			break;
		default:
			LOG_INFO("unknown command %c (0x%.2X) \n", strIn[0], strIn[0]);
			DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
			return;
		}
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_LAUNCH)
	{
		if (strIn[0] != 's')
		{
			// Allow test to be aborted.
			appTestModeReceivedPacketCnt[testModeDest.idxArr[testModeDest.currentIdx]] = 0;
			// Test aborted
			LOG_INFO("--- Test aborted ---\n");
			// Return to INIT state
			appTestModeState = APP_TEST_MODE_STATE_INIT;
			// Display Test Mode menu
			DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		}
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_RUNNING)
	{
		if (strIn[0] != 's')
		{
			// Allow test to be aborted.
			appTestModeReceivedPacketCnt[testModeDest.idxArr[testModeDest.currentIdx]] = 0;
			// Test aborted
			LOG_INFO("--- Test aborted ---\n");
			// Return to INIT state
			appTestModeState = APP_TEST_MODE_STATE_INIT;
			// Display Test Mode menu
			DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		}
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_CFG_DEST_IDX)
	{
		appTestModeProcessKeyForCFGDestinationIndex(strIn);
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_CFG_START_CONDITION)
	{
		appTestModeProcessKeyForCFGStartCondition(strIn[0]);
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_TX_OPTIONS)
	{
		appTestModeProcessKeyForTxOptions(strIn);
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_CFG_TX_POWER)
	{
		appTestModeProcessKeyForCFGTxPower(strIn[0]);
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_CFG_PAYLOAD)
	{
		appTestModeProcessKeyForCFGPayload(strIn);
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_CFG_MAX_DELAY)
	{
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].maxBackoffDuration =
				atoi(strIn);
		LOG_INFO("You set random maximum delay to: \t %d ms\n",
				appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].maxBackoffDuration);
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_SET_NUM_OF_RUNS)
	{
		appTestModeProcessKeyForNumOfRuns(strIn);
	}
	else if (appTestModeState == APP_TEST_MODE_STATE_CFG_NUM_OF_PACKETS)
	{
		appTestModeProcessKeyForNumOfPackets(strIn);
	}
}

/**************************************************************************************************
 * @fn          appTestModeProcessKeyForCFGDestinationIndex
 *
 * @brief       This function executes the Test Mode code for the
 * 				APP_TEST_MODE_STATE_CFG_DEST_IDX state
 *
 * input parameters
 *
 * @param keyStr - array of destination indices to test against
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeProcessKeyForCFGDestinationIndex(char* keyStr) {
	int i = 0, j = 0, tmp;
	char* pStr;
	// use strtok to split string and process each individual Profile ID
	// Get first token
	pStr = strtok(keyStr, " ,:;-|");
	while (pStr != NULL)
	{
		// Convert string to int
		tmp = atoi(pStr);
		// Add it to the list if it's a valid pairing reference
		if ((tmp < 0xFF) && (tmp >= 0))
		{
			// Fill test destination index array
			testModeDest.idxArr[j] = tmp;
			if (j == 0)
				testModeDest.currentIdx = 0;
			j++;
		}
		else
		{
			LOG_WARN("Invalid pairing reference %s (0x%.2X) \n", pStr, tmp);
		}
		i++;
		if (i > RCN_CAP_PAIR_TABLE_SIZE)
			break;
		// Now get next token
		pStr = strtok(NULL, " ,:;-|");
	}

	testModeDest.size = j;
	// Return to INIT state
	appTestModeState = APP_TEST_MODE_STATE_INIT;
	// Display Test Mode menu
	DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
}

/**************************************************************************************************
 * @fn          appTestModeProcessKeyForCFGStartCondition
 *
 * @brief       This function executes the Test Mode code for the
 * 				APP_TEST_MODE_STATE_CFG_START_CONDITION state
 *
 * input parameters
 *
 * @param key - key from key scanner to be processed
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeProcessKeyForCFGStartCondition(char key)
{
	switch (key)
	{
	case '1':
		// Set start condition to IMMEDIATE
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].startCondition = RSA_TEST_START_IMMEDIATE;
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		break;
	case '2':
		// Set start condition to ON BUTTON
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].startCondition = RSA_TEST_START_ON_BUTTON;
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		break;
	case 'm':
		// Display Test Mode menu
		DispTestModeCFGStartConditionMenuInit();
		break;
	default:
		LOG_WARN("unknown command %c (0x%.2X) \n", key, key);
		DispTestModeCFGStartConditionMenuInit();
		return;
	}
}

/**************************************************************************************************
 * @fn          appTestModeProcessKeyForTxOptions
 *
 * @brief       This function executes the Test Mode code for the
 * 				APP_TEST_MODE_STATE_CFG_START_CONDITION state
 *
 * input parameters
 *
 * @param key - key from key scanner to be processed
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeProcessKeyForTxOptions(char* strIn)
{
	if ((atoi(strIn) < 256) && (atoi(strIn) > 0))
	{
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txOptions =
				atoi(strIn);
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
	}
	else
	{
		if (strIn[0] != 'm')
		{
			LOG_WARN("unknown command %c (0x%.2X) \n", strIn[0], strIn[0]);
		}
		DispTestModeCFGTxOptions();
	}
}
/**************************************************************************************************
 * @fn          appTestModeProcessKeyForCFGTxPower
 *
 * @brief       This function executes the Test Mode code for the
 * 				APP_TEST_MODE_STATE_CFG_TX_POWER state
 *
 * input parameters
 *
 * @param key - key from key scanner to be processed
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeProcessKeyForCFGTxPower(char key)
{
	switch (key)
	{
	case '1':
		// Set Tx power to 7dBm
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower = 7;
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		break;
	case '2':
		// Set Tx power to 4.5dBm
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower = 4;
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		break;
	case '3':
		// Set Tx power to 0dBm
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower = 0;
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		break;
	case '4':
		// Set Tx power to -5.9dBm
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower = -5;
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		break;
	case '5':
		// Set Tx power to -9.9dBm
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower = -10;
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		break;
	case '6':
		// Set Tx power to -14.9dBm
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower = -15;
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		break;
	case '7':
		// Set Tx power to -18.7dBm
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].txPower = -19;
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
		break;
	case 'm':
		// Display Test Mode menu
		DispTestModeCFGTxPowerMenuInit();
		break;
	default:
		LOG_WARN("unknown command %c (0x%.2X) \n", key, key);
		DispTestModeCFGTxPowerMenuInit();
		return;
	}
}

/**************************************************************************************************
 * @fn          appTestModeProcessKeyForCFGPayload
 *
 * @brief       This function executes the Test Mode code for the
 * 				APP_TEST_MODE_STATE_CFG_PAYLOAD state
 *
 * input parameters
 *
 * @param key - key from key scanner to be processed
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeProcessKeyForCFGPayload(char* strIn)
{
	if ((atoi(strIn) <= 96) && (atoi(strIn) > 2))
	{
		// Set payload size
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].userDataSize = atoi(strIn);
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
	}
	else
	{
		if (strIn[0] != 'm')
		{
			LOG_WARN("unknown command %c (0x%.2X) \n", strIn[0], strIn[0]);
		}
		DispTestModeCFGPayloadSizeMenuInit();
	}
}

/**************************************************************************************************
 * @fn          appTestModeProcessKeyForNumOfRuns
 *
 * @brief       This function executes the Test Mode code for the
 * 				APP_TEST_MODE_STATE_SET_NUM_OF_RUNS state
 *
 * input parameters
 *
 * @param key - key from key scanner to be processed
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeProcessKeyForNumOfRuns(char* strIn)
{
	int runs, i;
	runs = atoi(strIn);
	if (runs != 0)
	{
		// Set number of runs for all test indices
		for (i = 0; i < testModeDest.size; i++)
		{
			appTestModeRunCount[testModeDest.idxArr[i]].goal = runs;
			// If goal > 1 we change the start condition as well, in case it was set to button
			if (appTestModeRunCount[testModeDest.idxArr[i]].goal > 1)
			{
				if (appTestModeParameters[testModeDest.idxArr[i]].startCondition != RSA_TEST_START_IMMEDIATE)
				{
					//Warn the user of this change
					LOG_WARN("Start condition changed to: %s for index 0x%.2X\n",
							testMode_startCondition_list[RSA_TEST_START_IMMEDIATE],
							testModeDest.idxArr[i]);
					appTestModeParameters[testModeDest.idxArr[i]].startCondition =
							RSA_TEST_START_IMMEDIATE;
				}
			}
		}
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
	}
	else
	{
		switch (strIn[0])
		{
		case 'm':
			// Display Test Mode menu
			DispTestModeSetNumOfRunsMenuInit();
			break;
		default:
			LOG_WARN("unknown command %c (0x%.2X) \n", strIn[0], strIn[0]);
			DispTestModeSetNumOfRunsMenuInit();
			break;
		}
	}
}

/**************************************************************************************************
 * @fn          appTestModeProcessKeyForNumOfPackets
 *
 * @brief       This function executes the Test Mode code for the
 * 				APP_TEST_MODE_STATE_CFG_NUM_OF_PACKETS state
 *
 * input parameters
 *
 * @param key - key from key scanner to be processed
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeProcessKeyForNumOfPackets(char* strIn)
{
	if (atoi(strIn) != 0)
	{
		// Set number of packets to send during the test
		appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]].numPackets =
				atoi(strIn);
		// Return to INIT state
		appTestModeState = APP_TEST_MODE_STATE_INIT;
		// Display Test Mode menu
		DispTestModeMenuInit(testModeDest.idxArr[testModeDest.currentIdx]);
	}
	else
	{
		if (strIn[0] != 'm')
		{
			LOG_WARN("unknown command %c (0x%.2X) \n", strIn[0], strIn[0]);
		}

//		DispTestModeCFGNumOfPacketsMenuInit(testModeNumPacketsArray);
	}
}

/**************************************************************************************************
 * @fn          appTestModeSendTestSettings
 *
 * @brief       This function sends the test settings to the Controller
 *
 * input parameters
 *
 * @param destIdx		- Pairing index of the wanted Controller to test
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appTestModeSendTestSettings(uint8 destIdx)
{
	uint8 pData[sizeof(appTestModeParameters[destIdx]) + 2];
	uint8 profileId, vendorId;

	profileId = RTI_PROFILE_ZRC;
	vendorId = RTI_VENDOR_TEXAS_INSTRUMENTS;
	pData[0] = RTI_PROTOCOL_TEST;
	pData[1] = RTI_CMD_TEST_PARAMETERS;

	memcpy((uint8*) &pData[2], (uint8*) &appTestModeParameters[destIdx],
			sizeof(appTestModeParameters[testModeDest.idxArr[testModeDest.currentIdx]]));

	RTI_SendDataReq(destIdx, profileId, vendorId,
			appTestModeParameters[destIdx].txOptions, sizeof(pData), pData);
}

void DispTestModeMenuInit(uint8 destIdx)
{
	printf("------------------------------------------------------\n");
	printf("- Test Mode MENU:\n");
	printf("- \t Current destination index: 0x%.2X\n", destIdx);
	printf("------------------------------------------------------\n");
	printf("r- Return to Main Menu\n");
	printf("s- Start Test\n");
	printf("1- Configure Destination Index\n");
	printf("2- Configure Start Condition\n");
	printf("3- Configure Tx POWER\n");
	printf("4- Configure Payload Size\n");
	printf("5- Configure Number of Packets\n");
	printf("6- Configure Maximum Addition Delay\n");
	printf("7- Set Number of Automatic Test Runs\n");
	printf("8- Load Predifine Throughput Test\n");
	printf("9- Load Predifine Latency Test\n");
	printf("f- Toggle TEst file save (default off)\n");
	printf("l- List current configuration\n");
	printf("m- Show This Menu\n");
}

void DispTestModeCurrentCfg(rrtTestReq_t testReq, uint8 destIdx, uint8 appTestModeVersion, uint8 numOfCycles, bool boolFileSave )
{
	printf("------------------------------------------------------\n");
	printf("- Current Test Mode Configuration:\n");
	printf("- \t Current destination index: 0x%.2X\n", destIdx);
	printf("------------------------------------------------------\n");
	printf("- \t Start condition: \t %s [0x%.2X]\n",
			testMode_startCondition_list[testReq.startCondition],
			testReq.startCondition);
	printf("- \t Test Type: \t \t %s [0x%.2X]\n",
			testMode_testType_list[testReq.testType],
			testReq.testType);
	printf("- \t Tx Options: \t \t \t 0x%.2X\n",testReq.txOptions);
	printf("- \t Tx Power: \t \t \t%s\n",testMode_txPower_list[(-(testReq.txPower - 7))]);
	printf("- \t Payload size: \t \t \t %.2d \tbytes\n",testReq.userDataSize);
	printf("- \t Number of packets: \t  %4s \040%.5d \n","", testReq.numPackets);
	printf("- \t Number of tests to run:  %d\n", numOfCycles );
	printf("- \t Max Duration between packets:  %d\n", testReq.maxBackoffDuration );
	if(boolFileSave)
		printf("- \t All test will be saved to file, path: .\\RTI_Test_result\\date\\ \n");
	else
		printf("- \t test not saved to file\n");


	printf("------------------------------------------------------\n");
	printf("m- Display Main Menu\n");


}

void DispTestModeCFGDestinationIndexMenuInit ( void )
{
	printf("------------------------------------------------------\n");
	printf("Test Mode Configuration - Select Destination Index(s):\n");
	printf("- \tEnter as string of bytes separated by any of \n");
	printf("- \tthese delimiters: ' ' , ; : - |\n");
	printf("- \tA valid destination index is in the range 0 - 0x%.2X\n", RCN_CAP_PAIR_TABLE_SIZE - 1);
}

void DispTestModeCFGStartConditionMenuInit (void )
{
	printf("------------------------------------------------------\n");
	printf("Test Mode Configuration - Select Start Condition:\n");
	printf("1- Immediate\n");
	printf("2- On Button\n");
	printf("m- Show This Menu\n");
}

void DispTestModeCFGTxPowerMenuInit( void )
{
	printf("------------------------------------------------------\n");
	printf("Test Mode Configuration - Select Tx Power:\n");
	printf("1- \t %2s 7\tdBm\n", "");
	printf("2- \t %2s 4.5\tdBm\n", "");
	printf("3- \t %2s 0\tdBm\n", "");
	printf("4- \t %1s -5.9\tdBm\n", "");
	printf("5- \t %1s -9.9\tdBm\n", "");
	printf("6- \t-14.9\tdBm\n");
	printf("7- \t-18.7\tdBm\n");
	printf("m- Show This Menu\n");
}

void DispTestModeCFGPayloadSizeMenuInit ( void )
{
	printf("------------------------------------------------------\n");
	printf("Test Mode Configuration - Select Payload size:\n");
	printf("6 - 96 \n");
	printf("m- Show This Menu\n");
}

void DispTestModeCFGTxOptions ( void )
{
	printf("------------------------------------------------------\n");
	printf("Test Mode Configuration - Select Tx Options:\n");
	printf("Do it yourself!!! \n");
	printf("example: \n");
	printf("ACK: add/sub 0x4 (4)\n");
	printf("SEC: add/sub 0x8 (8)\n");
	printf("SIngle channel: add/sub (0x10) (16)\n");
	printf("Vendor Spec: add/sub  (0x40) (64)\n");
	printf("ack + sec + vendor = 0x4C (76) \n");
	printf("Nack + vendor + single = 0x50 (80) \n");

}

void DispTestModeSetNumOfRunsMenuInit ( void )
{
	printf("------------------------------------------------------\n");
	printf("Test Mode Configuration - Set Number of Runs:\n");
	printf("Choose a value < 256\n");
	printf("m- Show This Menu\n");
}

void DispTestModeCFGNumOfPacketsMenuInit ()
{
	uint8 i;
	printf("------------------------------------------------------\n");
	printf("Test Mode Configuration - Select number of packets:\n");
	for (i = 0; i < 10; i++)
	{
//		printf("%i - %.5d\n", i, nofPacketsArray[i]);
	}
	printf("m- Show This Menu\n");
}

void DispTestModeCFGMaxAdditionalDelayMenuInit (uint16 appTestModeVersion)
{
	// Can only take 1 byte as input, so this is rather useless for now.
	printf("------------------------------------------------------\n");
	printf("Test Mode Configuration - Set Maximum Additional Random Delay:\n");
	printf("Enter maximum random delay in number of milliseconds.\n");
	printf("Press any key to return\n");
}

static const char * const binStr[29] =
{
		"\t \040\0400ms - \04010ms: ",
		"\t \04010ms - \04020ms: ",
		"\t \04020ms - \04030ms: ",
		"\t \04030ms - \04040ms: ",
		"\t \04040ms - \04050ms: ",
		"\t \04050ms - \04060ms: ",
		"\t \04060ms - \04070ms: ",
		"\t \04070ms - \04080ms: ",
		"\t \04080ms - \04090ms: ",
		"\t \04090ms - 100ms: ",
		"\t 100ms - 150ms: ",
		"\t 150ms - 200ms: ",
		"\t 200ms - 250ms: ",
		"\t 250ms - 300ms: ",
		"\t 300ms - 350ms: ",
		"\t 350ms - 400ms: ",
		"\t 400ms - 450ms: ",
		"\t 450ms - 500ms: ",
		"\t 500ms - 550ms: ",
		"\t 550ms - 600ms: ",
		"\t 600ms - 650ms: ",
		"\t 650ms - 700ms: ",
		"\t 700ms - 750ms: ",
		"\t 750ms - 800ms: ",
		"\t 800ms - 850ms: ",
		"\t 850ms - 900ms: ",
		"\t 900ms - 950ms: ",
		"\t 950ms -1000ms: ",
		"\t %2s  %2s \040> 1000ms: "
};

void DispTestModeReport(uint8 srcIdx,
		int16 runNumber,
		void* bin,
		int8 txPower,
		float PER,
		uint32 numOfSentPackets,
		uint32 numReceivedPackets,
		uint8 appTestModeVersion)
{
	uint8 i;

	printf("------------------------------------------------------\n");
	printf("- Test Mode Report, Index 0x%.2X\n", srcIdx);
	printf("- \tRun #%.3d\n", runNumber + 1);
	printf("------------------------------------------------------\n");

	if (PER == -1)
	{
		if (runNumber == -1)
			printf("-\tPER: %.2f\%% (sent %d + not sent %d = %d != received %d)\n",
					PER,
					numOfSentPackets,
					((uint32*)bin)[28],
					numOfSentPackets + ((uint32*)bin)[25],
					numReceivedPackets);
		else
			printf("-\tPER: %.2f\%% (sent %d + not sent %d = %d != received %d)\n",
					PER,
					numOfSentPackets,
					((uint16*)bin)[28],
					numOfSentPackets + ((uint16*)bin)[25],
					numReceivedPackets);
	}
	else
	{
		printf("-\tPER: %.2f\%% (%d/%d)\n", PER, numOfSentPackets, numReceivedPackets);
	}
	printf("-\tTx Power: \t \t \t%s\n", testMode_txPower_list[(-(txPower - 7))]);
	for (i = 0; i < 28; i++)
	{
		if (runNumber == -1)
			printf("-%s %d\n", binStr[i],
					((uint32 *)bin)[i]);
		else
			printf("-%s %d\n", binStr[i],
					((uint16*)bin)[i]);
	}
}
