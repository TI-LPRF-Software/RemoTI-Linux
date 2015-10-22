/**************************************************************************************************
 Filename:       liveGraph.c

 Description:    Convenience functions to start LiveGraph and write data

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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "liveGraph.h"

#define MIN_WINDOW_SIZE		500
#define SIZE_HYSTERESIS		500	// Converts to 10 seconds @500Hz
#define FILE_SIZE_LIMIT		(7 * 1024 * 1024) // If moving files, instead of cropping set limit to 7MB (LiveGraph limit)

char dataFilePath[512];
int windowSize = MIN_WINDOW_SIZE;

void moveFileAndRestart(FILE *dataFile);
void cropFile(FILE *dataFile, int lineSize);

int runLiveGraph(int numOfDataSeries, int port, int channel, int window)
{
	setDataSeriesSettings(numOfDataSeries);

	// Only apply if higher than minimum window size
	windowSize = (window > MIN_WINDOW_SIZE) ? window : MIN_WINDOW_SIZE;

	// Open data output file
	FILE *dataFile = fopen(dataFilePath, "w");
	if (dataFile)
	{
		char tmpStr[128] = "";
		fputs("##|##\n", dataFile);
		sprintf(tmpStr, "@Data from port %d - Channel %d\n", port, channel);
		fputs(tmpStr, dataFile);
		int i = 0;
		sprintf(tmpStr, "Data %d", i);
		fputs(tmpStr, dataFile);
		for (i = 1; i < numOfDataSeries; i++)
		{
			sprintf(tmpStr, "|Data %d", i);
			fputs(tmpStr, dataFile);
		}
		fputs("\n", dataFile);

		// Close file
		fclose(dataFile);
	}
	const char *startCommand = "java -jar $TORB/Downloads/LiveGraph.1.14.Application.bin/LiveGraph.1.14.Application.bin.jar "
			"-dfs startup.lgdfs "
			"-gs graph.lggs "
			"-dss data.lgdss &";

	// Start process with pipe to stdin, so that we can terminate the process upon request
	return system(startCommand);
}

void writeDataToLiveGraph(int size_t, int numOfChannels, int numOfValuesPerChannel, void *data)
{
	int i;
	static long lineSize = 0, startSize = 0;
	char tmpStr[128] = "";
	// Open data output file
	FILE *dataFile = fopen(dataFilePath, "a");
	// Only write output if data file is open
	if (dataFile)
	{
		int dataPointer;
		for (dataPointer = 0; dataPointer < (numOfChannels * numOfValuesPerChannel); dataPointer+=numOfChannels)
		{
			if (size_t == sizeof(long))
			{
				long *pData = (long *)data;
				sprintf(tmpStr, "%ld", pData[dataPointer]);
				fputs(tmpStr, dataFile);
				for (i = 1; i < numOfChannels; i++)
				{
					sprintf(tmpStr, "|%ld", pData[dataPointer + i]);
					fputs(tmpStr, dataFile);
				}
			}
			else if (size_t == sizeof(int))
			{
				int *pData = (int *)data;
				sprintf(tmpStr, "%d", pData[dataPointer]);
				fputs(tmpStr, dataFile);
				for (i = 1; i < numOfChannels; i++)
				{
					sprintf(tmpStr, "|%d", pData[dataPointer + i]);
					fputs(tmpStr, dataFile);
				}
			}
			else if (size_t == sizeof(short))
			{
				short *pData = (short *)data;
				sprintf(tmpStr, "%d", pData[dataPointer]);
				fputs(tmpStr, dataFile);
				for (i = 1; i < numOfChannels; i++)
				{
					sprintf(tmpStr, "|%d", pData[dataPointer + i]);
					fputs(tmpStr, dataFile);
				}
			}
			else if (size_t == sizeof(char))
			{
				char *pData = (char *)data;
				sprintf(tmpStr, "%d", pData[dataPointer]);
				fputs(tmpStr, dataFile);
				for (i = 1; i < numOfChannels; i++)
				{
					sprintf(tmpStr, "|%d", pData[dataPointer + i]);
					printf(":%d", pData[dataPointer + i]);
					fputs(tmpStr, dataFile);
				}
			}
			else
			{
				sprintf(tmpStr, "Unsupported type of size %d", size_t);
				fputs(tmpStr, dataFile);
			}
			fputs("\n", dataFile);
		}

		long currentSize = ftell(dataFile);
		if (startSize == 0)
		{
			startSize = currentSize;
			printf("Start size: %ld\n", startSize);
		}
		else if (lineSize == 0)
		{
			lineSize = (currentSize - startSize) / (numOfChannels * numOfValuesPerChannel);
//			printf("Line size: %ld, limit: %ld\n", lineSize, (lineSize * windowSize + SIZE_HYSTERESIS));
			printf("Line size: %ld, limit: %d\n", lineSize, FILE_SIZE_LIMIT);
		}
//		else if (currentSize > (lineSize * windowSize + SIZE_HYSTERESIS))
//		{
//			cropFile();
//			dataFile = NULL;
//		}
		else if (currentSize > FILE_SIZE_LIMIT)
		{
			moveFileAndRestart(dataFile);
			dataFile = NULL;
		}
		else
		{
			printf("Current size: %ld\n", currentSize);
		}

		// Close file
		if (dataFile != NULL)
		{
			fclose(dataFile);
		}
	}
	else
	{
		printf("Failed to open file to append new data\n");
	}
}

void setStartupSettings(const char *filePath, int updateFrequency)
{
	char tmpStr[128] = "";
	FILE *outputFile = fopen("startup.lgdfs", "w");
	if (outputFile)
	{
		fputs("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n", outputFile);
		fputs("<!DOCTYPE properties SYSTEM \"http://java.sun.com/dtd/properties.dtd\">\n", outputFile);
		fputs("<properties>\n", outputFile);
		fputs("<comment>LiveGraph version 1.1.4. DataFileSettings.</comment>\n", outputFile);
		fputs("<entry key=\"ShowOnlyTailData\">1</entry>\n", outputFile);
		fputs("<entry key=\"DataFile\">", outputFile);
		// Use the provided path
		fputs(filePath, outputFile);
		strcpy(dataFilePath, filePath);
		fputs("</entry>\n", outputFile);
		fputs("<entry key=\"UpdateFrequency\">", outputFile);
		sprintf(tmpStr, "%d", updateFrequency);
		fputs(tmpStr, outputFile);
		fputs("</entry>\n", outputFile);
		fputs("<entry key=\"DoNotCacheData\">0</entry>\n", outputFile);
		fputs("</properties>\n", outputFile);

		// Close file
		fclose(outputFile);
	}
}

void setGraphSettings(int minY, int maxY, int minX, int maxX)
{
	char tmpStr[128] = "";
	FILE *outputFile = fopen("graph.lggs", "w");
	if (outputFile)
	{
		fputs("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n", outputFile);
		fputs("<!DOCTYPE properties SYSTEM \"http://java.sun.com/dtd/properties.dtd\">\n", outputFile);
		fputs("<properties>\n", outputFile);
		fputs("<comment>LiveGraph version 1.1.4. GraphSettings.</comment>\n", outputFile);
		fputs("<entry key=\"VGridSize\">50</entry>\n", outputFile);
		fputs("<entry key=\"HGridSize\">50</entry>\n", outputFile);
		fputs("<entry key=\"HGridColour\">C0C0C0</entry>\n", outputFile);
		fputs("<entry key=\"XAxisParamValue\">1</entry>\n", outputFile);
		fputs("<entry key=\"MaxY\">", outputFile);
		if (maxY == INT_MAX)
		{
			//		Auto
			sprintf(tmpStr, "Auto");
		}
		else
		{
			sprintf(tmpStr, "%d", maxY);
		}
		fputs(tmpStr, outputFile);
		fputs("</entry>\n", outputFile);
		fputs("<entry key=\"MaxX\">", outputFile);
		if (maxX == INT_MAX)
		{
			//		Auto
			sprintf(tmpStr, "Auto");
		}
		else
		{
			sprintf(tmpStr, "%d", maxX);
		}
		fputs(tmpStr, outputFile);
		fputs("</entry>\n", outputFile);
		fputs("<entry key=\"MinY\">", outputFile);
		if (minY == INT_MIN)
		{
			//		Auto
			sprintf(tmpStr, "Auto");
		}
		else
		{
			sprintf(tmpStr, "%d", minY);
		}
		fputs(tmpStr, outputFile);
		fputs("</entry>\n", outputFile);
		fputs("<entry key=\"MinX\">", outputFile);
		if (minX == INT_MIN)
		{
			//		Auto
			sprintf(tmpStr, "Auto");
		}
		else
		{
			sprintf(tmpStr, "%d", minX);
		}
		fputs(tmpStr, outputFile);
		fputs("</entry>\n", outputFile);
		fputs("<entry key=\"XAxisSeriesIndex\">0</entry>\n", outputFile);
		fputs("<entry key=\"VGridColour\">C0C0C0</entry>\n", outputFile);
		fputs("<entry key=\"XAxisType\">XAxis_DSNum</entry>\n", outputFile);
		fputs("<entry key=\"VGridType\">VGrid_None</entry>\n", outputFile);
		fputs("<entry key=\"HGridType\">HGrid_None</entry>\n", outputFile);
		fputs("<entry key=\"HighlightDataPoints\">1</entry>\n", outputFile);
		fputs("</properties>\n", outputFile);

		// Close file
		fclose(outputFile);
	}
}

void setDataSeriesSettings(int numOfDataSeries)
{
	char tmpStr[128] = "";
	FILE *outputFile = fopen("data.lgdss", "w");
	if (outputFile)
	{
		fputs("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n", outputFile);
		fputs("<!DOCTYPE properties SYSTEM \"http://java.sun.com/dtd/properties.dtd\">\n", outputFile);
		fputs("<properties>\n", outputFile);
		fputs("<comment>LiveGraph version 1.1.4. DataSeriesSettings.</comment>\n", outputFile);
		fputs("<entry key=\"DescribedSeriesCount\">", outputFile);
//		sprintf(tmpStr, "%d", numOfDataSeries);
		sprintf(tmpStr, "%d", 0);
		fputs(tmpStr, outputFile);
		fputs("</entry>\n", outputFile);
		fputs("</properties>\n", outputFile);

		// Close file
		fclose(outputFile);
	}
}

void moveFileAndRestart(FILE *dataFile)
{
	char tmpStr[512];
	// Time to crop the file
	printf("Moving file %s\n", dataFilePath);
	// Copy first three and last windowSize lines to temporary file.
	// Reopen file to read.
	freopen(NULL, "r", dataFile);
	if (dataFile == NULL)
	{
		printf("Failed to reopen %s for reading\n", dataFilePath);
		return;
	}
	printf("1\n");
	fseek(dataFile, 0, SEEK_SET);
	printf("2\n");
	FILE *tmpFile = fopen("tmp.file", "w");
	int l;
	// First the first three lines
	for (l = 0; l < 3; l++)
	{
		fgets(tmpStr, sizeof(tmpStr), dataFile);
		fputs(tmpStr, tmpFile);
	}
	// Close data file, move it to storage, then move tmp to data file
	printf("3\n");
	fsync(fileno(tmpFile));
	printf("4\n");
	fclose(dataFile);
	fclose(tmpFile);

	static int fileCount = 0;
//	static time_t rawtime;
//	static struct tm *timeinfo;
//	time(&rawtime);
//	timeinfo = localtime(&rawtime);
	printf("5\n");
	strcpy(tmpStr, dataFilePath);
	printf("6\n");
	char *tmpTok = strtok(tmpStr, ".");
	printf("7\n");
	tmpTok = strtok(NULL, ".");
	printf("8\n");
//	sprintf(tmpTok - 1, "_%d-%d-%d_%d-%d-%d.dat", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
	sprintf(tmpTok - 1, "_%d.dat", fileCount++);
	printf("9\n");
	printf("Moving %s to %s\n", dataFilePath, tmpStr);
	printf("10\n");
	rename(dataFilePath, tmpStr);
	printf("11\n");
	printf("Moving tmp.file to %s\n", dataFilePath);
	rename("tmp.file", dataFilePath);
	printf("File moved to %s\n", tmpStr);
}

void cropFile(FILE *dataFile, int lineSize)
{
	char tmpStr[512];
	// Time to crop the file
	printf("Cropping file\n");
	// Copy first three and last windowSize lines to temporary file.
	// Reopen file to read.
	freopen(NULL, "r", dataFile);
	fseek(dataFile, 0, SEEK_SET);
	FILE *tmpFile = fopen("tmp.file", "w");
	int l;
	// First the first three lines
	for (l = 0; l < 3; l++)
	{
		fgets(tmpStr, sizeof(tmpStr), dataFile);
		fputs(tmpStr, tmpFile);
	}
	// Then then the last windowSize lines
	fseek(dataFile, (windowSize * lineSize), SEEK_END);
	for (l = 0; l < windowSize; l++)
	{
		fgets(tmpStr, sizeof(tmpStr), dataFile);
		fputs(tmpStr, tmpFile);
	}
	// Finally close data file, rename tmp file to overwrite datafile, then open datafile
	fsync(fileno(tmpFile));
	fclose(dataFile);
	fclose(tmpFile);
	rename("tmp.file", dataFilePath);

	printf("File crop completed\n");
}
