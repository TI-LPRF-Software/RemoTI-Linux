/**************************************************************************************************
 Filename:       configParser.c

 Description:    Linux Host application tool

   Copyright (C) {YEAR} Texas Instruments Incorporated - http://www.ti.com/


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

#include "configParser.h"
#include "pthread.h"

#ifdef __DEBUG_TIME__
#include "time_printf.h"
#endif //__DEBUG_TIME__

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#define debug_verbose_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
extern int __DEBUG_APP_ACTIVE;
#define debug_printf(fmt, ...) 			st(if (__DEBUG_APP_ACTIVE > 1) printf( fmt, ##__VA_ARGS__);)
#define debug_verbose_printf(fmt, ...) 	st(if (__DEBUG_APP_ACTIVE >= 3) printf( fmt, ##__VA_ARGS__);)
#endif

#ifdef __DEBUG_TIME__
#define debug_time_printf(str) 	st(if (__DEBUG_APP_ACTIVE > 1) time_printf(str);)
#else
#define time_printf(str) printf( "%s", str)
#define debug_time_printf(str) debug_printf( "%s", str)
#endif //__DEBUG_TIME__

static FILE *configFileFd;
static FILE *shadowFileFd;

static char tmpStrForTimePrint[1024];

static pthread_mutex_t cfgFileAccessMutex;

/**************************************************************************************************
 *
 * @fn          ConfigParserInit
 *
 * @brief       This function set the local static file descriptor to perform consecutive
 * 				searches.
 *
 * input parameters
 *
 * @param          configFilePath   - path to configuration file
 * @param          shadowPath   	- path to shadow file used for new entries. Caller is
 * 									  responsible to verify permission to write to this
 * 									  file.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
int ConfigParserInit(const char *configFilePath, const char *shadowPath)
{
	ConfigParserClose();

	debug_verbose_printf("[CFG_PRS] Init\n");

	configFileFd = fopen(configFilePath, "r");
	if (configFileFd)
	{
		if (shadowPath)
		{
			shadowFileFd = fopen(shadowPath, "w");
		}

		if ( (!shadowPath) || (shadowFileFd && shadowPath) )
		{
			return 0;
		}
		else
		{
			perror("[CFG_PRS] Failed to open shadow file when it was requested");
			return -1;
		}
	}
	else
	{
		perror("[CFG_PRS] Failed to open configuration file");
		return -1;
	}

	if (pthread_mutex_init(&cfgFileAccessMutex, NULL))
	{
		sprintf(tmpStrForTimePrint, "[CFG_PRS][ERROR] Failed To Initialize Mutex cfgFileAccessMutex\n");
		time_printf(tmpStrForTimePrint);
		return -1;
	}
}

/**************************************************************************************************
 *
 * @fn          ConfigParserClose
 *
 * @brief       Closes opened files
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void ConfigParserClose()
{
	debug_verbose_printf("[CFG_PRS] Closing (if open)\n");
	if (configFileFd)
	{
		// Close previous file descriptor if it exist.
		fclose(configFileFd);
		configFileFd = NULL;
	}
	if (shadowFileFd)
	{
		// Close previous file descriptor if it exist.
		fclose(shadowFileFd);
		shadowFileFd = NULL;
	}
}

/**************************************************************************************************
 *
 * @fn          ConfigParserGetBaseSettings
 *
 * @brief       This function set the local static file descriptor to perform consecutive
 * 				searches.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * @param          appBaseSetting_s   - pointer to structure to store the base settings
 *
 * @return      status.
 *
 **************************************************************************************************/
int ConfigParserGetBaseSettings(appBaseSetting_s *baseSettings)
{
	int ret = 0;

	char *strBuf = NULL;
	strBuf = (char*) malloc(2048);
	memset(strBuf, 0, 2048);

	if (configFileFd != NULL)
	{
		if (baseSettings != NULL)
		{
			if (0 == ConfigParser("DEBUG_MODE", "APP", strBuf))
			{
				baseSettings->debug.app = strtol(strBuf, NULL, 10);
				__DEBUG_APP_ACTIVE = baseSettings->debug.app;
				debug_printf("[CFG_PRS] baseSettings->debug.app = 0x%.2X\n", baseSettings->debug.app);
			}
			else
			{
				ret = -1;
			}
			if (baseSettings->ip_addr != NULL)
			{
				if (0 == ConfigParser("BASE_SETTINGS", "IP_ADDR", baseSettings->ip_addr))
				{
					debug_printf("[CFG_PRS] baseSettings->ip_addr = %s\n", baseSettings->ip_addr);
				}
				else
				{
					ret = -1;
				}
			}
			if (baseSettings->port != NULL)
			{
				if (0 == ConfigParser("BASE_SETTINGS", "PORT", baseSettings->port))
				{
					debug_printf("[CFG_PRS] baseSettings->port = %s\n", baseSettings->port);
				}
				else
				{
					ret = -1;
				}
			}
			if (0 == ConfigParser("BASE_SETTINGS", "FA_ENABLED", strBuf))
			{
				baseSettings->freqAgilityEnabled = strtol(strBuf, NULL, 10);
				debug_printf("[CFG_PRS] baseSettings->freqAgilityEnabled = 0x%.2X\n", baseSettings->freqAgilityEnabled);
			}
			else
			{
//				ret = -1;
			}
			if (0 == ConfigParser("BASE_SETTINGS", "MAC_CH", strBuf))
			{
				baseSettings->macChannel = strtol(strBuf, NULL, 10);
				debug_printf("[CFG_PRS] baseSettings->macChannel = 0x%.2X\n", baseSettings->macChannel);
			}
			else
			{
//				ret = -1;
			}
			if (0 == ConfigParser("DEBUG_MODE", "BIG", strBuf))
			{
				baseSettings->debug.big = strtol(strBuf, NULL, 10);
				debug_printf("[CFG_PRS] baseSettings->debug.big = 0x%.2X\n", baseSettings->debug.big);
			}
			else
			{
				ret = -1;
			}
			if (0 == ConfigParser("DEBUG_MODE", "TIMER", strBuf))
			{
				baseSettings->debug.timer = strtol(strBuf, NULL, 10);
				debug_printf("[CFG_PRS] baseSettings->debug.timer = 0x%.2X\n", baseSettings->debug.timer);
			}
			else
			{
				ret = -1;
			}
			if (0 == ConfigParser("DEBUG_MODE", "CLIENT", strBuf))
			{
				baseSettings->debug.client = strtol(strBuf, NULL, 10);
				debug_printf("[CFG_PRS] baseSettings->debug.client = 0x%.2X\n", baseSettings->debug.client);
			}
			else
			{
//				ret = -1;
			}
		}
		else
		{
			// Regard this as a successful attempt to check if Config Parser has been initialized.
			ret = 0;
		}
	}
	else
	{
		// ConfigParser has not been initialized.
		ret = -1;
	}

	free(strBuf);

	return ret;
}

/**************************************************************************************************
 *
 * @fn          ConfigParserSetValue
 *
 * @brief       This function searches for a key and returns its value.
 * 				NOTE! Config Parser must have been initialized first.
 *
 * input parameters
 *
 * @param          section          - section to search for
 * @param          key              - key to return value of within section
 *
 * output parameters
 * @param          resultstring     - return string, must be allocated by caller
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
int ConfigParser(const char* section, const char* key, char* resultString)
{
	if (configFileFd != NULL)
	{
		return ConfigParserFromFd(configFileFd, section, key, resultString);
	}
	else
	{
		return -1;
	}
}

int ConfigParserSet(const char *configFilePath, const char *shadowPath, const char* section, const char* key, char* newValue)
{
	int returnVal = 0;

	int debugSettingBefore = __DEBUG_APP_ACTIVE;
	__DEBUG_APP_ACTIVE = FALSE;

	// First initialize the ConfigParser
	ConfigParserInit(configFilePath, shadowPath);

	if (configFileFd && shadowFileFd)
	{
		returnVal = ConfigParserSetGetFromFd(configFileFd, section, key, NULL, newValue);
	}
	else
	{
		returnVal = -1;
	}

	// Now copy the updated shadow over to the NV file. First make sure the files are not
	// used by Config Parser
	ConfigParserClose();

	// The following function assumes that the system will overwrite the existing file.
	rename(shadowPath, configFilePath);

	__DEBUG_APP_ACTIVE = debugSettingBefore;

	return returnVal;
}

/**************************************************************************************************
 *
 * @fn          ConfigParserSetValue
 *
 * @brief       This function searches for a key and updates its value, in the shadow file
 * 				NOTE! Config Parser must have been initialized first.
 *
 * input parameters
 *
 * @param          section      - section to search for
 * @param          key          - key to return value of within section
 * @param          newValue     - string to set as new value
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
int ConfigParserSetValue(const char* section, const char* key, char* newValue)
{
	if (configFileFd && shadowFileFd)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

/**************************************************************************************************
 *
 * @fn          ConfigParserFromPath
 *
 * @brief       This function searches for a string a returns its value.
 * 				Use this if you don't want to keep the file opened. This function
 * 				will open the file for you based on the incoming path.
 *
 * input parameters
 *
 * @param          configFilePath   - path to configuration file
 * @param          section          - section to search for
 * @param          key              - key to return value of within section
 *
 * output parameters
 * @param          resultstring     - return string, must be allocated by caller
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
int ConfigParserFromPath(const char *configFilePath, const char* section,
		const char* key, char* resultString)
{
	FILE *configFileTmpFd;
	int res = 1;

	configFileTmpFd = fopen(configFilePath, "r");

	ConfigParserFromFd(configFileTmpFd, section, key, resultString);

	fclose(configFileTmpFd);

	return res;
}

/**************************************************************************************************
 *
 * @fn          ConfigParserFromFd
 *
 * @brief       This function searches for a string a returns its value. It searches
 * 				in the file you provide.
 *
 * input parameters
 *
 * @param          cfgFd   			- configuration file descriptor
 * @param          section          - section to search for
 * @param          key              - key to return value of within section
 *
 * output parameters
 * @param          resultstring     - return string, must be allocated by caller
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
int ConfigParserFromFd(FILE* cfgFd, const char* section,
		const char* key, char* resultString)
{
	return ConfigParserSetGetFromFd( cfgFd, section,
			key, resultString, NULL);
}

/**************************************************************************************************
 *
 * @fn          ConfigParserSetGetFromFd
 *
 * @brief       This function searches for a string a returns its value. It searches
 * 				in the file you provide. It also updates the shadow file if newValue string is
 * 				non-NULL
 *
 * input parameters
 *
 * @param          cfgFd   			- configuration file descriptor
 * @param          section          - section to search for
 * @param          key              - key to return value of within section
 * @param          newValue     	- string to set as new value
 *
 * output parameters
 * @param          resultstring     - return string, must be allocated by caller
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
int ConfigParserSetGetFromFd(FILE* cfgFd, const char* section,
		const char* key, char* resultString, char* newValue)
{
	uint8 sectionFound = FALSE, invalidLineLen = FALSE, newValueWritten = FALSE;
	char* resString = NULL;
	char* resStringToFree = NULL;
	char* lineToShadow = NULL;
	char* lineToShadowToFree = NULL;
	char* lineToShadowCreated = NULL;
	char* lineToShadowCreatedToFree = NULL;
	char* psStr; // Processing string pointer
	int res = -1;

	int debugSettingBefore = __DEBUG_APP_ACTIVE;
//	if (strcmp(key, "ValidationConfig") != 0)
//	{
//		__DEBUG_APP_ACTIVE = FALSE;
//	}
	// Get file access lock first
	pthread_mutex_lock(&cfgFileAccessMutex);

	resString = malloc (2048);
	lineToShadow = malloc (2048);
	lineToShadowCreated = malloc (1024);
	if (!resString || !lineToShadow || !lineToShadowCreated)
	{
		return res;
	}
	resStringToFree = resString;
	lineToShadowToFree = lineToShadow;
	lineToShadowCreatedToFree = lineToShadowCreated;
	debug_verbose_printf("[CFG_PRS]------------------------------------------------------\n");
	debug_verbose_printf("[CFG_PRS]Config Parsing:\n");
	debug_verbose_printf("[CFG_PRS]- \tSection \t%s:\n", section);
	debug_verbose_printf("[CFG_PRS]- \tKey \t\t%s:\n", key);

	// Do nothing if the file doesn't exist
	if (cfgFd != NULL)
	{
		sprintf(tmpStrForTimePrint, "[CFG_PRS][INFO] Resetting files - \tSection \t%s:\t%s:\n", section, key);
		debug_time_printf(tmpStrForTimePrint);
		if (shadowFileFd != NULL)
		{
			// Make sure we restart the shadow file
			fseek(shadowFileFd, 0, SEEK_SET);
		}
		// Make sure we start search from the beginning of the file
		fseek(cfgFd, 0, SEEK_SET);

		sprintf(tmpStrForTimePrint, "[CFG_PRS][INFO] Files reset, now searching\n");
		debug_time_printf(tmpStrForTimePrint);

		// Search through the configuration file for the wanted
		while ((resString = fgets(resString, 2048, cfgFd)) != NULL)
		{
			if (shadowFileFd && lineToShadow && newValue)
			{
				// Copy line to shadow line
				strcpy(lineToShadow, resString);
			}

			// Check if we have a valid line, i.e. begins with [.
			// Note! No valid line can span more than 2048 bytes. Hence we
			// must hold off parsing until we hit a newline.
			if (strlen(resString) == 2048)
			{
				invalidLineLen = TRUE;
				debug_verbose_printf("[CFG_PRS]Found line > 2048 bytes\r");
				fflush(stdout);
			}
			else
			{
				// First time we find a valid line length after having
				// found invalid line length may be the end of the
				// invalid line. Hence, do not process this string.
				// We set the invalidLineLen parameter to FALSE after
				// the processing logic.
				if (invalidLineLen == FALSE)
				{
					// Remove the newline character (ok even if line had length 2048)
					resString[strlen(resString) - 1] = '\0';

					debug_verbose_printf("[CFG_PRS]Found line < 2048 bytes\r");
					fflush(stdout);
					if (resString[0] == '[')
					{
						debug_verbose_printf("[CFG_PRS]Found section %s\n", resString);
						// Search for wanted section
						psStr = strstr(resString, section);
						if (psStr != NULL)
						{
							resString = psStr;
							// We found our wanted section. Now search for wanted key.
							sectionFound = TRUE;
							debug_verbose_printf("[CFG_PRS]Found wanted section!\n");
						}
						else
						{
							// We found another section.
							if (sectionFound == TRUE)
							{
								// We passed the whole section without finding the value. Add it manually instead
								if (shadowFileFd && lineToShadow && newValue)
								{
									if (newValueWritten == TRUE)
									{
										debug_verbose_printf("[CFG_PRS] Value already written, can move on!\n");
									}
									else
									{
										debug_printf("[CFG_PRS][NEW] Section found, but not value in it. We add it here: ");
										// Create new line based on known key and value
										lineToShadowCreated[0] = '\0';
										strcat(lineToShadowCreated, key);
										strcat(lineToShadowCreated, "=");
										strcat(lineToShadowCreated, newValue);
										strcat(lineToShadowCreated, "\n");
										// Write it to file
										fputs(lineToShadowCreated, shadowFileFd);
										debug_printf("%s", lineToShadowCreated);
										newValueWritten = TRUE;
									}
								}
							}
							sectionFound = FALSE;
						}
					}
					else if (sectionFound == TRUE)
					{
						debug_verbose_printf("[CFG_PRS]Line to process %s (strlen=%zd)\n",
								resString,
								strlen(resString));
						// We have found our section, now we search for wanted key
						// Check for commented lines, tagged with '#', and
						// lines > 0 in length
						if ((resString[0] != '#') && (strlen(resString) > 0))
						{
							// Search for wanted section
							psStr = strstr(resString, key);
							if (psStr != NULL)
							{
								debug_verbose_printf("[CFG_PRS]Found key \t'%s' in \t'%s'\n", key, resString);
								// The value is located after the '='
								// after the key.
								//                                                                                                                            printf("%s\n", psStr);
								psStr = strtok(psStr, "=");
								//                                                                                                                            printf("%s\n", psStr);
								psStr = strtok(NULL, "=;\"");
								//                                                                                                                            printf("%s\n", psStr);

								resString = psStr;
								res = 0;
								debug_verbose_printf("[CFG_PRS]Found value '%s'\n", resString);

								// Only copy result if it was asked for.
								if (resultString)
								{
									strcpy(resultString, resString);
									debug_verbose_printf("[CFG_PRS]Found value2 '%s'\n", resultString);
								}

								// Update shadow file if new value is provided
								if (shadowFileFd && lineToShadow && newValue)
								{
									if (newValueWritten == TRUE)
									{
										debug_verbose_printf("[CFG_PRS] Value already written!\n");
									}
									else
									{
										// Create new line based on known key and value
										strcpy(lineToShadow, key);
										strcat(lineToShadow, "=");
										strcat(lineToShadow, newValue);
										strcat(lineToShadow, "\n");
										debug_printf("[CFG_PRS] Updated line to '%s'\n", lineToShadow);
										newValueWritten = TRUE;
									}
									// Move back pointer
									resString = resStringToFree;
								}
								else
								{
									// We can return this string to the calling function

									// Only break if we're not trying to update file
									break;
								}
							}
						}
					}
					// Write line to shadow file
					if (shadowFileFd && lineToShadow && newValue)
					{
						debug_verbose_printf("[CFG_PRS][OLD] Writing existing line '%s' to shadow file...", lineToShadow);
						fflush(stdout);
						fputs(lineToShadow, shadowFileFd);
						debug_verbose_printf(" done\n");
					}
				}
				else
				{
					debug_verbose_printf("[CFG_PRS]Found end of line > 2048 bytes\n");
					invalidLineLen = FALSE;
				}
			}
		}

		sprintf(tmpStrForTimePrint, "[CFG_PRS][INFO] Search complete\n");
		debug_time_printf(tmpStrForTimePrint);
	}
	else
	{
		free(resStringToFree);
		free(lineToShadowToFree);
		free(lineToShadowCreatedToFree);
		return res;
	}

	// We passed the whole section without finding the value. Add it manually instead
	if (shadowFileFd && lineToShadow && newValue)
	{
		if (newValueWritten == TRUE)
		{
			debug_verbose_printf("[CFG_PRS] Value written successfully!\n");
		}
		else
		{
			if (sectionFound == FALSE)
			{
				debug_printf("[CFG_PRS] Section not found. We add it here\n");
				// Create new section line
				lineToShadowCreated[0] = '[';
				lineToShadowCreated[1] = '\0';
				strcat(lineToShadowCreated, section);
				strcat(lineToShadowCreated, "]");
				strcat(lineToShadowCreated, "\n");
				// Write it to file
				debug_printf("[CFG_PRS][NEW] Writing line '%s' to shadow file\n", lineToShadowCreated);
				fputs(lineToShadowCreated, shadowFileFd);
			}
			debug_printf("[CFG_PRS][NEW] Preparing line to shadow file\n");
			// Create new line based on known key and value
			lineToShadowCreated[0] = '\0';
			strcat(lineToShadowCreated, key);
			strcat(lineToShadowCreated, "=");
			strcat(lineToShadowCreated, newValue);
			strcat(lineToShadowCreated, "\n");
			debug_printf("[CFG_PRS][NEW] Writing line '%s' to shadow file\n", lineToShadowCreated);
			// Write it to file
			fputs(lineToShadowCreated, shadowFileFd);
		}
	}

	free(resStringToFree);
	free(lineToShadowToFree);
	free(lineToShadowCreatedToFree);

	// Release file access lock first
	pthread_mutex_unlock(&cfgFileAccessMutex);

	__DEBUG_APP_ACTIVE = debugSettingBefore;

	return res;
}
