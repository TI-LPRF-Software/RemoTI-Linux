#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>
#include <malloc.h>
#include <sys/stat.h>
#include <tiLogging.h>

#define SIZEMAX(a,b) ((sizeof(a) >= sizeof(b)) ? sizeof(a) : sizeof(b))

// Global variables (used by the logging macros)
int               __APP_LOG_LEVEL = MAX_LOG_LEVEL_TO_STDIO;
int               __BIG_DEBUG_ACTIVE = 0;
char const        *processLogPrefix = ""; // Default to empty string, NOT NULL, so can be used even if tiLogging_Init is never called.


static char const *levelToName(int level)
{
	static       char       illegalLevel[40];
	static const char const *kLevelNames[LOG_LEVEL_MAX+1] = {"ALWAYS (only those that cannot be turned off)", "FATAL", "ERROR", "WARN", "INFO", "DEBUG", "TRACE1", "TRACE2"};

	char const *name;

	if (level < 0 || level > LOG_LEVEL_MAX)
	{
		snprintf(illegalLevel, sizeof(illegalLevel), "Unrecognized value %d", level);
		name = illegalLevel;
	}
	else
	{
		name = kLevelNames[level];
	}
	return name;
}


/**************************************************************************************************
 * @fn      tiLogging_Init
 * @brief   Initializes logging code.  If never called, logging will still
 *          work, but the process name won't be shown and only the default
 *				log level will be possible.
 *
 * input parameters
 *
 * @param *configFileName: multi-purpose...
 *    If 'configFileName' as an unsigned integer is >0 and <= LOG_LEVEL_MAX
 *       it will use that as the logging level (Limitation: Using this method you cannot
 *       disable LOG_FATAL logging, but that's probably not anything you would want off.)
 *    Else if 'configFileName' is non-NULL,
 *       it will try to read the logging level from 'configFileName'.
 *    Else if 'configFileName' is NULL, then first looks for a file in the same base name as
 *       the process but with DEFAULT_LOG_CFG_SUFFIX, and will look for it in
 *       LOG_CONFIG_FILE_OVERRIDE_PATH1 and if not found then in LOG_CONFIG_FILE_OVERRIDE_PATH2.
 *       If it doesn't exist there, it will look in the path of the process. If it's not there
 *       it gives up and defaults to MAX_LOG_LEVEL_TO_STDIO (defined above).
 *    If it finds a config file and the value is out of range, it ignores it and defaults to
 *    MAX_LOG_LEVEL_TO_STDIO.
 *
 * @return  FALSE if previously initialized OR no config was found, otherwise TRUE.
 **************************************************************************************************
 */
bool tiLogging_Init(char const *configFileName)
{
	static const char kDefaultCfgExtension[] = DEFAULT_LOG_CFG_SUFFIX;
	static const char kConfigFileOverridePath1[] = LOG_CONFIG_FILE_OVERRIDE_PATH1;
	static const char kConfigFileOverridePath2[] = LOG_CONFIG_FILE_OVERRIDE_PATH2;
	static       bool alreadyInitialized = false;
	struct       stat statData;
	auto         int  statResult = 0;
	auto         bool foundConfig = false;

	if (!alreadyInitialized)
	{
		FILE *fp;
		char *allocatedPath = NULL;
		char *processName = NULL;
		char nameBuf[256] = {0};

		alreadyInitialized = true;

		// IMPORTANT: This call to setvbuf() MUST BE DONE FIRST.
		// It changes the buffering of stdout to line-based, and
		// that works ONLY if the stream has not been used yet, so
		// we MUST NOT printf/log anything before calling it...
		// Set stdout buffering to line-based buffering.  If
		// it is going to a terminal, it should be this by default,
		// but if it's being redirected to a file, files are block-
		// buffered by default and we want to be able to tail a
		// log file and see output in realtime, AND in the instance
		// of a crash, we don't want to lose a block that hadn't
		// been flushed yet.  We are NOT disabling buffering
		// altogether because that has more of an impact on performance.
		setvbuf(stdout, NULL, _IOLBF, 0); // Leaves buffer, changes mode.

		// Find our process name to be used as part of the logging prefix...
		if (!(fp=fopen("/proc/self/cmdline", "r")))
			LOG_ALWAYS("%s(): Can't open cmdline for reading to determine process name!\n", __FUNCTION__);
		else
		{
			if (!fgets(nameBuf, sizeof(nameBuf), fp))
				LOG_ALWAYS("%s(): Can't read cmdline to determine process name!\n", __FUNCTION__);
			else
			{
				char *ptr;

				fgets(nameBuf, sizeof(nameBuf), fp);

				ptr = nameBuf;
				while (*ptr && !isspace(*ptr)) // Find first whitespace (or EOL)
					++ptr;
				if (ptr)
					*ptr = 0; // Terminate string at first whitepace

				if (!(processName=strrchr(nameBuf, '/'))) // Get base name (no path) of process;
					processName = nameBuf;
				else
					++processName; // Skip past '/'

				if (processName && *processName)
				{
					int len = strlen(processName) + 4; // +4 is for brackets, space, and null terminator.
					ptr = malloc(len);
					if (ptr) // Shouldn't fail malloc, but playing it safe so processLogPrefix can't become NULL
					{
						snprintf(ptr, len, "[%s] ", processName);
						processLogPrefix = ptr;
					}
				}
			}
			fclose(fp);
		}

		if (configFileName && (((intptr_t)configFileName) <= LOG_LEVEL_MAX))
		{
			__APP_LOG_LEVEL = ((intptr_t)configFileName);
			LOG_ALWAYS("%s(): Called with hard-coded level. Logging threshold set to level %s.\n\n", __FUNCTION__, levelToName(__APP_LOG_LEVEL));
			configFileName = NULL;
			foundConfig = true;
		}
		else if ((!configFileName || !*configFileName) && processName && *processName)
		{
			// By now, if we were successful getting the process name from the cmdline,
			// it's stored in processLogPrefix, and the full path to it is still in nameBuf.
			int maxLen = strlen(nameBuf) + SIZEMAX(kConfigFileOverridePath1,kConfigFileOverridePath2) + sizeof(kDefaultCfgExtension) + 1;

			// Allocate space for the longest path/name of the locations we'll try.
			if ((allocatedPath = malloc(maxLen)))
			{
				snprintf(allocatedPath, maxLen, "%s%s%s", kConfigFileOverridePath1, processName, kDefaultCfgExtension);
				if ((statResult = stat(allocatedPath, &statData)) == 0)
					configFileName = allocatedPath;
				else
				{
					snprintf(allocatedPath, maxLen, "%s%s%s", kConfigFileOverridePath2, processName, kDefaultCfgExtension);
					if ((statResult = stat(allocatedPath, &statData)) == 0)
						configFileName = allocatedPath;
					else
					{
						snprintf(allocatedPath, maxLen, "%s%s", nameBuf, kDefaultCfgExtension);
						if ((statResult = stat(allocatedPath, &statData)) == 0)
							configFileName = allocatedPath;
					}
				}
			}
		}

		// Note that we're implementing our own log filter if file exists.
		// This should get set to a positive value. (See tiLogging.h)
		if (!foundConfig && configFileName && *configFileName)
		{
			if ((fp = fopen(configFileName, "r")) == NULL)
				LOG_ALWAYS("%s(): File %s exists, but unable to open it for reading.\n", __FUNCTION__, configFileName);
			else
			{
				char valBuf[10];
				bool skipNext = false;
				bool gotValueFromFile = false;

				while (fgets(valBuf, sizeof(valBuf), fp) && !gotValueFromFile)
				{
					if (!skipNext && ((gotValueFromFile = isdigit(*valBuf)))) // Allow ANY non-digit as first character to be taken as a comment line.
					{
						int tmp=atoi(valBuf);

						if (tmp <= 0 || tmp > LOG_LEVEL_MAX)
							LOG_ALWAYS("%s(): Invalid value found (%d) in %s.\n", __FUNCTION__, tmp, configFileName);
						else
						{
							__APP_LOG_LEVEL = tmp;
							LOG_ALWAYS("%s(): Found %s. Logging threshold set to level %s.\n\n", __FUNCTION__, configFileName, levelToName(__APP_LOG_LEVEL));
							foundConfig = true;
						}
					}
					// If fgets terminated due to length, not newline, skip reads until we hit a newline (or end of file).
					skipNext = valBuf[strlen(valBuf)-1] != '\n';
				}
				fclose(fp);

				if (!gotValueFromFile)
					LOG_ALWAYS("%s(): %s exists but has no numeric line.\n", __FUNCTION__, configFileName);
			}
		}

		if (configFileName == allocatedPath)
			configFileName = NULL;

		if (!foundConfig)
		{
			if (configFileName)
				LOG_ALWAYS("%s(): Could not find specified logging config file %s\n", __FUNCTION__, configFileName);
			else if (processName && *processName)
				LOG_DEBUG("%s(): No default logging config file %s%s in %s nor in %s nor in process directory.\n", __FUNCTION__, processName, kDefaultCfgExtension, kConfigFileOverridePath1, kConfigFileOverridePath2);
			else
				LOG_ALWAYS("%s(): Could not determine process name to extrapolate into logging config file name!\n", __FUNCTION__);

			__APP_LOG_LEVEL = MAX_LOG_LEVEL_TO_STDIO;
			LOG_ALWAYS("%s(): Defaulting logging threshold to %s.\n\n", __FUNCTION__, levelToName(__APP_LOG_LEVEL));
		}

		if (allocatedPath)
			free(allocatedPath);
	}

	return foundConfig;
}

