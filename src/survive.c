
// Copyright 2016 <>< C. N. Lohr, FULLY Under MIT/x11 License.
// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_internal.h"
#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

#include "stdarg.h"

#include "os_generic.h"
#include "survive_config.h"
#include "survive_default_devices.h"
#include "survive_kalman_lighthouses.h"
#include "survive_recording.h"

#include <stdarg.h>

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __APPLE__
#define z_const const
#endif

#include "survive_private.h"

#define DEFAULT_CONFIG_PATH "config.json"
STATIC_CONFIG_ITEM(SURVIVE_VERBOSE, "v", 'i', "Verbosity level", 0)
STATIC_CONFIG_ITEM(BLACKLIST_DEVS, "blacklist-devs", 's', "List any devs (or substrings of devs) to blacklist.", "")
STATIC_CONFIG_ITEM(CONFIG_FILE, "configfile", 's', "Default configuration file", DEFAULT_CONFIG_PATH)
STATIC_CONFIG_ITEM(INIT_CONFIG_FILE, "init-configfile", 's', "Initial configuration file", 0)
STATIC_CONFIG_ITEM(CONFIG_D_CALI, "disable-calibrate", 'b', "Enables or disables calibration", 0)
STATIC_CONFIG_ITEM(CONFIG_F_CALI, "force-calibrate", 'b', "Forces calibration even if one exists.", 0)
STATIC_CONFIG_ITEM(CONFIG_F_OOTX, "force-ootx", 'b', "Forces ootx capture even if its in the config file.", 0)
STATIC_CONFIG_ITEM(CONFIG_LIGHTHOUSE_COUNT, "lighthousecount", 'i', "How many lighthouses to look for.", 0)
STATIC_CONFIG_ITEM(LIGHTHOUSE_GEN, "lighthouse-gen", 'i',
				   "Which lighthouse gen to use -- 1 for LH1, 2 for LH2, 0 (default) for auto-detect", 0)
STATIC_CONFIG_ITEM(OUTPUT_CALLBACK_STATS, "output-callback-stats", 'f',
				   "Print cb stats every given number of seconds. 0 disables this output.", 0.);
STATIC_CONFIG_ITEM(THREADED_POSERS, "threaded-posers", 'b', "Whether or not to run each poser in their own thread.", 1)

STATIC_CONFIG_ITEM(LH_0_DISABLE, "lighthouse-0-disable", 'b', "Disable lh at idx 0", 0)
STATIC_CONFIG_ITEM(LH_1_DISABLE, "lighthouse-1-disable", 'b', "Disable lh at idx 1", 0)
STATIC_CONFIG_ITEM(LH_2_DISABLE, "lighthouse-2-disable", 'b', "Disable lh at idx 2", 0)
STATIC_CONFIG_ITEM(LH_3_DISABLE, "lighthouse-3-disable", 'b', "Disable lh at idx 3", 0)
STATIC_CONFIG_ITEM(USE_LIGHTHOUSES, "use-lighthouses", 's', "Comma-separated list of lighthouse IDs (hex) to use for tracking.", "")

STRUCT_CONFIG_SECTION(SurviveContext)
STRUCT_CONFIG_ITEM("lighthouse-max-update", "Maximum instant move for a lighthouse", .01, t->settings.lh_max_update);
STRUCT_CONFIG_ITEM("lighthouse-max-nudge-distance",
				   "If a proscribed LH move is more than this; do it instantly -- it's too far off to slerp over", .1,
				   t->settings.lh_max_nudge_distance);
STRUCT_CONFIG_ITEM("lighthouse-update-velocity", "Allowable velocity to update a lighthouse", .1,
				   t->settings.lh_update_velocity);
END_STRUCT_CONFIG_SECTION(SurviveContext)

const char *survive_config_file_name(struct SurviveContext *ctx) {
	return survive_configs(ctx, "configfile", SC_GET, DEFAULT_CONFIG_PATH);
}
#ifdef WIN32
#define RUNTIME_SYMNUM
#endif

#ifdef RUNTIME_SYMNUM
#include <symbol_enumerator.h>
static int did_runtime_symnum;
int SymnumCheck(const char *path, const char *name, void *location, long size) {
	if (strncmp(name, "REGISTER", 8) == 0) {
		typedef void (*sf)();
		sf fn = (sf)location;
		fn();
	}
	return 0;
}

#endif

static void reset_stderr(FILE *f) {
#ifndef _WIN32
	fprintf(f, "\033[0m");
#else
	HANDLE   hConsole = GetStdHandle(STD_ERROR_HANDLE);
	SetConsoleTextAttribute(hConsole, 7);
#endif
}

static void set_stderr_color(FILE *f, int c) {
#ifndef _WIN32
	fprintf(f, "\033[0;31m");
#else
	HANDLE   hConsole = GetStdHandle(STD_ERROR_HANDLE);
	SetConsoleTextAttribute(hConsole, c == 1 ? FOREGROUND_RED : (FOREGROUND_INTENSITY | FOREGROUND_RED));
#endif
}

static void survive_default_report_error_process(struct SurviveContext *ctx, SurviveError errorCode) {
	ctx->currentError = errorCode;
}

int survive_default_printf_process(struct SurviveContext *ctx, const char *format, ...) {
  if(ctx->log_target == 0) return 0;
	va_list args;
	va_start(args, format);
	int rtn = vfprintf(ctx->log_target, format, args);
	va_end(args);
	return rtn;
}

static void survive_default_error(struct SurviveContext *ctx, SurviveError errorCode, const char *fault) {
  if(ctx->log_target == 0) return;
	set_stderr_color(ctx->log_target, 2);
	SURVIVE_INVOKE_HOOK(printf, ctx, "Error %d: %s", errorCode, fault);
	reset_stderr(ctx->log_target);
	SURVIVE_INVOKE_HOOK(printf, ctx, "\n");
	fflush(ctx->log_target);
}

static void survive_default_info(struct SurviveContext *ctx, const char *fault) {
  	survive_recording_info_process(ctx, fault);
	
    if(ctx->log_target == 0) return;
	SURVIVE_INVOKE_HOOK(printf, ctx, "Info: %s\n", fault);
	fflush(ctx->log_target);
}

static void survive_default_warn(struct SurviveContext *ctx, const char *fault) {
  	survive_recording_info_process(ctx, fault);
    if(ctx->log_target == 0) return;
	set_stderr_color(ctx->log_target, 1);
	SURVIVE_INVOKE_HOOK(printf, ctx, "Warning: %s\n", fault);
	reset_stderr(ctx->log_target);
	fflush(ctx->log_target);
}

void survive_default_log_process(struct SurviveContext *ctx, SurviveLogLevel ll, const char *fault) {
	switch (ll) {
	case SURVIVE_LOG_LEVEL_ERROR:
		survive_default_error(ctx, ctx->currentError, fault);
		return;
	case SURVIVE_LOG_LEVEL_WARNING:
		survive_default_warn(ctx, fault);
		return;
	case SURVIVE_LOG_LEVEL_INFO:
		survive_default_info(ctx, fault);
		return;
	}
}
size_t survive_input_event_count(const SurviveContext *ctx) {
	if (ctx->buttonQueue.nextReadIndex > ctx->buttonQueue.nextWriteIndex) {
		return (ctx->buttonQueue.nextWriteIndex + sizeof(ctx->buttonQueue.entry) / sizeof(ctx->buttonQueue.entry[0])) -
			   ctx->buttonQueue.nextReadIndex;
	}
	return ctx->buttonQueue.nextWriteIndex - ctx->buttonQueue.nextReadIndex;
}
static void *button_servicer(void *context) {
	SurviveContext *ctx = (SurviveContext *)context;

	while (1) {
		OGLockSema(ctx->buttonQueue.buttonservicesem);

		if (ctx->state != SURVIVE_RUNNING) {
			// we're shutting down.  Close.
			return NULL;
		}

		ButtonQueueEntry *entry = &(ctx->buttonQueue.entry[ctx->buttonQueue.nextReadIndex]);
		if (entry->isPopulated == 0) {
			// should never happen.  indicates failure of code pushing stuff onto
			// the buttonQueue
			// if it does happen, it will kill all future button input
			SV_WARN("Unpopulated ButtonQueueEntry! NextReadIndex=%d\n", ctx->buttonQueue.nextReadIndex);
			return NULL;
		}

		// printf("ButtonEntry: eventType:%x, buttonId:%d, axis1:%d, axis1Val:%8.8x, axis2:%d, axis2Val:%8.8x\n",
		//	entry->eventType,
		//	entry->buttonId,
		//	entry->axis1Id,
		//	entry->axis1Val,
		//	entry->axis2Id,
		//	entry->axis2Val);

		survive_get_ctx_lock(ctx);
		survive_recording_button_process(entry->so, entry->eventType, entry->buttonId, entry->ids, entry->axisValues);
		SURVIVE_INVOKE_HOOK_SO(button, entry->so, entry->eventType, entry->buttonId, entry->ids, entry->axisValues);
		survive_release_ctx_lock(ctx);
		ctx->buttonQueue.processed_events++;

		ctx->buttonQueue.nextReadIndex++;
		if (ctx->buttonQueue.nextReadIndex >= BUTTON_QUEUE_MAX_LEN) {
			ctx->buttonQueue.nextReadIndex = 0;
		}
	};
	return NULL;
}

void survive_verify_FLT_size(uint32_t user_size) {
	if (sizeof(FLT) != user_size) {
		fprintf(stderr, "FLT type incompatible; the shared library libsurvive has FLT size %lu vs user program %lu\n",
			(unsigned long)sizeof(FLT), (unsigned long)user_size);
		fprintf(stderr, "Add '#define FLT %s' before including survive.h or recompile the shared library with the "
						"appropriate flag. \n",
				sizeof(FLT) == sizeof(double) ? "double" : "float");
		exit(-1);
	}
}

static void PrintMatchingDrivers( const char * prefix, const char * matchingparam )
{
	int i = 0;
	char stringmatch[128];
	snprintf( stringmatch, 127, "%s%s", prefix, matchingparam?matchingparam:"" );
	const char * DriverName;
	while ((DriverName = GetDriverNameMatching(stringmatch, i++))) 
	{
		printf( "%s ", DriverName+strlen(prefix) );
	}
}

SURVIVE_EXPORT int8_t survive_get_bsd_idx(SurviveContext *ctx, survive_channel channel) {
	if (channel < 0 || channel >= 16) {
		return -1;
	}

	if (ctx->lh_version == 0) {
		if (ctx->bsd[channel].mode == 0xFF) {
			ctx->bsd[channel].mode = channel;
			ctx->activeLighthouses++;
			SV_INFO("Adding lighthouse ch %d (cnt: %d)", channel, ctx->activeLighthouses);
		}
		return channel;
	}

	int8_t i = ctx->bsd_map[channel];
	if (i != -1)
		return i;

	for (i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (ctx->bsd[i].mode == 0xFF) {
			ctx->bsd[i].mode = channel;
			if (ctx->activeLighthouses < i + 1) {
				ctx->activeLighthouses = i + 1;
			}
			SV_INFO("Adding lighthouse ch %d (idx: %d, cnt: %d)", channel, i, ctx->activeLighthouses);
			return ctx->bsd_map[channel] = i;
		}
	}

	return -1;
}

void survive_get_ctx_lock(SurviveContext *ctx) {
	struct SurviveContext_private *pctx = ctx->private_members;
	// SV_VERBOSE(100, "Trying to get lock on %lx", pthread_self());
	OGLockSema(pctx->poll_sema);
	// SV_VERBOSE(100, "Got lock on %lx", pthread_self());
}
void survive_release_ctx_lock(SurviveContext *ctx) {
	struct SurviveContext_private *pctx = ctx->private_members;
	// SV_VERBOSE(100, "Releasing lock on %lx", pthread_self());
	OGUnlockSema(pctx->poll_sema);
	// SV_VERBOSE(100, "Signaled on %lx", pthread_self());
}

static inline bool find_correct_config_file(struct SurviveContext *ctx, const char **config_prefix_fields) {
	for (const char **name = config_prefix_fields; *name; name++) {
		if (survive_config_is_set(ctx, *name)) {
			char configfile[256] = {0};
			const char *recordname = survive_configs(ctx, *name, SC_GET, 0);

			// If a non-arg driver is used; just use the name of the driver
			if (recordname == 0)
				recordname = *name;

			const char *end = recordname + strlen(recordname);
			while (end != recordname && !(*end == '/' || *end == '\\'))
				end--;
			if (*end == '/' || *end == '\\')
				end++;

			if (end[0] != 0) {
				recordname = end;
				snprintf(configfile, 256, "%s.json", recordname);
				survive_configs(ctx, "configfile", SC_SET | SC_OVERRIDE, configfile);
				return true;
			}
		}
	}

	return false;
}

void survive_init_plugins() {
	static bool loaded = false;
	if (loaded)
		return;

	loaded = true;
	survive_load_plugins(0);

#ifdef RUNTIME_SYMNUM
	if (!did_runtime_symnum) {
		EnumerateSymbols(SymnumCheck);
		did_runtime_symnum = 1;
	}
#endif
}
static bool disable_colorization = false;
static void survive_process_env(SurviveContext *ctx, bool only_print) {
	char ** env;
#if defined(WIN) && (_MSC_VER >= 1900)
	env = *__p__environ();
#else
	extern char ** environ;
	env = environ;
#endif

#define ENV_PREFIX "SURVIVE_"
	for (; *env; ++env) {
		if(strncmp(*env, ENV_PREFIX, strlen(ENV_PREFIX)) == 0) {
			const char* entry = *env + strlen(ENV_PREFIX);
			char tag[32] = { 0 };
			const char* value = strchr(entry, '=') + 1;
			int offset = value - entry - 1;
			if(offset > 32) continue;
			for(int i = 0; i < offset; i++) tag[i] = tolower(entry[i]);
			if(tag[0]) {
				if(only_print) {
					SV_VERBOSE(100, "\t[ENV]'%s'",*env);
				} else {
					survive_configs(ctx, tag, SC_OVERRIDE | SC_SET, value);
				}
			}
		}
	}

}

SurviveContext *survive_init_internal(int argc, char *const *argv, void *userData, log_process_func log_func) {
	int i;

	survive_init_plugins();

	SurviveContext *ctx = SV_CALLOC(sizeof(SurviveContext));
	ctx->user_ptr = userData;
	ctx->poll_min_time_ms = 10;

	struct SurviveContext_private *pctx = ctx->private_members = SV_CALLOC(sizeof(struct SurviveContext_private));
	pctx->external2world.Rot[0] = 1;

	pctx->poll_sema = OGCreateSema();

	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		ctx->bsd[i].mode = -1;
		ctx->bsd_map[i] = -1;
	}
	ctx->state = SURVIVE_STOPPED;

#define SURVIVE_HOOK_PROCESS_DEF(hook) survive_install_##hook##_fn(ctx, 0);
#define SURVIVE_HOOK_FEEDBACK_DEF(hook) survive_install_##hook##_fn(ctx, 0);
#include "survive_hooks.h"

	survive_install_log_fn(ctx, log_func);

	ctx->global_config_values = SV_MALLOC(sizeof(config_group));
	ctx->temporary_config_values = SV_MALLOC(sizeof(config_group));
	ctx->lh_config = SV_MALLOC(sizeof(config_group) * NUM_GEN2_LIGHTHOUSES);

	// initdata
	init_config_group(ctx->global_config_values, 30, ctx);
	init_config_group(ctx->temporary_config_values, 30, ctx);
	for (i = 0; i < NUM_GEN2_LIGHTHOUSES; i++)
		init_config_group(&ctx->lh_config[i], 10, ctx);

	// Process command-line parameters.
	char *const *av = argv + 1;
	char *const *argvend = argv + argc;
	int list_for_autocomplete = 0;
	const char * autocomplete_match[3] = { 0, 0, 0};
	int showhelp = 0;
	bool showversion = false;
	for (; av < argvend; av++) {
		if ((*av)[0] == '-') {
			const char *vartoupdate = 0;

			switch ((*av)[1]) {
			case '-':
				vartoupdate = &(*av)[2];
				break;
			case 'h':
				showhelp = 1;
				break;
			case 'p':
				vartoupdate = "poser";
				break;
			case 'l':
				vartoupdate = "lighthousecount";
				break;
			case 'm':
				if( av + 1 < argvend ) autocomplete_match[0] = *(av+1);
				if( av + 2 < argvend ) autocomplete_match[1] = *(av+2);
				if( av + 3 < argvend ) autocomplete_match[2] = *(av+3);
				list_for_autocomplete = 1;
				av = argvend; //Eject immediately after processing -m
				break;
			case 'c':
				vartoupdate = "configfile";
				break;
			case 'v':
				vartoupdate = "v";
				break;
			default:
				break;
				// fprintf(stderr, "Error: unknown short parameter '%s'\n", *av);
				// showhelp = 1;
			}

			if (vartoupdate) {
				const char *name = vartoupdate;
				bool isLast = av + 1 == argvend;
				bool nextIsOption = !isLast && av[1][0] == '-';
				if (nextIsOption) {
					int n = atoi(*(av + 1));
					nextIsOption = n == 0;
				}
				bool flagArgument = isLast || nextIsOption;

				if (strcmp(name, "help") == 0) {
					showhelp = 1;
				} else if (strcmp(name, "version") == 0) {
					showversion = 1;
				} else if (flagArgument) {
					bool value = strncmp("no-", name, 3) != 0;
					if (value == 0) {
						name += 3; // Skip "no-"
					}
					survive_configi(ctx, name, SC_OVERRIDE | SC_SET, value);
				} else {
					const char *value = *(av + 1);
					survive_configs(ctx, name, SC_OVERRIDE | SC_SET, value);
					av++;
				}
			}
		}
	}

	survive_process_env(ctx, false);

	const char *log_file = survive_configs(ctx, "log", SC_GET, 0);
	int record_to_stdout = survive_configi(ctx, "record-stdout", SC_GET, 0);
	if (log_file || record_to_stdout)
		disable_colorization = true;
	ctx->log_target = log_file ? fopen(log_file, "w") : stdout;

	bool user_set_configfile = survive_config_is_set(ctx, "configfile");

	if (!user_set_configfile) {
		const char *playback_config_prefix_fields[] = {"playback", "usbmon-playback", "simulator", 0};
		find_correct_config_file(ctx, playback_config_prefix_fields);
	}

	ctx->log_level = survive_configi(ctx, "v", SC_SETCONFIG, 0);

	const char *init_config = survive_configs(ctx, "init-configfile", SC_GET, 0);
	char config_path[FILENAME_MAX];
	if (init_config == 0) {
		survive_config_file_path(ctx, config_path);
	} else {
		strncpy(config_path, init_config, FILENAME_MAX - 1);
		SV_INFO("Initial config file is %s", init_config);
	}
	config_read(ctx, config_path);
	SV_VERBOSE(5, "libsurvive version %s (backend %s)", survive_build_tag(), cnMatrixBackend());
	SV_VERBOSE(5, "Config file is %.512s", config_path);

	SV_VERBOSE(100, "Args: ");
	for(int i = 0;i < argc;i++) {
		SV_VERBOSE(100, "\t'%s'",argv[i]);
	}
	survive_process_env(ctx, true);

	const char *record_config_prefix_fields[] = {"record", "usbmon-record", 0};
	if (!user_set_configfile && find_correct_config_file(ctx, record_config_prefix_fields)) {
		survive_config_file_path(ctx, config_path);
		SV_VERBOSE(5, "Config file switched to %.512s", config_path);
		config_save(ctx);
	}
	SurviveContext_attach_config(ctx, ctx);

	ctx->lh_version = -1;
	ctx->lh_version_configed = survive_configi(ctx, "configed-lighthouse-gen", SC_GET, 0) - 1;
	ctx->lh_version_forced = survive_configi(ctx, "lighthouse-gen", SC_GET, 0) - 1;
	ctx->floor_offset = survive_configf(ctx, "floor-offset", SC_GET, 0);
	ctx->activeLighthouses = 0;

	pctx->callbackStatsTimeBetween = survive_configf(ctx, "output-callback-stats", SC_GET, 0.0);

	const char *whitelist = survive_configs(ctx, "use-lighthouses", SC_GET, "");
    bool use_whitelist = (whitelist && whitelist[0] != 0);
    if(use_whitelist) {
        SV_INFO("Using lighthouse whitelist: %s", whitelist);
    }

	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		// config_read_lighthouse returns true if it found a config
        bool has_config = config_read_lighthouse(ctx->lh_config, &(ctx->bsd[i]), i);
		if (has_config) {
			if (ctx->bsd[i].mode >= 0 && ctx->bsd[i].mode < 16)
				ctx->bsd_map[ctx->bsd[i].mode] = i;
			ctx->activeLighthouses++;
			SV_VERBOSE(50, "Adding LH %d mode: %d id: %08x", i, ctx->bsd[i].mode, (unsigned)ctx->bsd[i].BaseStationID);
		}
		char buffer[128] = {0};
		sprintf(buffer, "lighthouse-%d-disable", i);
		if (ctx->bsd[i].disable = survive_configi(ctx, buffer, SC_GET, 0)) {
			SV_WARN("Disabling LH %d", i);
		}
		if (use_whitelist && has_config && !ctx->bsd[i].disable) {
            char id_str[16] = {0};
            sprintf(id_str, "%X", ctx->bsd[i].BaseStationID); // Use ID from config
            
            // If ID is NOT in list, disable it
            if (strstr(whitelist, id_str) == NULL) {
                SV_WARN("Lighthouse %s (idx %d) is not in the whitelist. Disabling.", id_str, i);
                ctx->bsd[i].disable = 1;
            } else {
                SV_INFO("Lighthouse %s (idx %d) is in whitelist. Enabling.", id_str, i);
            }
        }
        
        // If whitelist is active but this LH has no config, disable it until OOTX can identify it.
        if (use_whitelist && !has_config) {
             ctx->bsd[i].disable = 1;
             SV_VERBOSE(50, "LH %d has no config, disabling pending OOTX identification.", i);
        }
		ctx->bsd[i].tracker = SV_MALLOC(sizeof(struct SurviveKalmanLighthouse));
		survive_kalman_lighthouse_init(ctx->bsd[i].tracker, ctx, i);
	};

	if( list_for_autocomplete )
	{
		const char * lastparam = (autocomplete_match[2]==0)?autocomplete_match[1]:autocomplete_match[2];
		const char * matchingparam = (autocomplete_match[2]==0)?0:autocomplete_match[1];

		// First see if any of the parameters perfectly match a config item, if so print some help.
		const char * checkconfig = matchingparam;
		if( matchingparam == 0 ) checkconfig = lastparam;

		if (checkconfig && strlen(checkconfig) > 2 && survive_print_help_for_parameter(ctx, checkconfig + 2)) {
			exit(0);
		}

		if (lastparam && strstr(lastparam, "poser"))
			PrintMatchingDrivers("Poser", matchingparam);
		else if (lastparam && strstr(lastparam, "disambiguator"))
			PrintMatchingDrivers("Disambiguator", matchingparam);
		else
		{
			printf( "-h -m -p -l -c " );
			survive_print_known_configs( ctx, 0 );
		}
		printf( "\n" );
		exit( 0 );
	}

	if( showhelp )
	{
		// Can't use SV_GENERAL_ERROR here since we don't have a context to send to yet.
		fprintf(stderr, "libsurvive version %s\n", survive_build_tag());
		fprintf(stderr, "Available flags:\n");
		fprintf(stderr, " -h                      - shows help.\n");
		fprintf(stderr, " -m                      - list parameters, for autocomplete.\n");
		fprintf(stderr, " -p [poser]              - use a specific poser.\n");
		fprintf(stderr, " -l [lighthouse count]   - use a specific number of lighthouses.\n");
		fprintf(stderr, " -c [config file]        - set config file\n\n");

		survive_print_known_configs( ctx, 1 );
		return 0;
	}
	if (showversion) {
		fprintf(stderr, "libsurvive version %s (backend: %s)\n", survive_build_tag(), cnMatrixBackend());
		return 0;
	}
	return ctx;
}

survive_timecode survive_timecode_difference(survive_timecode most_recent, survive_timecode least_recent) {
	survive_timecode diff = 0;
	if (most_recent > least_recent) {
		diff = most_recent - least_recent;
	} else {
		diff = least_recent - most_recent;
	}

	if (diff > 0xFFFFFFFF / 2)
		return 0xFFFFFFFF - diff;
	return diff;
}

survive_driver_fn GetDriverByConfig(SurviveContext *ctx, const char *name, const char *configname,
									const char *configdef) {
	const char *Preferred = survive_configs(ctx, configname, SC_SETCONFIG, configdef);
	const char *DriverName = 0;
	const char *picked = 0;
	int i = 0;
	survive_driver_fn func = 0;
	int prefixLen = strlen(name);

	SV_VERBOSE(1, "Available %ss:", name);
	while ((DriverName = GetDriverNameMatching(name, i++))) {
		survive_driver_fn p = GetDriver(DriverName);

		bool match = strcmp(DriverName, Preferred) == 0 || strcmp(DriverName + prefixLen, Preferred) == 0;
		SV_VERBOSE(1, "\t%c%s", match ? '*' : ' ', DriverName + prefixLen);
		if (!func || match) {
			func = p;
			picked = (DriverName + prefixLen);
		}
	}
	if (!func) {
		SV_WARN("Error.  Cannot find any valid %s.", name);
		return 0;
	}

	SV_VERBOSE(1, "Totals %d %ss.", i - 1, name);
	SV_VERBOSE(1, "Using '%s' for %s", picked, configname);

	return func;
}
static inline SurviveDeviceDriverReturn callDriver(SurviveContext *ctx, const char *DriverName, char *buffer) {
	DeviceDriver dd = (DeviceDriver)GetDriver(DriverName);
	if (dd == 0) {
		return -1;
	}

	SurviveDeviceDriverReturn r = dd(ctx);
	if (r < 0) {
		SV_WARN("Driver %s reports status %d", DriverName + strlen("DriverReg"), r);
		return r;
	}
	else {
		strcat(buffer, DriverName + strlen("DriverReg"));
		strcat(buffer, ", ");
	}
	return r;
}

static void warn_missing_drivers(SurviveContext *ctx, const char* name) {
  int manually_enabled = survive_config_is_set(ctx, name);
  char buffer[64];
  snprintf(buffer, 64, "DriverReg%s", name);
  const void* driver = GetDriverNameMatching(buffer, 0);
  if(manually_enabled && driver == 0) {
    SV_WARN("Could not find manually specified driver '%s'. Please make sure it is configured to build (eg ENABLE_driver_%s is 'ON' in CMakeCache.txt) and has all required dependencies. Run with an environment variable `SURVIVE_PLUGIN_DEBUG=1` for information on the plugin search path(s).", name, name);
    SV_INFO("Available drivers:");
    ListDrivers();
  }
}

int survive_startup(SurviveContext *ctx) {
	ctx->state = SURVIVE_RUNNING;

	survive_install_recording(ctx);

	// initialize the button queue
	memset(&(ctx->buttonQueue), 0, sizeof(ctx->buttonQueue));
	ctx->buttonQueue.buttonservicesem = OGCreateSema();

	// start the thread to process button data
	ctx->buttonservicethread = OGCreateThread(button_servicer, "Button service", ctx);

	PoserCB PreferredPoserCB = (PoserCB)GetDriverByConfig(ctx, "Poser", "poser", "MPFIT");
	ctx->lightcapproc = GetDriverByConfig(ctx, "Disambiguator", "disambiguator", "StateBased");

	const char *DriverName;

	warn_missing_drivers(ctx, "openvr");
	warn_missing_drivers(ctx, "playback");
	warn_missing_drivers(ctx, "usbmon");
	
	bool loadDefaultDriver = true;
	char buffer[1024] = "Loaded drivers: ";
	{
		int i = 0;
		while ((DriverName = GetDriverNameMatching("DriverReg", i++))) {
			char driverNameSuffix[256] = {0};
			char *driverNameSuffix_p = driverNameSuffix;
			for (const char *c = DriverName + strlen("DriverReg"); *c; c++) {
				*driverNameSuffix_p = tolower(*c);
				if (*driverNameSuffix_p == '_')
					*driverNameSuffix_p = '-';
				driverNameSuffix_p++;
			}

			int enabled = survive_configi(ctx, driverNameSuffix, SC_GET, 0) != 0 ||
						  survive_configs(ctx, driverNameSuffix, SC_GET, 0) != 0;
			int manually_enabled = survive_config_is_set(ctx, driverNameSuffix);

			// We load htcvive later if nothing else was loaded
			if (strcmp("htcvive", driverNameSuffix) != 0) {
				if (enabled) {
					int driverReturn = callDriver(ctx, DriverName, buffer); // == SURVIVE_DRIVER_NORMAL
					// Auto drivers dont preclude the default HTC driver
					if (manually_enabled && driverReturn != SURVIVE_DRIVER_PASSIVE) {
						loadDefaultDriver = false; 
					}
				}
			}
		}
	}

	if (ctx->currentError != SURVIVE_OK) {
		return ctx->currentError;
	}

	// Load the vive driver by default, even if not enabled as a flag
	if (loadDefaultDriver == true || survive_configi(ctx, "htcvive", SC_GET, 0) != 0) {
		callDriver(ctx, "DriverRegHTCVive", buffer);
	}

	if (ctx->currentError != SURVIVE_OK) {
		return ctx->currentError;
	}

	buffer[strlen(buffer) - 2] = 0;
	SV_INFO("%s", buffer);

	FLT playback_factor = survive_configf(ctx, "playback-factor", SC_GET, 1.);
	bool use_async_posers = survive_configi(ctx, THREADED_POSERS_TAG, SC_GET, 0) && playback_factor != 0.0;
	if (use_async_posers) {
		for (int i = 0; i < ctx->objs_ct; i++) {
			*survive_object_plugin_data(ctx->objs[i], survive_threaded_poser_fn) =
				survive_create_threaded_poser(ctx->objs[i], PreferredPoserCB);
		}
		ctx->PoserFn = survive_threaded_poser_fn;
	} else {
		ctx->PoserFn = PreferredPoserCB;
	}

	// saving the config extra to make sure that the user has a config file they can change.
	config_save(ctx);

	int calibrateMandatory = survive_configi(ctx, "force-calibrate", SC_GET, 0);
	if (calibrateMandatory) {
		SV_INFO("Force calibrate flag set -- clearing position on all lighthouses");
		ctx->floor_offset = 0;
		for (int i = 0; i < ctx->activeLighthouses; i++) {
			ctx->bsd[i].PositionSet = 0;
			ctx->bsd[i].Pose = (SurvivePose){0};
		}
	}

	FLT random_noise = survive_configf(ctx, "random-bsd-noise", SC_GET, -1);
	if (random_noise > 0) {
		for (int i = 0; i < ctx->activeLighthouses; i++) {
			for (int j = 0; j < 3; j++)
				ctx->bsd[i].Pose.Pos[j] += linmath_normrand(0, random_noise);
			for (int j = 0; j < 4; j++)
				ctx->bsd[i].Pose.Rot[j] += linmath_normrand(0, random_noise * .1);
			quatnormalize(ctx->bsd[i].Pose.Rot, ctx->bsd[i].Pose.Rot);
		}
	}

	int ootxMandatory = survive_configi(ctx, "force-ootx", SC_GET, 0);
	if (ootxMandatory) {
		SV_INFO("Force ootx flag set -- clearing ootx on all lighthouses");
		for (int i = 0; i < ctx->activeLighthouses; i++) {
			ctx->bsd[i].OOTXSet = 0;
			memset(ctx->bsd[i].fcal, 0, sizeof(ctx->bsd[i].fcal));
		}
	}

	int ootxuse = survive_configi(ctx, "use-ootx", SC_GET, 1);
	if (!ootxuse) {
		for (int i = 0; i < ctx->activeLighthouses; i++) {
			memset(ctx->bsd[i].fcal, 0, sizeof(ctx->bsd[i].fcal));
		}
	}

	const char *steamvr_path = survive_configs(ctx, "steamvr-calibration", SC_GET, "");
	if (steamvr_path == 0 || steamvr_path[0] != 0) {
		char configpath[1024] = {0};
		if (steamvr_path == 0) {
			const char *home = getenv("HOME");
			snprintf(configpath, sizeof(configpath) - 1, "%s/.steam/steam/config/lighthouse/lighthousedb.json", home);
			steamvr_path = configpath;
		}
		SV_VERBOSE(10, "Attempting to load lighthouse db from %s", steamvr_path);
		survive_load_steamvr_lighthousedb_from_file(ctx, steamvr_path);
		config_save(ctx);
	}

	// If lighthouse positions are known, broadcast them
	for (int i = 0; i < ctx->activeLighthouses; i++) {
		if (ctx->bsd[i].OOTXSet) {
			SURVIVE_INVOKE_HOOK(ootx_received, ctx, i);
		}
		if (ctx->bsd[i].PositionSet) {
			SURVIVE_INVOKE_HOOK(raw_lighthouse_pose, ctx, i, &ctx->bsd[i].Pose);
		}
	}

	if (ctx->objs_ct == 0 && ctx->driver_ct == 0) {
		SV_ERROR(SURVIVE_ERROR_NO_TRACKABLE_OBJECTS, "No trackable objects provided and no drivers are registered.");
	}

	return 0;
}
datalog_process_func survive_default_datalog_process = 0;

#define SURVIVE_HOOK_FN_DEF(hook)                                                                                      \
	SURVIVE_EXPORT void survive_install_##hook##_fn(SurviveContext *ctx, hook##_func fbp) {                            \
		ctx->hook##function = fbp ? fbp : survive_default_##hook;                                                      \
	}
#define SURVIVE_HOOK_PROCESS_DEF(hook)                                                                                 \
	SURVIVE_EXPORT hook##_process_func survive_install_##hook##_fn(SurviveContext *ctx, hook##_process_func fbp) {     \
		hook##_process_func rtn = ctx->hook##proc;                                                                     \
		ctx->hook##proc = fbp ? fbp : survive_default_##hook##_process;                                                \
		return rtn;                                                                                                    \
	}
#define SURVIVE_HOOK_FEEDBACK_DEF(hook)                                                                                \
	SURVIVE_EXPORT hook##_feedback_func survive_install_##hook##_fn(SurviveContext *ctx, hook##_feedback_func fbp) {   \
		hook##_feedback_func rtn = ctx->hook##function;                                                                \
		ctx->hook##function = fbp ? fbp : survive_default_##hook;                                                      \
		return rtn;                                                                                                    \
	}

#include "ootx_decoder.h"
#include "survive_hooks.h"
#include "survive_kalman_tracker.h"

void survive_default_disconnect_process(struct SurviveObject *so) {
	SurviveContext *ctx = so->ctx;
	survive_recording_disconnect_process(so);
	SV_VERBOSE(10, "Disconnecting device %s at %.7f", survive_colorize(so->codename), survive_run_time(ctx));
}
void survive_default_new_object_process(SurviveObject *so) {}
int survive_add_object(SurviveContext *ctx, SurviveObject *obj) {

	if (survive_get_so_by_name(ctx, obj->codename)) {
		SV_ERROR(SURVIVE_ERROR_INVALID_CONFIG, "Object named %s already exists", obj->codename);
		return 0;
	}

	SV_INFO("Adding tracked object %s from %s", survive_colorize(obj->codename), survive_colorize(obj->drivername));
	int oldct = ctx->objs_ct;
	ctx->objs = SV_REALLOC(ctx->objs, sizeof(SurviveObject *) * (oldct + 1));
	ctx->objs[oldct] = obj;
	ctx->objs_ct = oldct + 1;

	SURVIVE_INVOKE_HOOK_SO(new_object, obj);

	return 0;
}

void survive_remove_object(SurviveContext *ctx, SurviveObject *obj) {
	int obj_idx = 0;
	for (obj_idx = 0; obj_idx < ctx->objs_ct; obj_idx++) {
		if (ctx->objs[obj_idx] == obj)
			break;
	}

	if (obj_idx == ctx->objs_ct) {
		SV_INFO("Warning: Tried to remove un-added object %p(%s)", (void *)obj, obj->codename);
		return;
	}

	// Swap the last item into this items slot; this assumes order doesn't matter in this list
	if (obj_idx != ctx->objs_ct - 1) {
		ctx->objs[obj_idx] = ctx->objs[ctx->objs_ct - 1];
	}

	ctx->objs_ct--;

	// Blank out the spot; but this is only really necessary for diagnostic reasons -- presumably no one will ever read
	// past the end of the list
	ctx->objs[ctx->objs_ct] = 0;

	SV_INFO("Removing tracked object %s from %s", obj->codename, obj->drivername);
	free(obj);
}

const void *survive_get_driver(const SurviveContext *ctx, DeviceDriverCb pollFn) {
	for (int i = 0; i < ctx->driver_ct; i++) {
		if (ctx->driverpolls[i] == pollFn)
			return ctx->drivers[i];
	}
	return 0;
}

const void *survive_get_driver_by_closefn(const SurviveContext *ctx, DeviceDriverCb closeFn) {
	for (int i = 0; i < ctx->driver_ct; i++) {
		if (ctx->drivercloses[i] == closeFn)
			return ctx->drivers[i];
	}
	return 0;
}

void survive_reset_lighthouse_position(SurviveContext *ctx, int bsd_idx) {
	ctx->bsd[bsd_idx].PositionSet = false;
	ctx->bsd[bsd_idx].variance = (SurviveVelocity){0};
	survive_kalman_lighthouse_reset(ctx->bsd[bsd_idx].tracker);
};

SURVIVE_EXPORT const SurvivePose *survive_get_lighthouse_true_position(const SurviveContext *ctx, int bsd_idx) {
	if (ctx->bsd[bsd_idx].true_pos_time != 0) {
		return &ctx->bsd[bsd_idx].true_pos;
	}
	return &ctx->bsd[bsd_idx].Pose;
}
SURVIVE_EXPORT const SurvivePose *survive_get_lighthouse_position(const SurviveContext *ctx, int bsd_idx) {
	assert(bsd_idx >= 0);
	if (ctx->bsd[bsd_idx].true_pos_time != 0) {
		SurviveContext *mctx = (SurviveContext *)ctx;
		FLT t_diff = ctx->bsd[bsd_idx].true_pos_time - ctx->bsd[bsd_idx].old_pos_time;
		FLT t = (survive_run_time(ctx) - ctx->bsd[bsd_idx].old_pos_time) / t_diff;
		if (t > 1)
			t = 1;
		assert(t >= 0);
		if (t >= 0) {
			PoseSlerp(&mctx->bsd[bsd_idx].Pose, &ctx->bsd[bsd_idx].old_pos, &ctx->bsd[bsd_idx].true_pos, t);
		}
		if (t == 1) {
			mctx->bsd[bsd_idx].true_pos_time = 0;
			mctx->bsd[bsd_idx].old_pos_time = NAN;
		}
		SurvivePose p = ctx->bsd[bsd_idx].Pose;
		p.Pos[2] -= ctx->floor_offset;
		survive_recording_lighthouse_process(mctx, bsd_idx, &p);
	}
	return &ctx->bsd[bsd_idx].Pose;
}

void survive_reset_lighthouse_positions(SurviveContext *ctx) {
	// survive_get_ctx_lock(ctx);
	SV_VERBOSE(100, "survive_reset_lighthouse_positions called");
	ctx->floor_offset = 0;
	for (int i = 0; i < ctx->activeLighthouses; i++) {
		survive_reset_lighthouse_position(ctx, i);
	}
	for (int i = 0; i < ctx->objs_ct; i++) {
		survive_kalman_tracker_lost_tracking(ctx->objs[i]->tracker, false);
	}
	// survive_release_ctx_lock(ctx);
}

void survive_add_driver(SurviveContext *ctx, void *payload, DeviceDriverCb poll, DeviceDriverCb close) {
	int oldct = ctx->driver_ct;
	ctx->drivers = SV_REALLOC(ctx->drivers, sizeof(void *) * (oldct + 1));
	ctx->driverpolls = SV_REALLOC(ctx->driverpolls, sizeof(DeviceDriverCb *) * (oldct + 1));
	ctx->drivercloses = SV_REALLOC(ctx->drivercloses, sizeof(DeviceDriverCb *) * (oldct + 1));
	ctx->drivers[oldct] = payload;
	ctx->driverpolls[oldct] = poll;
	ctx->drivercloses[oldct] = close;
	ctx->driver_ct = oldct + 1;
}
struct survive_threaded_driver {
	void *driver_data;
	DeviceDriverCb thread_fn, close_fn;

	bool keep_running;
	og_thread_t thread;
};

static int threaded_driver_poll(struct SurviveContext *ctx, void *_driver) {
	struct survive_threaded_driver *driver = _driver;
	if (driver->keep_running == false)
		return -1;
	return 0;
}

static int threaded_driver_close(struct SurviveContext *ctx, void *_driver) {
	struct survive_threaded_driver *driver = _driver;
	driver->keep_running = false;
	survive_release_ctx_lock(ctx);
	OGJoinThread(driver->thread);
	survive_get_ctx_lock(ctx);
	driver->close_fn(ctx, driver->driver_data);

	free(driver);
	return 0;
}

bool *survive_add_threaded_driver(SurviveContext *ctx, void *_driver, const char *name, void *(routine)(void *),
								  DeviceDriverCb close) {
	struct survive_threaded_driver *driver = SV_CALLOC(sizeof(struct survive_threaded_driver));
	driver->driver_data = _driver;
	driver->close_fn = close;

	driver->keep_running = true;
	driver->thread = OGCreateThread(routine, name, _driver);

	survive_add_driver(ctx, driver, threaded_driver_poll, threaded_driver_close);
	return &driver->keep_running;
}

int survive_haptic(SurviveObject *so, FLT freq, FLT amp, FLT duration) {
	if (NULL == so || NULL == so->haptic) {
		return -404;
	}

	return so->haptic(so, freq, amp, duration);
}

void survive_output_callback_stats(SurviveContext *ctx) {
	SV_VERBOSE(10, "Callback statistics:");
#define SURVIVE_HOOK_PROCESS_DEF(hook)                                                                                 \
	SV_VERBOSE(10, "\t%-20s cnt: %7d avg time: %.7fms max time: %.7fms cnt over 1ms: %5d(%.7f%%)", #hook,               \
			   ctx->hook##_call_cnt, 1000. * ctx->hook##_call_time / (1e-5 + ctx->hook##_call_cnt),                    \
			   ctx->hook##_max_call_time * 1000., ctx->hook##_call_over_cnt,                                           \
			   ctx->hook##_call_over_cnt / (FLT)(ctx->hook##_call_cnt + .0001));                                       \
	ctx->hook##_call_cnt = 0;                                                                                          \
	ctx->hook##_max_call_time = ctx->hook##_call_time = 0.;                                                            \
	ctx->hook##_call_over_cnt = 0;
#include "survive_hooks.h"
}

void survive_close(SurviveContext *ctx) {
	const char *DriverName;
	int r = 0;

	ctx->state = SURVIVE_CLOSING;

	// unlock/ post to button service semaphore so the thread can kill itself
	OGUnlockSema(ctx->buttonQueue.buttonservicesem);
	OGJoinThread(ctx->buttonservicethread);
	OGDeleteSema(ctx->buttonQueue.buttonservicesem);
	ctx->buttonQueue.buttonservicesem = 0;

	SV_VERBOSE(10, "Button events processed: %d", (int)ctx->buttonQueue.processed_events);

	while ((DriverName = GetDriverNameMatching("DriverUnreg", r++))) {
		DeviceDriver dd = (DeviceDriver)GetDriver(DriverName);
		SV_INFO("De-registering driver %s", DriverName);
		r = dd(ctx);
		SV_INFO("Driver %s reports status %d", DriverName, r);
	}

	for (int i = 0; i < ctx->driver_ct; i++) {
		if (ctx->drivercloses[i]) {
			ctx->drivercloses[i](ctx, ctx->drivers[i]);
		} else {
			free(ctx->drivers[i]);
		}
	}

	for (int i = 0; i < ctx->objs_ct; i++) {
		PoserData pd;
		pd.pt = POSERDATA_DISASSOCIATE;
		if (ctx->PoserFn) {
			ctx->PoserFn(ctx->objs[i], &pd);
		}
		SURVIVE_INVOKE_HOOK_SO(lightcap, ctx->objs[i], 0);
	}
	ctx->PoserFn = 0;

	config_save(ctx);

	while (ctx->objs_ct) {
		size_t objs_ct = ctx->objs_ct;
		survive_destroy_device(ctx->objs[0]);
		assert(objs_ct != ctx->objs_ct);
	}

	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		survive_ootx_free_decoder_context(ctx, i);
		survive_kalman_lighthouse_free(ctx->bsd[i].tracker);
		ctx->bsd[i].tracker = 0;
	}

	survive_output_callback_stats(ctx);

	survive_destroy_recording(ctx);

	SurviveContext_detach_config(ctx, ctx);

	destroy_config_group(ctx->global_config_values);
	destroy_config_group(ctx->temporary_config_values);

	for (int lh = 0; lh < NUM_GEN2_LIGHTHOUSES; lh++) {
		ootx_decoder_context *decoderContext = ctx->bsd[lh].ootx_data;
		if (decoderContext) {
			ootx_free_decoder_context(decoderContext);
			free(decoderContext);
		}
		destroy_config_group(ctx->lh_config + lh);
	}

	struct SurviveContext_private *pctx = ctx->private_members;
	OGDeleteSema(pctx->poll_sema);
	free(pctx);

	free(ctx->objs);
	free(ctx->drivers);
	free(ctx->driverpolls);
	free(ctx->drivercloses);
	free(ctx->global_config_values);
	free(ctx->temporary_config_values);
	free(ctx->lh_config);
	free(ctx->recptr);

	free(ctx);
}

int survive_poll(struct SurviveContext *ctx) {
	int i, r;

	uint64_t timeStart = OGGetAbsoluteTimeMS();

	if (ctx->state == SURVIVE_STOPPED) {
		r = survive_startup(ctx);
		if (r) {
			return r;
		}
	}

	if (ctx->currentError != SURVIVE_OK) {
		return ctx->currentError;
	}

	int oldct = ctx->driver_ct;

	for (i = 0; i < oldct; i++) {
		if (ctx->driverpolls[i]) {
			r = ctx->driverpolls[i](ctx, ctx->drivers[i]);
			if (r) {
				SV_WARN("Driver reported %d", r);
				return r;
			}
		}
	}

	survive_release_ctx_lock(ctx);
	if (ctx->poll_min_time_ms > 0) {
		uint64_t timeNow = OGGetAbsoluteTimeMS();
		if ((timeStart + ctx->poll_min_time_ms) > timeNow) {
			uint64_t sleepTime = (timeStart + ctx->poll_min_time_ms) - timeNow;
			OGUSleep(sleepTime * 1000);
		}
	}

	struct SurviveContext_private *pctx = ctx->private_members;
	if (pctx->callbackStatsTimeBetween != 0.) {
		FLT now = OGRelativeTime();
		if (pctx->lastCallbackStats + pctx->callbackStatsTimeBetween < now) {
			survive_output_callback_stats(ctx);
			pctx->lastCallbackStats = now;
		}
	}
	survive_get_ctx_lock(ctx);

	return 0;
}

struct SurviveObject *survive_get_so_by_name(struct SurviveContext *ctx, const char *name) {
	int i;
	for (i = 0; i < ctx->objs_ct; i++) {
		if (strcmp(ctx->objs[i]->codename, name) == 0)
			return ctx->objs[i];
	}
	return 0;
}

#ifdef NOZLIB

#include <puff.h>

int survive_simple_inflate(struct SurviveContext *ctx, const uint8_t *input, int inlen, uint8_t *output, int outlen) {
	// Tricky: we actually get 2 bytes of data on the front.  I don't know what it's for. 0x78 0x9c - puff doesn't deal
	// with it well.
	unsigned long ol = outlen;
	unsigned long il = inlen - 2;
	int ret = puff(output, &ol, input + 2, &il);
	if (ret == 0)
		return ol;
	else {
		SV_INFO("puff returned error code %d\n", ret);
		return -5;
	}
}

#else

#include <zlib.h>

int survive_simple_inflate(struct SurviveContext *ctx, const uint8_t *input, int inlen, uint8_t *output, int outlen) {
	z_stream zs; // Zlib stream.  May only be used by configuration at beginning and by USB thread periodically.
	memset(&zs, 0, sizeof(zs));
	inflateInit(&zs); /// Consider checking error

	// XXX: Todo: If we find that this is not useful past the beginning (nix this here and move into the configuration
	// getter)
	zs.avail_in = inlen;
	zs.next_in = (z_const Bytef *)input;
	zs.avail_out = outlen;
	zs.next_out = output;

	int errorCode = inflate(&zs, Z_FINISH);
	if (errorCode != Z_STREAM_END) {
		SV_WARN("survive_simple_inflate could not inflate: %s %d (stream written to 'libz_error.stream')", zs.msg,
				errorCode);

		char fstname[128] = "libz_error.stream";
		FILE *f = fopen(fstname, "wb");
		fwrite(input, inlen, 1, f);
		fclose(f);

		inflateEnd(&zs);
		return -1;
	}
	int len = zs.total_out;
	inflateEnd(&zs);
	return len;
}

#endif

const SurvivePose *survive_object_last_imu2world(const SurviveObject *so) { return &so->OutPoseIMU; }
const char *survive_object_codename(SurviveObject *so) { return so->codename; }

const char *survive_object_drivername(SurviveObject *so) { return so->drivername; }
int8_t survive_object_charge(SurviveObject *so) { return so->charge; }
bool survive_object_charging(SurviveObject *so) { return so->charging; }

SURVIVE_EXPORT SurvivePluginData *survive_object_plugin_data(SurviveObject *so, SurvivePluginKey k) {
	for (size_t i = 0; i < so->PluginDataEntries_cnt; i++) {
		if (so->PluginDataEntries[i].key == k) {
			return &so->PluginDataEntries[i].data;
		}
	}

	if (so->PluginDataEntries_cnt >= so->PluginDataEntries_space) {
		so->PluginDataEntries_space += 8;
		so->PluginDataEntries =
			SV_REALLOC(so->PluginDataEntries, sizeof(SurvivePluginPair) * so->PluginDataEntries_space);
	}

	so->PluginDataEntries_cnt++;
	so->PluginDataEntries[so->PluginDataEntries_cnt - 1].key = k;
	so->PluginDataEntries[so->PluginDataEntries_cnt - 1].data = 0;
	return &so->PluginDataEntries[so->PluginDataEntries_cnt - 1].data;
}

const SurvivePose *survive_object_pose(SurviveObject *so) { return &so->OutPose; }

int8_t survive_object_sensor_ct(SurviveObject *so) { return so->sensor_ct; }
const FLT *survive_object_sensor_locations(SurviveObject *so) { return so->sensor_locations; }
const FLT *survive_object_sensor_normals(SurviveObject *so) { return so->sensor_normals; }

inline void survive_find_ang_velocity(SurviveAngularVelocity out, FLT tdiff, const LinmathQuat from,
									  const LinmathQuat to) {
	LinmathQuat vDiff = {1.};
	// quatfind(vDiff, comparison_pose.Rot.v, tracker->last_pose.Rot.v);
	quatfind(vDiff, from, to);

	quattoaxisanglemag(out, vDiff);
	scale3d(out, out, 1. / tdiff);
}
inline void survive_apply_ang_velocity(LinmathQuat out, const SurviveAngularVelocity v, FLT t, const LinmathQuat t0) {
	SurviveAngularVelocity vel;
	scale3d(vel, v, t);

	LinmathQuat rot_change;
	quatfromaxisanglemag(rot_change, vel);
	quatrotateabout(out, rot_change, t0);
	quatnormalize(out, out);
}
inline void survive_apply_ang_velocity_aa(LinmathAxisAngle out, const SurviveAngularVelocity v, FLT t,
										  const LinmathAxisAngle t0) {
	SurviveAngularVelocity vel;
	scale3d(vel, v, t);
	axisanglerotateabout(out, vel, t0);
}

/***
 * We add a small offset here so survive_run_tim can never be 0
 */
static double timestamp_in_s() { return OGGetAbsoluteTime() - OGStartTimeS() + 1e-3; }

double survive_run_time(const SurviveContext *ctx) {
	struct SurviveContext_private *pctx = ctx->private_members;
	if (pctx->runTimeFn) {
		return pctx->lastRunTime = pctx->runTimeFn(ctx, pctx->runTimeFnUser);
	}

	return pctx->lastRunTime = timestamp_in_s();
}

double survive_time_to_time_since_epoch(const SurviveContext *ctx, double t) { return t + OGStartTimeS(); }
double survive_run_time_since_epoch(const SurviveContext *ctx) { return survive_run_time(ctx) + OGStartTimeS(); }

double static_time(const SurviveContext *ctx, void *user) {
	struct SurviveContext_private *pctx = user;
	return pctx->lastRunTime;
}
void survive_install_run_time_fn(SurviveContext *ctx, survive_run_time_fn fn, void *user) {
	struct SurviveContext_private *pctx = ctx->private_members;
	if (fn == 0 && pctx->runTimeFn != 0) {
		pctx->runTimeFn = static_time;
		pctx->runTimeFnUser = pctx;
	} else {
		pctx->runTimeFn = fn;
		pctx->runTimeFnUser = user;
	}
}

const char *SurviveAxisStr(SurviveObjectSubtype objectSubtype, enum SurviveAxis b) {
	switch (objectSubtype) {
	case SURVIVE_OBJECT_SUBTYPE_INDEX_HMD:
		switch (b) {
		case SURVIVE_AXIS_IPD:
			return "IPD";
		case SURVIVE_AXIS_FACE_PROXIMITY:
			return "Face Proximity";
		}
		break;
	case SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R:
	case SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L:
	case SURVIVE_OBJECT_SUBTYPE_WAND:
		switch (b) {
		case SURVIVE_AXIS_TRACKPAD_FORCE:
			return "Trackpad force";
		case SURVIVE_AXIS_UNKNOWN:
			return "Unknown";
		case SURVIVE_AXIS_TRIGGER:
			return "Trigger";
		case SURVIVE_AXIS_TRACKPAD_X:
			return "Trackpad X";
		case SURVIVE_AXIS_TRACKPAD_Y:
			return "Trackpad Y";
		case SURVIVE_AXIS_JOYSTICK_X:
			return "Joystick X";
		case SURVIVE_AXIS_JOYSTICK_Y:
			return "Joystick Y";
		case SURVIVE_AXIS_MIDDLE_FINGER_PROXIMITY:
			return "Middle proximity";
		case SURVIVE_AXIS_RING_FINGER_PROXIMITY:
			return "Ring proximity";
		case SURVIVE_AXIS_PINKY_FINGER_PROXIMITY:
			return "Pinky proximity";
		case SURVIVE_AXIS_TRIGGER_FINGER_PROXIMITY:
			return "Trigger proximity";
		case SURVIVE_AXIS_GRIP_FORCE:
			return "Grip force";
		}
	default:
		break;
	}
	return 0;
}

const char *SurviveInputEventStr(enum SurviveInputEvent evt) {
	switch (evt) {
	case SURVIVE_INPUT_EVENT_NONE:
		return "None";
	case SURVIVE_INPUT_EVENT_BUTTON_DOWN:
		return "Button Down";
	case SURVIVE_INPUT_EVENT_BUTTON_UP:
		return "Button Up";
	case SURVIVE_INPUT_EVENT_AXIS_CHANGED:
		return "Axis Changed";
	case SURVIVE_INPUT_EVENT_TOUCH_DOWN:
		return "Touch Down";
	case SURVIVE_INPUT_EVENT_TOUCH_UP:
		return "Touch Up";
	default:
		break;
	}
	return 0;
}

const char *SurviveButtonsStr(SurviveObjectSubtype objectSubtype, enum SurviveButton b) {
	switch (objectSubtype) {
	case SURVIVE_OBJECT_SUBTYPE_INDEX_HMD:
		switch (b) {
		case SURVIVE_BUTTON_ON_FACE:
			return "On face";
		}
		break;
	case SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2:
	case SURVIVE_OBJECT_SUBTYPE_TRACKER:
	case SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R:
	case SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L:
	case SURVIVE_OBJECT_SUBTYPE_WAND:
		switch (b) {
		case SURVIVE_BUTTON_TRACKPAD:
			return "Trackpad";
		case SURVIVE_BUTTON_THUMBSTICK:
			return "Thumbstick";
		case SURVIVE_BUTTON_SYSTEM:
			return "System";
		case SURVIVE_BUTTON_A:
			return "A";
		case SURVIVE_BUTTON_B:
			return "B";
		case SURVIVE_BUTTON_TRIGGER:
			return "Trigger";
		case SURVIVE_BUTTON_MENU:
			return "Menu";
		case SURVIVE_BUTTON_GRIP:
			return "Grip";
		default:
			break;
		}
	default:
		break;
	}
	return 0;
}

const char *SurviveObjectTypeStr(SurviveObjectType t) {
	switch (t) {
	case SURVIVE_OBJECT_TYPE_HMD:
		return "HMD";
	case SURVIVE_OBJECT_TYPE_CONTROLLER:
		return "Controller";
	case SURVIVE_OBJECT_TYPE_OTHER:
		return "Other";
	default:
		break;
	}
	return "Unknown";
}
SURVIVE_EXPORT const char *SurviveObjectSubtypeStr(SurviveObjectSubtype t) {
	switch (t) {
	default:
		break;
	case SURVIVE_OBJECT_SUBTYPE_GENERIC:
		return "Generic";
	case SURVIVE_OBJECT_SUBTYPE_INDEX_HMD:
		return "Index HMD";
	case SURVIVE_OBJECT_SUBTYPE_WAND:
		return "Wand";
	case SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R:
		return "Knuckles(R)";
	case SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L:
		return "Knuckles(L)";
	case SURVIVE_OBJECT_SUBTYPE_TRACKER:
		return "Tracker";
	case SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2:
		return "Tracker 2";
	}
	return "Unknown";
}

uint32_t survive_hash(const uint8_t *data, size_t len) {
	// http://www.cse.yorku.ca/~oz/hash.html
	uint32_t hash = 5381;

	for (int i = 0; i < len; i++)
		hash = ((hash << 5) + hash) + data[i]; /* hash * 33 + c */

	return hash;
}
uint32_t survive_hash_str(const char *data) { return survive_hash((uint8_t *)data, strlen(data)); }
SURVIVE_EXPORT const char *survive_colorize(const char *str) {
	if (disable_colorization) {
		return str;
	}
#ifdef _WIN32
	return str;
#else
	static __thread int next_buffer = 0;
	static __thread char color_buffers[8][128] = {0};

	char *color_buffer = color_buffers[next_buffer++ % 8];
	size_t len = strlen(str);
	if (len > sizeof(color_buffers[0]) - 18 - 1)
		return str;

	size_t written_length =
		snprintf(color_buffer, sizeof(color_buffers[0]), SURVIVE_COLORIZED_FORMAT("%s"), SURVIVE_COLORIZED_STR(str));
	return color_buffer;
#endif
}

SURVIVE_EXPORT const char *survive_colorize_codename(const SurviveObject *so) {
	return survive_colorize(so ? so->codename : "unknown");
}
BaseStationCal *survive_basestation_cal(SurviveContext *ctx, int lh, int axis) {
	return &ctx->bsd[lh].fcal[axis];
	// return axis == 0 ? &ctx->bsd[lh].tracker->state.BSD0 : &ctx->bsd[lh].tracker->state.BSD1;
}
