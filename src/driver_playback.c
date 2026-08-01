#include <errno.h>
#include <math.h>

#include "os_generic.h"
#include "survive.h"

#include "survive_recording.h"
#include "survive_internal.h"

#include "survive_default_devices.h"

#include "survive_gz.h"

STATIC_CONFIG_ITEM(PLAYBACK_REPLAY_POSE, "playback-replay-pose", 'b', "Whether or not to output pose", 0)
STATIC_CONFIG_ITEM(PLAYBACK_REPLAY_EXTERNAL_POSE, "playback-replay-external-pose", 'b',
				   "Whether or not to output external pose", 0)
STATIC_CONFIG_ITEM(PLAYBACK, "playback", 's', "File to be used for playback if playing a recording.", 0)
STATIC_CONFIG_ITEM(PLAYBACK_FACTOR, "playback-factor", 'f',
				   "Time factor of playback -- 1 is run at the same timing as original, 0 is run as fast as possible.",
				   1.0f)
STATIC_CONFIG_ITEM(PLAYBACK_START_TIME, "playback-start-time", 'f', "Start time of playback", -1.0f)
STATIC_CONFIG_ITEM(PLAYBACK_TIME, "playback-time", 'f', "End time of playback", -1.0f)

STATIC_CONFIG_ITEM(PLAYBACK_RUN_TIME, "run-time", 'f', "How long to run for", -1.)


  
#ifdef _MSC_VER
typedef long ssize_t;
#define SSIZE_MAX LONG_MAX

ssize_t getdelim(char **lineptr, size_t *n, int delimiter, FILE *stream);
ssize_t getline(char **lineptr, size_t *n, FILE *stream);
#define RESTRICT_KEYWORD
#else
#define RESTRICT_KEYWORD restrict
#endif

ssize_t gzgetdelim(char **RESTRICT_KEYWORD lineptr, size_t *RESTRICT_KEYWORD n, int delimiter,
				   gzFile RESTRICT_KEYWORD stream);
ssize_t gzgetline(char **RESTRICT_KEYWORD lineptr, size_t *RESTRICT_KEYWORD n, gzFile RESTRICT_KEYWORD stream);

typedef struct SurvivePlaybackData {
    SurviveContext *ctx;
    const char *playback_dir;
    gzFile playback_file;
    int lineno;

	double time_start;
	double next_time_s;
	double time_now;
    FLT playback_factor;
	FLT playback_time;
	FLT playback_start_time;
	bool hasRawLight;
    bool hasSweepAngle;
	bool outputCalculatedPose, outputExternalPose;
    bool hasRawIMU;

	uint32_t total_sleep_time;
	bool *keepRunning;
} SurvivePlaybackData;

static double survive_playback_run_time(const SurviveContext *ctx, void *_sp) {
	const struct SurvivePlaybackData *sp = _sp;
	return sp->time_now;
}


static SurviveObject *find_or_warn(SurvivePlaybackData *driver, const char *dev) {
	SurviveContext *ctx = driver->ctx;
	SurviveObject *so = survive_get_so_by_name(driver->ctx, dev);
	if (!so) {
		static bool display_once = false;
		SurviveContext *ctx = driver->ctx;
		if (display_once == false) {
			SV_WARN("Could not find device named %s from lineno %d\r\n", dev, driver->lineno);
		}
		display_once = true;

		return 0;
	}
	return so;
}

static int parse_and_run_sweep(char *line, SurvivePlaybackData *driver) {
	if (driver->time_now < driver->playback_start_time)
		return 0;

	char dev[10];

	survive_channel channel;
	int sensor_id;
	survive_timecode timecode;
	uint8_t flag;

	int rr = sscanf(line, SWEEP_SCANF, SWEEP_SCANF_ARGS);
	if (rr != 5) {
		SurviveContext *ctx = driver->ctx;
		SV_WARN("Only got %d values for a sweep", rr);
		return -1;
	}

	SurviveObject *so = find_or_warn(driver, dev);
	if (!so) {
		return 0;
	}

	driver->hasSweepAngle = true;
	SURVIVE_INVOKE_HOOK_SO(sweep, so, channel, sensor_id, timecode, flag);
	return 0;
}

static int parse_and_run_sync(char *line, SurvivePlaybackData *driver) {
	if (driver->time_now < driver->playback_start_time)
		return 0;

	char dev[10];

	survive_channel channel;
	survive_timecode timecode;
	uint8_t ootx, gen;

	int rr = sscanf(line, SYNC_SCANF, SYNC_SCANF_ARGS);
	if (rr != 5) {
		SurviveContext *ctx = driver->ctx;
		SV_WARN("Only got %d values for a sync", rr);
		return -1;
	}

	SurviveObject *so = find_or_warn(driver, dev);
	if (!so) {
		return 0;
	}

	SURVIVE_INVOKE_HOOK_SO(sync, so, channel, timecode, ootx, gen);
	return 0;
}

static int parse_and_run_sweep_angle(char *line, SurvivePlaybackData *driver) {
	if (driver->time_now < driver->playback_start_time)
		return 0;

	char dev[10];

	survive_channel channel;
	int sensor_id;
	survive_timecode timecode;
	int8_t plane;
	FLT angle;

	int rr = sscanf(line, SWEEP_ANGLE_SCANF, SWEEP_ANGLE_SCANF_ARGS);

	if (rr != 6) {
		SurviveContext *ctx = driver->ctx;
		SV_WARN("Only got %d values for sweep angle", rr);
		return -1;
	}

	SurviveObject *so = find_or_warn(driver, dev);
	if (!so) {
		return 0;
	}

	SURVIVE_INVOKE_HOOK_SO(sweep_angle, so, channel, sensor_id, timecode, plane, angle);
	return 0;
}

static int parse_and_run_pose(const char *line, SurvivePlaybackData *driver) {
	char name[128] = "replay_";
	SurvivePose pose;

	int rr = sscanf(line, "%s POSE " SurvivePose_sformat "\r\n", name + strlen(name), &pose.Pos[0], &pose.Pos[1],
					&pose.Pos[2], &pose.Rot[0], &pose.Rot[1], &pose.Rot[2], &pose.Rot[3]);

	SurviveContext *ctx = driver->ctx;
	if (rr != 8) {
		SV_WARN("Only got %d values for a pose", rr);
		return 0;
	}

	SURVIVE_INVOKE_HOOK(external_pose, ctx, name, &pose);
	return 0;
}

static int parse_and_run_velocity(const char *line, SurvivePlaybackData *driver) {
	char name[128] = "replay_";
	SurviveVelocity velocity;

	int rr =
		sscanf(line, "%s VELOCITY " SurviveVel_sformat "\r\n", name + strlen(name), &velocity.Pos[0], &velocity.Pos[1],
			   &velocity.Pos[2], &velocity.AxisAngleRot[0], &velocity.AxisAngleRot[1], &velocity.AxisAngleRot[2]);

	SurviveContext *ctx = driver->ctx;
	if (rr != 7) {
		SV_WARN("Only got %d values for a pose", rr);
		return 0;
	}

	SURVIVE_INVOKE_HOOK(external_velocity, ctx, name, &velocity);
	return 0;
}

static int parse_and_set_imu_scales(const char* line, SurvivePlaybackData *driver) {
    char dev[10];
    int gyro_mode = 0, acc_mode = 0;
    SurviveContext *ctx = driver->ctx;

    char i_char = 0;

    int rr = sscanf(line, "%s IMU_SCALES %d %d", dev, &gyro_mode, &acc_mode);

    SurviveObject *so = find_or_warn(driver, dev);
    if (so) {
        survive_default_set_imu_scale_modes(so, gyro_mode, acc_mode); 
    }

    return 0;
}
static int parse_and_run_imu(const char *line, SurvivePlaybackData *driver, bool raw) {
	if (driver->time_now < driver->playback_start_time)
		return 0;

	char dev[10];
	survive_timecode timecode = 0;
	FLT accelgyro[9] = { 0 };
	int mask;
	int id;
	SurviveContext *ctx = driver->ctx;

	char i_char = 0;

	int rr = sscanf(line,
					"%s %c %d %u " FLT_sformat " " FLT_sformat " " FLT_sformat " " FLT_sformat " " FLT_sformat
					" " FLT_sformat " " FLT_sformat " " FLT_sformat " " FLT_sformat "%d",
					dev, &i_char, &mask, &timecode, &accelgyro[0], &accelgyro[1], &accelgyro[2], &accelgyro[3],
					&accelgyro[4], &accelgyro[5], &accelgyro[6], &accelgyro[7], &accelgyro[8], &id);

	if (rr == 11) {
		// Older formats might not have mag data
		id = accelgyro[6];
		accelgyro[6] = 0;
	} else if (rr != 14) {
		SV_WARN("On line %d, only %d values read: '%s'", driver->lineno, rr, line);
		return -1;
	}

	assert(raw ^ i_char == 'I');

	SurviveObject *so = find_or_warn(driver, dev);
	if (so) {
		if (raw) {
            driver->hasRawIMU = true;
            SURVIVE_INVOKE_HOOK_SO(raw_imu, so, mask, accelgyro, timecode, id);
		} else if(!driver->hasRawIMU){
			SURVIVE_INVOKE_HOOK_SO(imu, so, mask, accelgyro, timecode, id);
		}
	}

	return 0;
}

static int parse_and_run_lhpose(const char *line, struct SurvivePlaybackData *driver) {
	SurvivePose pose;
	int lh = -1;
	uint32_t basestationId = 0;
	int rr = sscanf(line, "%d LH_POSE " SurvivePose_sformat "%u\n", &lh, &pose.Pos[0], &pose.Pos[1], &pose.Pos[2],
					&pose.Rot[0], &pose.Rot[1], &pose.Rot[2], &pose.Rot[3], &basestationId);

	SurviveContext *ctx = driver->ctx;
	if (driver->outputCalculatedPose) {
		char buffer[32] = {0};
		snprintf(buffer, 31, "previous_LH%d", lh);
		SURVIVE_INVOKE_HOOK(external_pose, ctx, buffer, &pose);
	}
	return 0;
}

static int parse_and_run_externalpose(const char *line, SurvivePlaybackData *driver) {
	if (driver->time_now < driver->playback_start_time)
		return 0;

	char name[128] = { 0 };
	SurvivePose pose;

	if (driver->outputExternalPose) {
		int rr = sscanf(line, "%s EXTERNAL_POSE " SurvivePose_sformat "\n", name, &pose.Pos[0], &pose.Pos[1],
						&pose.Pos[2], &pose.Rot[0], &pose.Rot[1], &pose.Rot[2], &pose.Rot[3]);

		SurviveContext *ctx = driver->ctx;
		SURVIVE_INVOKE_HOOK(external_pose, ctx, name, &pose);
	}
	return 0;
}

static int parse_and_run_externalvelocity(const char *line, SurvivePlaybackData *driver) {
	if (driver->time_now < driver->playback_start_time)
		return 0;

	char name[128] = {0};
	SurviveVelocity pose;

	if (driver->outputExternalPose) {
		int rr = sscanf(line, "%s EXTERNAL_VELOCITY " SurviveVel_sformat "\n", name, &pose.Pos[0], &pose.Pos[1],
						&pose.Pos[2], &pose.AxisAngleRot[0], &pose.AxisAngleRot[1], &pose.AxisAngleRot[2]);

		SurviveContext *ctx = driver->ctx;
		SURVIVE_INVOKE_HOOK(external_velocity, ctx, name, &pose);
	}
	return 0;
}

static int parse_and_run_config(const char *line, SurvivePlaybackData *driver) {
	const char *configStart = line;
	SurviveContext *ctx = driver->ctx;

	char dev[10] = {0};
	for (int i = 0; i < sizeof(dev) && *configStart != ' '; i++) {
		dev[i] = *configStart++;
	}

	SurviveObject *old_so = survive_get_so_by_name(ctx, dev);
	if (old_so) {
		survive_destroy_device(old_so);
	}

	configStart += strlen("CONFIG") + 1;

	size_t len = strlen(configStart);

	SurviveObject *so = survive_create_device(ctx, "replay", driver, dev, 0);
	if(so == 0) {
        return 0;
    }
	survive_add_object(ctx, so);

	char *config = SV_CALLOC(len + 1);
	memcpy(config, configStart, len);

	if (ctx->configproc(so, config, len) == 0) {
		SV_INFO("Found %s in playback file...", dev);
	} else {
		SV_WARN("Found %s in playback file, but could not read config description", dev);
	}
	return 0;
}

static int parse_and_run_rawlight(const char *line, SurvivePlaybackData *driver) {
	if (driver->time_now < driver->playback_start_time)
		return 0;

	driver->hasRawLight = 1;

	char dev[10];
	char op[10];
	LightcapElement le;
	int rr = sscanf(line, "%s %s %hhu %u %hu\n", dev, op, &le.sensor_id, &le.timestamp, &le.length);

	SurviveObject *so = find_or_warn(driver, dev);
	if (so) {
		handle_lightcap(so, &le);
	}
	return 0;
}

static int parse_and_run_lightcode(const char *line, SurvivePlaybackData *driver) {
	if (driver->time_now < driver->playback_start_time)
		return 0;

	char lhn[10];
	char axn[10];
	char dev[10];
	uint32_t timecode = 0;
	int sensor_id = 0;
	int acode = 0;
	int timeinsweep = 0;
	uint32_t length = 0;
	uint32_t lh = 0;
	SurviveContext *ctx = driver->ctx;

	int rr = sscanf(line, "%8s %8s %8s %d %d %d %u %u %u\n", dev, lhn, axn, &sensor_id, &acode, &timeinsweep, &timecode,
					&length, &lh);

	if (rr != 9) {
		SV_WARN("Warning:  On line %d, only %d values read: '%s'\n", driver->lineno, rr, line);
		return -1;
	}

	SurviveObject *so = find_or_warn(driver, dev);
	if (so)
		SURVIVE_INVOKE_HOOK_SO(light, so, sensor_id, acode, timeinsweep, timecode, length, lh);
	return 0;
}



static int playback_pump_msg(struct SurviveContext *ctx, void *_driver) {
	SurvivePlaybackData *driver = _driver;
	gzFile f = driver->playback_file;

	if (f && !gzeof(f) && !gzerror_dropin(f)) {
		driver->lineno++;
		char *line = 0;

		if (driver->next_time_s == 0) {
			size_t n = 0;
			ssize_t r = gzgetdelim(&line, &n, ' ', f);
			if (r <= 0) {
				free(line);
				return 0;
			}

			if (sscanf(line, "%lf", &driver->next_time_s) != 1) {
				size_t n = 0;
				free(line);
				line = 0;

				ssize_t r = gzgetline(&line, &n, f);
				free(line);

				return 0;
			}

			if(!isfinite(driver->next_time_s)) {
				driver->next_time_s = 0;
			}
			free(line);
			line = 0;
		}

		if (driver->next_time_s * driver->playback_factor > (OGRelativeTime() + driver->time_start))
			return 0;

		driver->time_now = driver->next_time_s;
		driver->next_time_s = 0;

		size_t n = 0;
		ssize_t r = gzgetline(&line, &n, f);

		if (r <= 0) {
			free(line);
			return 0;
		}
		while (r && (line[r - 1] == '\n' || line[r - 1] == '\r')) {
			line[--r] = 0;
		}
		char dev[32];
		char op[32];
		if (sscanf(line, "%31s %31s", dev, op) < 2) {
			free(line);
			return 0;
		}

		const char *ignore_ops[] = {"OPTION", "EXTERNAL_TO_WORLD", "SPHERE", "LH_UP"};
		for (int i = 0; i < SURVIVE_ARRAY_SIZE(ignore_ops); i++) {
			if (strcmp(dev, ignore_ops[i]) == 0) {
				free(line);
				return 0;
			}
		}

		survive_get_ctx_lock(ctx);
		switch (op[0]) {
		case 'F':
			if (strcmp(op, "FULL_STATE") == 0 || strcmp(op, "FULL_COVARIANCE") == 0) {
			}
			break;
		case 'D':
			break;
		case 'W':
			if (op[1] == 0)
				parse_and_run_sweep(line, driver);
			break;
		case 'B':
			if (op[1] == 0 && driver->hasSweepAngle == false)
				parse_and_run_sweep_angle(line, driver);
			break;
		case 'Y':
			if (op[1] == 0)
				parse_and_run_sync(line, driver);
			break;
		case 'E':
			if (strcmp(op, "EXTERNAL_POSE") == 0) {
				parse_and_run_externalpose(line, driver);
				break;
			} else if (strcmp(op, "EXTERNAL_VELOCITY") == 0) {
				parse_and_run_externalvelocity(line, driver);
				break;
			}
		case 'C':
			if (op[1] == 0) {
				parse_and_run_rawlight(line, driver);
			} else if (strcmp(op, "CONFIG") == 0) {
				parse_and_run_config(line, driver);
			}
			break;
		case 'L':
			if (strcmp(op, "LH_POSE") == 0) {
				parse_and_run_lhpose(line, driver);
				break;
			}
		case 'R':
			if (op[1] == 0 && driver->hasRawLight == false)
				parse_and_run_lightcode(line, driver);
			break;
		case 'i':
			if (op[1] == 0)
				parse_and_run_imu(line, driver, true);
			break;
		case 'I':
			if (op[1] == 0)
				parse_and_run_imu(line, driver, false);
			else if(strcmp(op, "IMU_SCALES") == 0) {
                parse_and_set_imu_scales(line, driver);
			}
			break;
		case 'P':
			if (strcmp(op, "POSE") == 0 && driver->outputCalculatedPose)
				parse_and_run_pose(line, driver);
			break;
		case 'A':
		case 'V':
			if (strcmp(op, "VELOCITY") == 0 && driver->outputCalculatedPose)
				parse_and_run_velocity(line, driver);
			break;
		default:
			SV_WARN("Playback doesn't understand '%.10s' op in '%.20s'", op, line);
		}
		survive_release_ctx_lock(ctx);

		free(line);
	} else {
		SV_VERBOSE(100, "EOF for playback received.");
		if (f) {
			gzclose(driver->playback_file);
		}
		driver->playback_file = 0;
		return -1;
	}

	return 0;
}

static void *playback_thread(void *_driver) {
	SurvivePlaybackData *driver = _driver;
	int last_output_minute = 0;
	while (driver->keepRunning == 0 || *driver->keepRunning) {
		double next_time_s_scaled = driver->next_time_s * driver->playback_factor;

		double time_now = OGRelativeTime() + driver->time_start;
		int output_minute = (driver->time_now / 60.);
		if (driver->playback_time >= 0 && driver->time_now > driver->playback_time) {
			*driver->keepRunning = false;
			return 0;
		}
		if (next_time_s_scaled == 0 || next_time_s_scaled < time_now) {
			int rtnVal = playback_pump_msg(driver->ctx, driver);
			SurviveContext *ctx = driver->ctx;
			if (last_output_minute != output_minute) {
				SV_VERBOSE(10, "Playback thread played back %6.2fs in %6.2fs real-time... (%6.2fx)", driver->time_now,
						   time_now, driver->time_now / (time_now + 1e-10));
				last_output_minute = output_minute;
			}
			if (rtnVal < 0)
				*driver->keepRunning = false;
		} else {
			int sleep_time_ms = 1 + (next_time_s_scaled - time_now) * 1000.;
			int sr = OGUSleep(sleep_time_ms * 1000);
			if (sr == 0)
				driver->total_sleep_time += sleep_time_ms;
		}
	}
	return 0;
}

static int playback_close(struct SurviveContext *ctx, void *_driver) {
	SurvivePlaybackData *driver = _driver;

	SV_VERBOSE(100, "Waiting on playback thread...");
	survive_release_ctx_lock(ctx);
	survive_get_ctx_lock(ctx);
	SV_VERBOSE(50, "Playback thread slept for %" PRIu32 "ms", driver->total_sleep_time);
	SV_VERBOSE(10, "Playback thread played back %6.2fs in %6.2fs real-time", driver->time_now, OGRelativeTime());
	if (driver->playback_file)
		gzclose(driver->playback_file);
	driver->playback_file = 0;

	survive_detach_config(ctx, PLAYBACK_START_TIME_TAG, &driver->playback_start_time);
	survive_detach_config(ctx, "playback-factor", &driver->playback_factor);
	survive_detach_config(ctx, "playback-time", &driver->playback_time);

	survive_install_run_time_fn(ctx, 0, 0);
	free(driver);
	return 0;
}

int DriverRegPlayback(SurviveContext *ctx) {
	const char *playback_file = survive_configs(ctx, "playback", SC_GET, 0);

	if (playback_file == 0 || strlen(playback_file) == 0) {
		SV_WARN("The playback argument requires a filename");
		return -1;
	}

	if (strstr(playback_file, ".pcap")) {
		int (*usb_driver)(SurviveContext *) = (int (*)(SurviveContext *))GetDriver("DriverRegUSBMon_Playback");
		if (usb_driver) {
			return usb_driver(ctx);
		}
		SV_WARN("Playback file %s is a USB packet capture, but the usbmon playback driver does not exist.",
				playback_file);
		return -1;
	}

	SurvivePlaybackData *sp = SV_CALLOC(sizeof(SurvivePlaybackData));
	sp->ctx = ctx;
	sp->playback_dir = playback_file;

	sp->outputCalculatedPose = survive_configi(ctx, "playback-replay-pose", SC_GET, 0);
	sp->outputExternalPose = survive_configi(ctx, PLAYBACK_REPLAY_EXTERNAL_POSE_TAG, SC_GET, 0);

	sp->playback_file = gzopen(playback_file, "r");
	if (sp->playback_file == 0) {
		SV_ERROR(SURVIVE_ERROR_INVALID_CONFIG, "Could not open playback events file %s", playback_file);
		return -1;
	}
	survive_install_run_time_fn(ctx, survive_playback_run_time, sp);
	survive_attach_configf(ctx, "playback-factor", &sp->playback_factor);
	survive_attach_configf(ctx, "playback-time", &sp->playback_time);
	survive_attach_configf(ctx, PLAYBACK_START_TIME_TAG, &sp->playback_start_time);

	SV_INFO("Using playback file '%s' with timefactor of %f until %f", playback_file, sp->playback_factor,
			sp->playback_time);

	FLT time = 0;
	char *line = 0;
	size_t n;
	int r = gzgetline(&line, &n, sp->playback_file);

	if (r > 0) {
		if (line[0] == 0x1f) {
			SV_ERROR(SURVIVE_ERROR_INVALID_CONFIG, "Attempting to playback a gz compressed file without gz support.");
			free(line);
			return -1;
		}

		char dev[32];
		char command[32];

		if (sscanf(line, FLT_sformat " %s %s", &time, dev, command) == 3) {
			sp->time_start = time;
		}
	}
	if (sp->time_start < sp->playback_start_time)
		sp->time_start = sp->playback_start_time;
	free(line);
	gzseek(sp->playback_file, 0, SEEK_SET); // same as rewind(f);

	sp->keepRunning = survive_add_threaded_driver(ctx, sp, "playback", playback_thread, playback_close);
	return 0;
}

REGISTER_LINKTIME(DriverRegPlayback)

  
#define _GETDELIM_GROWBY 128 /* amount to grow line buffer by */
#define _GETDELIM_MINLEN 4   /* minimum line buffer size */

#if defined(__has_feature)
#if __has_feature(memory_sanitizer)
#define MEMORY_SANITIZER_IGNORE __attribute__((no_sanitize("memory")))
#endif
#endif
#ifndef MEMORY_SANITIZER_IGNORE
#define MEMORY_SANITIZER_IGNORE
#endif

MEMORY_SANITIZER_IGNORE ssize_t gzgetdelim(char **RESTRICT_KEYWORD lineptr, size_t *RESTRICT_KEYWORD n, int delimiter,
										   gzFile RESTRICT_KEYWORD stream) {
	char *buf = 0, *pos = 0;
	int c = 0;
	ssize_t bytes = 0;

	if (lineptr == NULL || n == NULL) {
		errno = EINVAL;
		return -1;
	}
	if (stream == NULL) {
		errno = EBADF;
		return -1;
	}

	/* resize (or allocate) the line buffer if necessary */
	buf = *lineptr;
	if (buf == NULL || *n < _GETDELIM_MINLEN) {
		buf = SV_REALLOC(*lineptr, _GETDELIM_GROWBY);
		if (buf == NULL) {
			/* ENOMEM */
			return -1;
		}
		*n = _GETDELIM_GROWBY;
		*lineptr = buf;
	}

	/* read characters until delimiter is found, end of file is reached, or an
	   error occurs. */
	bytes = 0;
	pos = buf;
	while ((c = gzgetc(stream)) != EOF) {
		if (bytes + 1 >= (SIZE_MAX / 2)) {
			errno = EOVERFLOW;
			return -1;
		}
		bytes++;
		if (bytes >= *n - 1) {
			buf = SV_REALLOC(*lineptr, *n + _GETDELIM_GROWBY);
			if (buf == NULL) {
				/* ENOMEM */
				return -1;
			}
			*n += _GETDELIM_GROWBY;
			pos = buf + bytes - 1;
			*lineptr = buf;
		}

		*pos++ = (char)c;
		if (c == delimiter) {
			break;
		}
	}

	if (gzerror_dropin(stream) || (gzeof(stream) && (bytes == 0))) {
		/* EOF, or an error from getc(). */
		return -1;
	}

	*pos = '\0';
	return bytes;
}

ssize_t gzgetline(char **RESTRICT_KEYWORD lineptr, size_t *RESTRICT_KEYWORD n, gzFile RESTRICT_KEYWORD stream) {
	return gzgetdelim(lineptr, n, '\n', stream);
}
