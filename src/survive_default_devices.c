#include "survive_default_devices.h"
#include "assert.h"
#include "json_helpers.h"
#include "survive_internal.h"
#include "survive_kalman_tracker.h"
#include <jsmn.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

#define HMD_IMU_HZ 1000.0f
#define VIVE_DEFAULT_IMU_HZ 250.0f


static inline bool check_str(const char* blacklist, const char* name) {
	const char* s = strstr(blacklist, name);
	while(s) {
		if ((s[strlen(name)] == 0 || s[strlen(name)] == ',') && (s == blacklist || s[-1] == ',')) {
				return true;
			}
			s = strstr(s + 1, name);
	}
	return false;
}

SurviveObject *survive_create_device(SurviveContext *ctx, const char *driver_name, void *driver,
									 const char *device_name, haptic_func fn) {
    const char *blacklist = survive_configs(ctx, "blacklist-devs", SC_GET, "-");
    if(check_str(blacklist, device_name)) {
        return 0;
    }

    SurviveObject *device = SV_CALLOC(sizeof(SurviveObject));

	device->ctx = ctx;
	device->driver = driver;

	memcpy(device->drivername, driver_name, strlen(driver_name));
	memcpy(device->codename, device_name, strlen(device_name));
	for (int i = 0; ctx && i < ctx->objs_ct; i++) {
		if (memcmp(device->codename, ctx->objs[i]->codename, sizeof(driver_name)) == 0) {
			i = 0;
			device->codename[2]++;
		}
	}

	device->object_type = SURVIVE_OBJECT_TYPE_OTHER;
	device->timebase_hz = 48000000;
	device->imu_freq = VIVE_DEFAULT_IMU_HZ;
	device->haptic = fn;

	device->imu2trackref.Rot[0] = 1.;
	device->head2trackref.Rot[0] = 1.;
	device->sensor_scale = 1;
	device->sensor_scale_var = 0.1;

	for (int i = 0; i < 3; i++) {
		device->gyro_scale[i] = device->acc_scale[i] = 1.0;
	}

	SurviveSensorActivations_ctor(device, &device->activations);

	FLT playback_factor = survive_configf(ctx, "playback-factor", SC_GET, 1.);
	bool use_async_posers = survive_configi(ctx, "threaded-posers", SC_GET, 1) && playback_factor != 0;
	if (use_async_posers) {
		PoserCB PreferredPoserCB = (PoserCB)GetDriverByConfig(ctx, "Poser", "poser", "MPFIT");
		*survive_object_plugin_data(device, survive_threaded_poser_fn) =
			survive_create_threaded_poser(device, PreferredPoserCB);
	}

	device->tracker = SV_MALLOC(sizeof(struct SurviveKalmanTracker));
	survive_kalman_tracker_init(device->tracker, device);

	return device;
}

static int jsoneq(const char *json, const jsmntok_t *tok, const char *s) {
	if (tok && tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
		strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}

static int ParsePoints(SurviveContext *ctx, SurviveObject *so, char *ct0conf, FLT **floats_out, const jsmntok_t *t) {
  char ctt[128] = { 0 };
	int pts = t[1].size;
	const jsmntok_t *tk;
	
	so->sensor_ct = 0;
	assert(*floats_out == 0);
	*floats_out = SV_CALLOC(sizeof(**floats_out) * 32 * 3);

	for (int k = 0; k < pts; k++) {
		tk = &t[2 + k * 4];

		for (int m = 0; m < 3; m++) {

			tk++;
			int elemlen = tk->end - tk->start;

			if (tk->type != 4 || elemlen > sizeof(ctt) - 1) {
			  SV_GENERAL_ERROR("Parse error in JSON %d %d %lu", tk->type, elemlen, sizeof(ctt));
				return 1;
			}

			memcpy(ctt, ct0conf + tk->start, elemlen);
			ctt[elemlen] = 0;
			FLT f = atof(ctt);
			int id = so->sensor_ct * 3 + m;
			(*floats_out)[id] = f;
		}
		so->sensor_ct++;
	}
	return 0;
}

static void vive_json_pose_to_survive_pose(const FLT *values, SurvivePose *pose) {
	for (int i = 0; i < 3; i++) {
		pose->Pos[i] = values[4 + i];
		pose->Rot[1 + i] = values[i];
	}
	pose->Rot[0] = values[3];
}

typedef struct stack_entry_s {
	struct stack_entry_s *previous;
	jsmntok_t *key;
} stack_entry_t;

typedef struct {
	FLT position[3];
	FLT plus_x[3];
	FLT plus_z[3];
} vive_pose_t;

int solve_vive_pose(SurvivePose *pose, const vive_pose_t *vpose) {
	if (vpose->plus_x[0] == 0.0 && vpose->plus_x[1] == 0.0 && vpose->plus_x[2] == 0.0)
		return 0;

	if (vpose->plus_z[0] == 0.0 && vpose->plus_z[1] == 0.0 && vpose->plus_z[2] == 0.0)
		return 0;

	copy3d(pose->Pos, vpose->position);

	LinmathVec3d plus_y;
	cross3d(plus_y, vpose->plus_z, vpose->plus_x);
	FLT m[] = {vpose->plus_x[0], plus_y[0],		   vpose->plus_z[0], vpose->plus_x[1], plus_y[1],
			   vpose->plus_z[1], vpose->plus_x[2], plus_y[2],		 vpose->plus_z[2]};

	quatfrommatrix33(pose->Rot, m);

	// Double check the math above; I'm 95% sure its right but kabsch tells us for sure; but is expensive. Remove this
	// when it's 100% clear its correct
	FLT axis[] = {1, 0, 0, 0, 0, 1};
	LinmathQuat rAssert;
	KabschCentered(rAssert, axis, vpose->plus_x, 2);

	LinmathQuat diff;
	quatfind(diff, rAssert, pose->Rot);
	assert(norm3d(diff + 1) < .001);

	return 1;
}

typedef struct {
	SurviveObject *so;
	vive_pose_t imu_pose;
	vive_pose_t head;
	FLT sensor_scale;
} scratch_space_t;

static scratch_space_t scratch_space_init(SurviveObject *so) { return (scratch_space_t){.so = so}; }

static bool parse_ctx_sensitive_vive_pose_t(char *ct0conf, stack_entry_t *stack, const char *field_name,
											vive_pose_t *output) {
	if (stack->previous && jsoneq(ct0conf, stack->previous->key, field_name) == 0) {
		struct field {
			const char *name;
			FLT *vals;
		};

		struct field imufields[] = {
			{"plus_x", output->plus_x}, {"plus_z", output->plus_z}, {"position", output->position}};

		const jsmntok_t *tk = stack->key;
		for (int i = 0; i < sizeof(imufields) / sizeof(struct field); i++) {
			if (jsoneq(ct0conf, tk, imufields[i].name) == 0) {
				int32_t count = (tk + 1)->size;
				assert(count == 3);
				if (count == 3) {
					parse_float_array_in_place(ct0conf, tk + 2, imufields[i].vals, count);
				}
				break;
			}
		}

		return true;
	}

	return false;
}

static int process_jsonarray(scratch_space_t *scratch, char *ct0conf, stack_entry_t *stack) {
	SurviveObject *so = scratch->so;
	jsmntok_t const *const tk = stack->key;
	SurviveContext *ctx = so->ctx;

	/// CONTEXT FREE FIELDS
	if (jsoneq(ct0conf, tk, "modelPoints") == 0) {
		if (ParsePoints(ctx, so, ct0conf, &so->sensor_locations, tk)) {
			return -1;
		}
	} else if (jsoneq(ct0conf, tk, "modelNormals") == 0) {
		if (ParsePoints(ctx, so, ct0conf, &so->sensor_normals, tk)) {
			return -1;
		}
	} else if (jsoneq(ct0conf, tk, "channelMap") == 0) {
		int32_t count = (tk + 1)->size;
		int *values = NULL;
		if (parse_int_array(ct0conf, tk + 2, &values, count)) {
			int max_port = 32;
			so->channel_map = SV_MALLOC(sizeof(int) * max_port);
			for (int i = 0; i < max_port; i++)
				so->channel_map[i] = -1;

			for (int i = 0; i < count; i++) {
			  if(values[i] >= 0) {
				so->channel_map[values[i]] = i;
			  }
			} 
		}
		free(values);
	} else if (jsoneq(ct0conf, tk, "acc_bias") == 0) {
		int32_t count = (tk + 1)->size;
		if (parse_float_array_in_place(ct0conf, tk + 2, so->acc_bias, count) <= 0) {
			SV_WARN("Could not parse acc_bias");
		}
	} else if (jsoneq(ct0conf, tk, "acc_scale") == 0) {
		int32_t count = (tk + 1)->size;
		if (parse_float_array_in_place(ct0conf, tk + 2, so->acc_scale, count) <= 0) {
			SV_WARN("Could not parse acc_scale");
		}
	} else if (jsoneq(ct0conf, tk, "gyro_bias") == 0) {
		int32_t count = (tk + 1)->size;
		if (parse_float_array_in_place(ct0conf, tk + 2, so->gyro_bias, count) <= 0) {
			SV_WARN("Could not parse gyro_bias");
		}
	} else if (jsoneq(ct0conf, tk, "gyro_scale") == 0) {
		int32_t count = (tk + 1)->size;
		if (parse_float_array_in_place(ct0conf, tk + 2, so->gyro_scale, count) <= 0) {
			SV_WARN("Could not parse gyro_scale");
		}
	} else if (jsoneq(ct0conf, tk, "trackref_from_imu") == 0) {
		int32_t count = (tk + 1)->size;
		if (count == 7) {
			FLT *values = NULL;
			if (parse_float_array(ct0conf, tk + 2, &values, count) > 0) {
				vive_json_pose_to_survive_pose(values, &so->imu2trackref);
				free(values);
			}
		}
	} else if (jsoneq(ct0conf, tk, "trackref_from_head") == 0) {
		int32_t count = (tk + 1)->size;
		if (count == 7) {
			FLT *values = NULL;
			if (parse_float_array(ct0conf, tk + 2, &values, count) > 0) {
				vive_json_pose_to_survive_pose(values, &so->head2trackref);
				free(values);
			}
		}
	}

	/// Context sensitive fields
	else {
		parse_ctx_sensitive_vive_pose_t(ct0conf, stack, "imu", &scratch->imu_pose);
		parse_ctx_sensitive_vive_pose_t(ct0conf, stack, "head", &scratch->head);
	}

	return 0;
}

struct model_number_metadata {
	const char *key;
	int32_t value;
	FLT sensor_scale;
};

struct model_number_metadata model_number_subtypes[] = {
	{"Knuckles Right", SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R, 0.999},
	{"Knuckles EV3.0 Right", SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R, 0.999},
	{"Knuckles Left", SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L, 0.999},
	{"Knuckles EV3.0 Left", SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L, 0.999},
	{"Utah MP", SURVIVE_OBJECT_SUBTYPE_INDEX_HMD, 1.0048},
	{"Vive Controller MV", SURVIVE_OBJECT_SUBTYPE_WAND, 1.},
	{"Vive. Controller MV", SURVIVE_OBJECT_SUBTYPE_WAND, 1.},
	{"VIVE Tracker Pro MV", SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2, 1.0034},
	{"Vive. Tracker MV", SURVIVE_OBJECT_SUBTYPE_TRACKER, 1.00585},
	{"Vive Tracker MV", SURVIVE_OBJECT_SUBTYPE_TRACKER, 1.00585},
	{"Vive MV.", SURVIVE_OBJECT_SUBTYPE_VIVE_HMD, 1.004},
	{"Vive MV", SURVIVE_OBJECT_SUBTYPE_VIVE_HMD, 1.004},
	{"Vive. MV", SURVIVE_OBJECT_SUBTYPE_VIVE_HMD, 1.004},
	{"Vive DVT", SURVIVE_OBJECT_SUBTYPE_VIVE_HMD, 1.004},
	{"Vive_Pro MV", SURVIVE_OBJECT_SUBTYPE_VIVE_HMD, 1.0028},
	{"VIVE_Pro MV", SURVIVE_OBJECT_SUBTYPE_VIVE_HMD, 1.0028},
	{"VIVE Tracker 3.0 MV", SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2, 1.0034},
	{"REF-HMD", SURVIVE_OBJECT_SUBTYPE_VIVE_HMD, 1.},
	{"Tundra Tracker", SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2, 1.},
	{"VIVE_Pro 2 MV", SURVIVE_OBJECT_TYPE_HMD, 1.},
	{"VIVE Controller Pro MV", SURVIVE_OBJECT_SUBTYPE_WAND, 1.},
};

static int process_jsontok(scratch_space_t *scratch, char *d, stack_entry_t *stack, jsmntok_t *t, int count) {
	int i, j, k;
	assert(count >= 0);
	if (count == 0) {
		return 0;
	}
	if (t->type == JSMN_PRIMITIVE) {
		return 1;
	} else if (t->type == JSMN_STRING) {
		if (stack && stack->key) {
			if (jsoneq(d, stack->key, "device_class") == 0) {
				if (strncmp("controller", d + t->start, t->end - t->start) == 0) {
					scratch->so->object_type = SURVIVE_OBJECT_TYPE_CONTROLLER;
				} else if (strncmp("hmd", d + t->start, t->end - t->start) == 0) {
					scratch->so->object_type = SURVIVE_OBJECT_TYPE_HMD;
				} else if (strncmp("generic_tracker", d + t->start, t->end - t->start) == 0) {
					scratch->so->object_type = SURVIVE_OBJECT_TYPE_OTHER;
				}
			} else if (jsoneq(d, stack->key, "device_serial_number") == 0) {
				int size = sizeof(scratch->so->serial_number);
				if (size > t->end - t->start)
					size = t->end - t->start;

				memcpy(scratch->so->serial_number, d + t->start, size);
			} else if (jsoneq(d, stack->key, "model_number") == 0) {

				const char *str = d + t->start;
				size_t len = t->end - t->start;

				for (int idx = 0; idx < sizeof(model_number_subtypes) / sizeof(model_number_subtypes[0]); idx++) {
					if (strncmp(model_number_subtypes[idx].key, str, len) == 0) {
						scratch->so->object_subtype = model_number_subtypes[idx].value;
						scratch->sensor_scale = model_number_subtypes[idx].sensor_scale;
						break;
					}
				}

				if (scratch->so->object_subtype == SURVIVE_OBJECT_SUBTYPE_GENERIC) {
					SurviveContext *ctx = scratch->so->ctx;
					SV_WARN(
						"Unknown model_number '%.*s'. Please submit an issue with this value describing your device "
						"so it can be added to the known list.",
						(int)len, str);
				}
			} else if (jsoneq(d, stack->key, "model_name") == 0) {
				if (strncmp("Vive Tracker MV", d + t->start, t->end - t->start) == 0) {
					scratch->so->object_subtype = SURVIVE_OBJECT_SUBTYPE_TRACKER;
				}
			}
		}

		return 1;
	} else if (t->type == JSMN_OBJECT) {
		stack_entry_t entry;
		entry.previous = stack;
		j = 0;
		for (i = 0; i < t->size; i++) {
			entry.key = t + 1 + j;
			// print_stack_spot(d, &entry);
			// Skip the key
			j++;
			// Read value
			j += process_jsontok(scratch, d, &entry, t + 1 + j, count - j);
		}
		return j + 1;
	} else if (t->type == JSMN_ARRAY) {
		process_jsonarray(scratch, d, stack);

		stack_entry_t entry;
		entry.previous = stack;
		j = 0;
		for (i = 0; i < t->size; i++) {
			entry.key = t + 1 + j;
			j += process_jsontok(scratch, d, &entry, t + 1 + j, count - j);
		}
		return j + 1;
	}
	return 0;
}

STATIC_CONFIG_ITEM(IGNORE_CONFIG_IMU_BIAS, "ignore-config-imu-bias", 'b',
				   "Ignore the bias set for imu devices in the config", 0)

int survive_load_htc_config_format(SurviveObject *so, char *ct0conf, int len) {
	if (len == 0)
		return -1;

	SurviveContext *ctx = so->ctx;
	// From JSMN example.
	jsmn_parser p = {0};
	jsmn_init(&p);

	int r = jsmn_parse(&p, ct0conf, len);
	if (r < 0) {
		SV_INFO("Failed to parse JSON in HMD configuration: %d\n", r);
		jsmn_free(&p);
		return -1;
	}
	if (r < 1 || p.token_pool[0].type != JSMN_OBJECT) {
		SV_INFO("Object expected in HMD configuration\n");
		jsmn_free(&p);
		return -2;
	}

	so->object_type = SURVIVE_OBJECT_TYPE_OTHER;
	scratch_space_t scratch = scratch_space_init(so);
	process_jsontok(&scratch, ct0conf, 0, p.token_pool, r);

	solve_vive_pose(&so->imu2trackref, &scratch.imu_pose);
	solve_vive_pose(&so->head2trackref, &scratch.head);

	SurvivePose trackref2imu = InvertPoseRtn(&so->imu2trackref);

	bool sensorsAreZero = true;
	for (int i = 0; i < so->sensor_ct; i++) {
		if (norm3d(&so->sensor_locations[i]) > .001) {
			sensorsAreZero = false;
		}
		if (scratch.sensor_scale != 0.0) {
			scale3d(&so->sensor_locations[i * 3], &so->sensor_locations[i * 3], scratch.sensor_scale);
		}
		ApplyPoseToPoint(&so->sensor_locations[i * 3], &trackref2imu, &so->sensor_locations[i * 3]);
		quatrotatevector(&so->sensor_normals[i * 3], trackref2imu.Rot, &so->sensor_normals[i * 3]);
	}

	so->has_sensor_locations = !sensorsAreZero;

	ApplyPoseToPose(&so->head2imu, &trackref2imu, &so->head2trackref);

	// Handle device-specific scaling.
	if (strcmp(so->codename, "HMD") == 0 || so->object_type == SURVIVE_OBJECT_TYPE_HMD) {
		SV_INFO("%s is treated as HMD device", so->codename);
        if(so->raw_acc_scale == 0) {
            so->raw_acc_scale = 1. / 8192.;
        }
		scale3d(so->acc_bias, so->acc_bias, 1. / 1000.); // Odd but seems right.
		so->imu_freq = HMD_IMU_HZ;

		FLT deg_per_sec = 500;
        if(so->raw_gyro_scale == 0) {
            so->raw_gyro_scale = deg_per_sec / (1 << 15) * LINMATHPI / 180.;
        }
	} else if (memcmp(so->codename, "WM", 2) == 0) {
        if(so->raw_acc_scale == 0) {
			so->raw_acc_scale = 2. / 8192.;
		}
		scale3d(so->acc_bias, so->acc_bias, 1. / 1000.); // Need to verify.

		FLT deg_per_sec = 2000;
		if(so->raw_gyro_scale == 0) {
            so->raw_gyro_scale = deg_per_sec / (1 << 15) * LINMATHPI / 180.;
        }
	} else // Verified on WW, Tracker, both RF and wired
	{
		// 1G for accelerometer, from MPU6500 datasheet
		// this can change if the firmware changes the sensitivity.
		// When coming off of USB, these values are in units of .5g -JB
		if(so->raw_acc_scale == 0) {
			so->raw_acc_scale = 2. / 8192.;
		}
		// If any other device, we know we at least need this.
		scale3d(so->acc_bias, so->acc_bias, 1. / 1000.);

		// From datasheet, can be 250, 500, 1000, 2000 deg/s range over 16 bits
		FLT deg_per_sec = 2000;
		if(so->raw_gyro_scale == 0) {
            so->raw_gyro_scale = deg_per_sec / (1 << 15) * LINMATHPI / 180.;
        }
		// scale3d(so->gyro_scale, so->gyro_scale, 3.14159 / 1800. / 1.8);
	}

	if (survive_configb(ctx, IGNORE_CONFIG_IMU_BIAS_TAG, SC_GET, 0)) {
		scale3d(so->gyro_bias, so->gyro_bias, 0);
		scale3d(so->acc_bias, so->acc_bias, 0);
	}

	SV_VERBOSE(110, "Device %s has gyro bias " Point3_format " scale " Point3_format, survive_colorize_codename(so),
			   LINMATH_VEC3_EXPAND(so->gyro_bias), LINMATH_VEC3_EXPAND(so->gyro_scale));
	SV_VERBOSE(110, "Device %s has acc bias  " Point3_format " scale " Point3_format, survive_colorize_codename(so),
			   LINMATH_VEC3_EXPAND(so->acc_bias), LINMATH_VEC3_EXPAND(so->acc_scale));
	SV_VERBOSE(50, "Read config for %s", survive_colorize(so->codename));
	jsmn_free(&p);
	return 0;
}

struct lhdb_ctx {
	SurviveContext *ctx;
	uint32_t lh_found;
	uint32_t serials[NUM_GEN2_LIGHTHOUSES];

	SurvivePose poses[NUM_GEN2_LIGHTHOUSES];

	struct json_stack_entry_s *parsingBSD;
	FLT pitch, roll;
};
static void lhdb_begin_object(struct json_callbacks *cb, struct json_stack_entry_s *obj) {
	struct lhdb_ctx *lhdb = cb->user;

	if (!lhdb->parsingBSD && json_has_ancestor_tag("base_stations", obj) &&
		json_has_ancestor_tag("known_universes", obj)) {
		lhdb->parsingBSD = obj;
		lhdb->lh_found++;
		SurviveContext *ctx = lhdb->ctx;
		SV_VERBOSE(105, "Found base station object definition");
	}
}
static void lhdb_end_object(struct json_callbacks *cb, struct json_stack_entry_s *obj) {
	struct lhdb_ctx *lhdb = cb->user;

	if (obj && obj == lhdb->parsingBSD) {
		lhdb->parsingBSD = 0;
		SurviveContext *ctx = lhdb->ctx;
		SV_VERBOSE(105, "Exiting base station object definition");
	}
}
static void lhdb_tag_value(struct json_callbacks *cb, struct json_stack_entry_s *obj) {
	struct lhdb_ctx *lhdb = cb->user;
	SurviveContext *ctx = lhdb->ctx;

	if (strcmp("base_serial_number", json_stack_tag(obj)) == 0) {
		lhdb->serials[lhdb->lh_found - 1] = atoi(json_stack_value(obj));
		SV_VERBOSE(105, "\tSerial number %8u", lhdb->serials[lhdb->lh_found - 1]);
	} else if (json_has_ancestor_tag("pose", obj)) {
		FLT v = atof(json_stack_value(obj));
		int idx = json_stack_index(obj);
		if (idx >= 4) {
			lhdb->poses[lhdb->lh_found - 1].Pos[idx - 4] = v;
		} else if (idx <= 2) {
			lhdb->poses[lhdb->lh_found - 1].Rot[idx + 1] = v;
		} else {
			lhdb->poses[lhdb->lh_found - 1].Rot[0] = v;
		}
		SV_VERBOSE(105, "\tPose index %d %f", idx, v);
	} else if (strcmp("pitch", json_stack_tag(obj)) == 0 && json_has_ancestor_tag("known_universes", obj)) {
		lhdb->pitch = atof(json_stack_value(obj));
	} else if (strcmp("roll", json_stack_tag(obj)) == 0 && json_has_ancestor_tag("known_universes", obj)) {
		lhdb->roll = atof(json_stack_value(obj));
	}
}

SURVIVE_EXPORT int survive_load_steamvr_lighthousedb(SurviveContext *ctx, char *ct0conf, int len) {
	if (len == 0)
		return -1;

	jsmn_parser p = {0};
	jsmn_init(&p);

	int r = jsmn_parse(&p, ct0conf, len);
	if (r < 0) {
		SV_WARN("Failed to parse JSON in lighthouse db: %d\n", r);
		jsmn_free(&p);
		return -1;
	}
	struct lhdb_ctx lhctx = {.ctx = ctx};
	struct json_callbacks cbs = {.user = &lhctx,
								 .json_begin_object = lhdb_begin_object,
								 .json_end_object = lhdb_end_object,
								 .json_tag_value = lhdb_tag_value};
	json_run_callbacks(&cbs, ct0conf, len);

	FLT euler[] = {lhctx.roll, lhctx.pitch, 0};
	LinmathPose vr2bsd = {0, .Rot = {1}};
	// quatfromeuler(vr2bsd.Rot, euler);

	LinmathQuat q = {1};
	LinmathPoint3d survivePts[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	LinmathPoint3d openvrPts[] = {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};
	// KabschCentered(q, (FLT *)openvrPts, (FLT *)survivePts, 3);
	quatrotateabout(vr2bsd.Rot, q, vr2bsd.Rot);
	SurvivePose bsdup2realup = {.Rot = {0.}};

	for (int i = 0; i < ctx->activeLighthouses; i++) {
		for (int j = 0; j < lhctx.lh_found; j++) {
			if (ctx->bsd[i].BaseStationID == lhctx.serials[j]) {
				lhctx.poses[j] = InvertPoseRtn(&lhctx.poses[j]);
				SV_VERBOSE(50, "Basestation ID %8u (%d) has " SurvivePose_format, lhctx.serials[j], i,
						   SURVIVE_POSE_EXPAND(lhctx.poses[j]));
				ApplyPoseToPose(&ctx->bsd[i].Pose, &vr2bsd, &lhctx.poses[j]);

				if (quatiszero(bsdup2realup.Rot)) {
					LinmathPoint3d real_up = {0, 0, 1};
					LinmathPoint3d bsd_up = {0};
					normalize3d(bsd_up, ctx->bsd[i].accel);
					quatrotatevector(bsd_up, ctx->bsd[i].Pose.Rot, bsd_up);

					quatfind_between_vectors(bsdup2realup.Rot, bsd_up, real_up);
				}

				ApplyPoseToPose(&ctx->bsd[i].Pose, &bsdup2realup, &ctx->bsd[i].Pose);

				ctx->bsd[i].PositionSet = true;
				ctx->request_floor_set = true;
			}
		}
	}

	SV_VERBOSE(50, "Read lighthouse db config file");
	jsmn_free(&p);
	return 0;
}
SURVIVE_EXPORT int survive_load_steamvr_lighthousedb_from_file(SurviveContext *ctx, const char *filename) {
	if (ctx == 0)
		return -1;

	FILE *fp = fopen(filename, "r");
	if (fp) {
		fseek(fp, 0L, SEEK_END);
		int len = ftell(fp);
		fseek(fp, 0L, SEEK_SET);
		if (len > 0) {
			char *ct0conf = (char *)malloc(len);
			size_t read = fread(ct0conf, 1, len, fp);
			survive_load_steamvr_lighthousedb(ctx, ct0conf, len);
			free(ct0conf);
			fclose(fp);
		}
		return 0;
	}

	SV_WARN("Could not open lighthouse db file at '%s' (%d)", filename, errno);
	return -1;
}

int survive_load_htc_config_format_from_file(SurviveObject *so, const char *filename) {
	if (so == 0 || so->ctx == 0)
		return -1;

	SurviveContext *ctx = so->ctx;

	FILE *fp = fopen(filename, "r");
	if (fp) {
		fseek(fp, 0L, SEEK_END);
		int len = ftell(fp);
		fseek(fp, 0L, SEEK_SET);
		if (len > 0) {
			char *ct0conf = (char *)malloc(len);
			size_t read = fread(ct0conf, 1, len, fp);
			SURVIVE_INVOKE_HOOK_SO(config, so, ct0conf, read);
			fclose(fp);
		}
		return 0;
	}
	return -1;
}

void survive_destroy_device(SurviveObject *so) {
	SurviveContext *ctx = so->ctx;
	SURVIVE_INVOKE_HOOK_SO(disconnect, so);

	size_t idx = 0;
	if (ctx->objs) {
		for (idx = 0; idx < ctx->objs_ct && ctx->objs[idx] != so; idx++)
			;
		ctx->objs[idx] = ctx->objs[ctx->objs_ct - 1];
		ctx->objs_ct--;
	}

	PoserData pd;
	pd.pt = POSERDATA_DISASSOCIATE;
	if (ctx->PoserFn) {
		ctx->PoserFn(so, &pd);
	}
	SURVIVE_INVOKE_HOOK_SO(lightcap, so, 0);

	SV_VERBOSE(5, "Statistics for %s (driver %s)", so->codename, so->drivername);
	SV_VERBOSE(5, "\tExtent hits               %6u", so->stats.extent_hits);
	SV_VERBOSE(5, "\tNaive hits                %6u", so->stats.naive_hits);
	SV_VERBOSE(5, "\tExtent misses             %6u", so->stats.extent_misses);
	SV_VERBOSE(5, "\tExtent min                %6.4f", so->stats.min_extent);
	SV_VERBOSE(5, "\tExtent max                %6.4f", so->stats.max_extent);

	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (so->stats.hit_from_lhs[i]) {
			SV_VERBOSE(5, "\tLH %2d", i);
			SV_VERBOSE(5, "\t\tSyncs:             %8u", so->stats.syncs[i]);
			SV_VERBOSE(5, "\t\tSkipped Syncs:     %8u", so->stats.skipped_syncs[i]);
			SV_VERBOSE(5, "\t\tSync resets:       %8u", so->stats.sync_resets[i]);
			SV_VERBOSE(5, "\t\tBad Syncs:         %8u", so->stats.bad_syncs[i]);
			SV_VERBOSE(5, "\t\tHits:              %8u", so->stats.hit_from_lhs[i]);
			SV_VERBOSE(5, "\t\tDrops:             %8u", so->stats.dropped_light[i]);
			SV_VERBOSE(5, "\t\tRejected Data:     %8u (%3.3f%%)", so->stats.rejected_data[i],
					   so->stats.rejected_data[i] * 100. / (FLT)so->stats.hit_from_lhs[i]);
		}
	}

	survive_kalman_tracker_free(so->tracker);
	SurviveSensorActivations_dtor(so);
	free(so->tracker);
	free(so->sensor_locations);
	free(so->sensor_normals);
	free(so->conf);
	free(so->channel_map);
	free(so->PluginDataEntries);
	so->PluginDataEntries_space = so->PluginDataEntries_cnt = 0;
	free(so);
}
