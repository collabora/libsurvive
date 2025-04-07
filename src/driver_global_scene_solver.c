#include "os_generic.h"
#include "survive.h"
#include "survive_recording.h"

#include <stdio.h>
#include <stdlib.h>
#include <survive_optimizer.h>
#include <survive_reproject_gen2.h>

STATIC_CONFIG_ITEM(GSS_ENABLE, "globalscenesolver", 'i', "Enable global scene solver", 1)

#ifndef GSS_NUM_STORED_SCENES
#define GSS_NUM_STORED_SCENES 32
#endif

typedef struct global_scene_solver {
	struct SurviveContext *ctx;

	size_t scenes_cnt;
	struct PoserDataGlobalScene scenes[GSS_NUM_STORED_SCENES];

	size_t last_capture_time_cnt;
	survive_long_timecode *last_capture_time;

	int solve_counts;
	int solve_count_max;

	bool needsSolve;
	FLT last_addition;
	int desired_coverage;
	bool auto_floor;

	imu_process_func imu_fn;
	sync_process_func prior_sync_fn;
	light_pulse_process_func prior_light_pulse;
	ootx_received_process_func prior_ootx_fn;

#define NUM_BINS 5
	int coverage[NUM_GEN2_LIGHTHOUSES][2][NUM_BINS];

	bool threaded;
	og_thread_t thread;
	bool active;
	og_cv_t data_available;
	og_mutex_t data_available_lock;
	og_mutex_t scenes_lock;
	int run_count;

} global_scene_solver;

STRUCT_CONFIG_SECTION(global_scene_solver)
	STRUCT_CONFIG_ITEM("gss-threaded", "Thread GSS iterations", 1, t->threaded)
	STRUCT_CONFIG_ITEM("gss-desired-coverage", "Number of measurements to saturate a bin", 30, t->desired_coverage)
	STRUCT_CONFIG_ITEM("gss-auto-floor-height", "Automatically use the lowest position to set the floor offset", 1, t->auto_floor)
END_STRUCT_CONFIG_SECTION(global_scene_solver)

static size_t add_scenes(struct global_scene_solver *gss, SurviveObject *so) {
	size_t rtn = 0;
	SurviveContext *ctx = so->ctx;

	survive_long_timecode sensor_time_window = SurviveSensorActivations_stationary_time(&so->activations) / 2;

	SurviveSensorActivations *activations = &so->activations;

	// TT clean up scenes with invalid SurviveObjects
	unsigned int free_idx = 0;
	for (int i = 0; i < GSS_NUM_STORED_SCENES; i++) {
		if (gss->scenes[i].so == NULL) {
			continue;
		}
		
		bool is_still_valid = false;
		if (gss->scenes[i].so == so) {
			is_still_valid = true;
		}
		for (int j = 0; j < ctx->objs_ct; j++) {
			if (gss->scenes[i].so == ctx->objs[j]) {
				is_still_valid = true;
			}
		}

		if (is_still_valid) {
			gss->scenes[free_idx].so = gss->scenes[i].so;
			free_idx++;
		} else {
			SV_WARN("in GlobalSceneSolver: deleted scene %d with SurviveObject at adress &%p", i, (void* )so);
		}
	}
	gss->scenes_cnt = free_idx;

	struct PoserDataGlobalScene *scene = &gss->scenes[gss->scenes_cnt % GSS_NUM_STORED_SCENES];

	scene->pose = so->OutPoseIMU;

	scene->so = so;
	copy3d(scene->accel, activations->accel);
	scene->meas_cnt = 0;
	scene->meas = SV_REALLOC(scene->meas, 32 * 2 * ctx->activeLighthouses * sizeof(scene->meas[0]));

	int scene_coverage[NUM_GEN2_LIGHTHOUSES][2][NUM_BINS] = {0};

	bool useful = gss->desired_coverage < 0;
	size_t lh_meas[NUM_GEN2_LIGHTHOUSES] = {0};
	for (uint8_t lh = 0; lh < ctx->activeLighthouses; lh++) {
		for (uint8_t sensor = 0; sensor < so->sensor_ct; sensor++) {
			for (uint8_t axis = 0; axis < 2; axis++) {
				bool isReadingValid =
					SurviveSensorActivations_is_reading_valid(activations, sensor_time_window, sensor, lh, axis);

				if (isReadingValid) {
					const FLT *a = activations->angles[sensor][lh];

					PoserDataGlobalSceneMeasurement *meas = scene->meas + scene->meas_cnt;

					meas->axis = axis;
					meas->value = a[axis];
					meas->sensor_idx = sensor;
					meas->lh = lh;
					FLT range = 2.0944; // 120 degrees

					int bin = meas->value / range * NUM_BINS + (NUM_BINS / 2.);
					if (bin < 0)
						bin = 0;
					if (bin >= NUM_BINS)
						bin = NUM_BINS - 1;
					scene_coverage[lh][axis][bin]++;

					useful |= gss->coverage[lh][axis][bin] < gss->desired_coverage;

					lh_meas[lh]++;
					scene->meas_cnt++;
				}
			}
		}
	}

	if (useful && scene->meas_cnt > 10) {
		gss->scenes_cnt++;
		rtn++;

		for (uint8_t lh = 0; lh < ctx->activeLighthouses; lh++) {
			for (uint8_t axis = 0; axis < 2; axis++) {
				for (int i = 0; i < NUM_BINS; i++) {
					gss->coverage[lh][axis][i] += scene_coverage[lh][axis][i];
				}
			}
		}

		for (int i = 0; i < ctx->activeLighthouses; i++) {
			SV_VERBOSE(100, "Scene %s %d for lh %d", survive_colorize_codename(so), (int)lh_meas[i], i);
		}
	} else {
		SV_VERBOSE(100, "Scene rejected; meas %d", (int)scene->meas_cnt);
	}

	return rtn;
}

static bool run_optimization(global_scene_solver *gss) {
	if (gss->solve_counts > gss->solve_count_max && gss->solve_count_max > 0)
		return false;

	OGLockMutex(gss->scenes_lock);

	PoserDataGlobalScenes pgss = {
		.hdr = {.pt = POSERDATA_GLOBAL_SCENES}, .scenes_cnt = gss->scenes_cnt, .scenes = gss->scenes};
	if (pgss.scenes_cnt > GSS_NUM_STORED_SCENES)
		pgss.scenes_cnt = GSS_NUM_STORED_SCENES;
	gss->solve_counts++;

	bool success = gss->ctx->PoserFn(gss->ctx->objs[0], (PoserData *)&pgss) == 0;
	if(success) {
		if(gss->auto_floor) {
			FLT min_z = gss->ctx->floor_offset;
			for (int i = 0; i < gss->scenes_cnt; i++) {
				min_z = linmath_min(min_z, gss->scenes[i].pose.Pos[2]);
			}
			if (isfinite(min_z))
				survive_set_floor_offset(gss->ctx, min_z);
		}

		for (int i = 0; i < gss->scenes_cnt; i++) {
			SurvivePose p = gss->scenes[i].pose;

			if (!quatiszero(p.Rot)) {
				p.Pos[2] -= gss->ctx->floor_offset;
				survive_recording_write_to_output(gss->ctx->recptr, "SPHERE %s_%d %f %d " Point3_format "\n",
												  gss->scenes[i].so->codename, (int)gss->scenes_cnt, .05, 0xFF,
												  LINMATH_VEC3_EXPAND(p.Pos));
			}
		}
	}

	OGUnlockMutex(gss->scenes_lock);
	return success;
}

static void notify_global_data_available(global_scene_solver *gss, SurviveObject *so) {
	PoserDataGlobalScenes pgss = {.hdr = {.pt = POSERDATA_GLOBAL_SCENES}, .scenes_cnt = 0, .scenes = 0};

	gss->ctx->PoserFn(so, (PoserData *)&pgss);
}

static void check_for_new_objects(global_scene_solver *gss) {
	if (OGTryLockMutex(gss->scenes_lock) != 0)
		return;

	SurviveContext *ctx = gss->ctx;
	if (ctx->objs_ct > gss->last_capture_time_cnt) {
		gss->last_capture_time = SV_REALLOC(gss->last_capture_time, ctx->objs_ct * sizeof(survive_long_timecode));
		for (int i = gss->last_capture_time_cnt; i < ctx->objs_ct; i++) {
			gss->last_capture_time[i] = 0;

			notify_global_data_available(gss, ctx->objs[i]);
		}
		gss->last_capture_time_cnt = ctx->objs_ct;
	}

	OGUnlockMutex(gss->scenes_lock);
}

static void set_needs_solve(global_scene_solver *gss) {
	FLT now = survive_run_time(gss->ctx);

	for (int lh = 0; lh < gss->ctx->activeLighthouses; lh++) {
		if (!gss->ctx->bsd[lh].OOTXSet)
			return;
	}

	gss->needsSolve = true;
	gss->last_addition = now;
}

static size_t check_object(global_scene_solver *gss, int i, SurviveObject *so) {
	if (gss->solve_counts > gss->solve_count_max && gss->solve_count_max > 0)
		return false;

	if (OGTryLockMutex(gss->scenes_lock) != 0)
		return 0;

	size_t scenes_added = 0;
	SurviveContext *ctx = gss->ctx;

	survive_long_timecode last_event_time = SurviveSensorActivations_last_time(&so->activations);
	survive_long_timecode last_change = so->activations.last_light_change;
	survive_long_timecode standstill_time = SurviveSensorActivations_stationary_time(&so->activations);
	survive_long_timecode lockout_time = so->timebase_hz * .1;

	bool activations_changed = so->activations.last_light_change != gss->last_capture_time[i];
	bool spreadout = (last_event_time - gss->last_capture_time[i]) > so->timebase_hz * 3;
	bool light_static = (last_event_time - last_change) > lockout_time;
	bool not_moving = (standstill_time > SurviveSensorActivations_default_tolerance * 16);
	// SV_VERBOSE(100, "%s %d %d %d %d %lu", survive_colorize_codename(so), activations_changed, spreadout,
	// light_static, not_moving, gss->last_capture_time[i]);
	if (activations_changed && spreadout && light_static && not_moving) {
		size_t new_scenes = add_scenes(gss, so);
		if (new_scenes) {
			scenes_added += new_scenes;
			SV_VERBOSE(10, "Adding scene (%d) for %s at %6.4f (%f)", (int)gss->scenes_cnt % GSS_NUM_STORED_SCENES,
					   so->codename, survive_run_time(ctx),
					   SurviveSensorActivations_stationary_time(&so->activations) / 48000000.);
		}
		gss->last_capture_time[i] = so->activations.last_light_change;
	}

	if (scenes_added) {
		set_needs_solve(gss);
	}

	OGUnlockMutex(gss->scenes_lock);

	if (gss->needsSolve && (gss->last_addition + 1) < survive_run_time(ctx)) {
		if (!gss->threaded) {
			gss->needsSolve = false;
			run_optimization(gss);
		} else {
			OGLockMutex(gss->data_available_lock);
			OGSignalCond(gss->data_available);
			OGUnlockMutex(gss->data_available_lock);
		}
	}

	return scenes_added;
}

static int DriverRegGlobalSceneSolverPoll(struct SurviveContext *ctx, void *driver) { return 0; }

static int DriverRegGlobalSceneSolverClose(struct SurviveContext *ctx, void *driver) {
	global_scene_solver *gss = (global_scene_solver *)driver;
	global_scene_solver_detach_config(ctx, driver);

	SV_VERBOSE(10, "Global Scene Solver:");
	SV_VERBOSE(10, "\tScenes:       %8d", (int)gss->scenes_cnt);
	for (int i = 0; i < ctx->activeLighthouses; i++) {
		for (int j = 0; j < 2; j++) {
			SV_VERBOSE(10, "\tCoverage %02d.%02d     %4d %4d %4d %4d %4d ", i, j,
					   LINMATH_VEC5_EXPAND(gss->coverage[i][j]));
		}
	}

	if (gss->threaded) {
		OGLockMutex(gss->data_available_lock);
		gss->needsSolve = 0;
		gss->active = 0;
		OGSignalCond(gss->data_available);
		OGUnlockMutex(gss->data_available_lock);
		OGJoinThread(gss->thread);
		OGDeleteConditionVariable(gss->data_available);

		OGDeleteMutex(gss->data_available_lock);
	}

	OGDeleteMutex(gss->scenes_lock);

	free(gss->last_capture_time);
	for (int i = 0; i < GSS_NUM_STORED_SCENES; i++) {
		free(gss->scenes[i].meas);
	}
	free(driver);
	return 0;
}

static int survive_get_so_idx(const SurviveObject *so) {
	for (int i = 0; i < so->ctx->objs_ct; i++) {
		if (so == so->ctx->objs[i]) {
			return i;
		}
	}
	return -1;
}

static void light_pulse_fn(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode, FLT length,
						   uint32_t lh) {
	global_scene_solver *gss =
		(global_scene_solver *)survive_get_driver_by_closefn(so->ctx, DriverRegGlobalSceneSolverClose);
	gss->prior_light_pulse(so, sensor_id, acode, timecode, length, lh);

	check_for_new_objects(gss);
	check_object(gss, survive_get_so_idx(so), so);
}
static void imu_fn(SurviveObject *so, int mask, const FLT *accelgyro, survive_timecode timecode, int id) {
	global_scene_solver *gss =
		(global_scene_solver *)survive_get_driver_by_closefn(so->ctx, DriverRegGlobalSceneSolverClose);
	gss->imu_fn(so, mask, accelgyro, timecode, id);

	check_for_new_objects(gss);
	check_object(gss, survive_get_so_idx(so), so);
}
static void sync_fn(SurviveObject *so, survive_channel channel, survive_timecode timeofsync, bool ootx, bool gen) {
	global_scene_solver *gss =
		(global_scene_solver *)survive_get_driver_by_closefn(so->ctx, DriverRegGlobalSceneSolverClose);
	gss->prior_sync_fn(so, channel, timeofsync, ootx, gen);

	check_for_new_objects(gss);
	check_object(gss, survive_get_so_idx(so), so);
}

global_scene_solver *global_scene_solver_init(global_scene_solver *driver, SurviveContext *ctx) {
	driver->ctx = ctx;
	driver->last_capture_time_cnt = 0;
	driver->last_capture_time = SV_CALLOC_N(driver->last_capture_time_cnt, sizeof(survive_long_timecode) * 4);

	return driver;
}

static void ootx_recv(struct SurviveContext *ctx, uint8_t bsd_idx) {
	global_scene_solver *gss =
		(global_scene_solver *)survive_get_driver_by_closefn(ctx, DriverRegGlobalSceneSolverClose);

	gss->prior_ootx_fn(ctx, bsd_idx);

	set_needs_solve(gss);
}

void *survive_threaded_gss_thread_fn(void *_poser) {
	struct global_scene_solver *self = (struct global_scene_solver *)_poser;
	OGLockMutex(self->data_available_lock);
	while (self->active) {
		OGWaitCond(self->data_available, self->data_available_lock);

		while (self->needsSolve) {
			OGUnlockMutex(self->data_available_lock);
			self->needsSolve = false;
			survive_get_ctx_lock(self->ctx);
			run_optimization(self);
			survive_release_ctx_lock(self->ctx);
			self->run_count++;

			OGLockMutex(self->data_available_lock);
		}
	}
	OGUnlockMutex(self->data_available_lock);
	return 0;
}

int DriverRegGlobalSceneSolver(SurviveContext *ctx) {
	int disableCalibrate = survive_configi(ctx, "disable-calibrate", SC_GET, 0);

	if(disableCalibrate) {
		SV_VERBOSE(10, "Not running global solver since disable-calibrate is specfied");
		return SURVIVE_DRIVER_NORMAL;
	}

	global_scene_solver *driver = SV_NEW(global_scene_solver, ctx);
	global_scene_solver_attach_config(ctx, driver);

	int flag = survive_configi(ctx, GSS_ENABLE_TAG, SC_GET, 1);
	driver->solve_count_max = flag > 1 ? flag : -1;

	driver->imu_fn = survive_install_imu_fn(ctx, imu_fn);
	driver->prior_sync_fn = survive_install_sync_fn(ctx, sync_fn);
	driver->prior_light_pulse = survive_install_light_pulse_fn(ctx, light_pulse_fn);
	driver->prior_ootx_fn = survive_install_ootx_received_fn(ctx, ootx_recv);

	FLT playback_factor = survive_configf(ctx, "playback-factor", SC_GET, 1.);
	if (playback_factor == 0)
		driver->threaded = false;

	driver->scenes_lock = OGCreateMutex();

	if (driver->threaded) {
		driver->data_available = OGCreateConditionVariable();
		driver->data_available_lock = OGCreateMutex();
		driver->active = 1;
		driver->thread = OGCreateThread(survive_threaded_gss_thread_fn, "threaded poser", driver);
	}

	survive_add_driver(ctx, driver, DriverRegGlobalSceneSolverPoll, DriverRegGlobalSceneSolverClose);
	return SURVIVE_DRIVER_PASSIVE;
}

REGISTER_LINKTIME(DriverRegGlobalSceneSolver)
