#define SURVIVE_ENABLE_FULL_API
#include "stdbool.h"

#include <assert.h>
#include <survive_api.h>

#include "inttypes.h"
#include "os_generic.h"
#include "stdio.h"
#include "string.h"
#include "survive.h"

struct SurviveExternalObject {
	SurvivePose pose;
	SurviveVelocity velocity;
};

struct SurviveLighthouseData {
	int lighthouse;
	char serial_number[16];
};

struct SurviveSimpleObject {
	struct SurviveSimpleContext *actx;

	enum SurviveSimpleObject_type type;

	union {
		struct SurviveLighthouseData lh;
		struct SurviveObject *so;
		struct SurviveExternalObject seo;
	} data;

	char name[32];
	bool has_update;

	SurviveSimpleObject *next;
};

struct SurviveSimpleObjectList {
	size_t cnt;
	SurviveSimpleObject *head, *tail;
};

#define MAX_EVENT_SIZE 64
struct SurviveSimpleContext {
	SurviveContext *ctx;
	SurviveSimpleLogFn log_fn;
	void *user;

	bool running;
	og_thread_t thread;
	og_mutex_t poll_mutex;
	og_cv_t update_cv;

	size_t events_cnt;
	size_t event_next_write;
	struct SurviveSimpleEvent events[MAX_EVENT_SIZE];

	struct SurviveSimpleObjectList objects;
};

static enum SurviveSimpleObject_type to_simple_type(SurviveObjectType sot) {
	switch (sot) {
		case SURVIVE_OBJECT_TYPE_HMD:
			return SurviveSimpleObject_HMD;
		case SURVIVE_OBJECT_TYPE_OTHER:
		case SURVIVE_OBJECT_TYPE_CONTROLLER:
			return SurviveSimpleObject_OBJECT;
		default:
			return SurviveSimpleObject_UNKNOWN;
	}
}

static void unlock_and_notify_change(SurviveSimpleContext *actx) {
	OGBroadcastCond(actx->update_cv);
	OGUnlockMutex(actx->poll_mutex);
}

static void insert_into_event_buffer(SurviveSimpleContext *actx, const SurviveSimpleEvent *event) {
	bool buffer_full = actx->events_cnt == MAX_EVENT_SIZE;

	actx->events[actx->event_next_write] = *event;
	actx->event_next_write = (actx->event_next_write + 1) % MAX_EVENT_SIZE;
	if (!buffer_full)
		actx->events_cnt++;
	unlock_and_notify_change(actx);
}

static bool pop_from_event_buffer(SurviveSimpleContext *actx, SurviveSimpleEvent *event) {
	if (actx->events_cnt == 0)
		return false;

	size_t read_idx = (MAX_EVENT_SIZE + actx->event_next_write - actx->events_cnt) % MAX_EVENT_SIZE;
	*event = actx->events[read_idx];

	// Mark read events as invalid and assert if they are about to be passed on. This means there is a logic bug in the
	// circular buffer
	assert(event->event_type != -1);
	actx->events[read_idx].event_type = -1;

	actx->events_cnt--;
	return true;
}

static void SurviveSimpleObjectList_add(struct SurviveSimpleObjectList *list, SurviveSimpleObject *so) {
	list->cnt++;
	if (list->head == 0) {
		list->head = so;
	} else {
		assert(list->tail);
		list->tail->next = so;
	}

	list->tail = so;
}

static SurviveSimpleObject *find_or_create_external(SurviveSimpleContext *actx, const char *name) {
	for (struct SurviveSimpleObject *so = actx->objects.head; so; so = so->next) {
		if (strncmp(name, so->name, 32) == 0) {
			return so;
		}
	}

	SurviveSimpleObject *so = SV_CALLOC(sizeof(struct SurviveSimpleObject));
	so->type = SurviveSimpleObject_EXTERNAL;
	so->actx = actx;
	strncpy(so->name, name, sizeof(so->name) - 1);
	SurviveSimpleObjectList_add(&actx->objects, so);

	return so;
}

static void external_velocity_fn(SurviveContext *ctx, const char *name, const SurviveVelocity *velocity) {
	SurviveSimpleContext *actx = ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_external_velocity_process(ctx, name, velocity);

	SurviveSimpleObject *so = find_or_create_external(actx, name);
	so->has_update = true;
	so->data.seo.velocity = *velocity;
	unlock_and_notify_change(actx);
}

static void external_pose_fn(SurviveContext *ctx, const char *name, const SurvivePose *pose) {
	SurviveSimpleContext *actx = ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_external_pose_process(ctx, name, pose);

	SurviveSimpleObject *so = find_or_create_external(actx, name);
	so->has_update = true;
	so->data.seo.pose = *pose;
	unlock_and_notify_change(actx);
}
static void pose_fn(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *pose) {
	SurviveSimpleContext *actx = so->ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_pose_process(so, timecode, pose);

	struct SurviveSimpleObject *sao = so->user_ptr;
	sao->has_update = true;
	unlock_and_notify_change(actx);
}

static inline SurviveSimpleObject *create_lighthouse(SurviveSimpleContext *actx, size_t i) {
	SurviveSimpleObject *obj = SV_CALLOC(sizeof(struct SurviveSimpleObject));
	obj->data.lh.lighthouse = i;
	obj->type = SurviveSimpleObject_LIGHTHOUSE;
	obj->actx = actx;

	SurviveContext *ctx = actx->ctx;
	obj->has_update = ctx->bsd[i].PositionSet;
	ctx->bsd[i].user_ptr = obj;
	snprintf(obj->name, 32, "LH%" PRIdPTR, i);
	snprintf(obj->data.lh.serial_number, 16, "LHB-%X", (unsigned)ctx->bsd[i].BaseStationID);
	SurviveSimpleObjectList_add(&actx->objects, obj);

	OGLockMutex(actx->poll_mutex);
	SurviveSimpleEvent event = {.event_type = SurviveSimpleEventType_DeviceAdded,
								.d = {.object_event = {
										  .time = survive_simple_run_time_since_epoch(actx),
										  .object = obj,
									  }}};
	insert_into_event_buffer(actx, &event);

	return obj;
}

static void lh_fn(SurviveContext *ctx, uint8_t lighthouse, const SurvivePose *lighthouse_pose) {
	SurviveSimpleContext *actx = ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_lighthouse_pose_process(ctx, lighthouse, lighthouse_pose);

	struct SurviveSimpleObject *sao = ctx->bsd[lighthouse].user_ptr;
	if (sao == 0)
		sao = create_lighthouse(actx, lighthouse);
	sao->has_update = true;

	unlock_and_notify_change(actx);
}

static void button_fn(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
					  const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals) {
	SurviveSimpleContext *actx = so->ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_button_process(so, eventType, buttonId, axisIds, axisVals);
	struct SurviveSimpleObject *sao = so->user_ptr;

	sao->has_update = true;

	SurviveSimpleEvent event = {.event_type = SurviveSimpleEventType_ButtonEvent,
								.d = {.button_event = {
										  .time = survive_simple_run_time_since_epoch(actx),
										  .object = sao,
										  .event_type = eventType,
										  .button_id = buttonId,
									  }}};

	for (int i = 0; i < 16 && axisIds && axisIds[i] != 255; i++) {
		event.d.button_event.axis_count++;
		event.d.button_event.axis_ids[i] = axisIds[i];
		event.d.button_event.axis_val[i] = axisVals[i];
	}

	insert_into_event_buffer(actx, &event);
}

static int config_fn(struct SurviveObject *so, char *ct0conf, int len) {
	int res = survive_default_config_process(so, ct0conf, len);
	SurviveSimpleContext *actx = so->ctx->user_ptr;

	OGLockMutex(actx->poll_mutex);
	SurviveSimpleObject *sso = so->user_ptr;
	sso->type = to_simple_type(so->object_type);

	struct SurviveSimpleEvent event = {.event_type = SurviveSimpleEventType_ConfigEvent,
									   .d = {.config_event = {.time = survive_simple_run_time_since_epoch(actx),
															  .object = sso,
															  .cfg = survive_simple_json_config(sso)}}};

	insert_into_event_buffer(actx, &event);

	return res;
}

SURVIVE_EXPORT SurviveSimpleContext *survive_simple_init(int argc, char *const *argv) {
	return survive_simple_init_with_logger(argc, argv, 0);
}

SURVIVE_EXPORT void survive_simple_set_user(SurviveSimpleContext *actx, void *user) { actx->user = user; }
SURVIVE_EXPORT void *survive_simple_get_user(SurviveSimpleContext *actx) { return actx->user; }

static void simple_log_fn(SurviveContext *ctx, SurviveLogLevel logLevel, const char *msg) {
	SurviveSimpleContext *actx = ctx->user_ptr;
	if (actx == 0 || actx->log_fn == 0) {
		survive_default_log_process(ctx, logLevel, msg);
		return;
	}

	actx->log_fn(actx, logLevel, msg);
}

static void new_object_fn(SurviveObject *so) {
	SurviveSimpleContext *actx = so->ctx->user_ptr;

	SurviveSimpleObject *obj = SV_CALLOC(sizeof(struct SurviveSimpleObject));
	obj->data.so = so;
	obj->type = to_simple_type(so->object_type);
	obj->actx = actx;
	obj->data.so->user_ptr = (void *)obj;
	strncpy(obj->name, obj->data.so->codename, sizeof(obj->name));

	SurviveSimpleObjectList_add(&actx->objects, obj);

	OGLockMutex(actx->poll_mutex);
	survive_default_new_object_process(so);
	SurviveSimpleEvent event = {.event_type = SurviveSimpleEventType_DeviceAdded,
								.d = {.object_event = {
										  .time = survive_simple_run_time_since_epoch(actx),
										  .object = obj,
									  }}};
	insert_into_event_buffer(actx, &event);
}

SURVIVE_EXPORT SurviveSimpleContext *survive_simple_init_with_logger(int argc, char *const *argv,
																	 SurviveSimpleLogFn fn) {
	SurviveSimpleContext *actx = SV_CALLOC(sizeof(SurviveSimpleContext));
	actx->log_fn = fn;

	SurviveContext *ctx = survive_init_with_logger(argc, argv, actx, simple_log_fn);
	if (ctx == 0) {
		free(actx);
		return 0;
	}

	survive_install_new_object_fn(ctx, new_object_fn);

	if (ctx == 0) {
		free(actx);
		return 0;
	}
	ctx->user_ptr = actx;
	actx->ctx = ctx;
	actx->poll_mutex = OGCreateMutex();
	actx->update_cv = OGCreateConditionVariable();

	survive_startup(ctx);

	intptr_t i = 0;
	for (i = 0; i < ctx->activeLighthouses; i++) {
		create_lighthouse(actx, i);
	}

	survive_install_pose_fn(ctx, pose_fn);
	survive_install_external_pose_fn(ctx, external_pose_fn);
	survive_install_external_velocity_fn(ctx, external_velocity_fn);
	survive_install_button_fn(ctx, button_fn);
	survive_install_lighthouse_pose_fn(ctx, lh_fn);
	survive_install_config_fn(ctx, config_fn);
	return actx;
}

int survive_simple_stop_thread(SurviveSimpleContext *actx) {
	actx->running = false;
	intptr_t error = (intptr_t)OGJoinThread(actx->thread);
	actx->thread = 0;
	if (error != 0) {
		SurviveContext *ctx = actx->ctx;
		SV_INFO("Warning: Loop exited with error %" PRIdPTR, error);
	}
	return error;
}

void survive_simple_close(SurviveSimpleContext *actx) {
	if (actx->running) {
		survive_simple_stop_thread(actx);
	}

	survive_close(actx->ctx);

	for (struct SurviveSimpleObject *n = actx->objects.head; n;) {
		struct SurviveSimpleObject *freeMe = n;
		n = n->next;
		free(freeMe);
	}

	OGDeleteMutex(actx->poll_mutex);
	OGJoinThread(actx->thread);

	OGDeleteConditionVariable(actx->update_cv);
	actx->thread = 0;
	free(actx);
}

static inline void *__simple_thread(void *_actx) {
	SurviveSimpleContext *actx = _actx;
	intptr_t error = 0;
	while (actx->running && error == 0) {
		error = survive_poll(actx->ctx);
	}
	actx->running = false;
	return (void*)error; 
}
bool survive_simple_is_running(SurviveSimpleContext *actx) { return actx->running; }
void survive_simple_start_thread(SurviveSimpleContext *actx) {
	actx->running = true;
	actx->thread = OGCreateThread(__simple_thread, "survive simple api", actx);
}

const SurviveSimpleObject *survive_simple_get_next_object(SurviveSimpleContext *actx, const SurviveSimpleObject *curr) {
	return curr->next;
}

SurviveSimpleObject *survive_simple_get_object(SurviveSimpleContext *actx, const char *name) {
	for (struct SurviveSimpleObject *n = actx->objects.head; n; n = n->next) {
		if (strcmp(name, n->name) == 0) {
			return n;
		}
	}
	return 0;
}

const SurviveSimpleObject *survive_simple_get_first_object(SurviveSimpleContext *actx) { return actx->objects.head; }

size_t survive_simple_get_object_count(SurviveSimpleContext *actx) { return actx->objects.cnt; }

const SurviveSimpleObject *survive_simple_get_next_updated(SurviveSimpleContext *actx) {
	for (struct SurviveSimpleObject *n = actx->objects.head; n; n = n->next) {
		if (n->has_update) {
			n->has_update = false;
			return n;
		}
	}
	return 0;
}

FLT survive_simple_object_get_latest_velocity(const SurviveSimpleObject *sao, SurviveVelocity *velocity) {
	FLT timecode = 0;
	OGLockMutex(sao->actx->poll_mutex);

	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE:
		if (velocity)
			*velocity = (SurviveVelocity){ 0 };
		timecode = survive_simple_run_time_since_epoch(sao->actx);
		break;
	case SurviveSimpleObject_HMD:
	case SurviveSimpleObject_OBJECT:
		if (velocity)
			*velocity = sao->data.so->velocity;
		timecode = SurviveSensorActivations_runtime(&sao->data.so->activations, sao->data.so->velocity_timecode) * 1e-6;
		break;
	case SurviveSimpleObject_EXTERNAL:
		if (velocity)
			*velocity = sao->data.seo.velocity;
		break;

	default: {
		SurviveContext *ctx = sao->actx->ctx;
		SV_GENERAL_ERROR("Invalid object type %d", sao->type);
	}
	}

	OGUnlockMutex(sao->actx->poll_mutex);
	return timecode;
}

void survive_simple_object_get_transform_to_imu(const SurviveSimpleObject *sao, SurvivePose *pose) {
    *pose = LinmathPose_Identity;
    const SurviveObject *so = survive_simple_get_survive_object(sao);
    if (so == 0)
        return;
    *pose = so->head2imu;
}

FLT survive_simple_object_get_latest_pose(const SurviveSimpleObject *sao, SurvivePose *pose) {
	FLT timecode = 0;
	OGLockMutex(sao->actx->poll_mutex);

	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE: {
		if (pose)
			*pose = *survive_get_lighthouse_position(sao->actx->ctx, sao->data.lh.lighthouse);
		timecode = survive_simple_run_time_since_epoch(sao->actx);
		break;
	}
	case SurviveSimpleObject_HMD:
	case SurviveSimpleObject_OBJECT:
		if (pose)
			*pose = sao->data.so->OutPose;
		timecode = SurviveSensorActivations_runtime(&sao->data.so->activations, sao->data.so->OutPose_timecode) * 1e-6;
		break;
	case SurviveSimpleObject_EXTERNAL:
		if (pose) {
			ApplyPoseToPose(pose, survive_external_to_world(sao->actx->ctx), &sao->data.seo.pose);
		}
		break;

	default: {
		SurviveContext *ctx = sao->actx->ctx;
		SV_GENERAL_ERROR("Invalid object type %d", sao->type);
	}
	}

	OGUnlockMutex(sao->actx->poll_mutex);
	return timecode;
}

SURVIVE_EXPORT bool survive_simple_object_charging(const SurviveSimpleObject *sao) {
	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE: {
		return true;
	}
	case SurviveSimpleObject_HMD:
	case SurviveSimpleObject_OBJECT:
		return sao->data.so->charging;
	case SurviveSimpleObject_EXTERNAL:
	default:
		return false;
	}
}

SURVIVE_EXPORT uint8_t survive_simple_object_charge_percent(const SurviveSimpleObject *sao) {
	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE: {
		return 100;
	}
	case SurviveSimpleObject_HMD:
	case SurviveSimpleObject_OBJECT:
		return sao->data.so->charge;
	case SurviveSimpleObject_EXTERNAL:
	default:
		return 0;
	}
}

const char *survive_simple_object_name(const SurviveSimpleObject *sao) { return sao->name; }
const char *survive_simple_serial_number(const SurviveSimpleObject *sao) {
	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE: {
		return sao->data.lh.serial_number;
	}
	case SurviveSimpleObject_HMD:
	case SurviveSimpleObject_OBJECT:
		return sao->data.so->serial_number;
	case SurviveSimpleObject_EXTERNAL:
	default:
		return survive_simple_object_name(sao);
	}
}
SURVIVE_EXPORT const char *survive_simple_json_config(const SurviveSimpleObject *sao) {
	switch (sao->type) {
	case SurviveSimpleObject_HMD:
	case SurviveSimpleObject_OBJECT:
		return sao->data.so->conf;
	case SurviveSimpleObject_EXTERNAL:
	case SurviveSimpleObject_LIGHTHOUSE:
	default:
		return 0;
	}
	return 0;
}

void survive_simple_lock(SurviveSimpleContext *actx) { OGLockMutex(actx->poll_mutex); }

void survive_simple_unlock(SurviveSimpleContext *actx) { OGUnlockMutex(actx->poll_mutex); }

SurviveContext *survive_simple_get_ctx(SurviveSimpleContext *actx) { return actx->ctx; }

SurviveObject *survive_simple_get_survive_object(const SurviveSimpleObject *sao) {
	switch (sao->type) {
	case SurviveSimpleObject_HMD:
	case SurviveSimpleObject_OBJECT:
		return sao->data.so;
	default:
		return NULL;
	}
}

BaseStationData *survive_simple_get_bsd(const SurviveSimpleObject *sao) {
	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE:
		return &sao->actx->ctx->bsd[sao->data.lh.lighthouse];
	default:
		return NULL;
	}
}

const SurviveSimpleConfigEvent *survive_simple_get_config_event(const SurviveSimpleEvent *event) {
	if (event->event_type == SurviveSimpleEventType_ConfigEvent)
		return &event->d.config_event;
	return NULL;
}

const SurviveSimpleButtonEvent *survive_simple_get_button_event(const SurviveSimpleEvent *event) {
	if (event->event_type == SurviveSimpleEventType_ButtonEvent)
		return &event->d.button_event;
	return NULL;
}

const struct SurviveSimplePoseUpdatedEvent *survive_simple_get_pose_updated_event(const SurviveSimpleEvent *event) {
	if (event->event_type == SurviveSimpleEventType_PoseUpdateEvent) {
		return &event->d.pose_event;
	}
	return NULL;
}

bool survive_simple_wait_for_update(SurviveSimpleContext *actx) {
	OGLockMutex(actx->poll_mutex);
	OGWaitCondTimeout(actx->update_cv, actx->poll_mutex, 100);
	OGUnlockMutex(actx->poll_mutex);
	return survive_simple_is_running(actx);
}

enum SurviveSimpleEventType survive_simple_wait_for_event(SurviveSimpleContext *actx, SurviveSimpleEvent *event) {
	survive_simple_wait_for_update(actx);
	return survive_simple_next_event(actx, event);
}

enum SurviveSimpleEventType survive_simple_next_event(SurviveSimpleContext *actx, SurviveSimpleEvent *event) {
	event->event_type = SurviveSimpleEventType_None;

	OGLockMutex(actx->poll_mutex);
	pop_from_event_buffer(actx, event);
	OGUnlockMutex(actx->poll_mutex);

	if (event->event_type == SurviveSimpleEventType_None) {
		const SurviveSimpleObject *sso = survive_simple_get_next_updated(actx);
		if (sso) {
			event->event_type = SurviveSimpleEventType_PoseUpdateEvent;
			event->d.pose_event = (SurviveSimplePoseUpdatedEvent){
				.object = sso,
			};
			event->d.pose_event.time = survive_simple_object_get_latest_pose(sso, &event->d.pose_event.pose);
			survive_simple_object_get_latest_velocity(sso, &event->d.pose_event.velocity);
			return event->event_type;
		}
	}

	if (event->event_type == SurviveSimpleEventType_None && survive_simple_is_running(actx) == false) {
		return event->event_type = SurviveSimpleEventType_Shutdown;
	}

	return event->event_type;
}

enum SurviveSimpleObject_type survive_simple_object_get_type(const struct SurviveSimpleObject *sao) {
	return sao->type;
}
SurviveSimpleSubobject_type survive_simple_object_get_subtype(const struct SurviveSimpleObject *sao) {
	const SurviveObject *so = survive_simple_get_survive_object(sao);
	if (so == 0)
		return SURVIVE_OBJECT_SUBTYPE_GENERIC;

	return so->object_subtype;
}
int survive_simple_object_haptic(struct SurviveSimpleObject *sao, FLT frequency, FLT amplitude, FLT time_s) {
	SurviveObject *so = survive_simple_get_survive_object(sao);
	return survive_haptic(so, frequency, amplitude, time_s);
}
SurviveAxisVal_t survive_simple_object_get_input_axis(const struct SurviveSimpleObject *sao, enum SurviveAxis axis) {
	SurviveObject *so = survive_simple_get_survive_object(sao);
	if (so == 0 || axis > 16)
		return 0;

	return so->axis[axis];
}
SURVIVE_EXPORT int32_t survive_simple_object_get_button_mask(const struct SurviveSimpleObject *sao) {
    const SurviveObject *so = survive_simple_get_survive_object(sao);
    if (so == 0)
        return 0;
    return so->buttonmask;
}
SURVIVE_EXPORT int32_t survive_simple_object_get_touch_mask(const struct SurviveSimpleObject *sao)  {
    const SurviveObject *so = survive_simple_get_survive_object(sao);
    if (so == 0)
        return 0;
    return so->touchmask;
}

const struct SurviveSimpleObjectEvent *survive_simple_get_object_event(const SurviveSimpleEvent *event) {
	switch (event->event_type) {
	case SurviveSimpleEventType_DeviceAdded:
	case SurviveSimpleEventType_ButtonEvent:
	case SurviveSimpleEventType_ConfigEvent:
	case SurviveSimpleEventType_PoseUpdateEvent:
		return &event->d.object_event;
	case SurviveSimpleEventType_Shutdown:
		return NULL;
	}
	return NULL;
}
FLT survive_simple_run_time_since_epoch(const struct SurviveSimpleContext *actx) {
	if (actx->ctx == 0)
		return 0;
	return survive_run_time_since_epoch(actx->ctx);
}
FLT survive_simple_run_time(const struct SurviveSimpleContext *actx) {
	if (actx->ctx == 0)
		return 0;
	return survive_run_time(actx->ctx);
}
