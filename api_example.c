#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

static volatile int keepRunning = 1;

#ifdef __linux__

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

void intHandler(int dummy) {
	if (keepRunning == 0)
		exit(-1);
	keepRunning = 0;
}

#endif

static void log_fn(SurviveSimpleContext *actx, SurviveLogLevel logLevel, const char *msg) {
	fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

int main(int argc, char **argv) {
#ifdef __linux__
	signal(SIGINT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGKILL, intHandler);
#endif

	SurviveSimpleContext *actx = survive_simple_init_with_logger(argc, argv, log_fn);
	if (actx == 0) // implies -help or similiar
		return 0;

	double start_time = OGGetAbsoluteTime();
	survive_simple_start_thread(actx);

	for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
		 it = survive_simple_get_next_object(actx, it)) {
		printf("Found '%s'\n", survive_simple_object_name(it));
	}

    struct SurviveSimpleEvent event = {0};
	while (keepRunning && survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown) {
		switch (event.event_type) {
		case SurviveSimpleEventType_PoseUpdateEvent: {
			const struct SurviveSimplePoseUpdatedEvent *pose_event = survive_simple_get_pose_updated_event(&event);
			SurvivePose pose = pose_event->pose;
			FLT timecode = pose_event->time;
			printf("%s %s (%7.3f): pos [%f, %f, %f] quat [%f, %f, %f, %f]\n",
				   survive_simple_object_name(pose_event->object),
				   survive_simple_serial_number(pose_event->object), timecode,
				   pose.Pos[0], pose.Pos[1], pose.Pos[2],
				   pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
			fflush(stdout);
			break;
		}
		case SurviveSimpleEventType_ButtonEvent: {
			const struct SurviveSimpleButtonEvent *button_event = survive_simple_get_button_event(&event);
			SurviveObjectSubtype subtype = survive_simple_object_get_subtype(button_event->object);
			printf("%s input %s (%d) ", survive_simple_object_name(button_event->object),
				   SurviveInputEventStr(button_event->event_type), button_event->event_type);

			FLT v1 = survive_simple_object_get_input_axis(button_event->object, SURVIVE_AXIS_TRACKPAD_X) / 2. + .5;

			if (button_event->button_id != 255) {
				printf(" button %16s (%2d) ", SurviveButtonsStr(subtype, button_event->button_id),
					   button_event->button_id);

				if (button_event->button_id == SURVIVE_BUTTON_SYSTEM) {
					FLT v = 1 - survive_simple_object_get_input_axis(button_event->object, SURVIVE_AXIS_TRIGGER);
					survive_simple_object_haptic(button_event->object, 30, v, .5);
				}
			}
			for (int i = 0; i < button_event->axis_count; i++) {
				printf(" %20s (%2d) %+5.4f   ", SurviveAxisStr(subtype, button_event->axis_ids[i]),
					   button_event->axis_ids[i], button_event->axis_val[i]);
			}
			printf("\n");
			break;
		}
		case SurviveSimpleEventType_ConfigEvent: {
			const struct SurviveSimpleConfigEvent *cfg_event = survive_simple_get_config_event(&event);
			printf("(%f) %s received configuration of length %u type %d-%d\n", cfg_event->time,
				   survive_simple_object_name(cfg_event->object), (unsigned)strlen(cfg_event->cfg),
				   survive_simple_object_get_type(cfg_event->object),
				   survive_simple_object_get_subtype(cfg_event->object));
			break;
		}
		case SurviveSimpleEventType_DeviceAdded: {
			const struct SurviveSimpleObjectEvent *obj_event = survive_simple_get_object_event(&event);
			printf("(%f) Found '%s'\n", obj_event->time, survive_simple_object_name(obj_event->object));
			break;
		}
		case SurviveSimpleEventType_None:
			break;
		}
	}

	printf("Cleaning up\n");
	survive_simple_close(actx);
	return 0;
}
