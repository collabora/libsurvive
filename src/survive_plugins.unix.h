#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dirent.h>
#include <dlfcn.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#include "assert.h"

#pragma GCC diagnostic ignored "-Wpedantic"

#ifdef __APPLE__
static const char* plugin_ext() { return ".dylib"; }
#else
static const char* plugin_ext() { return ".so"; }
#endif

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

static const char *get_so_filename() {
	static char so_path[PATH_MAX] = {0};
	if (so_path[0] == 0) {

		Dl_info dl_info;
		dladdr((void *)get_so_filename, &dl_info);
		char *dst = realpath(dl_info.dli_fname, so_path);
		if (dst == 0) {
			strncpy(so_path, dl_info.dli_fname, sizeof(so_path) - 1);
		}
	}
	return so_path;
}

static const char *get_exe_filename() {
	static char exe_path[PATH_MAX] = { 0 };
	if (exe_path[0] == 0) {
#ifdef __APPLE__
		uint32_t size = sizeof(exe_path);
		if (_NSGetExecutablePath(exe_path, &size) != 0) {
			exe_path[0] = 0;
		}
#else
		ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
		if (len > 0) {
			exe_path[len] = 0;
		} else {
			exe_path[0] = 0;
		}
#endif
	}
	return exe_path;
}

static void* survive_load_plugin(const char* path) {
	return  dlopen(path, RTLD_NOW | RTLD_GLOBAL);
}

const char* survive_load_plugin_error() {
	return dlerror();
}
