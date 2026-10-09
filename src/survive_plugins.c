#ifndef SURVIVE_DISABLE_PLUGINS

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

#include "assert.h"

#ifdef _WIN32
#include "survive_plugins.windows.h"
#else
#include "survive_plugins.unix.h"
#endif

static const char *get_so_filename();
static void* survive_load_plugin(const char* path);
static const char *get_exe_filename();
static const char* plugin_ext();

static char *strip_filename(char *buffer) {
	int n = strlen(buffer);
	char *ptr = buffer + n;
	while (ptr > buffer && *ptr != '/' && *ptr != '\\')
		ptr--;

	if (*ptr == '/' || *ptr == '\\') {
		*ptr = 0;
		ptr++;
	}

	return ptr;
}

static bool has_suffix(const char *str, const char *suffix) {
	int s_len = strlen(str);
	int suffix_len = strlen(suffix);

	if (suffix_len > s_len)
		return false;

	return strcmp(str + s_len - suffix_len, suffix) == 0;
}

typedef struct list_t {
	size_t size;
	char **data;
} list_t;

static void list_add(list_t *list, const char *item) {
	list->size++;
	list->data = SV_REALLOC(list->data, sizeof(item) * list->size);
	list->data[list->size - 1] = SV_MALLOC(strlen(item) + 1);
	strcpy(list->data[list->size - 1], item);
}

static bool list_find(list_t* plugin_list, const char* item) {
	const char *item_name = strrchr(item, '/');
	item_name = item_name ? item_name + 1 : item;
	for (size_t i = 0; i < plugin_list->size; i++) {
		const char *plugin_path = plugin_list->data[i];
		const char *plugin_name = strrchr(plugin_path, '/');
		plugin_name = plugin_name ? plugin_name + 1 : plugin_path;
		// Build and install copies of a plugin must only be loaded once.
		if (strcmp(plugin_name, item_name) == 0) {
			return true;
		}
	}
	return false;
}

void survive_load_plugins(const char *plugin_dir) {
	// The basic strategy here is to compile a list of all possible plugins, then try to load them. Some
	// will fail if they have a dependency on other plugins which aren't loaded; and that is fine -- we
	// just keep loading the list until no new libraries are accepted.
	//
	// If there are still unresolved symbols, errors are reported.
	bool verbose = getenv("SURVIVE_PLUGIN_DEBUG") != 0;
	const char *check_from_files[] = {get_so_filename(), get_exe_filename(), getenv("SURVIVE_PLUGINS"), 0};
	const char *plugin_dirs[] = {"plugins", "libsurvive/plugins", plugin_dir, 0};

	list_t plugin_list = { 0 };

	for (const char **check_from_file = check_from_files; *check_from_file; check_from_file++) {
		const size_t dirname_len = strlen(*check_from_file) + 1;
		char *dirname = alloca(dirname_len);
		strcpy(dirname, *check_from_file);
		char *filename = strip_filename(dirname);

		for (const char **plugin_dir = plugin_dirs; *plugin_dir; plugin_dir++) {
			int plugindirname_len = strlen(*check_from_file) + strlen(*plugin_dir) + 2;
			char *plugindirname = alloca(plugindirname_len);
			strcpy(plugindirname, dirname);

			strcat(plugindirname, "/");
			strcat(plugindirname, *plugin_dir);

			if (verbose)
				printf("survive plugins: Looking in %s\n", plugindirname);

			DIR *dir_handle = opendir(plugindirname);
			if (dir_handle == 0)
				continue;

			struct dirent *dir_entry = 0;
			while (dir_entry = readdir(dir_handle)) {
				if (strcmp(dir_entry->d_name, filename) != 0 && has_suffix(dir_entry->d_name, plugin_ext())) {
					char full_path[1024] = { 0 };
					snprintf(full_path, 1024, "%s/%s", plugindirname, dir_entry->d_name);

					if (!list_find(&plugin_list, full_path)) {
						if (verbose) {
							printf("survive plugins: Adding %s to plugin check list\n", full_path);
						}
						list_add(&plugin_list, full_path);
					}
				}
			}

			closedir(dir_handle);
		}
	}

	bool change = true;
	while (change) {
		change = false;
		for (size_t i = 0; i < plugin_list.size; i++) {
			char *plugin_path = plugin_list.data[i];
			if (plugin_path) {
				// Global is important to share symbols
				void *handle = survive_load_plugin(plugin_path);
				if (handle) {
					if (verbose) {
						printf("survive plugins: Loaded %s\n", plugin_path);
					}
					change = true;
					free(plugin_path);
					plugin_list.data[i] = 0;
				} else {
					if (verbose) {
						printf("Initial error for %s; will retry: %s\n", plugin_path, survive_load_plugin_error());
					}
				}
			}
		}
	}

	for (size_t i = 0; i < plugin_list.size; i++) {
		if (plugin_list.data[i]) {
			void *handle = survive_load_plugin(plugin_list.data[i]);
			assert(handle == false);
			fprintf(stderr, "Error loading %s: %s\n", plugin_list.data[i], survive_load_plugin_error());
			free(plugin_list.data[i]);
			plugin_list.data[i] = 0;
		}
	}

	free(plugin_list.data);
}
#else
void survive_load_plugins(const char *plugin_dir) {}
#endif
