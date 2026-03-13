#pragma once
#include "linmath.h"

struct variance_measure {
	size_t size;
	FLT sum[16];
	FLT sumSq[16];
	size_t n;
};

static inline void variance_measure_add(struct variance_measure *meas, const FLT *d) {
	if (meas->size == 0)
		meas->size = 1;

	/* Guard against non-finite values. Corrupt optical angles (e.g. bad FPGA
	 * timestamps during USB disturbances) can reach here as NaN or Inf. The
	 * assert below would crash the process and corrupt the on-disk config;
	 * dropping one sample is strictly safer and has negligible effect on the
	 * variance estimate. */
	for (int i = 0; i < meas->size; i++) {
		if (!isfinite(d[i])) {
			fprintf(stderr, "[libsurvive] variance_measure_add: non-finite d[%d]=%f, dropping measurement\n", i, (double)d[i]);
			return;
		}
	}

	meas->n++;
	addnd(meas->sum, meas->sum, d, meas->size);
	for (int i = 0; i < meas->size; i++) {
		meas->sumSq[i] += d[i] * d[i];
	}
}

static inline void variance_measure_reset(struct variance_measure *meas) {
	meas->n = 0;
	memset(meas->sum, 0, sizeof(FLT) * meas->size);
	memset(meas->sumSq, 0, sizeof(FLT) * meas->size);
}
static inline void variance_measure_calc(struct variance_measure *meas, FLT *d) {
	if (meas->n == 0) {
		memset(d, 0, meas->n * sizeof(FLT));
		return;
	}

	for (int i = 0; i < meas->size; i++) {
		d[i] = (meas->sumSq[i] - (meas->sum[i] * meas->sum[i]) / meas->n) / meas->n;
	}
}

struct variance_tracker {
	FLT variances[16];
	size_t counts;
	struct variance_measure variance;
};

static inline void variance_tracker_reset(struct variance_tracker *meas) {
	if (meas->variance.n == 0)
		return;

	FLT v[16] = {0};
	variance_measure_calc(&meas->variance, v);
	addnd(meas->variances, v, meas->variances, meas->variance.size);
	meas->counts += meas->variance.n;
	variance_measure_reset(&meas->variance);
}

static inline void variance_tracker_add(struct variance_tracker *meas, const FLT *d, size_t size) {
	meas->variance.size = size;
	variance_measure_add(&meas->variance, d);
}
static inline void variance_tracker_calc(struct variance_tracker *meas, FLT *d) {
	scalend(d, meas->variances, 1. / meas->counts, meas->variance.size);
}