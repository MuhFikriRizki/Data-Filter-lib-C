/*
 * dataFilter.c
 *
 *  Created on: Mar 7, 2025
 *      Author: coope
 */
#include "dataFilter.h"

/* Initialize Struct tiap filter BEGIN */
MovingMedian* createMovingMedian(int window_size) {
	MovingMedian *mm = (MovingMedian*)malloc(sizeof(MovingMedian));
    if (mm == NULL) return NULL; // Pastikan alokasi berhasil

    mm->buffer = (float*)malloc(window_size * sizeof(float));
    if (mm->buffer == NULL) {
        free(mm);
        return NULL; // Jika gagal alokasi buffer, bebaskan memori struct
    }

    mm->index = 0;
    mm->size = window_size;

    return mm;
}

MovingAverage* createMovingAverage(int window_size) {
    MovingAverage *ma = (MovingAverage*)malloc(sizeof(MovingAverage));
    if (ma == NULL) return NULL; // Pastikan alokasi berhasil

    ma->buffer = (float*)malloc(window_size * sizeof(float));
    if (ma->buffer == NULL) {
        free(ma);
        return NULL; // Jika gagal alokasi buffer, bebaskan memori struct
    }

    ma->sum = 0;
    ma->index = 0;
    ma->count = 0;
    ma->size = window_size;

    return ma;
}

ExponentialMovingAverage* createExponentialMovingAverage(float alpha){
	ExponentialMovingAverage *ema = (ExponentialMovingAverage*)malloc(sizeof(ExponentialMovingAverage));
    if (ema == NULL) return NULL; // Pastikan alokasi berhasil

    ema->alpha = alpha;
    ema->value = 0;

    return ema;
}

KalmanFilter* createKalmanFilter(float P, float Q, float R){
    KalmanFilter *kf = (KalmanFilter*)malloc(sizeof(KalmanFilter));
    if (kf == NULL) return NULL; // Pastikan alokasi berhasil

	kf->P = P;
	kf->Q = Q;
	kf->R = R;

	return kf;
}

/* Initialize Struct tiap filter END */


/* Program Moving Median BEGIN */

// **Fungsi untuk menghitung median**
float computeMedian(float *arr, int size) {
    float temp[size];

    // Copy array untuk pengurutan
    for (int i = 0; i < size; i++) {
        temp[i] = arr[i];
    }

    // Sort array dengan metode bubble sort (sederhana untuk data kecil)
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (temp[i] > temp[j]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    return temp[size / 2];  // Ambil nilai tengah (median)
}

// **Fungsi untuk memperbarui Moving Median**
float movingMedian(MovingMedian *mm, float new_value) {
    mm->buffer[mm->index] = new_value;  // Simpan nilai baru
    mm->index = (mm->index + 1) % mm->size;  // Perbarui indeks secara melingkar

    return computeMedian(mm->buffer, mm->size);  // Hitung dan kembalikan median
}

/* Program Moving Median END */


/* Program Moving Average BEGIN */

// Fungsi untuk menghitung moving average
float movingAverage(MovingAverage *ma, float new_value) {
    // Jika buffer belum penuh, tambahkan nilai baru tanpa menghapus nilai lama
    if (ma->count < ma->size) {
        ma->sum += new_value;
        ma->buffer[ma->index] = new_value;
        ma->count++;
    } else {
        // Jika buffer sudah penuh, geser window: kurangi nilai lama, tambahkan nilai baru
        ma->sum = ma->sum - ma->buffer[ma->index] + new_value;
        ma->buffer[ma->index] = new_value;
    }

    // Update indeks untuk memasukkan data berikutnya
    ma->index = (ma->index + 1) % ma->size;

    // Return hasil moving average
    return ma->sum / (ma->count < ma->size ? ma->count : ma->size);
}

/* Program Moving Average END */

/* Program Exponential Moving Average BEGIN */
float exponentialMovingAverage(ExponentialMovingAverage *ema, float newValue) {
	ema->value = ema->alpha * newValue + (1 - ema->alpha) * ema->value;

    return ema->value;
}
/* Program Exponential Moving Average END */

/* Program Kalman Filter BEGIN */

float kalmanFilterAdaptif(KalmanFilter *kf, float measurement, float alpha, float beta) {
	// Prediksi
	float x_pred = kf->X;
	float P_pred = kf->P + kf->Q;	// Prediksi (Tidak ada model dinamis, jadi hanya mempertahankan nilai sebelumnya)
    kf->K = P_pred / (P_pred + kf->R);	// Update (Kalman Gain)
    kf->X = x_pred + kf->K * (measurement - x_pred);	// Koreksi estimasi
    kf->P = (1 - kf->K) * P_pred;	// Update ketidakpastian estimasi

    // Adaptasi R (berdasarkan error sensor)
	float error = measurement - x_pred;
	kf->R = alpha * kf->R + (1 - alpha) * (error * error);

	// Adaptasi Q (berdasarkan perubahan estimasi)
	float delta_x = kf->X - x_pred;
	kf->Q = beta * kf->Q + (1 - beta) * (delta_x * delta_x);

    return kf->X;
}

float kalmanFilter(KalmanFilter *kf, float measurement) {
	kf->P = kf->P + kf->Q;	// Prediksi (Tidak ada model dinamis, jadi hanya mempertahankan nilai sebelumnya)
    kf->K = kf->P / (kf->P + kf->R);	// Update (Kalman Gain)
    kf->X = kf->X + kf->K * (measurement - kf->X);	// Koreksi estimasi
    kf->P = (1 - kf->K) * kf->P;	// Update ketidakpastian estimasi

    return kf->X;
}
/* Program Kalman Filter END */


