/*
 * dataFilter.h
 *
 *  Created on: Mar 7, 2025
 *      Author: coope
 */

#ifndef INC_DATAFILTER_H_
#define INC_DATAFILTER_H_

#include <stdio.h>
#include <stdlib.h>

typedef struct {
    float *buffer;  	// Buffer penyimpanan data
    int index;     	// Index untuk pergeseran data
    int size;
} MovingMedian;

typedef struct {
    float *buffer;  	// Buffer nilai terakhir
    float sum;  	// Total nilai dalam jendela
    int index;  	// Indeks posisi dalam buffer
    int count; 	 	// Jumlah elemen yang sudah masuk
    int size;
} MovingAverage;

typedef struct {
    float alpha;  // Faktor smoothing
    float value;  // Nilai EMA terakhir
} ExponentialMovingAverage;

typedef struct{
	float X;   	// Estimasi awal
	float P;   	// Perkiraan kesalahan awal
	float Q;  	// Noise sistem (proses)
	float R;   	// Noise pengukuran (sensor)
	float K;	// Kalman Gain
}KalmanFilter;

typedef struct{
	MovingMedian *MM;
	MovingAverage *MA;
	ExponentialMovingAverage *EMA;
	KalmanFilter *KF;
}filter;


MovingMedian* createMovingMedian(int window_size);
MovingAverage* createMovingAverage(int window_size);
ExponentialMovingAverage* createExponentialMovingAverage(float alpha);
KalmanFilter* createKalmanFilter(float P, float Q, float R);

float computeMedian(float *arr, int size);
float movingMedian(MovingMedian *mm, float new_value);

float movingAverage(MovingAverage *ma, float new_value);

float exponentialMovingAverage(ExponentialMovingAverage *ema, float newValue);

float kalmanFilterAdaptif(KalmanFilter *kf, float measurement, float alpha, float beta);
float kalmanFilter(KalmanFilter *kf, float measurement);

#endif /* INC_DATAFILTER_H_ */
