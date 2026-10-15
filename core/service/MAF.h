/*
 * MAF.h
 *
 *  Created on: 2026. 9. 16.
 *      Author: h04
 */

#ifndef CORE_SERVICE_MAF_H_
#define CORE_SERVICE_MAF_H_

#include <stdio.h>
#include <stdint.h>
#include "SystemPara.h"

// -----------------------------------------------------------------------------
// 1. 이동 평균 필터 설정 및 데이터 구조체 정의
// -----------------------------------------------------------------------------

//10kHz 제어주기에 맞춰서 하려면 필터 사이즈는 4 or 8이 적당
//N = 4이면 지연시간이 10us(PI제어 주기의 10%) / N = 8이면 지연시간이 23.3us(PI제어 주기의 23%)
//delay = (N-1)/2 x Ts

#define FILTER_SIZE  8   // 이동 평균 필터의 샘플 개수 (N = 8)
                         // 2의 거듭제곱(2, 4, 8, 16...)으로 설정하면 나눗셈 연산 속도를 최적화

typedef struct {
    float32_t buffer[FILTER_SIZE]; // 최근 N개의 ADC 데이터를 저장할 원형 버퍼
    uint16_t index;               // 버퍼에서 현재 새로 들어올 데이터의 위치 (인덱스)
    float32_t sum;                 // 최근 N개 데이터의 총합 (매번 새로 계산하지 않고 누적 처리)
    float32_t filtered_val;        // 필터링이 완료된 최종 평균 결과값
} MovingAvgFilter;

extern MovingAvgFilter stMAF_HVI_out;
extern MovingAvgFilter stMAF_LVV_out;
extern MovingAvgFilter stMAF_LVI_out;

extern float32_t apply_moving_avg(MovingAvgFilter* filter, const float32_t new_adc_raw);
extern void init_MAF(void);

#endif /* CORE_SERVICE_MAF_H_ */
