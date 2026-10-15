/*
 * MAF.c
 *
 *  Created on: 2026. 9. 16.
 *      Author: h04
 */
#include "MAF.h"

MovingAvgFilter stMAF_HVI_out;
MovingAvgFilter stMAF_LVV_out;
MovingAvgFilter stMAF_LVI_out;

// -----------------------------------------------------------------------------
// 2. 필터 초기화 함수
// -----------------------------------------------------------------------------
void init_moving_avg_filter(MovingAvgFilter* filter) {
    uint16_t i;

    filter->index = 0;
    filter->sum = 0;
    filter->filtered_val = 0;

    // 버퍼의 모든 요소를 0으로 초기화
    for(i = 0; i < FILTER_SIZE; i++) {
        filter->buffer[i] = 0;
    }
}

void init_MAF(void){
    init_moving_avg_filter(&stMAF_HVI_out);
    init_moving_avg_filter(&stMAF_LVV_out);
    init_moving_avg_filter(&stMAF_LVI_out);
}

// -----------------------------------------------------------------------------
// 3. 이동 평균 필터 핵심 연산 함수
// -----------------------------------------------------------------------------
float32_t apply_moving_avg(MovingAvgFilter* filter, const float32_t new_adc_raw) {
    // [단계 1] 누적 합(sum)에서 가장 오래된 데이터(버퍼의 현재 위치에 있는 값)를 뺍니다.
    filter->sum -= filter->buffer[filter->index];

    // [단계 2] 버퍼의 현재 위치에 새로운 ADC 데이터를 저장합니다.
    filter->buffer[filter->index] = new_adc_raw;

    // [단계 3] 누적 합(sum)에 새 데이터를 더합니다.
    filter->sum += new_adc_raw;

    // [단계 4] 다음 데이터 저장을 위해 인덱스를 1 증가시킵니다.
    filter->index++;

    // [단계 5] 인덱스가 버퍼 크기에 도달하면 0으로 되돌려 원형 버퍼 구조를 만듭니다.
    if (filter->index >= FILTER_SIZE) {
        filter->index = 0;
    }

    // [단계 6] 총합을 샘플 개수(FILTER_SIZE)로 나누어 평균값을 구합니다.
    // (FILTER_SIZE가 2의 거듭제곱일 경우 비트 시프트 연산 'sum >> 3' 으로 치환하면 MCU 속도가 빨라집니다)
    filter->filtered_val = (float32_t)(filter->sum * 0.125f);
    //비트 시프트는 정수만 돼서 플록은 나눗셈을 그냥 곱하기로 바꿈

    return filter->filtered_val;
}
