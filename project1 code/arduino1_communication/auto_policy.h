#ifndef AUTO_POLICY_H
#define AUTO_POLICY_H
#include <math.h>
#include "controller_types.h"
#include "scanner_config.h"

namespace AutoPolicy {
// z가 일정한 평면. r의 원점은 보호 영역 중심, theta=북0/서90/남180/동270도.
// vr: 바깥쪽 +, vtheta: 반시계 접선 방향 +. 두 속도 모두 mm/s.
// 예측 구간 동안 '직교 좌표계에서의 속도 벡터'가 일정하다고 가정한다.
// vr와 vtheta가 미래에도 일정하다는 가정은 아니다.
inline bool positionValid(const Target *target) {
    if (!target || !target->valid || !target->polar_valid || !target->id ||
        target->r_mm > SCANNER_MAX_RADIUS_MM || target->theta_cdeg >= 36000U) return false;
#if SCANNER_MIN_QUALITY > 0
    if (target->quality < SCANNER_MIN_QUALITY) return false;
#endif
    return true;
}

// 반환 true: 미래 경계 접촉 시간이 존재. 이미 내부/경계이면 0초.
// 접선으로 딱 닿는 경우도 거리 <= R 기준에 포함한다.
inline bool entryTimeSeconds(const Target *target, float *seconds) {
    if (!seconds || !positionValid(target)) return false;
    const float r = (float)target->r_mm;
    const float radius = (float)AUTO_DANGER_RADIUS_MM;
    if (r <= radius) { *seconds = 0.0f; return true; }
    if (!target->velocity_valid || target->vr_mm_s >= 0) return false;
    const float vr = (float)target->vr_mm_s;
    const float vt = (float)target->vtheta_mm_s;
    const float a = vr * vr + vt * vt;
    const float c = (r - radius) * (r + radius);
    // (vr^2 + vt^2)t^2 + 2*r*vr*t + (r^2-R^2) = 0.
    // 판별식 / 4. 접선 성분 때문에 단순히 (r-R)/(-vr)를 쓰면 안 된다.
    const float discriminant = a * radius * radius - r * r * vt * vt;
    if (a <= 0.0f || discriminant < 0.0f) return false;
    // 작은 근의 안정적인 형태. 비슷한 양수끼리 빼서 생기는 오차를 줄인다.
    *seconds = c / (-r * vr + sqrtf(discriminant));
    return *seconds >= 0.0f;
}

inline bool dangerous(const Target *target) {
    float seconds;
    return entryTimeSeconds(target, &seconds) &&
           seconds <= (float)AUTO_LOOKAHEAD_MS * 0.001f;
}

inline float priority(const Target *target, uint8_t mode) {
    if (!positionValid(target)) return 0.0f;
    // 수동 모드: 위험 시간 조건과 무관하게 가까운 유효 표적을 선택한다.
    if (mode == MODE_MANUAL) return 1.0f / (1.0f + (float)target->r_mm);
    if (mode != MODE_AUTO) return 0.0f;
    // 속도 미제공 또는 정지 표적은 도달 시간 대신 현재 거리로 선택한다.
    if (!target->velocity_valid ||
        (target->vr_mm_s == 0 && target->vtheta_mm_s == 0))
        return 1.0f / (1.0f + (float)target->r_mm);
    float seconds;
    if (!entryTimeSeconds(target, &seconds) ||
        seconds > (float)AUTO_LOOKAHEAD_MS * 0.001f) return 0.0f;
    // 큰 점수 우선이라는 기존 인터페이스에 맞춘 단조 변환. 시간별 반복 계산 없음.
    return 1.0f / (1.0f + seconds);
}
}
#endif
