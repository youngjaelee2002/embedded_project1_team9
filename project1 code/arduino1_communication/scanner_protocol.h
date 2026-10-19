#ifndef SCANNER_PROTOCOL_H
#define SCANNER_PROTOCOL_H
#include <math.h>
#include <string.h>
#include "controller_types.h"
#include "scanner_config.h"
namespace ScannerProtocol {
inline uint16_t readU16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
inline int16_t readI16(const uint8_t *p) {
    uint16_t v=readU16(p); return v<0x8000U ? (int16_t)v : (int16_t)((int32_t)v-65536L);
}
inline uint8_t crc8(const uint8_t *data,uint8_t length) {
    uint8_t crc=SCANNER_CRC_INITIAL;
    for(uint8_t i=0;i<length;++i) {
        crc^=data[i];
        for(uint8_t bit=0;bit<8;++bit)
            crc=(uint8_t)((crc&0x80) ? (crc<<1)^SCANNER_CRC_POLYNOMIAL : crc<<1);
    }
    return crc^SCANNER_CRC_XOR_OUT;
}
inline bool crcOk(const uint8_t *data,uint8_t length) {
    return data && length==SCANNER_FRAME_BYTES && crc8(data,47)==data[47];
}
inline int16_t roundedCoordinate(float v) {
    if(v>32767.0f)return 32767;
    if(v< -32768.0f)return -32768;
    return (int16_t)(v<0 ? v-0.5f : v+0.5f);
}
inline int16_t roundedVelocity(float v) {
    // 실제 3번의 cylindrical 변환 clampVelocity()와 같은 포화 범위.
    if(v>32000.0f)return 32000;
    if(v< -32000.0f)return -32000;
    return roundedCoordinate(v);
}
inline bool telemetryReady(uint8_t mode,uint8_t status) {
    const uint8_t required=SCANNER_HALL_HOME_VALID|SCANNER_TOF_READY;
    if(mode<1 || mode>3 || (status&required)!=required || (status&SCANNER_REHOME_PENDING))return false;
#if SCANNER_COMMAND_LINK_REQUIRED
    if(status&SCANNER_COMMAND_LINK_TIMEOUT)return false;
#endif
    return true;
}
// 내부 SCAN_*와 wire mode는 숫자가 다르므로 명시적으로 변환한다.
inline uint8_t encodeCommand(const ScannerCommand *command,uint8_t *out) {
    if(!command || !out)return 0;
    uint8_t mode;
    switch(command->command) {
    case SCAN_STOP: case SCAN_WAIT: mode=0;break;
    case SCAN_SLOW: mode=1;break;
    case SCAN_FAST: mode=2;break;
    case SCAN_TRACK: mode=3;break;
    case SCAN_HOME: mode=4;break;
    default:return 0;
    }
    memset(out,0,SCANNER_FRAME_BYTES); // flags=0, 예약 byte 6..46=0
    out[0]=SCANNER_SYNC;out[1]=SCANNER_VERSION;out[2]=(uint8_t)command->command_seq;
    out[3]=mode;out[5]=command->target_id;out[47]=crc8(out,47);
    return SCANNER_FRAME_BYTES;
}
// 실패 시 out 보존. 내부는 스캐너 중심 x=동/y=북, 각도는 북0/반시계.
inline bool decode(const uint8_t *data,uint8_t length,DetectionFrame *out) {
    if(!out || !crcOk(data,length) || data[0]!=SCANNER_SYNC || data[1]!=SCANNER_VERSION ||
       data[3]>SCANNER_MODE_TRACK_TARGET || data[42]>SCANNER_STATE_FAULT || data[5]>MAX_TARGETS)return false;
    DetectionFrame f;memset(&f,0,sizeof(f));
    f.sequence_valid=1;f.sequence=data[2];f.scan_mode=data[3];f.status_flags=data[4];f.count=data[5];
    f.scanner_state=data[42];f.hall_strikes=data[43];f.selected_target_id=data[44];
    f.accepted_cmd_seq=data[45];f.last_error=data[46];
    f.scanner_ready=telemetryReady(f.scan_mode,f.status_flags) && f.scanner_state==SCANNER_STATE_MOVING;
    for(uint8_t i=0;i<f.count;++i) {
        const uint8_t *s=data+6+i*SCANNER_SLOT_BYTES;Target &t=f.targets[i];
        if(!s[0] || (s[1]&0xF8))return false;
        for(uint8_t j=0;j<i;++j)if(f.targets[j].id==s[0])return false;
        t.id=s[0];t.velocity_valid=(s[1]&SCANNER_VELOCITY_VALID)!=0;
        t.quality=s[10];t.age_scans=s[11];t.fresh=!t.age_scans && !(s[1]&SCANNER_OBJECT_STALE);
        t.polar_valid=1;
        if(f.status_flags&SCANNER_CYLINDRICAL_RECORDS) {
            t.r_mm=readU16(s+2);uint16_t east=readU16(s+4);
            if(east>=36000U)return false;
            t.theta_cdeg=(uint16_t)(((uint32_t)east+27000UL)%36000UL);
            t.vr_mm_s=readI16(s+6);t.vtheta_mm_s=readI16(s+8);
            float rad=(float)east*0.000174532925199433f;
            t.x=roundedCoordinate(t.r_mm*cosf(rad));t.y=roundedCoordinate(t.r_mm*sinf(rad));
        } else {
            int32_t dx=(int32_t)readI16(s+2)-SCANNER_ORIGIN_X_MM;
            int32_t dy=(int32_t)readI16(s+4)-SCANNER_ORIGIN_Y_MM;
            float r=sqrtf((float)dx*dx+(float)dy*dy);
            t.r_mm=(uint16_t)(r+0.5f);t.x=roundedCoordinate((float)dx);t.y=roundedCoordinate((float)dy);
            int32_t east=(int32_t)roundedCoordinate(atan2f((float)dy,(float)dx)*5729.5779513f);
            if(east<0)east+=36000L;
            t.theta_cdeg=(uint16_t)((east+27000L)%36000L);
            t.vx=readI16(s+6);t.vy=readI16(s+8);
            if(t.velocity_valid && r>0.0f) {
                t.vr_mm_s=roundedVelocity(((float)dx*t.vx+(float)dy*t.vy)/r);
                t.vtheta_mm_s=roundedVelocity((-(float)dy*t.vx+(float)dx*t.vy)/r);
            }
        }
        // OUT_OF_BOUNDS/STALE는 프레임 오류가 아니라 해당 표적만 사용 불가.
        t.valid=(s[1]&SCANNER_IN_BOUNDS) && t.fresh && t.r_mm<=SCANNER_MAX_RADIUS_MM;
        if(t.valid)++f.valid_count;
    }
    *out=f;return true;
}
}
#endif
