#ifndef TURRET4_PROTOCOL_H
#define TURRET4_PROTOCOL_H
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "controller_types.h"

// 새 4번과 센서 회전축을 원점으로 공유한다. 원점 이동은 하지 않는다.
// 물리 설치는 두 보드의 CFG와 일치시켜야 한다. 이 값은 프로토콜 좌표계 확인 표시.
#ifndef TURRET_WORLD_FRAME_CONFIRMED
#define TURRET_WORLD_FRAME_CONFIRMED 1
#endif
namespace Turret4Link {
const uint8_t ADDRESS=0x20, STATUS_BYTES=8;
const uint32_t COMMAND_GUARD_MS=40;
const uint8_t ANGLE_TOLERANCE_DEG=2;
const float MIN_DIST_MM=50.0f;
struct Config { float r, theta, trim; int8_t dir; uint8_t minDeg,maxDeg; };
// 받은 4번 CFG와 동일한 값. 팀원이 CFG를 바꾸면 이 표도 함께 변경한다.
const Config CFG[2]={{424.3f,135.0f,45.0f,1,0,90},{424.3f,225.0f,-45.0f,1,90,180}}; // 600x600, 4번과 동일
inline uint16_t wireTheta(uint16_t cdeg) { return (uint16_t)(((uint32_t)cdeg+5UL)/10UL%3600UL); }
inline void cylToXY(float r,float theta,float &x,float &y) {
    float t=(float)(theta*3.14159265358979323846/180.0);
    x=(float)(-r*sin(t));y=(float)(r*cos(t));
}
inline float bearing(float dx,float dy) {
    float b=(float)(atan2(-dx,dy)*180.0/3.14159265358979323846);
    if(b<0)b+=360.0;
    return b;
}
inline float wrap180(float a) {while(a>180.0)a-=360.0;while(a<=-180.0)a+=360.0;return a;}
inline float mountBearing(uint8_t index) {
    float px,py;cylToXY(CFG[index].r,CFG[index].theta,px,py);
    return bearing(-px,-py);
}
inline uint16_t servoBearing(uint8_t index,int servo) {
    float b=mountBearing(index)+CFG[index].trim+(servo-90)/(float)CFG[index].dir;
    b=(float)fmod(b,360.0);if(b<0)b+=360.0;
    return (uint16_t)lround(b*10)%3600;
}
inline bool goalAngle(uint8_t index,uint16_t r,uint16_t theta_cdeg,uint8_t *goal) {
    if(index>1 || !goal || theta_cdeg>=36000U)return false;
    const Config &c=CFG[index];float px,py,tx,ty;
    cylToXY(c.r,c.theta,px,py);
    // 4번이 실제로 받는 0.1도 각도로 범위/조준을 판단한다.
    cylToXY(r,(float)(wireTheta(theta_cdeg)/10.0),tx,ty);
    float dx=tx-px,dy=ty-py;
    float delta=wrap180(bearing(dx,dy)-bearing(-px,-py)-c.trim);
    int servo=(int)lround(90+c.dir*delta);
    if(sqrt(dx*dx+dy*dy)<MIN_DIST_MM || servo<c.minDeg || servo>c.maxDeg)return false;
    *goal=(uint8_t)servo;return true;
}
inline uint8_t reachableMask(uint16_t r,uint16_t theta_cdeg) {
    uint8_t goal,mask=0;
    if(goalAngle(0,r,theta_cdeg,&goal))mask|=TURRET_A;
    if(goalAngle(1,r,theta_cdeg,&goal))mask|=TURRET_B;
    return mask;
}
inline void write16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
inline uint16_t read16(const uint8_t *p){return (uint16_t)p[0]|((uint16_t)p[1]<<8);}
inline uint8_t encode(const TurretCommand *command,uint8_t *out) {
    if(!command || !out)return 0;
    if(command->command==GUN_SAFE){out[0]=3;return 1;}
    uint8_t mask=command->turret_mask;
    if(!mask || mask>TURRET_BOTH)return 0;
    if(command->command==GUN_ENABLE || command->command==GUN_FIRE){
        out[0]=4;out[1]=mask;out[2]=command->command==GUN_FIRE?3:0;return 3;
    }
    if(command->command!=GUN_AIM || command->theta_cdeg>=36000U ||
       (reachableMask(command->r_mm,command->theta_cdeg)&mask)!=mask)return 0;
    uint8_t start;
    if(mask==TURRET_BOTH){out[0]=1;start=1;}
    else {out[0]=2;out[1]=mask==TURRET_A?0:1;start=2;}
    write16(out+start,command->r_mm);write16(out+start+2,wireTheta(command->theta_cdeg));
    write16(out+start+4,0); // z 예약: 좌우 회전만 사용하므로 0 전송.
    return start+6;
}
inline bool decode(const uint8_t *data,uint8_t length,TurretReport *out,uint32_t now) {
    if(!out || !data || length!=STATUS_BYTES || (data[0]&0x80) || data[5]>180 || data[6]>180)return false;
    uint8_t sum=0;for(uint8_t i=0;i<7;++i)sum^=data[i];
    if(sum!=data[7] || read16(data+1)>=3600 || read16(data+3)>=3600)return false;
    if((data[0]&1 && data[0]&4) || (data[0]&2 && data[0]&8))return false;
    TurretReport r;memset(&r,0,sizeof(r));
    r.feedback_kind=TURRET_FEEDBACK_ANGLES;r.received_ms=now;r.aimed_mask=data[0]&3;
    r.firing_mask=(data[0]>>5)&3; // 순간 출력. 3회 점멸 전체 진행 여부는 전송되지 않음.
    r.bearing_a_ddeg=read16(data+1);r.bearing_b_ddeg=read16(data+3);
    r.angle_a=data[5];r.angle_b=data[6];
    r.error_a=(data[0]&4?1:0)|(data[0]&0x10?2:0);
    r.error_b=(data[0]&8?1:0)|(data[0]&0x10?2:0);
    *out=r;return true; // 응답에 없는 ID/seq/활성 여부는 만들어 넣지 않는다.
}
inline uint16_t bearingDistance(uint16_t a,uint16_t b){
    uint16_t d=a>b?a-b:b-a;return d>1800?3600-d:d;
}
// 명령 실행 ACK나 물리 각도 피드백이 아니다. 소프트웨어 서보 위치와 방위각을 함께 대조.
class AimTracker {
    bool valid_;TurretCommand aim_;uint32_t sent_ms_,last_sample_ms_;uint8_t matching_samples_;
public:
    AimTracker():valid_(false),aim_(),sent_ms_(0),last_sample_ms_(0),matching_samples_(0){}
    void reset(){valid_=false;matching_samples_=0;}
    void sent(const TurretCommand *c,bool ok,uint32_t now){
        if(!ok || !c || c->command==GUN_SAFE){reset();return;}
        if(c->command!=GUN_AIM)return;
        bool same=valid_ && aim_.target_id==c->target_id && aim_.aim_seq==c->aim_seq &&
            aim_.turret_mask==c->turret_mask && aim_.r_mm==c->r_mm && aim_.theta_cdeg==c->theta_cdeg;
        if(!same){aim_=*c;valid_=true;sent_ms_=last_sample_ms_=now;matching_samples_=0;}
    }
    bool poseMatches(const TurretReport *r) const {
        if(!valid_ || !r || r->feedback_kind!=TURRET_FEEDBACK_ANGLES ||
           (r->aimed_mask&aim_.turret_mask)!=aim_.turret_mask)return false;
        for(uint8_t i=0;i<2;++i){
            if(!(aim_.turret_mask&(1<<i)))continue;
            uint8_t goal;int angle=i?r->angle_b:r->angle_a;uint16_t b=i?r->bearing_b_ddeg:r->bearing_a_ddeg;
            if((i?r->error_b:r->error_a) || !goalAngle(i,aim_.r_mm,aim_.theta_cdeg,&goal) ||
                abs(angle-(int)goal)>ANGLE_TOLERANCE_DEG ||
                bearingDistance(b,servoBearing(i,angle))>1)return false;
        }
        return true;
    }
    void observe(const TurretReport *r){
        if(!valid_ || !r){matching_samples_=0;return;}
        uint32_t elapsed=(uint32_t)(r->received_ms-sent_ms_);
        if(elapsed<COMMAND_GUARD_MS || elapsed>=0x80000000UL || !poseMatches(r)){matching_samples_=0;return;}
        if(r->received_ms!=last_sample_ms_){last_sample_ms_=r->received_ms;if(matching_samples_<2)++matching_samples_;}
    }
    bool ready(uint8_t mask,uint8_t id,uint16_t seq,uint16_t r,uint16_t theta,const TurretReport *report) const {
        return matching_samples_>=2 && valid_ && aim_.turret_mask==mask && aim_.target_id==id &&
            aim_.aim_seq==seq && aim_.r_mm==r && aim_.theta_cdeg==theta && poseMatches(report);
    }
};
}
#endif
