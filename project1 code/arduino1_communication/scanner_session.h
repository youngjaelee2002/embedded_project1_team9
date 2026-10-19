#ifndef SCANNER_SESSION_H
#define SCANNER_SESSION_H
#include "scanner_protocol.h"
namespace ScannerSession {
// 전체 탐색 유지 정책. UI STOP/표적 선택과 독립. 자동 REHOME/ToF 재초기화 없음.
struct CommandLink {
    ScannerCommand command;
    bool automatic_start;
    CommandLink(){reset();}
    void reset(){command.command=SCAN_STOP;command.target_id=0;command.command_seq=0;automatic_start=true;}
    void request(uint8_t requested_command,uint8_t target_id) {
        command.command=requested_command;
        command.target_id=requested_command==SCAN_TRACK?target_id:0;
        ++command.command_seq; // 같은 모드 재요청도 Arduino 3에서 새 요청으로 처리
        automatic_start=false;
    }
    void observe(const DetectionFrame &f) {
        const uint8_t required=SCANNER_HALL_HOME_VALID|SCANNER_TOF_READY;
        // ACK는 접수 번호일 뿐. 실제 원점 보정/준비 상태도 확인한다.
        bool canStart=f.scanner_state==SCANNER_STATE_IDLE && f.scan_mode==SCANNER_MODE_STOP_IDLE &&
            (f.status_flags&required)==required &&
            !(f.status_flags&(SCANNER_REHOME_PENDING|SCANNER_COMMAND_LINK_TIMEOUT));
        if(automatic_start && canStart &&
           f.accepted_cmd_seq==(uint8_t)command.command_seq) {
            command.command=SCAN_SLOW;command.target_id=0;++command.command_seq;
        }
        // 중복 명령은 3번의 watchdog만 갱신한다. 새 동작이 필요할 때만 seq 증가.
    }
};
// v1의 seq는 전송 번호. 측정 시각이 없으므로 레코드 변화 시각만 갱신 근거로 쓴다.
// 완전히 같은 새 관측과 재전송은 구별 불가: 정지 표적도 만료될 수 있다.
struct SampleClock {
    struct Stamp {uint8_t record[12];uint32_t changed_ms;} stamps[MAX_TARGETS];
    uint8_t coordinate_kind;
    SampleClock(){reset();}
    void reset(){memset(stamps,0,sizeof(stamps));coordinate_kind=0xFF;}
    void apply(DetectionFrame &f,const uint8_t *wire,uint32_t now) {
        uint8_t kind=f.status_flags&SCANNER_CYLINDRICAL_RECORDS;
        bool kindChanged=coordinate_kind!=0xFF && coordinate_kind!=kind;coordinate_kind=kind;
        // 스냅샷에서 빠진 ID만 삭제. 만료 후에도 반복되는 ID 이력은 보존.
        for(uint8_t j=0;j<MAX_TARGETS;++j) {
            bool present=false;
            for(uint8_t i=0;i<f.count;++i)if(f.targets[i].id==stamps[j].record[0])present=true;
            if(!present)memset(&stamps[j],0,sizeof(Stamp));
        }
        f.sample_times_valid=1;f.valid_count=0;
        for(uint8_t i=0;i<f.count;++i) {
            Target &t=f.targets[i];const uint8_t *s=wire+6+12*i;uint8_t j=0;
            while(j<MAX_TARGETS && stamps[j].record[0]!=t.id)++j;
            if(j==MAX_TARGETS){j=0;while(j<MAX_TARGETS && stamps[j].record[0])++j;}
            if(j==MAX_TARGETS){t.valid=0;continue;}
            Stamp &stamp=stamps[j];
            if(!stamp.record[0] || (!kindChanged && memcmp(stamp.record,s,12)))stamp.changed_ms=now;
            memcpy(stamp.record,s,12);t.updated_ms=stamp.changed_ms;
            if((uint32_t)(now-stamp.changed_ms)>=TARGET_TIMEOUT_MS)t.valid=0;
            if(t.valid)++f.valid_count;
        }
    }
};
}
#endif
