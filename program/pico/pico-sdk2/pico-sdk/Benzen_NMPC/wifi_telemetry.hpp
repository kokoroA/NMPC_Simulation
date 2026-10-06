#ifndef WIFI_TELEMETRY_HPP
#define WIFI_TELEMETRY_HPP

#include <stdint.h>
#include <stdbool.h>

#define NMPC_UDP_MAGIC      0xBEEFu
// ver 4: NmpcTelPkt に ftc / failsafe を追加 (旧 _pad + 1 バイト)
// ver 5: 末尾に rx_flags / rx_framelost_run / rx_framelost_cnt を追加 (188 -> 196 バイト)。
//        既存フィールドのオフセットは変えていない。
// ver 6: 最終 PWM duty から伝播した a_hat_p/q/r を追加 (196 -> 208 バイト)。
#define NMPC_UDP_VER        6u
#define NMPC_PORT_CMD       5000u
#define NMPC_PORT_TEL       5001u

#define NMPC_WIFI_AP_SSID   "Benzen_NMPC"
#define NMPC_WIFI_AP_PASS   "benzen2026"

enum NmpcUdpType : uint8_t {
    NMPC_PKT_REGISTER   = 0x01,
    NMPC_PKT_REGISTER_ACK = 0x02,
    NMPC_PKT_SET_COST   = 0x10,
    NMPC_PKT_COST_ACK   = 0x11,
    NMPC_PKT_TEL        = 0x20,
};

#pragma pack(push, 1)
struct NmpcPktHdr {
    uint16_t magic;
    uint8_t  type;
    uint8_t  ver;
    uint32_t seq;
};

struct NmpcSetCostPkt {
    NmpcPktHdr hdr;
    float Q_phi;
    float Q_theta;
    float Q_p;
    float Q_q;
    float Q_r;
    float R_u;
    float terminal_scale;
};

struct NmpcCostAckPkt {
    NmpcPktHdr hdr;
    float Q_phi;
    float Q_theta;
    float Q_p;
    float Q_q;
    float Q_r;
    float R_u;
    float terminal_scale;
    uint8_t applied;  // 1 = accepted (pending on core1)
};

struct NmpcTelPkt {
    NmpcPktHdr hdr;
    uint8_t  arm_flag;
    uint8_t  nmpc_active;
    uint8_t  nmpc_reset;
    uint8_t  reset_cause;
    uint8_t  step_path;
    uint8_t  tr1_applied;
    uint8_t  ftc;       // 耐故障モード 0=健全 / 1=FTC (旧 _pad)
    uint8_t  failsafe;  // FS_* の原因コード (0=正常)
    int32_t  cycles_since_reset;
    uint32_t cnt_tr1_path;
    uint32_t cnt_tr1_applied;
    uint32_t cnt_tr1_skipped;
    uint32_t cnt_reset_period;
    uint32_t cnt_reset_external;
    uint32_t cnt_reset_sr2_bad_c;
    uint32_t cnt_reset_sr2_unhealthy;
    uint32_t cnt_reset_after_sr2;
    float phi;
    float theta;
    float p;
    float q;
    float r;
    float phi_ref;
    float theta_ref;
    float psi_ref;
    float T_ref;
    float u_base;
    float schedule_alpha;
    float iter_us;
    float loop_us;
    float cost;
    float iter_us_max;
    float u_fr;
    float u_fl;
    float u_mr;
    float u_ml;
    float u_rr;
    float u_rl;
    float a_hat_p;
    float a_hat_q;
    float a_hat_r;
    float d_fr;
    float d_fl;
    float d_mr;
    float d_ml;
    float d_rr;
    float d_rl;
    float Q_phi;
    float Q_theta;
    float Q_p;
    float Q_q;
    float Q_r;
    float R_u;
    float terminal_scale;
    // ---- ver 5 追加: 受信品質の計装 ----
    // rx_flags は SBUS フラグバイト (Chdata[16] = sbus_data[23]) の生値。
    // 0x04=FRAME_LOST / 0x08=FAILSAFE。どちらが起きたのかを後から判別するために出す。
    uint8_t  rx_flags;
    uint8_t  rx_framelost_run;   // 0x04 が連続しているフレーム数 (255 で飽和)
    uint16_t _rsv16;             // rx_framelost_cnt を 4 バイト境界に置くための予約
    uint32_t rx_framelost_cnt;   // 0x04 の累積回数
};
#pragma pack(pop)

static_assert(sizeof(NmpcTelPkt) == 208, "NmpcTelPkt は 208 バイト (tools/nmpc_udp_client.py の TEL_FMT と一致させること)");

int wifi_telemetry_init(void);
bool wifi_telemetry_ready(void);
void wifi_telemetry_send_bench(const NmpcTelPkt &tel);

#endif
