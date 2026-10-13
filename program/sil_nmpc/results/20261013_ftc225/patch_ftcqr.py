import sys
d = sys.argv[1]
p = d + '/nmpc.cpp'; s = open(p, 'rb').read().decode()
a = 'bool nmpc_cost_update_scheduled(float u_hover, NmpcContext &ctx)'
assert s.count(a) == 1
s = s.replace(a, '''// ---- SIL 比較用（ファーム複製のみ）: FTC 中だけヨー率の重み Q_r を差し替える ----
// nmpc_cost_set_pending() は R_u を 100 に丸める（clamp_cost_params）ので使わず、Q_r だけを直接書き換える。
// Q は H に入るのでフル再構築を要求する（1 回だけ 26-30 ms 止まる）。
void sil_ftc_set_qr(NmpcContext &ctx, float q_r)
{
    (void)cost_diag_ref();
    if (g_cost_active.Q_r == q_r) return;   // 起動時（g_motor_eta_dirty=1）に値を変えずに再構築を足さない＝健全飛行はビット単位で現行と同じ
    g_cost_active.Q_r = q_r;
    build_cost_diag(g_cost_diag, g_cost_active);
    nmpc_mark_external_rebuild(ctx);
}

''' + a, 1)
open(p, 'wb').write(s.encode())
p = d + '/nmpc.hpp'; s = open(p, 'rb').read().decode()
a = 'NmpcCostParams nmpc_cost_defaults();'
s = s.replace(a, a + '\nvoid sil_ftc_set_qr(NmpcContext &ctx, float q_r);   // SIL 比較用（複製のみ）', 1)
open(p, 'wb').write(s.encode())
p = d + '/control.cpp'; s = open(p, 'rb').read().decode()
a = '        nmpc::nmpc_set_motor_eta(g_nmpc_ctx, Motor_eta);'
assert s.count(a) == 1
nl = '\r\n' if '\r\n' in s else '\n'
s = s.replace(a, a + nl + '#ifdef SIL_FTC_QR' + nl + '        nmpc::sil_ftc_set_qr(g_nmpc_ctx, Ftc_mode ? (float)(SIL_FTC_QR) : nmpc::cost::Q_r);' + nl + '#endif', 1)
open(p, 'wb').write(s.encode())
print('patched')
