// wifi_telemetry.cpp — Wi-Fi AP + UDP テレメトリ / NMPC コスト重みコマンド
#include "wifi_telemetry.hpp"

#ifdef NMPC_WIFI_TELEM

#include <string.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "cyw43_config.h"
#include "lwip/def.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "dhcpserver.h"
#include "nmpc.hpp"

static struct udp_pcb *s_cmd_pcb;
static struct udp_pcb *s_tel_pcb;
static dhcp_server_t s_dhcp;
static ip_addr_t s_client_addr;
static bool s_client_known;
static bool s_wifi_ready;

static void register_client(const ip_addr_t *addr)
{
    s_client_addr = *addr;
    s_client_known = true;
}

static void send_register_ack(const ip_addr_t *addr, uint16_t port, uint32_t seq)
{
    NmpcPktHdr ack = {};
    ack.magic = NMPC_UDP_MAGIC;
    ack.type = NMPC_PKT_REGISTER_ACK;
    ack.ver = NMPC_UDP_VER;
    ack.seq = seq;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(ack), PBUF_RAM);
    if (!p)
        return;
    memcpy(p->payload, &ack, sizeof(ack));

    cyw43_arch_lwip_begin();
    udp_sendto(s_cmd_pcb, p, addr, port);
    cyw43_arch_lwip_end();
    pbuf_free(p);

#if defined(ip4_addr1) && defined(ip4_addr2) && defined(ip4_addr3) && defined(ip4_addr4)
    printf("#wifi: client %u.%u.%u.%u registered, tel -> UDP %u\r\n",
           ip4_addr1(ip_2_ip4(addr)), ip4_addr2(ip_2_ip4(addr)),
           ip4_addr3(ip_2_ip4(addr)), ip4_addr4(ip_2_ip4(addr)),
           (unsigned)NMPC_PORT_TEL);
#else
    printf("#wifi: client registered, tel -> UDP %u\r\n", (unsigned)NMPC_PORT_TEL);
#endif
}

static void send_cost_ack(const ip_addr_t *addr, uint16_t port, uint32_t seq, uint8_t applied)
{
    NmpcCostAckPkt ack = {};
    ack.hdr.magic = NMPC_UDP_MAGIC;
    ack.hdr.type = NMPC_PKT_COST_ACK;
    ack.hdr.ver = NMPC_UDP_VER;
    ack.hdr.seq = seq;
    ack.applied = applied;

    nmpc::NmpcCostParams cp = nmpc::nmpc_cost_get_active();
    ack.Q_phi = cp.Q_phi;
    ack.Q_theta = cp.Q_theta;
    ack.Q_p = cp.Q_p;
    ack.Q_q = cp.Q_q;
    ack.Q_r = cp.Q_r;
    ack.R_u = cp.R_u;
    ack.terminal_scale = cp.terminal_scale;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(ack), PBUF_RAM);
    if (!p)
        return;
    memcpy(p->payload, &ack, sizeof(ack));

    cyw43_arch_lwip_begin();
    udp_sendto(s_cmd_pcb, p, addr, port);
    cyw43_arch_lwip_end();
    pbuf_free(p);
}

static void handle_set_cost(const NmpcSetCostPkt *pkt, const ip_addr_t *from, uint16_t port)
{
    nmpc::NmpcCostParams p = {};
    p.Q_phi = pkt->Q_phi;
    p.Q_theta = pkt->Q_theta;
    p.Q_p = pkt->Q_p;
    p.Q_q = pkt->Q_q;
    p.Q_r = pkt->Q_r;
    p.R_u = pkt->R_u;
    p.terminal_scale = pkt->terminal_scale;

    nmpc::nmpc_cost_set_pending(p);
    register_client(from);
    send_cost_ack(from, port, pkt->hdr.seq, 1);
}

static void cmd_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                     const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)pcb;
    if (!p)
        return;

    if (p->len < (int)sizeof(NmpcPktHdr)) {
        pbuf_free(p);
        return;
    }

    const uint8_t *data = (const uint8_t *)p->payload;
    const NmpcPktHdr *hdr = (const NmpcPktHdr *)data;
    if (hdr->magic != NMPC_UDP_MAGIC || hdr->ver != NMPC_UDP_VER) {
        pbuf_free(p);
        return;
    }

    switch (hdr->type) {
    case NMPC_PKT_REGISTER:
        register_client(addr);
        send_register_ack(addr, port, hdr->seq);
        break;
    case NMPC_PKT_SET_COST:
        if (p->len >= (int)sizeof(NmpcSetCostPkt))
            handle_set_cost((const NmpcSetCostPkt *)data, addr, port);
        break;
    default:
        break;
    }

    pbuf_free(p);
}

int wifi_telemetry_init(void)
{
    if (cyw43_arch_init()) {
        printf("#wifi: cyw43_arch_init failed\r\n");
        return -1;
    }

    cyw43_arch_enable_ap_mode(NMPC_WIFI_AP_SSID, NMPC_WIFI_AP_PASS,
                              CYW43_AUTH_WPA2_AES_PSK);
    sleep_ms(500);  // AP netif 安定待ち

#if LWIP_IPV6
#define IP(x) ((x).u_addr.ip4)
#else
#define IP(x) (x)
#endif

    ip_addr_t gw;
    ip_addr_t mask;
    IP(gw).addr = PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS);
    IP(mask).addr = PP_HTONL(CYW43_DEFAULT_IP_MASK);

#undef IP

    dhcp_server_init(&s_dhcp, &gw, &mask);

    cyw43_arch_lwip_begin();
    s_cmd_pcb = udp_new();
    if (!s_cmd_pcb) {
        cyw43_arch_lwip_end();
        printf("#wifi: udp_new cmd failed\r\n");
        return -1;
    }
    udp_bind(s_cmd_pcb, IP_ANY_TYPE, NMPC_PORT_CMD);
    udp_recv(s_cmd_pcb, cmd_recv, nullptr);

    s_tel_pcb = udp_new();
    if (!s_tel_pcb) {
        cyw43_arch_lwip_end();
        printf("#wifi: udp_new tel failed\r\n");
        return -1;
    }
    cyw43_arch_lwip_end();

    s_client_known = false;
    s_wifi_ready = true;
    printf("#wifi: AP '%s' IP 192.168.4.1 CMD:%u TEL:%u\r\n",
           NMPC_WIFI_AP_SSID, (unsigned)NMPC_PORT_CMD, (unsigned)NMPC_PORT_TEL);
    return 0;
}

bool wifi_telemetry_ready(void)
{
    return s_wifi_ready;
}

void wifi_telemetry_send_bench(const NmpcTelPkt &tel)
{
    if (!s_wifi_ready || !s_client_known || !s_tel_pcb)
        return;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(tel), PBUF_RAM);
    if (!p)
        return;
    memcpy(p->payload, &tel, sizeof(tel));

    cyw43_arch_lwip_begin();
    udp_sendto(s_tel_pcb, p, &s_client_addr, NMPC_PORT_TEL);
    cyw43_arch_lwip_end();
    pbuf_free(p);
}

#else  // NMPC_WIFI_TELEM

int wifi_telemetry_init(void) { return 0; }
bool wifi_telemetry_ready(void) { return false; }
void wifi_telemetry_send_bench(const NmpcTelPkt &) {}

#endif
