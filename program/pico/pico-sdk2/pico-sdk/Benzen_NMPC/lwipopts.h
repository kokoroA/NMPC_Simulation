#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Minimal lwIP for UDP-only AP telemetry (RAM constrained)
#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    8192
#define PBUF_POOL_SIZE              24
#define MEMP_NUM_UDP_PCB            8
#define MEMP_NUM_TCP_PCB            0
#define MEMP_NUM_TCP_SEG            0
#define MEMP_NUM_ARP_QUEUE          6
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    0
#define LWIP_UDP                    1
#define LWIP_TCP                    0
#define LWIP_DHCP                   0
#define LWIP_DNS                    0
#define LWIP_IPV4                   1
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETCONN                0
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define LWIP_CHKSUM_ALGORITHM       3
#define MEM_STATS                   0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0

#endif
