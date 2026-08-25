// SIL shim: pico/binary_info.h（実機のメタデータ埋め込みは SIL では不要）
#ifndef _PICO_BINARY_INFO_H
#define _PICO_BINARY_INFO_H
#define bi_decl(x)
#define bi_2pins_with_func(a, b, c) 0
#define bi_4pins_with_func(a, b, c, d, e) 0
#define bi_1pin_with_name(a, b) 0
#define bi_program_description(x) 0
#endif
