 .section .rodata
    .global _binary_iq_ps5258_bin_start
    .align  4
_binary_iq_ps5258_bin_start:
    .incbin "iq_ps5258.bin"
_binary_iq_ps5258_bin_end:
    .global _binary_iq_ps5258_bin_size
    .align  4
_binary_iq_ps5258_bin_size:
    .int    _binary_iq_ps5258_bin_end - _binary_iq_ps5258_bin_start
	.end