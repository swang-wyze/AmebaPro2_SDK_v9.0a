 .section .rodata
    .global _binary_sensor_gc2053_bin_start
    .align  4
_binary_sensor_gc2053_bin_start:
    .incbin "sensor_gc2053.bin"
_binary_sensor_gc2053_bin_end:
    .global _binary_sensor_gc2053_bin_size
    .align  4
_binary_sensor_gc2053_bin_size:
    .int    _binary_sensor_gc2053_bin_end - _binary_sensor_gc2053_bin_start
	.end