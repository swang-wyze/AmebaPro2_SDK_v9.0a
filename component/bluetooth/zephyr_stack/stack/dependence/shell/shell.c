/*
 * Copyright (c) 2021, Realsil Semiconductor Corporation. All rights reserved.
 */

#include <shell/shell.h>
#include <ctype.h>

void shell_hexdump_line(const struct shell *shell, unsigned int offset,
		const uint8_t *data, size_t len)
{
	int i;

	shell_fprintf(shell, SHELL_NORMAL, "%08X: ", offset);

	for (i = 0; i < SHELL_HEXDUMP_BYTES_IN_LINE; i++) {
		if (i > 0 && !(i % 8)) {
			shell_fprintf(shell, SHELL_NORMAL, " ");
		}

		if (i < len) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x ",
					data[i] & 0xFF);
		} else {
			shell_fprintf(shell, SHELL_NORMAL, "   ");
		}
	}

	shell_fprintf(shell, SHELL_NORMAL, "|");

	for (i = 0; i < SHELL_HEXDUMP_BYTES_IN_LINE; i++) {
		if (i > 0 && !(i % 8)) {
			shell_fprintf(shell, SHELL_NORMAL, " ");
		}

		if (i < len) {
			char c = data[i];

			shell_fprintf(shell, SHELL_NORMAL, "%c",
					isprint((int)c) ? c : '.');
		} else {
			shell_fprintf(shell, SHELL_NORMAL, " ");
		}
	}

	shell_print(shell, "|");
}

void shell_hexdump(const struct shell *shell, const uint8_t *data, size_t len)
{
	const uint8_t *p = data;
	size_t line_len;

	while (len) {
		line_len = MIN(len, SHELL_HEXDUMP_BYTES_IN_LINE);

		shell_hexdump_line(shell, p - data, p, line_len);

		len -= line_len;
		p += line_len;
	}
}

void shell_help(const struct shell *shell)
{
    shell_fprintf(shell, SHELL_NORMAL, "[shell help] %s: %s\n\r", 
        shell->ctx->active_cmd.syntax, shell->ctx->active_cmd.help);
}

