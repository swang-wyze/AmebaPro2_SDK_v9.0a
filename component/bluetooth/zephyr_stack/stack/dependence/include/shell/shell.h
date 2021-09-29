/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_H__
#define SHELL_H__

#include <zephyr.h>
#include <logging/log.h>
#include <sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

enum shell_vt100_color {
	SHELL_VT100_COLOR_DEFAULT,
	SHELL_VT100_COLOR_BLACK,
	SHELL_VT100_COLOR_RED,
	SHELL_VT100_COLOR_GREEN,
	SHELL_VT100_COLOR_YELLOW,
	SHELL_VT100_COLOR_BLUE,
	SHELL_VT100_COLOR_MAGENTA,
	SHELL_VT100_COLOR_CYAN,
	SHELL_VT100_COLOR_WHITE,

	VT100_COLOR_END
};

#define SHELL_NORMAL	SHELL_VT100_COLOR_DEFAULT
#define SHELL_INFO	    SHELL_VT100_COLOR_GREEN
#define SHELL_OPTION	SHELL_VT100_COLOR_CYAN
#define SHELL_WARNING	SHELL_VT100_COLOR_YELLOW
#define SHELL_ERROR	    SHELL_VT100_COLOR_RED

#define SHELL_HEXDUMP_BYTES_IN_LINE	16

#define SHELL_CMD_ARG_REGISTER(syntax, subcmd, help, handler, mandatory, optional)

#define SHELL_CMD_REGISTER(syntax, subcmd, help, handler)

struct shell_static_entry {
	const char *syntax;			/*!< Command syntax strings. */
	const char *help;			/*!< Command help string. */
};

struct shell_ctx {
	const char *prompt; /*!< shell current prompt. */
	struct shell_static_entry active_cmd; /*!< Currently executed command.*/
};

struct shell {
	const void *log_backend;
    struct shell_ctx *ctx; /*!< Internal context.*/
};

typedef int (*_cmd_func)(const struct shell *shell, size_t argc, char *argv[]);
typedef struct _cmd_entry {
	const char *name;
	const char *help;
	const _cmd_func handle;
} cmd_entry;

/**
 * @brief Define ending subcommands set.
 *
 */
#define SHELL_SUBCMD_SET_END {NULL}

/**
 * @brief Macro for creating a subcommand set. It must be used outside of any
 * function body.
 *
 * Example usage:
 * SHELL_STATIC_SUBCMD_SET_CREATE(
 *	foo,
 *	SHELL_CMD(abc, ...),
 *	SHELL_CMD(def, ...),
 *	SHELL_SUBCMD_SET_END
 * )
 *
 * @param[in] name	Name of the subcommand set.
 * @param[in] ...	List of commands created with @ref SHELL_CMD_ARG or
 *			or @ref SHELL_CMD
 */
#define SHELL_STATIC_SUBCMD_SET_CREATE(name, ...)			\
        const cmd_entry name##_table[] = {    \
            __VA_ARGS__                     \
        }

/**
 * @brief Initializes a shell command with arguments.
 *
 * @note If a command will be called with wrong number of arguments shell will
 * print an error message and command handler will not be called.
 *
 * @param[in] syntax	 Command syntax (for example: history).
 * @param[in] subcmd	 Pointer to a subcommands array.
 * @param[in] help	 Pointer to a command help string.
 * @param[in] handler	 Pointer to a function handler.
 * @param[in] mand	 Number of mandatory arguments includig command name.
 * @param[in] opt	 Number of optional arguments.
 */
#define SHELL_CMD_ARG(syntax, subcmd, help, handler, mand, opt) \
	{STRINGIFY(syntax), help, handler}

/**
 * @brief Initializes a shell command.
 *
 * @param[in] _syntax	Command syntax (for example: history).
 * @param[in] _subcmd	Pointer to a subcommands array.
 * @param[in] _help	Pointer to a command help string.
 * @param[in] _handler	Pointer to a function handler.
 */
#define SHELL_CMD(_syntax, _subcmd, _help, _handler) \
	SHELL_CMD_ARG(_syntax, _subcmd, _help, _handler, 0, 0)

/**
 * @brief printf-like function which sends formatted data stream to the shell.
 *
 * This function can be used from the command handler or from threads, but not
 * from an interrupt context.
 *
 * @param[in] shell	Pointer to the shell instance.
 * @param[in] color	Printed text color.
 * @param[in] fmt	Format string.
 * @param[in] ...	List of parameters to print.
 */
//void shell_fprintf(const struct shell *shell, enum shell_vt100_color color, const char *fmt, ...);
#define shell_fprintf(shell, color, fmt, ...) printk(fmt, ##__VA_ARGS__)

/**
 * @brief Print a line of data in hexadecimal format.
 *
 * Each line shows the offset, bytes and then ASCII representation.
 *
 * For example:
 *
 * 00008010: 20 25 00 20 2f 48 00 08  80 05 00 20 af 46 00
 *	| %. /H.. ... .F. |
 *
 * @param[in] shell	Pointer to the shell instance.
 * @param[in] offset	Offset to show for this line.
 * @param[in] data	Pointer to data.
 * @param[in] len	Length of data.
 */
void shell_hexdump_line(const struct shell *shell, unsigned int offset,
			const uint8_t *data, size_t len);

/**
 * @brief Print data in hexadecimal format.
 *
 * @param[in] shell	Pointer to the shell instance.
 * @param[in] data	Pointer to data.
 * @param[in] len	Length of data.
 */
void shell_hexdump(const struct shell *shell, const uint8_t *data, size_t len);

/**
 * @brief Print info message to the shell.
 *
 * See @ref shell_fprintf.
 *
 * @param[in] _sh Pointer to the shell instance.
 * @param[in] _ft Format string.
 * @param[in] ... List of parameters to print.
 */
#define shell_info(_sh, _ft, ...) \
	shell_fprintf(_sh, SHELL_INFO, _ft "\n", ##__VA_ARGS__)

/**
 * @brief Print normal message to the shell.
 *
 * See @ref shell_fprintf.
 *
 * @param[in] _sh Pointer to the shell instance.
 * @param[in] _ft Format string.
 * @param[in] ... List of parameters to print.
 */
#define shell_print(_sh, _ft, ...) \
	shell_fprintf(_sh, SHELL_NORMAL, _ft "\n", ##__VA_ARGS__)

/**
 * @brief Print warning message to the shell.
 *
 * See @ref shell_fprintf.
 *
 * @param[in] _sh Pointer to the shell instance.
 * @param[in] _ft Format string.
 * @param[in] ... List of parameters to print.
 */
#define shell_warn(_sh, _ft, ...) \
	shell_fprintf(_sh, SHELL_WARNING, _ft "\n", ##__VA_ARGS__)

/**
 * @brief Print error message to the shell.
 *
 * See @ref shell_fprintf.
 *
 * @param[in] _sh Pointer to the shell instance.
 * @param[in] _ft Format string.
 * @param[in] ... List of parameters to print.
 */
#define shell_error(_sh, _ft, ...) \
	shell_fprintf(_sh, SHELL_ERROR, _ft "\n", ##__VA_ARGS__)

/**
 * @brief Prints the current command help.
 *
 * Function will print a help string with: the currently entered command
 * and subcommands (if they exist).
 *
 * @param[in] shell      Pointer to the shell instance.
 */
void shell_help(const struct shell *shell);

/* @brief Command's help has been printed */
#define SHELL_CMD_HELP_PRINTED	(1)

/**
 * @}
 */

/**
 * @brief Execute shell cmds.
 *
 * Function will execute all shell cmds.
 *
 * @param[in] argc      The number of arg.
 * @param[in] argv      Pointer to the arg list.
 */
int execute_shell_cmds(int argc, char* argv[]);

#ifdef __cplusplus
}
#endif

#endif /* SHELL_H__ */
