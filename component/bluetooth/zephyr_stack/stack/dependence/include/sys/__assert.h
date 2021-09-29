/*
 * Copyright (c) 2011-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SYS___ASSERT_H_
#define ZEPHYR_INCLUDE_SYS___ASSERT_H_

#ifdef CONFIG_ASSERT

#include <stdbool.h>
#include <sys/printk.h>

#ifdef CONFIG_ASSERT
#ifndef __ASSERT_ON
#define __ASSERT_ON 1
#endif
#endif

#define __ASSERT_LOC(test) \
    printk("ASSERTION FAIL [%s] @ %s:%d\n", \
		Z_STRINGIFY(test), __FILE__, __LINE__)

#ifdef CONFIG_ASSERT_NO_MSG_INFO
#define __ASSERT_MSG_INFO(fmt, ...)
#else
#define __ASSERT_MSG_INFO(fmt, ...) printk("\t" fmt "\n", ##__VA_ARGS__)
#endif

extern void k_panic(void);
#define __ASSERT_POST_ACTION(x) k_panic()

#define __ASSERT(test, fmt, ...)                                          \
	do {                                                              \
		if (!(test)) {                                            \
			__ASSERT_MSG_INFO(fmt, ##__VA_ARGS__);            \
			__ASSERT_LOC(test);									\
			__ASSERT_POST_ACTION(test);                           \
		}                                                         \
	} while (false)

#define __ASSERT_NO_MSG(test)                                             \
	do {                                                              \
		if (!(test)) {                                            \
			__ASSERT_LOC(test);									 \
			__ASSERT_POST_ACTION(test);                           \
		}                                                         \
	} while (false)

#define __ASSERT_EVAL(expr1, expr2, test, fmt, ...) expr1
#else
#define __ASSERT(test, fmt, ...) { }
#define __ASSERT_NO_MSG(test) { }
#define __ASSERT_EVAL(expr1, expr2, test, fmt, ...) expr1
#endif

#endif /* ZEPHYR_INCLUDE_SYS___ASSERT_H_ */
