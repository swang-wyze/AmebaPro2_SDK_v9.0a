/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_LOGGING_LOG_H_
#define ZEPHYR_INCLUDE_LOGGING_LOG_H_

#define LOG_MODULE_REGISTER(...)
#define LOG_MODULE_DECLARE(...)
#define LOG_STR(x) #x
#define LOG_NAME(x) LOG_STR(x)

#define LOG_LEVEL_NONE 0U /* No message */
#define LOG_LEVEL_ERR  1U /* Error conditions */
#define LOG_LEVEL_WRN  2U /* Warning conditions */
#define LOG_LEVEL_INF  3U /* Informational message */
#define LOG_LEVEL_DBG  4U /* Debug-level message */
#define LOG_LEVEL_ALL  5U /* Show all message */

#undef LOG_DBG
#undef LOG_INF
#undef LOG_WRN
#undef LOG_ERR

#define LOG_FORMAT_TAG    0x08
#define LOG_FORMAT_LEVEL  0x04
#define LOG_FORMAT_MODULE 0x02
#define LOG_FORMAT_FUNC   0x01

extern unsigned char g_log_format;
extern unsigned char g_log_level;

#define LOG_PROTO(level, fmt, ...) \
    printk("%s%s%s%s%s%s\t"fmt, \
        (g_log_format&LOG_FORMAT_TAG)? "[BT]":"", \
        (g_log_format&LOG_FORMAT_LEVEL)? "["LOG_NAME(level)"]":"", \
        (g_log_format&LOG_FORMAT_MODULE)? "["LOG_NAME(LOG_MODULE_NAME)"]":"", \
        (g_log_format&LOG_FORMAT_FUNC)? "[":"", \
        (g_log_format&LOG_FORMAT_FUNC)? __FUNCTION__:"", \
        (g_log_format&LOG_FORMAT_FUNC)? "]":"", \
        ##__VA_ARGS__); printk("\n\r");

#if (CONFIG_BT_LOG_LEVEL < LOG_LEVEL_DBG)
    #define LOG_DBG(fmt, ...)
#else
    #define LOG_DBG(fmt, ...) \
    do { \
        if (g_log_level >= LOG_LEVEL_DBG) { \
            LOG_PROTO(DBG, fmt, ##__VA_ARGS__) \
        } \
    } while(0)
#endif

#if (CONFIG_BT_LOG_LEVEL < LOG_LEVEL_INF)
    #define LOG_INF(fmt, ...)
#else
    #define LOG_INF(fmt, ...) \
    do { \
        if (g_log_level >= LOG_LEVEL_INF) { \
            LOG_PROTO(INF, fmt, ##__VA_ARGS__) \
        } \
    } while(0)
#endif

#if (CONFIG_BT_LOG_LEVEL < LOG_LEVEL_WRN)
    #define LOG_WRN(fmt, ...)
#else
    #define LOG_WRN(fmt, ...) \
    do { \
        if (g_log_level >= LOG_LEVEL_WRN) { \
            LOG_PROTO(WRN, fmt, ##__VA_ARGS__) \
        } \
    } while(0)
#endif

#if (CONFIG_BT_LOG_LEVEL < LOG_LEVEL_ERR)
    #define LOG_ERR(fmt, ...)
#else
    #define LOG_ERR(fmt, ...) \
    do { \
        if (g_log_level >= LOG_LEVEL_ERR) { \
            LOG_PROTO(ERR, fmt, ##__VA_ARGS__) \
        } \
    } while(0)
#endif

#if (CONFIG_BT_LOG_LEVEL < LOG_LEVEL_DBG)
    #define LOG_HEXDUMP_DBG(_data, _length, _str)
#else
    #define LOG_HEXDUMP_DBG(_data, _length, _str) \
    do { \
        if(g_log_level >= LOG_LEVEL_DBG) { \
            LOG_PROTO(DBG, "%s\n\r", "") \
            printk(_str); printk("\n\r"); \
            for(int i = 0; i < _length; i++) { \
                printk("%02x", *(char *)(_data + i)); \
            } \
            printk("\n\r"); \
        } \
    }while(0)
#endif

static inline char *log_strdup(const char *str)
{
	return (char *)str;
}

#endif /* ZEPHYR_INCLUDE_LOGGING_LOG_H_ */
