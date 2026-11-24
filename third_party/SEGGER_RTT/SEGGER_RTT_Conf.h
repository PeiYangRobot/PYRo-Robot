/* EDT 定制配置：2 上行（EDT_SCOPE 示波帧 / EDT_LOG 日志）+ 1 下行（EDT_CTRL 命令） */
#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#define SEGGER_RTT_MAX_NUM_UP_BUFFERS     (3)
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS   (2)
#define BUFFER_SIZE_UP                    (1024)
#define BUFFER_SIZE_DOWN                  (256)

/* 与 FreeRTOSConfig.h 中 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5 << 4) 对齐 */
#define SEGGER_RTT_MAX_INTERRUPT_PRIORITY (0x50)

#endif
