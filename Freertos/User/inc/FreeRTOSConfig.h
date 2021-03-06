/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/* Ensure stdint is only used by the compiler, and not the assembler. */
#ifdef __GNUC__
	#include <stdint.h>
	extern uint32_t SystemCoreClock;
#endif

	/*
	 * 置 0： RTOS 使用协作式调度器（时间片）
	 * 置 1： RTOS 使用抢占式调度器
	 * 注：在多任务管理机制上，操作系统可以分为抢占式和协作式两种。
	 * 协作式操作系统是任务主动释放 CPU 后，切换到下一个任务。
	 * 任务切换的时机完全取决于正在运行的任务。
	 * */
#define configUSE_PREEMPTION			1

/*
 * 置 0：忽略空闲钩子
 * 置 1：使用空闲钩子（Idle Hook 类似于回调函数）
 *
  *  空闲任务钩子是一个函数，这个函数由用户来实现，
 * FreeRTOS 规定了函数的名字和参数： void vApplicationIdleHook(void )，
  * 这个函数在每个空闲任务周期都会被调用
  * 对于已经删除的 RTOS 任务，空闲任务可以释放分配给它们的堆栈内存。
  * 因此必须保证空闲任务可以被 CPU 执行
  * 使用空闲钩子函数设置 CPU 进入省电模式是很常见的
  * 不可以调用会引起空闲任务阻塞的 API 函数
 * */
#define configUSE_IDLE_HOOK				0

/*
 * 置 1：使能低功耗 tickless 模式；
 * 置 0：保持系统节拍（tick）中断一直运行
 */
#define configUSE_TICK_HOOK				0

#define configCPU_CLOCK_HZ				( SystemCoreClock )
#define configTICK_RATE_HZ				( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES			( 32 )
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 128 )
#define configTOTAL_HEAP_SIZE			( ( size_t ) ( 36 * 1024 ) )
#define configMAX_TASK_NAME_LEN			( 16 )

/*
 * 可视化跟踪调试
 */
#define configUSE_TRACE_FACILITY		0

/*
 * 系统节拍计数器变量数据类型，
 * 1 表示为 16 位无符号整形
 * 0 表示为 32 位无符号整形
 * */
#define configUSE_16_BIT_TICKS			0

/*
 * 空闲任务放弃 CPU 使用权给其他同优先级的用户任务
 */
#define configIDLE_SHOULD_YIELD			1

/*
 * 使用互斥信号量
 */
#define configUSE_MUTEXES				1

/*
 * 可以注册的信号量和消息队列个数
 */
#define configQUEUE_REGISTRY_SIZE		10

/*
 * 大于0时启用堆栈溢出检测功能
 * 如果使用此功能，用户必须提供一个栈溢出钩子函数
 * 如果使用的话，此值可以为 1 或者 2，因为有两种栈溢出检测方法
 */
#define configCHECK_FOR_STACK_OVERFLOW	0
/*
 * 递归互斥信号量
 */
#define configUSE_RECURSIVE_MUTEXES		1

/*内存申请失败钩子函数
 *
 */
#define configUSE_MALLOC_FAILED_HOOK	0

#define configUSE_APPLICATION_TASK_TAG	0

/*
 * 为1时使用计数信号量
 */
#define configUSE_COUNTING_SEMAPHORES	1
/*
 *启用运行时间统计功能
 */
#define configGENERATE_RUN_TIME_STATS	0

/*****************与协程有关的配置选项****************/
/* Co-routine definitions. */
//启用协程，启用后必须添加文件 croutine.c
#define configUSE_CO_ROUTINES 		0
//协程的有效优先数目
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Software timer definitions. */
//启用软件定时器
#define configUSE_TIMERS				1
//软件定时器优先级
#define configTIMER_TASK_PRIORITY		( configMAX_PRIORITIES - 1 )
//软件定时器队列长度
#define configTIMER_QUEUE_LENGTH		10
//软件定时器任务堆栈大小
#define configTIMER_TASK_STACK_DEPTH	( configMINIMAL_STACK_SIZE * 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	1
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
	/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
	#define configPRIO_BITS       		__NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       		4        /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			0xf

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
	
/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }	
	
/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */

