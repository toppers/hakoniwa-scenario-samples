#ifndef _MROS_OS_CONFIG_H_
#define _MROS_OS_CONFIG_H_

#include "kernel.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void sub_task(void);
extern void pub_task(void);
extern void xml_slv_task(void);
extern void xml_mas_task(void);
extern void cyclic_handler(intptr_t exinf);

/****************************************
 * kernel cfg
 ****************************************/
#ifndef MROS_USR_TASK_PRI
/*
 * main task priority
 *
 * do not change this parameter
 */
#define MAIN_TASK_PRI 			7
/*
 * mROS task priority
 *
 * do not change this parameter
 */
#define MROS_TASK_PRI  			6

/*
 * user task priority
 *
 * following config parameter is an example.
 */
#define MROS_USR_TASK1_PRI  	9
#define MROS_USR_TASK2_PRI  	9
/*8
 * user task max priority8
 *8
 * please set max priority of user tasks.
 */
#define MROS_USR_TASK_PRI		8
#endif /* ROS_USR_TASK_PRI */


#ifndef MROS_TASK_STACK_SIZE
/*
 * user task stack size
 */
#define MROS_USR9_STACK_SIZE 1024 * 1   //for startup task
#define MROS_USR8_STACK_SIZE 1024 * 1   //for user task8
#define MROS_USR7_STACK_SIZE 1024 * 1   //for user task7
#define MROS_USR6_STACK_SIZE 1024 * 1   //for user task6
#define MROS_USR5_STACK_SIZE 1024 * 1   //for user task5
#define MROS_USR4_STACK_SIZE 1024 * 1   //for user task4
#define MROS_USR3_STACK_SIZE 1024 * 1   //for user task3
#define MROS_USR2_STACK_SIZE 1024 * 1	//for user task2
#define MROS_USR1_STACK_SIZE 1024 * 1 	//for user task1
/*
 * mROS task stack size
 *
 * do not change this parameter
 */
#define MROS_TASK_STACK_SIZE 1024 * 2	//for mros task
#endif	/*MROS_TASK_STACK_SIZE*/

#ifndef KMM_SIZE
#define MBED_TASK_STACK_SIZE 1024 * 8	//for mbed task
#define	KMM_SIZE	(MBED_TASK_STACK_SIZE * 16)	/* kernel assign */
#endif /* KMM_SIZE */						/* size of memory */

/*
 * do not change this parameter
 */
#ifndef CYC
#define MROS_LOOP_RATE 100
#define CYC
#endif	/*CYC*/

/*
 * do not change this parameter
 */
#ifndef LOOP_REF
#define LOOP_REF		ULONG_C(1000000)	/* number of loops to evaluate speed */
#endif /* LOOP_REF */

#ifdef __cplusplus
}
#endif


#endif /* _MROS_OS_CONFIG_H_ */
