/* task.h - 裸机周期任务调度器
   用法：Task_Add(函数, 周期ms) 按顺序注册任务，然后在 main 的 while(1) 里反复调用 Task_Run()
   任务函数绝对不能阻塞：不许 Delay、不许用 while 等标志位，否则后面所有任务都会延误 */

#ifndef __TASK_H__
#define __TASK_H__

#include <stdint.h>

typedef void (*Task_Func_t)(void);

uint8_t Task_Add(Task_Func_t Func, uint32_t PeriodMs);  //返回值：0-失败，1-成功
void    Task_Run(void);

#endif
