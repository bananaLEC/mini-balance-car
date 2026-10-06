/* task.c - 裸机周期任务调度器 */
#include "task.h"
#include "bsp_delay.h"

#define TASK_MAX_NUM  8u

typedef struct
{
	Task_Func_t Func;
	uint32_t    Period;  //周期 ms
	uint32_t    Last;    //上次运行时刻 ms
} Task_t;

static Task_t  tasks[TASK_MAX_NUM];
static uint8_t task_num = 0;

// 注册任务，Func 必须非阻塞，PeriodMs 单位 ms
// 返回 0 失败（函数为空、周期为 0 或任务表已满），1 成功
uint8_t Task_Add(Task_Func_t Func, uint32_t PeriodMs)
{
	if(Func == 0 || PeriodMs == 0)
	{
		return 0;
	}
	if(task_num >= TASK_MAX_NUM)
	{
		return 0;
	}

	tasks[task_num].Func   = Func;
	tasks[task_num].Period = PeriodMs;
	//以当前时刻作为起点，任务等一个周期后才首次执行，避免上电时任务挤在一起
	tasks[task_num].Last   = GetTick();

	task_num++;
	return 1;
}

// 执行所有到时间的任务，在 main 的 while 循环里反复调用
void Task_Run(void)
{
	uint32_t now = GetTick();
	uint8_t i;

	for(i = 0; i < task_num; i++)
	{
		if((uint32_t)(now - tasks[i].Last) >= tasks[i].Period)
		{
			//按周期累加而不是重置为当前时刻，长期运行速率才准确、不漂移
			tasks[i].Last += tasks[i].Period;

			//落后超过一个周期说明任务超时或被暂停过，放弃追赶、对齐到当前时刻，避免连续补跑
			if((uint32_t)(now - tasks[i].Last) >= tasks[i].Period)
			{
				tasks[i].Last = now;
			}

			tasks[i].Func();
		}
	}
}
