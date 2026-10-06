/* button.c - 按键驱动：消抖、单击/连击、长按识别，全部时间单位为 ms */

#include "button.h"
#include "bsp_delay.h"

#define BUTTON_SETTLING_TIME             10   // 消抖时间 ms
#define BUTTON_CLICK_INTERVAL            200  // 连击判定间隔 ms，两次点击超此时长即各自计数
#define BUTTON_LONG_PRESS_THRESHOLD      1000 // 长按阈值 ms
#define BUTTON_LONG_PRESS_TICK_INTERNVAL 100  // 长按后回调的重复触发间隔 ms

static void OnButtonPressed(Button_TypeDef *Button);
static void OnButtonReleased(Button_TypeDef *Button);
static void OnButtonEveryPolled(Button_TypeDef *Button, uint8_t State, uint32_t currentTime);
static void GPIOClockCmd(GPIO_TypeDef *GPIOx, uint8_t Enable);

// 初始化按钮：配置引脚为上拉输入，清空状态并注册回调
void My_Button_Init(Button_TypeDef *Button, Button_InitTypeDef *Button_InistStruct)
{
	Button->GPIOx = Button_InistStruct->GPIOx;
	Button->GPIO_Pin = Button_InistStruct->GPIO_Pin;
	Button->button_pressed_cb = 0;
	Button->button_released_cb = 0;
	Button->button_clicked_cb = 0;
	Button->button_long_pressed_cb = 0;
	Button->LongPressThreshold = BUTTON_LONG_PRESS_THRESHOLD;
	Button->ClickInterval = BUTTON_CLICK_INTERVAL;
	Button->LongPressTickInterval = BUTTON_LONG_PRESS_TICK_INTERNVAL;
	
	GPIOClockCmd(Button->GPIOx, 1);

	GPIO_InitTypeDef gpio_init_struct;
	
	gpio_init_struct.GPIO_Pin = Button->GPIO_Pin;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(Button->GPIOx, &gpio_init_struct);
	
	if(Button->LongPressThreshold == 0)
	{
		Button->LongPressThreshold = BUTTON_LONG_PRESS_THRESHOLD;
	}
	
	if(Button->LongPressTickInterval == 0)
	{
		Button->LongPressTickInterval = BUTTON_LONG_PRESS_TICK_INTERNVAL;
	}
	
	if(Button->ClickInterval == 0)
	{
		Button->ClickInterval = BUTTON_CLICK_INTERVAL;
	}
	
	Button->LastState = 0;
	Button->ChangePending = 0; 
	Button->PendingTime = 0;
	Button->LastPressedTime = 0;
	Button->LastReleasedTime = 0;
	Button->LongPressTicks = 0;
	Button->ClickCnt = 0;
}

// 按键轮询处理，需在 main 的 while 循环中反复调用
void My_Button_Proc(Button_TypeDef *Button)
{
	uint8_t currentState;

	uint32_t currentTime = GetTick();

	// 消抖：状态变化后等 BUTTON_SETTLING_TIME 再确认
	if(Button->ChangePending)
	{
		if (currentTime >= Button->PendingTime + BUTTON_SETTLING_TIME)
		{
			currentState = GPIO_ReadInputDataBit(Button->GPIOx, Button->GPIO_Pin) == Bit_RESET ? 1 : 0;
			
			if(currentState != Button->LastState)
			{
				if(currentState == 1)
					OnButtonPressed(Button);
				else
					OnButtonReleased(Button);
			}
			Button->LastState = currentState;
			Button->ChangePending = 0;
		}
	}
	else
	{
		currentState = GPIO_ReadInputDataBit(Button->GPIOx, Button->GPIO_Pin) == Bit_RESET ? 1 : 0;
		
		if(currentState != Button->LastState)
		{
			Button->PendingTime = currentTime;
			Button->ChangePending = 1;
		}
	}
	
	OnButtonEveryPolled(Button, Button->LastState, currentTime);
}

// 返回按钮当前状态：0 松开，1 按下
uint8_t MyButton_GetState(Button_TypeDef *Button)
{
	return Button->LastState;
}

// 设置单击间隔，单位 ms，小于此间隔的连续点击算作连击
void My_Button_ClickIntervalConfig(Button_TypeDef *Button, uint32_t Interval)
{
	Button->ClickInterval = Interval;
}

void My_Button_LongPressConfig(Button_TypeDef *Button, uint32_t Throshold, uint32_t TickInterval)
{
	Button->LongPressThreshold = Throshold;
	Button->LongPressTickInterval = TickInterval;
}


void My_Button_SetLongPressCb(Button_TypeDef *Button, void (*LongPressCb)(uint8_t ticks))
{
	Button->button_long_pressed_cb = LongPressCb;
}

void My_Button_SetPressCb(Button_TypeDef *Button, void (*PressCb)(void))
{
	Button->button_pressed_cb = PressCb;
}

void My_Button_SetReleaseCb(Button_TypeDef *Button, void (*ReleaseCb)(void))
{
	Button->button_released_cb = ReleaseCb;
}

void My_Button_SetClickCb(Button_TypeDef *Button, void (*ClickCb)(uint8_t clicks))
{
	Button->button_clicked_cb = ClickCb;
}

// 按下：记录按下时刻并调用按下回调
static void OnButtonPressed(Button_TypeDef *Button)
{
	Button->LastPressedTime = GetTick();

	if(Button->button_pressed_cb != 0)
	{
		Button->button_pressed_cb();
	}
}

// 松开：记录松开时刻、调用松开回调，并按按下时长更新连击计数
static void OnButtonReleased(Button_TypeDef *Button)
{
	Button->LastReleasedTime = GetTick();

	if(Button->button_released_cb != 0)
	{
		Button->button_released_cb();
	}

	Button->LongPressTicks = 0;

	if(Button->LastReleasedTime - Button->LastPressedTime < Button->LongPressThreshold)
	{
		Button->ClickCnt++;
	}
	else
	{
		Button->ClickCnt = 0;
	}
}

// 每次轮询都会调用，负责长按与连击的判定
static void OnButtonEveryPolled(Button_TypeDef *Button, uint8_t State, uint32_t CurrentTime)
{
	/* 长按处理：未触发过则等阈值，触发过则按 Tick 间隔重复回调 */

	if(Button->LastState == 1)
	{
		if(Button->LongPressTicks == 0)
		{
			if(Button->LastPressedTime!= 0
				&& CurrentTime - Button->LastPressedTime > Button->LongPressThreshold)
			{
				Button->LongPressTicks = 1;
			
				if(Button->button_long_pressed_cb)
				{
					Button->button_long_pressed_cb(Button->LongPressTicks);
				}

				Button->LastLongPressTickTime = GetTick();
			}
		}
		else
		{
			if(CurrentTime - Button->LastLongPressTickTime > Button->LongPressTickInterval)
			{
				Button->LastLongPressTickTime = GetTick();
				
				Button->LongPressTicks++;
				
				if(Button->button_long_pressed_cb)
				{
					Button->button_long_pressed_cb(Button->LongPressTicks);
				}
			}
		}
	}

	/* 连击处理：松开后静置超过间隔即上报点击次数 */
	
	if(Button->ClickCnt > 0 && Button->LastState == 0 && (GetTick() - Button->LastReleasedTime) > Button->ClickInterval)
	{
		if(Button->button_clicked_cb)
		{
			Button->button_clicked_cb(Button->ClickCnt);
		}
		
		Button->ClickCnt = 0;
	}
}

static void GPIOClockCmd(GPIO_TypeDef *GPIOx, uint8_t Enable)
{
	FunctionalState newState = Enable ? ENABLE : DISABLE;
	
	if(GPIOx == GPIOA)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, newState);
	}
	else if(GPIOx == GPIOB)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, newState);
	}
	else if(GPIOx == GPIOC)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, newState);
	}
	else if(GPIOx == GPIOD)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, newState);
	}
}
