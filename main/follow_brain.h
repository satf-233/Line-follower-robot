//include本文件可以调用循迹函数follow，执行红外巡线功能

#ifndef FOLLOW_BRAIN_H
#define FOLLOW_BRAIN_H

// 初始化四路红外传感器引脚为输入（启动时调用一次，之后才能用 follow 读取电平）
void ir_init(void);

//循迹函数，自动调取红外传感器四路数据，判断路况，并发出转向指令
void follow();

#endif // FOLLOW_BRAIN_H
