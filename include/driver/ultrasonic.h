#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// 超声波驱动接口函数声明
void UltrasonicDriver_Init(void);
float UltrasonicDriver_ReadFront(void);
float UltrasonicDriver_ReadLeft(void);
float UltrasonicDriver_ReadRight(void);

#ifdef __cplusplus
}
#endif

#endif // ULTRASONIC_H
