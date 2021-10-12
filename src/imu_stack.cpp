#include <stdlib.h>
#include <Arduino.h>
#include <Kaatru_sensor.h>


struct Stack* create_imu_stack(unsigned capacity){
    struct Stack* stack = (struct Stack*)malloc(sizeof(struct Stack));
    stack -> capacity = capacity;
    stack -> array = (struct imu_data*)malloc(stack->capacity * sizeof(struct imu_data));
    return stack;
}

int isFull(struct Stack* stack)
{
    return stack->top == stack->capacity - 1;
}
 
int isEmpty(struct Stack* stack)
{
    return stack->top == -1;
}
 
void push(struct Stack* stack, imu_data item)
{
    if (isFull(stack))
        return;
    stack->array[++stack->top] = item;
}
 
imu_data pop(struct Stack* stack)
{
    if (isEmpty(stack)){
        struct imu_data empty ;
        return empty;
    }
    return stack->array[stack->top--];
}
 
imu_data peek(struct Stack* stack)
{
    if (isEmpty(stack)){
        struct imu_data empty ;
        return empty;
    }
    return stack->array[stack->top];
}

