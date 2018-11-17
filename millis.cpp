/* 
 * @author: Natasha Sarkar, 2018
 */

#include "mbed.h"
#include "millis.h"
 
volatile unsigned long  _millis;
 
void millis_begin(void) {
    SysTick_Config(SystemCoreClock / 1000);
}
 
extern "C" void SysTick_Handler(void) {
    _millis++;
}
 
unsigned long millis(void) {
    return _millis;
}
