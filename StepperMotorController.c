// StepperMotorController.c starter file EE319K Lab 5
// Runs on TM4C123
// Finite state machine to operate a stepper motor.  
// Jonathan Valvano
// September 2, 2019

// Hardware connections (External: two input buttons and four outputs to stepper motor)
//  PA5 is Wash input  (1 means pressed, 0 means not pressed)
//  PA4 is Wiper input  (1 means pressed, 0 means not pressed)
//  PE5 is Water pump output (toggle means washing)
//  PE4-0 are stepper motor outputs 
//  PF1 PF2 or PF3 control the LED on Launchpad used as a heartbeat
//  PB6 is LED output (1 activates external LED on protoboard)

#include "SysTick.h"
#include "TExaS.h"
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

void EnableInterrupts(void);
// edit the following only if you need to move pins from PA4, PE3-0      
// logic analyzer on the real board
#define PA4       (*((volatile unsigned long *)0x40004040))
#define PE50      (*((volatile unsigned long *)0x400240FC))
void SendDataToLogicAnalyzer(void){
  UART0_DR_R = 0x80|(PA4<<2)|PE50;
}

struct State {
	uint8_t Next[4];; // depends on 2-bit input
	uint8_t LED;
	uint8_t Motor;
	uint32_t Dwell;
};
typedef const struct State STyp;
#define NoButton   0
#define Swipe1 1
#define Swipe2 2
#define Swipe3 3
#define Swipe4 4
#define Swipe5 5
#define Swipe6 6
#define Swipe7 7	
#define Swipe8 8
#define Swipe9 9
#define Swipe10 10
#define Swipe11 11
#define Swipe12 12
#define Swipe13 13
#define Swipe14 14
#define Swipe15 15
#define Swipe16 16
#define Swipe17 17
#define Swipe18 18
#define Swipe19 19
#define Wash1   20
#define Wash2   21
#define Wash3   22
#define Wash4   23
#define Wash5   24
#define Wash6   25
#define Wash7   26
#define Wash8   27
#define Wash9   28
#define Wash10   29
#define Wash11   30
#define Wash12   31
#define Wash13   32
#define Wash14   33
#define Wash15   34
#define Wash16   35
#define Wash17   36
#define Wash18   37
#define Wash19   38

STyp FSM[39]={
{/*NoButton*/ {NoButton,Swipe1,Wash1,Wash1},0,1,4000},
{/*Swipe1*/ {NoButton,Swipe2,Wash2,Wash2},0,1,4000},
{/*Swipe2*/ {Swipe1,Swipe3,Wash3,Wash3},0,2,4000},
{/*Swipe3*/ {Swipe2,Swipe4,Wash4,Wash4},0,4,4000},
{/*Swipe4*/ {Swipe3,Swipe5,Wash5,Wash5},0,8,4000},
{/*Swipe5*/ {Swipe4,Swipe6,Wash6,Wash6},0,16,4000},
{/*Swipe6*/ {Swipe5,Swipe7,Wash7,Wash7},0,1,4000},
{/*Swipe7*/ {Swipe6,Swipe8,Wash8,Wash8},0,2,4000},
{/*Swipe8*/ {Swipe7,Swipe9,Wash9,Wash9},0,4,4000},
{/*Swipe9*/ {Swipe8,Swipe10,Wash10,Wash10},0,8,4000},
{/*Swipe10*/ {Swipe9,Swipe11,Wash11,Wash11},0,16,4000},
{/*Swipe11*/ {Swipe10,Swipe12,Wash12,Wash12},0,8,4000},
{/*Swipe12*/ {Swipe11,Swipe13,Wash13,Wash13},0,4,4000},
{/*Swipe13*/ {Swipe12,Swipe14,Wash14,Wash14},0,2,4000},
{/*Swipe14*/ {Swipe13,Swipe15,Wash15,Wash15},0,1,4000},
{/*Swipe15*/ {Swipe14,Swipe16,Wash16,Wash16},0,16,4000},
{/*Swipe16*/ {Swipe15,Swipe17,Wash17,Wash17},0,8,4000},
{/*Swipe17*/ {Swipe16,Swipe18,Wash18,Wash18},0,4,4000},
{/*Swipe18*/ {Swipe17,Swipe19,Wash19,Wash19},0,2,4000},
{/*Swipe19*/ {Swipe18,Swipe1,Wash1,Wash1},0,1,4000},
{/*Wash1*/ {NoButton,Swipe2,Wash2,Wash2},1,1,4000},
{/*Wash2*/ {Swipe1,Swipe3,Wash3,Wash3},1,2,4000},
{/*Wash3*/ {Swipe2,Swipe4,Wash4,Wash4},1,4,4000},
{/*Wash4*/ {Swipe3,Swipe5,Wash5,Wash5},1,8,4000},
{/*Wash5*/ {Swipe4,Swipe6,Wash6,Wash6},1,16,4000},
{/*Wash6*/ {Swipe5,Swipe7,Wash7,Wash7},1,1,4000},
{/*Wash7*/ {Swipe6,Swipe8,Wash8,Wash8},1,2,4000},
{/*Wash8*/ {Swipe7,Swipe9,Wash9,Wash9},1,4,4000},
{/*Wash9*/ {Swipe8,Swipe10,Wash10,Wash10},1,8,4000},
{/*Wash10*/ {Swipe9,Swipe11,Wash11,Wash11},1,16,4000},
{/*Wash11*/ {Swipe10,Swipe12,Wash12,Wash12},1,8,4000},
{/*Wash12*/ {Swipe11,Swipe13,Wash13,Wash13},1,4,4000},
{/*Wash13*/ {Swipe12,Swipe14,Wash14,Wash14},1,2,4000},
{/*Wash14*/ {Swipe13,Swipe15,Wash15,Wash15},1,1,4000},
{/*Wash15*/ {Swipe14,Swipe16,Wash16,Wash16},1,16,4000},
{/*Wash16*/ {Swipe15,Swipe17,Wash17,Wash17},1,8,4000},
{/*Wash17*/ {Swipe16,Swipe18,Wash18,Wash18},1,4,4000},
{/*Wash18*/ {Swipe17,Swipe19,Wash19,Wash19},1,2,4000},
{/*Wash19*/ {Swipe18,Swipe1,Wash1,Wash1},1,1,4000},

};
int main(void){ 
  TExaS_Init(&SendDataToLogicAnalyzer);    // activate logic analyzer and set system clock to 80 MHz
  SysTick_Init();   
// you initialize your system here
	uint8_t cs;
	uint8_t input;
	cs=NoButton;
  EnableInterrupts();   
  while(1){
// output
// wait
// input
// next		
  }
}


