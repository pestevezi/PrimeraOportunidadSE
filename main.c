/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
int state = 0;
bool greeno, redo;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void led_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOD->PSOR = (1 << 5);

  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR = (1 << 29);

}

void button_init(){


  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

  PORTC->PCR[3] |= PORT_PCR_MUX(1);
  PORTC->PCR[3] |= PORT_PCR_PE(1);
  PORTC->PCR[3] |= PORT_PCR_PS(1);
  PORTC->PCR[3] |= PORT_PCR_IRQC(10);
  GPIOC->PDDR &= ~(1 << 3);

  PORTC->PCR[12] |= PORT_PCR_MUX(1);
  PORTC->PCR[12] |= PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_PS(1);
  PORTC->PCR[12] |= PORT_PCR_ISF(1);
  PORTC->PCR[12] |= PORT_PCR_IRQC(0b1010);
  GPIOC->PDDR &= ~(1 << 12);

  NVIC_EnableIRQ(PORTC_PORTD_IRQn);

}


void red_toggle(void) {
  GPIOE->PTOR = (1 << 29);
  redo = !redo;
}


void green_toggle() {
  GPIOD->PTOR = (1 << 5);
  greeno = !greeno;
}

void red_on(bool on) {
  if(on){
//    GPIOE->PDOR = 1;
    if(!redo)
      red_toggle();
   }
  else{
    if(redo)
      red_toggle();
  }
}


void green_on(bool on) {
  if(on){
    if(!greeno)
      green_toggle();
  }
  else{
    if(greeno)
      green_toggle();
  }
}

void blink(){
  green_toggle();
  red_toggle();
}

void unblink(){

}

void pit_init(uint8_t mult){
  SIM->SCGC6 |= SIM_SCGC6_PIT(1);
  PIT->MCR &= PIT_MCR_MDIS(0);

  PIT->CHANNEL[0].LDVAL = 20900000 * mult;
  PIT->CHANNEL[0].TCTRL &= PIT_TCTRL_CHN(0);
  PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN(1);
  PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE(1);
  NVIC_SetPriority(PIT_IRQn, 3); 
  NVIC_ClearPendingIRQ(PIT_IRQn);
  NVIC_EnableIRQ(PIT_IRQn);
}


void rb_state_machine(){
  switch(state){
    case 0:           // green, RED
      green_on(0);
      red_on(1);
      break;          
    case 1:           // GREEN, red
      green_on(1);
      red_on(0);
      break;
    case 2:           // GREEN, RED
      green_on(1);
      red_on(1);
      break;
    case 3:           // green, red
      green_on(0);
      red_on(0);
      break;
  }
  if(++state >= 4)
    state = 0;
}

void PORTD_Int_Handler(void){  
   bool left = PORTC->PCR[12]>>24, right = PORTC->PCR[3]>>24;

   if (left){
    rb_state_machine();
   }
   if(right){
    red_toggle();
    green_toggle();
   }

  PORTC->PCR[12] |= PORT_PCR_ISF(1);
  PORTC->PCR[3] |= PORT_PCR_ISF(1);

}

void PIT_Int_Handler(void) {
  blink();

  PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF(1); //clear the flag
}


int main(void)
{
  char ch, command[200];
  int i = 0;//, e;

  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  led_init();
  button_init();

  PRINTF("\r\nReinicio!\r\n");

  while (1) {
      i = 0;
      bool valid = false, blinko = false;

      PRINTF("$");
      do
        {
            ch = GETCHAR();
            PUTCHAR(ch);

/*            if(ch != '\0'){
              command[i] = ch;

              if(ch = '')
                ;
              
              if(ch == '\b')
                i--;
              else
                i++;
            }*/

            switch(ch){
              case '\0':
                break;
              case '\b':
                i--;
                break;
              case ' ':
                command[i] = '\0';
                if (!strcmp(command, "blink")){
                  blinko = true;
                  i = 0;
                }
                break;
               default:
                command[i] = ch;
                i++;
                break;
            }

        } while (ch != '\r');
        command[i-1] = '\0';
        PRINTF("\r\n");
        if (!strcmp(command, "led1")){
          red_toggle();
          PRINTF("Led rojo : %d\r\n", redo);
          valid = true;
        }
        if (!strcmp(command, "led2")){
          green_toggle();
          PRINTF("Led verde : %d\r\n", greeno);
          valid = true;
        }
        if (!strcmp(command, "off")){
          green_on(0);
          red_on(0);
          PRINTF("Led rojo : %d\r\n", redo);
          PRINTF("Led verde : %d\r\n", greeno);
          valid = true;
        }
        if (!strcmp(command, "toggle")){
          blink();
          PRINTF("Led rojo : %d\r\n", redo);
          valid = true;
        }
        if(blinko){
          uint8_t value = command[0] - 0x30U;
          if(value != 0){
            pit_init(value);
            PRINTF("Ciclo de %d segundos", value);
          }
          else
            unblink();

          valid = true;
        }
        if (!strcmp(command, "unblink")){
          blink();
          PRINTF("Led rojo : %d\r\n", redo);
          valid = true;
        }
        if(!valid)
          PRINTF("Comando inválido\r\n");
    }
}
