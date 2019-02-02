/*
 * MSST_Pwm.c
 *
 *  Created on: Oct 18, 2016
 *      Author: Yang Lei
 */

#include "F28x_Project.h"

#define PWM_PERIOD      4000

void Epwm1Init()
{
    EPwm1Regs.TBPRD = PWM_PERIOD - 1;

    EPwm1Regs.TBCTL.bit.SYNCOSEL = 0;
    EPwm1Regs.TBCTL.bit.PHSEN = 0;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;

    EPwm1Regs.AQCTLA.bit.CAU = 1; // Clear output A at TBCTR = CMPA
    EPwm1Regs.AQCTLA.bit.ZRO = 2; // Set output A at TBCTR = 0
    EPwm1Regs.CMPA.bit.CMPA = 1000;

    EPwm1Regs.TBCTL.bit.CTRMODE = 0; // Count-up mode, start the counter

    EPwm1Regs.ETSEL.bit.SOCASEL = 1; // SOCA at counter equals to 0
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  // Enable SOCA
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;  // Pulse at every event
}

void Epwm2Init()
{
    EPwm2Regs.TBPRD = PWM_PERIOD - 1;

    EPwm2Regs.TBCTL.bit.SYNCOSEL = 0;
    EPwm2Regs.TBCTL.bit.PHSEN = 1;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;

    EPwm2Regs.AQCTLA.bit.CAU = 1; // Clear output A at TBCTR = CMPA
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // Set output A at TBCTR = 0
    EPwm2Regs.CMPA.bit.CMPA = 1000;
    EPwm2Regs.TBPHS.bit.TBPHS = 1000;

    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Count-up mode, start the counter
}

void Epwm3Init()
{
    EPwm3Regs.TBPRD = PWM_PERIOD - 1;

    EPwm3Regs.TBCTL.bit.SYNCOSEL = 0;
    EPwm3Regs.TBCTL.bit.PHSEN = 1;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;

    EPwm3Regs.AQCTLA.bit.CAU = 1; // Clear output A at TBCTR = CMPA
    EPwm3Regs.AQCTLA.bit.ZRO = 2; // Set output A at TBCTR = 0
    EPwm3Regs.CMPA.bit.CMPA = 1000;
    EPwm3Regs.TBPHS.bit.TBPHS = 2000;

    EPwm3Regs.TBCTL.bit.CTRMODE = 0; // Count-up mode, start the counter
}

void Epwm4Init()
{
    EPwm4Regs.TBPRD = PWM_PERIOD - 1;

    EPwm4Regs.TBCTL.bit.SYNCOSEL = 0;
    EPwm4Regs.TBCTL.bit.PHSEN = 1;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;

    EPwm4Regs.AQCTLA.bit.CAU = 1; // Clear output A at TBCTR = CMPA
    EPwm4Regs.AQCTLA.bit.ZRO = 2; // Set output A at TBCTR = 0
    EPwm4Regs.CMPA.bit.CMPA = 1000;
    EPwm4Regs.TBPHS.bit.TBPHS = 3000;

    EPwm4Regs.TBCTL.bit.CTRMODE = 0; // Count-up mode, start the counter
}

void EpwmInit()
{
    Epwm1Init();
    Epwm2Init();
    Epwm3Init();
    Epwm4Init();
}

