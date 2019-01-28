

/**
 * main.c
 */

#include "F28x_Project.h"
#include "Syncopation_SCI.h"
#include "Syncopation_Data.h"

#define DSP_LED_1_OFF       GpioDataRegs.GPDSET.bit.GPIO109 = 1
#define DSP_LED_1_ON        GpioDataRegs.GPDCLEAR.bit.GPIO109 = 1
#define DSP_LED_1_TGL       GpioDataRegs.GPDTOGGLE.bit.GPIO109 = 1

#define DSP_LED_2_OFF       GpioDataRegs.GPDSET.bit.GPIO110 = 1
#define DSP_LED_2_ON        GpioDataRegs.GPDCLEAR.bit.GPIO110 = 1
#define DSP_LED_2_TGL       GpioDataRegs.GPDTOGGLE.bit.GPIO110 = 1

#define FPGA_RESET          GpioDataRegs.GPASET.bit.GPIO24 = 1
#define FPGA_RELEASE        GpioDataRegs.GPACLEAR.bit.GPIO24 = 1

extern void Syncopation_Init(void);

// EMIF1_CS2n - Program + Data  ||  2M × 16  ||  0x0010 0000  ||  0x002F FFFF  ||
Uint16 *fpga_led    =   (Uint16 *)0x00100010;
Uint16 *fpga_relay  =   (Uint16 *)0x00100040;

extern Uint16 vac_reading;
extern Uint16 vdc_reading;
extern Uint16 iac_reading;

void main(void) {
    InitSysCtrl();
    Syncopation_Init();
    SCI_Config();

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  // Enable the PIE block
    IER = M_INT1 | M_INT3 | M_INT9;
//    IER |= M_INT13;
    FPGA_RESET;

    FPGA_RELEASE;
    DSP_LED_1_OFF;
    DSP_LED_2_OFF;

    EINT;

    CpuTimer1Regs.TCR.bit.TIF = 1;
    CpuTimer1Regs.TCR.all = 0x4000;

    DSP_LED_1_OFF;
    DSP_LED_2_OFF;

    while(1)
    {
        DELAY_US(2000);

        SCI_UpdatePacketFloat(0, (float)vac_reading);
        SCI_UpdatePacketFloat(1, (float)vdc_reading);
        SCI_UpdatePacketFloat(2, (float)iac_reading);

        SCI_SendPacket();
    }
}




//#define TEST_SIZE 5000000
//Uint16 *sd_ram_start = (Uint16 *)0x80000000;
//Uint32 wr_index = 0;
//Uint32 rd_index = 0;
//Uint32 error_count = 0;
//
//void sdram_test()
//{
//    Uint16 data_wr = 0;
//    for(wr_index=0;wr_index<TEST_SIZE;wr_index++)
//    {
//        *(sd_ram_start + wr_index) = data_wr;
//        data_wr += 3;
//    }
//
//    data_wr = 0;
//
//    Uint16 data_rd = 0;
//
//    for(rd_index=0;rd_index<TEST_SIZE;rd_index++)
//    {
//        data_rd = *(sd_ram_start + rd_index);
//        if(data_rd != data_wr)
//            error_count++;
//        data_wr += 3;
//    }
//
//}


#pragma CODE_SECTION(CpuTimerIsr, ".TI.ramfunc");
interrupt void CpuTimerIsr()
{
    CpuTimer1Regs.TCR.bit.TIF = 1;
}

void Relay_mainClose()
{
    *fpga_relay |= 1;
}

void Relay_mainOpen()
{
    *fpga_relay &= (~1);
}

void Relay_SsrClose()
{
    *fpga_relay |= 2;
}

void Relay_SsrOpen()
{
    *fpga_relay &= (~2);
}

