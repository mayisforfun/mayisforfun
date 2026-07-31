#include "uart.h"
#include "ti_msp_dl_config.h"
#include "step_motor.h"

volatile uint8_t rx_data;

//                                帧头(2)            编号         方向        转动的步(2)       校验     帧尾(2)
volatile uint8_t rx_order[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFF, 0xFA};

volatile uint8_t last_data = 0x00;
volatile uint8_t pt = 0x00;

extern struct STEP_MOTOR step_motor_lr;
extern struct STEP_MOTOR step_motor_ud;


void send_char(UART_Regs *uart, uint8_t data)
{
    while (DL_UART_isBusy(uart) == true) {
    
    }
    DL_UART_transmitData(uart, data);
}

void send_str(UART_Regs *uart, uint8_t * data)
{
    while (*data!=0 && data!=0) 
    {
        send_char(uart, *data);
        data ++;
    }
}

void UART0_rx_dataframe()
{
    rx_data = DL_UART_receiveData(UART_0_INST);
    if(last_data == rx_data && rx_data == 0xAA)
    {
        pt = 2;
        rx_order[0] = 0xAA;
        rx_order[1] = 0xAA;
    }
    else if(last_data == rx_data && rx_data == 0xFF)
    {
        if(rx_order[2] + rx_order[3] + rx_order[4] + rx_order[5] == rx_order[6])
        {
            if(rx_order[2] == 0)
            {
                if(rx_order[3] == 0)
                {
                    step_motor_lr.remain_steps = (uint32_t) rx_order[4] * 256 + (uint32_t) rx_order[5];
                }
                if(rx_order[3] == 1)
                {
                    step_motor_lr.remain_steps = -(uint32_t) rx_order[4] * 256 - (uint32_t) rx_order[5];
                }
            }
            else if(rx_order[2] == 1)
            {
                if(rx_order[3] == 0)
                {
                    step_motor_ud.remain_steps = (uint32_t) rx_order[4] * 256 + (uint32_t) rx_order[5];

                }
                if(rx_order[3] == 1)
                {
                    step_motor_ud.remain_steps = -(uint32_t) rx_order[4] * 256 - (uint32_t) rx_order[5];
                }
            }
            
        }
    }
    else 
    {
        rx_order[pt] = rx_data;
        pt ++;
        pt %= 9;
    }
    last_data = rx_data;

}

void UART_0_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_0_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            UART0_rx_dataframe();
            break;

        default://其他的串口中断
            break;
    }
}

