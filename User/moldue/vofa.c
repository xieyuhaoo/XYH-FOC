#include "common.h"
#include "modlue.h"
#include "usbd_cdc_if.h"

#define MAX_BUFFER_SIZE 256
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

/**
***********************************************************************
* @brief:      vofa_start(void)
* @param:	void
* @retval:     void
* @details:    发送
***********************************************************************
**/
float adc_value[3];
extern uint16_t adc2_buff[4];

/*
 * VOFA+ JustFloat 通道列表:
 *   CH0:  i_alph       α轴电流 (A)
 *   CH1:  i_beta       β轴电流 (A)
 *   CH2:  Ealph        LPF反电势α — 幅值随转速增大
 *   CH3:  Ebeta        LPF反电势β
 *   CH4:  pos_e        PLL估算电角度 (rad) — 开环阶段是否收敛到强拖?
 *   CH5:  drag_pe      IF强拖角度 (rad)   — 对比CH4
 *   CH6:  we_pll       PLL输出电角速度 (rad/s)
 *   CH7:  i_shunt_1    第一次母线采样 (A)
 *   CH8:  i_shunt_2    第二次母线采样 (A)
 *   CH9:  run_stage    运行阶段: 0=对齐 1=IF爬速 2=SMO闭环
 */
void vofa_start(void)
{
	vofa_send_data(0,  pm.foc.i_d);
	vofa_send_data(1,  pm.foc.i_q);
	// vofa_send_data(2,  pm.foc.v_d);
	// vofa_send_data(3,  pm.foc.v_q);
	// vofa_send_data(4,  pm.id_pi.i_term);
	// vofa_send_data(5,  pm.id_pi.out_value);
	vofa_send_data(6,  pm.foc.arr1_up);
	vofa_send_data(7,  pm.foc.arr2_up);
	vofa_send_data(8,  pm.foc.arr3_up);
	// vofa_send_data(9,  pm.foc.theta);
	// vofa_send_data(10, (float)pm.period.start_cnt);
	// vofa_send_data(11, pm.foc.i_shunt_1);
	// vofa_send_data(12, pm.foc.i_shunt_2);
	// vofa_send_data(13, pm.foc.v_alph);
	// vofa_send_data(14, pm.foc.v_beta);
	// vofa_send_data(15, pm.foc.vbus);
	vofa_send_data(16, pm.foc.i_a);
	vofa_send_data(17, pm.foc.i_b);
	vofa_send_data(18, pm.foc.i_c);
	// vofa_send_data(22, (float)pm.foc.shunt_case);//移相采样标志
	// vofa_send_data(23, (float)pm.foc.svm_sector);
	// vofa_send_data(0, pm.foc.i_alph);
	// vofa_send_data(1, pm.foc.i_beta);
	// vofa_send_data(2, pm.esmo.Ealph);
	// vofa_send_data(3, pm.esmo.Ebeta);
	vofa_send_data(4, pm.esmo.pos_e);
	vofa_send_data(5, pm.ctrl.drag_pe);
	// vofa_send_data(6, pm.esmo.we_pll);
	vofa_send_data(9, (float)pm.foc.shunt_case);//移相采样标志
	vofa_send_data(10, (float)pm.foc.svm_sector*10);
	vofa_sendframetail();
}

/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		   void
* @retval:     void
* @details:    �޸�ͨ�Ź��ߣ�USART����USB
***********************************************************************
**/
void vofa_transmit(uint8_t* buf, uint16_t len)
{
	/* 方式1: USB CDC (无需设置波特率, VOFA+选JustFloat协议) */
	CDC_Transmit_FS((uint8_t *)buf, len);

	/* 方式2: UART3 (如果USB CDC不通, 取消下面注释, 注释掉上面CDC行)
	 * VOFA+ 选串口COM端口, 波特率与 MX_USART3_UART_Init 中一致 */
	// HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 0xFFFF);
}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: ���ݱ�� data: ���� 
* @retval:     void
* @details:    ���������ݲ�ֳɵ��ֽ�
***********************************************************************
**/
_RAM_FUNC void vofa_send_data(uint8_t num, float data) 
{

	data_u f;
	f.f_val = data;
	
	send_buf[cnt++] = f.u8_val[0];
	send_buf[cnt++] = f.u8_val[1];
	send_buf[cnt++] = f.u8_val[2];
	send_buf[cnt++] = f.u8_val[3];
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL 
* @retval     void
* @details:   �����ݰ�����֡β
***********************************************************************
**/
void vofa_sendframetail(void) 
{
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x80;
	send_buf[cnt++] = 0x7f;
	
	/* �����ݺ�֡β������� */
	vofa_transmit((uint8_t *)send_buf, cnt);
	cnt = 0;// ÿ�η�����֡β����Ҫ����
}
/**
***********************************************************************
* @brief      vofa_demo(void)
* @param      NULL 
* @retval     void
* @details:   demoʾ��
***********************************************************************
**/
void vofa_demo(void) 
{
	static float scnt = 0.0f;

	scnt += 0.01f;

	if(scnt >= 360.0f)
		scnt = 0.0f;

	float v1 = scnt;
	float v2 = sin((double)scnt / 180 * 3.14159) * 180 + 180;
	float v3 = sin((double)(scnt + 120) / 180 * 3.14159) * 180 + 180;
	float v4 = sin((double)(scnt + 240) / 180 * 3.14159) * 180 + 180;

	// Call the function to store the data in the buffer
	vofa_send_data(0, v1);
	vofa_send_data(1, v2);
	vofa_send_data(2, v3);
	vofa_send_data(3, v4);

	// Call the function to send the frame tail
	vofa_sendframetail();
}













