extern "C"
{
#include "stdio.h"
#include "string.h"
#include <LQ_UART.h>
#include "math.h"
}
#include "pa_UartManager.h"
#include "pa_globalCpp.h"

void checkUartData()
{
	if (uartHasReceivedData())
	{
		unsigned char recData[30] = {0};
		UART_GetBuff(UART2, recData, UART_GetCount(UART2));

		int index = 2;
		char state1 = 0, state2 = 0;
		char dataValid = 0;
		char xiaoshu = 0;
		float data = 0;
		state1 = recData[0];
		state2 = recData[1];

		if(state1=='o'&& recData[2]=='!'){
			pa_GlobalCpp * globalCpp=(pa_GlobalCpp::getGlobalCpp());
			switch (state2)
			{
			case '1':
				globalCpp->micOutPutMode=OutputMode_Mic;
				UART_PutStr(UART2, "output changed to Mic!\r\n");
				break;
			case '2':
				globalCpp->micOutPutMode=OutputMode_cross;
				UART_PutStr(UART2, "output changed to Cross!\r\n");
				break;
			case '3':
				globalCpp->micOutPutMode=OutputMode_motor;
				UART_PutStr(UART2, "output changed to Motor!\r\n");
				break;
			case '4':
				globalCpp->micOutPutMode=OutputMode_crossDetail;
				UART_PutStr(UART2, "output changed to CrossDetail!\r\n");
				break;
			case '5':
				globalCpp->micOutPutMode=OutputMode_UltrasonicDistance;
				UART_PutStr(UART2, "output changed to UltrasonicDistance\r\n");
				break;
			default:
				break;
			}
			return;
		}

		while (recData[index])
		{
			if ((state1 == 's'||state1 == 'd') && (state2 == 'p' || state2 == 'i' || state2 == 'd' || state2 == 's'||state2 == 'm'))
			{
				if (recData[index] <= '9' && recData[index] >= '0')
				{
					if (!xiaoshu)
					{
						data = recData[index] - 48 + data * 10;
					}
					else
					{
						data = data + pow(0.1, xiaoshu) * (recData[index] - 48);
						xiaoshu++;
					}
				}
				else if (recData[index] == '.')
				{
					xiaoshu = 1;
				}
				else if (recData[index] == '!')
				{
					dataValid = 1;
					break;
				}
			}
			else
			{
				break;
			}
			index++;
		}
		if (dataValid)
		{
			pa_GlobalCpp * globalCpp=(pa_GlobalCpp::getGlobalCpp());
			if (state1 == 's')
			{

				switch (state2)
				{
				case 'p':

				{
					char buffer[30] = "";
					sprintf(buffer, "set p from %f to %f\r\n", globalCpp->pid_Motor1.kp, data);
					UART_PutStr(UART2, buffer);
				}
					globalCpp->pid_Motor1.kp = data;
					globalCpp->pid_Motor2.kp = data;
					globalCpp->pid_Motor3.kp = data;
					globalCpp->pid_Motor4.kp = data;
					break;
				case 'i':
				{
					char buffer[30] = "";
					sprintf(buffer, "set i from %f to %f\r\n", globalCpp->pid_Motor1.ki, data);
					UART_PutStr(UART2, buffer);
				}
					globalCpp->pid_Motor1.iSum = 0;
					globalCpp->pid_Motor2.iSum = 0;
					globalCpp->pid_Motor3.iSum = 0;
					globalCpp->pid_Motor4.iSum = 0;

					globalCpp->pid_Motor1.ki = data;
					globalCpp->pid_Motor2.ki = data;
					globalCpp->pid_Motor3.ki = data;
					globalCpp->pid_Motor4.ki = data;
					break;
				case 'd':
				{
					char buffer[30] = "";
					sprintf(buffer, "set d from %f to %f\r\n", globalCpp->pid_Motor1.kd, data);
					UART_PutStr(UART2, buffer);
				}
					globalCpp->pid_Motor1.kd = data;
					globalCpp->pid_Motor2.kd = data;
					globalCpp->pid_Motor3.kd = data;
					globalCpp->pid_Motor4.kd = data;
					/* code */
					break;
				case 's':
				{
					char buffer[30] = "";
					sprintf(buffer, "set s from %f to %f\r\n", globalCpp->targetSpeed, data);
					UART_PutStr(UART2, buffer);
				}
					globalCpp->targetSpeed = data;
					/* code */
					break;
				default:
					break;
				}
			}else if(state1=='d'){
				switch (state2)
				{
				case 'p':

				{
					char buffer[30] = "";
					sprintf(buffer, "set dir_p from %f to %f\r\n", globalCpp->pid_Direction.kp, data);
					UART_PutStr(UART2, buffer);
				}
					globalCpp->pid_Direction.kp = data;
					break;
				case 'i':
				{
					char buffer[30] = "";
					sprintf(buffer, "set dir_i from %f to %f\r\n", globalCpp->pid_Direction.ki, data);
					UART_PutStr(UART2, buffer);
				}
					globalCpp->pid_Direction.iSum = 0;

					globalCpp->pid_Direction.ki = data;
					break;
				case 'd':
				{
					char buffer[30] = "";
					sprintf(buffer, "set dir_d from %f to %f\r\n", globalCpp->pid_Direction.kd, data);
					UART_PutStr(UART2, buffer);
				}
					globalCpp->pid_Direction.kd = data;
					/* code */
					break;
				case 'm':
					pa_GlobalCpp::getGlobalCpp()->motorDisable=!pa_GlobalCpp::getGlobalCpp()->motorDisable;
					if(pa_GlobalCpp::getGlobalCpp()->motorDisable){
						UART_PutStr(UART2, "motorDisabled!\r\n");
					}else{
						UART_PutStr(UART2, "motorEnabled!\r\n");
					}
					break;
				default:
					break;
				}
			}
		}
		else
		{
			UART_PutStr(UART2, "invalidData:");
			UART_PutStr(UART2, (char *)recData);
			UART_PutStr(UART2, "\r\n");
		}

		changeLED();
	}
}
