程序整体思路
使用变量标定工作模式，共分为两种
1正常工作模式
2复位工作模式
3标定模式
4不工作
实现尽量在主函数中实现

需要存储的数据

4报警



2月20日
记录完成
完成位置标记


需要完成内容


6显示













//#ifdef TIM4_CH3
// 		if(TIM4CH3_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
//		{
//			temp1_1=1000000/(((TIM4CH3_CAPTURE_STA&0X3F)*65536)+TIM4CH3_CAPTURE_VAL);
//			TIM4CH3_CAPTURE_STA=0;//开启下一次捕获
//				temp1_1 = speed - temp1_1;
//				pwm_wide_oc1 = pwm_wide_oc1 + temp1_1*p;	
//				if(pwm_wide_oc1 > 4000)
//				{
//				pwm_wide_oc1 = 4000;
//				} 				
//				TIM3->CCR1 = pwm_wide_oc1;			
//				printf("1.%d,%d\n",pwm_wide_oc1,temp1_1);//打印总的高点平时间			
//			
//		}
//#endif

//		if(one_second_flag==1)
//		{
//			before_one_sec_vol1=motor_adc_value1;//temp_value3????0??,?????
//			before_one_sec_vol2=motor_adc_value2;//temp_value3????0??,?????

//			printf("2\r\n");
//		}
//		else if(one_second_flag==16)
//		{
////			if(((before_one_sec_vol1-dn_cur)>motor_value1||motor_value1>(before_one_sec_vol1+up_cur))||
////			((before_one_sec_vol2-dn_cur)>motor_value2||motor_value2>(before_one_sec_vol2+up_cur)))
//			if((motor_adc_value1>(before_one_sec_vol1+up_cur))||(motor_adc_value2>(before_one_sec_vol2+up_cur)))
//			{				
//				dn_zero_count++;
//				printf("3\r\n");
//			}
//		}
//		else if(one_second_flag==32)
//		{
//			printf("4\r\n");
//			if(dn_zero_count>0)
//			{
////				if(((before_one_sec_vol1-dn_cur)>motor_value1||motor_value1>(before_one_sec_vol1+up_cur))||
////				((before_one_sec_vol2-dn_cur)>motor_value2||motor_value2>(before_one_sec_vol2+up_cur)))
//				if((motor_adc_value1>(before_one_sec_vol1+up_cur))||(motor_adc_value2>(before_one_sec_vol2+up_cur)))
//				{
//				anytime_stop=1;//??????
//				sure_block_flag=1;//??????
//				}
//			}
//			else
//				dn_zero_count = 0;
////				printf("motor_adc_value1 = %d\n",a);
////				printf("motor_adc_value2 = %d\n",b);			
//			one_second_flag=0;		
//		}



中断   
1定时器3
2定时器4
3串口dma中断



8没法试
9没有断电处理
1 2  4  5  6  7  10

