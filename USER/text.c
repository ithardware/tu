					key = 0;
					TIM_Cmd(TIM2, DISABLE);  //使能TIMx	
					dn_zero_count = 0;
					dn_zero_count1 = 0;
					sure_block_flag = 0;						
					anytime_stop=1;
					un_alarm_flag=1;  
					delay_ms(1);
					USART_str_send("erro:",1,facility_save);	
					error_flog=1;
					motor1_error=1;
#ifdef Close_PWM_Flag
					close_pwm();
#endif
					delay_ms(500);
					printf("500 yanshi wancheng");
					delay_ms(500);
					key = 0;
					USART_str_send("erro:",1,facility_save);	
					anytime_stop=1;
					un_alarm_flag=1; 
					ext_count1 = 0;
					ext_count2 = 0;
#if (leg_count==3) 	
					ext_count3 = 0;
#endif
					dn_zero_count = 0;
					dn_zero_count1 = 0;
					sure_block_flag = 0;
					save_reset();
					TIM_Cmd(TIM2, ENABLE);  //使能TIMx	