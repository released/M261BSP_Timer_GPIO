# M261BSP_Timer_GPIO
 M261BSP_Timer_GPIO

update @ 2022/11/15

1. initial TIMER 3 (4us interrupt) , EINT0 (PB.5) , pulse output (I/O : PB.2)

2. scenario : after EINT0 (PB.5) trigger with rising , TIMER3 start to count and execute pulse output process

	- Trigger by raising edge of the external interrupt  

	- after 13us, timer output goes high  

	- after 250us, timer output goes low 

	- keeping timer output status and waiting for next trigger (external interrupt)

3. use EPWM0 CH2 (PA.3) and connect to EINT0 (PB.5) to simulate period pulse INPUT

4. below is scope capture (CH0 : EINT0 / EPWM0 CH2 , CH1 : output pulse output

![image](https://github.com/released/M261BSP_Timer_GPIO/blob/main/period_00.jpg)	

first period , 

![image](https://github.com/released/M261BSP_Timer_GPIO/blob/main/period_01.jpg)	

second period , 

![image](https://github.com/released/M261BSP_Timer_GPIO/blob/main/period_02.jpg)	

third period , 

![image](https://github.com/released/M261BSP_Timer_GPIO/blob/main/period_03.jpg)	


