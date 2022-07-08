#include "app.h"

u8 get_decode_data[]=
{	
	0x23,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00
};


float acc_m;
float theta = 0.0f;
_debug_data Debug_Data;
u8 Oldman_i = 0;

void Oscilloscope(void)
{
	uint8_t i;

//	Oldman_i++;
  
	theta = theta+sensor.gyro.x*ts;
	if(++Oldman_i >= 10)
	{
		Oldman_i = 0;
		

		Debug_Data.data1 = sensor.gyro.x;	//IMU
		Debug_Data.data2 = sensor.gyro.y;		
		Debug_Data.data3 = sensor.gyro.z;			
		Debug_Data.data4 = sensor.acc.x;//att_air_G.x;
		Debug_Data.data5 = sensor.acc.y;//att_air_G.y;
		Debug_Data.data6 = sensor.acc.z;//tao;
	
		Debug_Data.data7 = Angle.x; //вкл╛╫г//Angle.x;
		Debug_Data.data8 = Angle.y;//Angle.y;
		Debug_Data.data9 = Angle.z;//Angle.z; 

		Debug_Data.data10  = att_air.x;//att_air.x;//Encoder.Angle_P; 
		Debug_Data.data11  = att_air.y;//att_air.y;//Encoder.Angle_R; 
		Debug_Data.data12  = att_air.z
		
		;//att_air.z;//Encoder.Angle_Y; 
//Ouler_Point_Air.z = att_air_G.z;		
		Debug_Data.data13 = Ouler_Point.x;//att_air.x;//gyro_air_n.x;//gyro_air.x;/KF_w[1];//Encoder.Angle_P;  gyro_air_n.x
		Debug_Data.data14 = Ouler_Point.y;//att_air.y;//gyro_air_n.y;//gyro_air.y;//KF_w[1];//Fd_InsData.rev;   gyro_air_n.y
		Debug_Data.data15 = Ouler_Point.z;//att_air.z;//gyro_air_n.z;//gyro_air.z;//KF_w[2];//Fd_InsData_State; gyro_air_n.z
		
		Debug_Data.data16  = sensor.temp;//.Source;	 

			
	  get_decode_data[0] = 0x23;
		memcpy(get_decode_data+1,&Debug_Data,sizeof(Debug_Data));
		
		get_decode_data[65]=0;
		for(i=1;i<65;i++)
		{	  				    
			get_decode_data[65] += get_decode_data[i];
		}	
		UART1_SendDataDMA(get_decode_data,66);
	}
}

