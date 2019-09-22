#include "IMU.h"
#include "delay.h"
#include "algorithm"
IMU_Accel accel, rvector;
float rangle;
IMU_Gyro  gyro, gyro_offset = { 0 };
IMU_Mag   mag, mag_asa = { 0 }, mag_offset = { 0 }, mag_sca = { 1,1,1 };
IMU_Angle angle;

void Delayms(uint32_t m);

void Delayms(uint32_t m)
{
	//uint32_t i;
	delay_us(1000 * m);
	//for (; m != 0; m--)
		//for (i = 0; i < 50000; i++);
}

void I2C_GPIO_Config(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_6;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_7;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void I2C_delay(void)
{
	//uint8_t i = 5; //ÕâÀï¿ÉÒÔÓÅ»¯ËÙ¶È	£¬¾­²âÊÔ×îµÍµ½5»¹ÄÜÐ´Èë
	//while (i)
	//{
	//	i--;
	//}
	delay_us(1);
}
bool I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if (!SDA_read)return false;	//SDAÏßÎªµÍµçÆ½Ôò×ÜÏßÃ¦,ÍË³ö
	SDA_L;
	I2C_delay();
	if (SDA_read) return false;	//SDAÏßÎª¸ßµçÆ½Ôò×ÜÏß³ö´í,ÍË³ö
	SDA_L;
	I2C_delay();
	return true;
}
void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}
void I2C_Ack(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}
void I2C_NoAck(void)
{
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}
bool I2C_WaitAck(void) 	 //·µ»ØÎª:=1ÓÐACK,=0ÎÞACK
{
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	if (SDA_read)
	{
		SCL_L;
		I2C_delay();
		return false;
	}
	SCL_L;
	I2C_delay();
	return true;
}
void I2C_SendByte(uint8_t SendByte)
{
	uint8_t i = 8;
	while (i--)
	{
		SCL_L;
		I2C_delay();
		if (SendByte & 0x80)
			SDA_H;
		else
			SDA_L;
		SendByte <<= 1;
		I2C_delay();
		SCL_H;
		I2C_delay();
	}
	SCL_L;
}
uint8_t I2C_ReadByte(void)
{
	uint8_t ReceiveByte = 0;
	SDA_H;
	for (uint8_t i = 0; i != 8; ++i)
	{
		ReceiveByte <<= 1;
		SCL_L;
		I2C_delay();
		SCL_H;
		I2C_delay();
		if (SDA_read)ReceiveByte |= 0x01;
	}
	SCL_L;
	return ReceiveByte;
}
bool Single_Write(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)
{
	if (!I2C_Start())return false;
	I2C_SendByte(SlaveAddress);
	if (!I2C_WaitAck()) { I2C_Stop(); return false; }
	I2C_SendByte(REG_Address);
	I2C_WaitAck();
	I2C_SendByte(REG_data);
	I2C_WaitAck();
	I2C_Stop();
	delay_us(1);
	return true;
}
uint8_t Single_Read(uint8_t SlaveAddress, uint8_t REG_Address)
{
	if (!I2C_Start())return false;
	I2C_SendByte(SlaveAddress);
	if (!I2C_WaitAck()) { I2C_Stop(); return false; }
	I2C_SendByte(REG_Address);
	I2C_WaitAck();
	I2C_Start();
	I2C_SendByte(SlaveAddress + 1);
	I2C_WaitAck();

	uint8_t REG_data = I2C_ReadByte();
	I2C_NoAck();
	I2C_Stop();
	return REG_data;
}

inline int16_t getshort(const uint8_t& l, const uint8_t& h)
{
	return (h << 8) | l;
}

void IMU_Init(void)
{
	/*
	Single_Write(GYRO_ADDRESS,PWR_M, 0x80);   //
	Single_Write(GYRO_ADDRESS,SMPL, 0x07);    //
	Single_Write(GYRO_ADDRESS,DLPF, 0x1E);    //¡À2000¡ã
	Single_Write(GYRO_ADDRESS,INT_C, 0x00 );  //
	Single_Write(GYRO_ADDRESS,PWR_M, 0x00);   //
	*/
	I2C_GPIO_Config();
	delay_ms(1);

	Single_Write(GYRO_ADDRESS, PWR_MGMT_1, 0x01);//set the clock reference to X axis gyroscope to get a better accuracy
	Single_Write(GYRO_ADDRESS, SMPLRT_DIV, 0x07);
	Single_Write(GYRO_ADDRESS, CONFIG, 0x05);
	Single_Write(GYRO_ADDRESS, GYRO_CONFIG, 0x08);
	Single_Write(GYRO_ADDRESS, ACCEL_CONFIG_1, 0x08);
	Single_Write(GYRO_ADDRESS, ACCEL_CONFIG_2, 0x05);

	Single_Write(GYRO_ADDRESS, 0x37, 0x02);//turn on Bypass Mode
	Single_Write(MAG_ADDRESS, 0x0A, 0x1F);
	mag_asa.x = ((uint16_t)Single_Read(MAG_ADDRESS, ASAX) - 128)*0.5f / 128 + 1;
	mag_asa.y = ((uint16_t)Single_Read(MAG_ADDRESS, ASAY) - 128)*0.5f / 128 + 1;
	mag_asa.z = ((uint16_t)Single_Read(MAG_ADDRESS, ASAZ) - 128)*0.5f / 128 + 1;

	//CNTL1_AD 0x0A
	Single_Write(MAG_ADDRESS, 0x0A, 0);
	Single_Write(MAG_ADDRESS, 0x0A, 0x16);
	//----------------
	//	Single_Write(GYRO_ADDRESS,0x6A,0x00);//close Master Mode	
}
bool IMU_Measure(void)
{
	const float gravity = 1.f;// 9.7887f;
	const float accel_scale = 4 * gravity / 32767.f;
	const float gyro_scale = 500.f *(PI / 180.f) / 32767.f;
	const float mag_scale = 0.15f;

	accel.x = getshort(Single_Read(ACCEL_ADDRESS, ACCEL_XOUT_L), Single_Read(ACCEL_ADDRESS, ACCEL_XOUT_H))*accel_scale;
	accel.y = getshort(Single_Read(ACCEL_ADDRESS, ACCEL_YOUT_L), Single_Read(ACCEL_ADDRESS, ACCEL_YOUT_H))*accel_scale;
	accel.z = getshort(Single_Read(ACCEL_ADDRESS, ACCEL_ZOUT_L), Single_Read(ACCEL_ADDRESS, ACCEL_ZOUT_H))*accel_scale;

	gyro.x = getshort(Single_Read(GYRO_ADDRESS, GYRO_XOUT_L), Single_Read(GYRO_ADDRESS, GYRO_XOUT_H))*gyro_scale - gyro_offset.x;
	gyro.y = getshort(Single_Read(GYRO_ADDRESS, GYRO_YOUT_L), Single_Read(GYRO_ADDRESS, GYRO_YOUT_H))*gyro_scale - gyro_offset.y;
	gyro.z = getshort(Single_Read(GYRO_ADDRESS, GYRO_ZOUT_L), Single_Read(GYRO_ADDRESS, GYRO_ZOUT_H))*gyro_scale - gyro_offset.z;

	if (Single_Read(MAG_ADDRESS, 0x02) & 1 == 1)
	{
		mag.x = (getshort(Single_Read(MAG_ADDRESS, MAG_XOUT_L), Single_Read(MAG_ADDRESS, MAG_XOUT_H)) *mag_scale - mag_offset.x)*mag_sca.x;
		mag.y = (getshort(Single_Read(MAG_ADDRESS, MAG_YOUT_L), Single_Read(MAG_ADDRESS, MAG_YOUT_H)) *mag_scale - mag_offset.y)*mag_sca.y;
		mag.z = (getshort(Single_Read(MAG_ADDRESS, MAG_ZOUT_L), Single_Read(MAG_ADDRESS, MAG_ZOUT_H)) *mag_scale - mag_offset.z)*mag_sca.z;
		Single_Read(MAG_ADDRESS, 0x09);
		return true;
	}
	return false;
}

extern float32_t MagConst[3];

void IMU_Calibrate(void)
{
	const float gyro_scale = 500.f *(PI / 180.f) / 32767.f;
	uint32_t samples = 200;
	const float mag_scale = 0.15f;

	for (uint32_t i = 0; i != samples; ++i)
	{
		gyro_offset.x += getshort(Single_Read(GYRO_ADDRESS, GYRO_XOUT_L), Single_Read(GYRO_ADDRESS, GYRO_XOUT_H))*gyro_scale;
		gyro_offset.y += getshort(Single_Read(GYRO_ADDRESS, GYRO_YOUT_L), Single_Read(GYRO_ADDRESS, GYRO_YOUT_H))*gyro_scale;
		gyro_offset.z += getshort(Single_Read(GYRO_ADDRESS, GYRO_ZOUT_L), Single_Read(GYRO_ADDRESS, GYRO_ZOUT_H))*gyro_scale;
	}
	gyro_offset.x /= samples;
	gyro_offset.y /= samples;
	gyro_offset.z /= samples;

/*
	offset_x = (max(x) + min(x)) / 2;
	offset_y = (max(y) + min(y)) / 2;
	offset_z = (max(z) + min(z)) / 2;

	avg_delta_x = (max(x) - min(x)) / 2;
	avg_delta_y = (max(y) - min(y)) / 2;
	avg_delta_z = (max(z) - min(z)) / 2;

	avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

	scale_x = avg_delta / avg_delta_x;
	scale_y = avg_delta / avg_delta_y;
	scale_z = avg_delta / avg_delta_z;

	x = (x - offset_x).*scale_x;
	y = (y - offset_y).*scale_y;
	z = (z - offset_z).*scale_z;
	*/
	IMU_Mag mag_max = { -10000,-10000,-10000 };
	IMU_Mag mag_min = { 10000,10000,10000 };

	HAL_Delay(2000);
	samples = 500;

	//for (ii = 0; ii < sample_count; ii++) {
	//	getRawCompassData();  // Read the mag data   
	//	for (int jj = 0; jj < 3; jj++) {
	//		if (Mxyz[jj] > mag_max[jj]) mag_max[jj] = Mxyz[jj];
	//		if (Mxyz[jj] < mag_min[jj]) mag_min[jj] = Mxyz[jj];
	//	}
	//	HAL_Delay(200);
	//}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	for (uint32_t i = 0; i != samples; ++i)
	{
		if (Single_Read(MAG_ADDRESS, 0x02) & 1 == 1)
		{
			mag.x = getshort(Single_Read(MAG_ADDRESS, MAG_XOUT_L), Single_Read(MAG_ADDRESS, MAG_XOUT_H)) *mag_scale;
			mag.y = getshort(Single_Read(MAG_ADDRESS, MAG_YOUT_L), Single_Read(MAG_ADDRESS, MAG_YOUT_H)) *mag_scale;
			mag.z = getshort(Single_Read(MAG_ADDRESS, MAG_ZOUT_L), Single_Read(MAG_ADDRESS, MAG_ZOUT_H)) *mag_scale;
			Single_Read(MAG_ADDRESS, 0x09);

			mag_max.x = std::max(mag.x, mag_max.x);
			mag_max.y = std::max(mag.y, mag_max.y);
			mag_max.z = std::max(mag.z, mag_max.z);

			mag_min.x = std::min(mag.x, mag_min.x);
			mag_min.y = std::min(mag.y, mag_min.y);
			mag_min.z = std::min(mag.z, mag_min.z);
		}
		HAL_Delay(10);
	}
	mag_offset.x = (mag_max.x + mag_min.x) / 2;
	mag_offset.y = (mag_max.y + mag_min.y) / 2;
	mag_offset.z = (mag_max.z + mag_min.z) / 2;

	float delta_x = (mag_max.x - mag_min.x) / 2;
	float delta_y = (mag_max.y - mag_min.y) / 2;
	float delta_z = (mag_max.z - mag_min.z) / 2;

	float ave_delta = (delta_x + delta_y + delta_z) / 3;

	mag_sca.x = ave_delta / delta_x;
	mag_sca.y = ave_delta / delta_y;
	mag_sca.z = ave_delta / delta_z;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}


