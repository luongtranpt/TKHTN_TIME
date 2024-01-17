#include "LCD_I2C.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c1;  // handle xu ly i2c
#define addr_pcf8574 0x4E // dia chi cua slave i2c

void LCD_I2C_Write_CMD(uint8_t data)
{
	uint8_t buf[4] = { (data&0xF0) | 0x0C ,(data &0xF0) |0x08 , (data<<4) |0x0C ,(data<<4)|0x08 }; // truyen 4 byte du lieu trong do 
//truyen 4 byte cao truoc , 0x0C và 0x08 là de chon chan RW tu 0->1 truyen cmd
	HAL_I2C_Master_Transmit(&hi2c1, addr_pcf8574,buf,4, 100); // truyen i2c trong STM32
}
void LCD_I2C_WRITE_DATA(uint8_t data)
{
	uint8_t buf[4] = { (data& 0xF0) | 0x0D ,(data & 0xF0) |0x09 , (data << 4) |0x0D ,(data << 4)| 0x09 };// truyen 4 byte du lieu trong do 
//truyen 4 byte cao truoc , 0x0D và 0x09 là de chon chan RS tu 0->1 truyen data
	HAL_I2C_Master_Transmit(&hi2c1, addr_pcf8574, buf,4, 100);// truyen 4 byte du lieu di 
}	
void LCD_I2C_Init()
{ 
	LCD_I2C_Write_CMD(0x33);
	HAL_Delay(50);
	LCD_I2C_Write_CMD(0x32);
	HAL_Delay(50);
  LCD_I2C_Write_CMD(0x28); // Che do 4 bit 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(0x01); // Xoa man hinh LCD 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(0x06); // Di chuyen tro xuat 1 ky tu 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(0x0C);// Tat con tro 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(0x02);// Con tro ve dau man hinh 
	HAL_Delay(50);
	LCD_I2C_Write_CMD(0x80); // Con tro ve dong 1 
	HAL_Delay(2);
}

void LCD_I2C_Clear()// xoa man hinh 
{
	LCD_I2C_Write_CMD(0x01); 
	HAL_Delay(2);
}

void LCD_I2C_Set_Cursor(uint8_t x,uint8_t y) // dat vi tri cho con tro LCD 
{
	if( x == 0)
		LCD_I2C_Write_CMD(0x80+y); 
	else if(x == 1)
		LCD_I2C_Write_CMD(0xC0+y);	
}

void  LCD_I2C_Write_String(char *string)  // in ra chuoi 
{
	for(uint8_t i=0; i < strlen(string); i++ ){
    LCD_I2C_WRITE_DATA( string[i] );
	}
}	
void LCD_I2C_WRITE_NUMBER(int number) // in ra so nguyen
{ 
   char buf[8];
	 sprintf(buf,"%d",number);
	 LCD_I2C_Write_String(buf);
}
void LCD_I2C_WRITE_Num_f(double var)  // in ra so dang thuc 
{ 
   char a[5];
	 sprintf(a, "%3.1f", var);
	 LCD_I2C_Write_String(a);
}