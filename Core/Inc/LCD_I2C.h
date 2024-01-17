#ifndef __LCD_I2C_H
#define __LCD_I2C_H

#include "main.h"

void LCD_I2C_Write_CMD(uint8_t); // Ham gui lenh

void LCD_I2C_WRITE_DATA(uint8_t); // Ham gui data

void LCD_I2C_Init(void); // Ham khoi dong LCD

void LCD_I2C_Clear(void);// Ham xoa man hinh LCD

void LCD_I2C_Set_Cursor(uint8_t,uint8_t); // Ham dat vi tri con tro LCD 

void LCD_I2C_Write_String(char *); // Ham gui mot chuoi len LCD

void LCD_I2C_WRITE_NUMBER(int);// Ham gui mot so nguyen len LCD

void LCD_I2C_WRITE_Num_f(double);// Ham gui mot so thuc len LCD
#endif