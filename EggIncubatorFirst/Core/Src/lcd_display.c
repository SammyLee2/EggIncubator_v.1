/*
 * lcd_display.c
 *
 *  Created on: Aug 13, 2024
 *      Author: sam
 */
#include "lcd_display.h"
#define bit(b) (1UL << (b))

static SPI_HandleTypeDef * m_hspi;


void lcd_init(SPI_HandleTypeDef * hspi){
	m_hspi = hspi;
}



unsigned int hexDigitValue[] = {
    0xFC,    /* Segments to light for 0  */ //11111100 00000011
    0x60,    /* Segments to light for 1  */ //01100000
    0xDA,    /* Segments to light for 2  */ //11011010
    0xF2,    /* Segments to light for 3  */ //11110010
    0x66,    /* Segments to light for 4  */ //01100110
    0xB6,    /* Segments to light for 5  */ //10110110
    0xBE,    /* Segments to light for 6  */ //10111110
    0xE0,    /* Segments to light for 7  */ //11100000
    0xFE,    /* Segments to light for 8  */ //11111110
    0xF6     /* Segments to light for 9  */ //11110110
};


void setDigit(uint8_t row, uint8_t digit, int decimalPoint)
{
	//3 1 0
	uint8_t rowSelector;
      uint8_t data;
      rowSelector = bit(3-row)<<4;


      data =  ~  (hexDigitValue[ digit & 0xF]);
      if(decimalPoint){
    	  data &= 0xFE;
      }

	     //HAL_SPI_Transmit(m_hspi, &digit, 1, 100);
	     HAL_SPI_Transmit(m_hspi, &data, 1, 100);

    	      // Now shift 4 row bits into the first 74HC595 and latch
    	      //digitalWrite(ST_CP, LOW);
//    	     HAL_SPI_Transmit(m_hspi, &row, 1, 100);
    	     HAL_SPI_Transmit(m_hspi, &rowSelector, 1, 100);

    	     HAL_GPIO_WritePin(PA6_RCLK_GPIO_Port, PA6_RCLK_Pin, 1);


    	     HAL_GPIO_WritePin(PA6_RCLK_GPIO_Port, PA6_RCLK_Pin, 0);

      }


      // First 8 data bits all the way into the second 74HC595
      //shiftOut(DS, SH_CP, LSBFIRST, data );


      //shiftOut(DS, SH_CP, LSBFIRST, rowSelector );

      //digitalWrite(ST_CP, HIGH);




/* Displays a number as a 4-digit decimal number on the display
 *   Note this is multiplexed, so you need to keep calling it
 *   or you'll end up with just one digit lit.
 */
void displayNumber(unsigned int number){
    for(unsigned int i=0; i<4; i++){
      setDigit(i, number % 10, 0); // display righmost 4 bits (1 digit)
      number = number / 10;  // roll on to the next digit
      HAL_Delay(1);
    }
}
/*
void displayTemperature(int temperature){

	if(temperature<100){
	    for(unsigned int i=0; i<3; i++){
	      setDigit(i, temperature % 10, 1); // display righmost 4 bits (1 digit) //% only works on integer...
	      temperature = temperature / 10;  // roll on to the next digit
	      HAL_Delay(1);






	    }
	}
	else if(temperature>100){
	    for(unsigned int i=0; i<4; i++){
	      setDigit(i, temperature % 10, 0); // display righmost 4 bits (1 digit)
	      temperature = temperature / 10;  // roll on to the next digit
	      HAL_Delay(1);
	      //123.4
	    }
	}

}
*/

static uint8_t m_tempercount = 0;

void displayTemperature(int temperature){

	int n1, n2, n3, n4;

/*	n1 = (int) temperature % 10; //4
	n2 = (int) ((temperature % 100)) / 10; // 3
	n3 = (int) ((temperature)) / 100; //2
	n4 = (int) ((temperature)) / 1000; //1
*/
	n1 = (int) temperature % 10; //4
	n2 = (int) (temperature/10) % 10; //3
	n3 = (int) (temperature/100) % 10; //2
	n4 = (int) (temperature/1000) % 10; // 1
/*
	 	  setDigit(0, 1234%10,0);
	      setDigit(1, (1234/10)%10,0);
	      setDigit(2, (1234/100)%10,0);
	      setDigit(3, (1234/1000)%10,0);
*/
	switch (m_tempercount) {

	case 0:
		//	      setDigit(i, temperature % 10, 0); // display righmost 4 bits (1 digit)

		//send_port(_LED_0F[n1], 0b0001); //0
		setDigit(0, n1, 0);
		break;
	case 1:
//		send_port(_LED_0F[n2] & 0x7F, 0b0010); //1
		setDigit(1, n2, 1);
		break;
	case 2:
		if (temperature > 99) {
//얘가 문제같
			setDigit(2, n3, 0);
			//send_port(_LED_0F[n3], 0b0100);
		}
		break;
	case 3:
		if (temperature > 999) {
			setDigit(3, n4, 0);
			//send_port(_LED_0F[n4], 0b1000);
		}
		break;
	default:
		break;
	}

	m_tempercount++;

	if (temperature > 999 && m_tempercount >= 5) {
		m_tempercount = 0;
	}else if(temperature > 99 && m_tempercount >= 4){
		m_tempercount = 0;
	}else if(temperature <= 99 && m_tempercount >= 3){
		m_tempercount = 0;
	}

}






	  /*0x90 = 1001 0000
	   * 0xD0 = 1101 0000
	   *
	   */
/*//	setDigit(0b01000000,0b10011111,0);
	setDigit(0x40,0xFF,0);//0000 0001 1000 0000
	// 0x40이 두번째자리 0x40 = 0100 0000

	  HAL_Delay(1);
		setDigit(0x20,0xF,0);//0x20 = 0010 0000
		//0x20이 지금 첫번째 자리.

//	  setDigit(0b00100000,0b10011111,0);
	  HAL_Delay(1);
		setDigit(0x10,0x0,0);//0000 0001 1000 0000

	  //	setDigit(0b00010000,0b10011111,0);
	  HAL_Delay(1);*/
