/*
 * distortion.c
 *
 *  Created on: 14.04.2012
 * ------------------------------------------------------------------------------------------------------------------------
 *  Copyright 2013 Julian Schmidt
 *  Julian@sonic-potions.com
 * ------------------------------------------------------------------------------------------------------------------------
 *  This file is part of the Sonic Potions LXR drumsynth firmware.
 * ------------------------------------------------------------------------------------------------------------------------
 *  Redistribution and use of the LXR code or any derivative works are permitted
 *  provided that the following conditions are met:
 *
 *       - The code may not be sold, nor may it be used in a commercial product or activity.
 *
 *       - Redistributions that are modified from the original source must include the complete
 *         source code, including the source code for all components used by a binary built
 *         from the modified sources. However, as a special exception, the source code distributed
 *         need not include anything that is normally distributed (in either source or binary form)
 *         with the major components (compiler, kernel, and so on) of the operating system on which
 *         the executable runs, unless that component itself accompanies the executable.
 *
 *       - Redistributions must reproduce the above copyright notice, this list of conditions and the
 *         following disclaimer in the documentation and/or other materials provided with the distribution.
 * ------------------------------------------------------------------------------------------------------------------------
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ------------------------------------------------------------------------------------------------------------------------
 */

#include "distortion.h"
#include "math.h"

#include "config.h"


// rstephane : declare function for bit wise manipulation
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define BIT(x) (0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))


//--------------------------------------------------
__inline void setDistortionShape(Distortion *dist, uint8_t shape)
{
	dist->shape = 2*(shape/128.f)/(1-(shape/128.f));
}
//--------------------------------------------------
void calcDistBlock(const Distortion *dist, int16_t* buf, const uint8_t size)
{
	uint8_t i;
	for(i=0;i<size;i++)
	{
			float x = buf[i]/32767.f;
			x = (1+dist->shape)*x/(1+dist->shape*fabsf(x));
			buf[i] = (x*32767);
	}
}
//--------------------------------------------------

float distortion_calcSampleFloat(const Distortion *dist, float x)
{
	return (1+dist->shape)*x/(1+dist->shape*fabsf(x));
}

// rstephane : DELAY
void calcDelayBlock(uint8_t delay, int16_t* buf, const uint8_t size)
{

	uint8_t j,i;
	int16_t SampleRate=44000;
	int16_t DlyBuffer[size];
	
	for(i=0;i<size;i++)
	{
		DlyBuffer[ i ] = buf[ i ];
		j = i - ((delay) * SampleRate);

		if( j < 0 )
		    j = SampleRate + j;
		buf[i] += DlyBuffer[ j ];
	}
}

// rstephane : OTO biscuit
void calcOTOFxBlock(uint8_t maskType, int16_t* buf,const uint8_t size, uint8_t otoAmount)
{
	uint8_t i,j;
	uint16_t temp;
	int16_t bufTemp[size];
	
	// TO change the range from 0 to 127 to 0.0 to 1.0
	float old_min = 0;
	float old_max = 127;
	float new_min = 0.0;
	float new_max = 1.0;
	float knobValue, dry, wet;
	
	// dry and Wet infor
	float dryFloatTemp;
	float wetFloatTemp;
	
	// knob Value (0 to 1 <--> 0 to 127) 
	knobValue = (  ( (otoAmount - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min  );
	dry = (1-knobValue);
	wet = fabs((1-knobValue)-1);
	
	// WE copy the Sounds before manipulating it 
	for(i=0;i<size;i++)
		bufTemp[i] = buf[i] ;
					
	switch(maskType)
	{
		case 1 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x0100100F; // ou 7F avce en plus COA à 0 !!!
			break;
		case 2 :
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x00000FFF; // remove 12 TOP BIts 16 bits to 8 bitmap 
			break;
		case 3 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x000007FF; // remove 8 TOP BIts 16 bits to 8 bitmap 
			break;
		case 4 : 
			for(i=0;i<size;i++)
				bufTemp[i] = bufTemp[i] << 1 ;	
			break;
		case 5 : 
			for(i=0;i<size;i++)
				bufTemp[i] = bufTemp[i] << 2 ;	
			break;
		case 6 :
			for(i=0;i<size;i++)
			{
				bufTemp[i] = bufTemp[i] << 3 ;	
			}
			break;
		/* nice effect !!!!
		case 6 :
			for(i=0;i<size;i++)
			{
				bufTemp[i] = buf[i] << 3 ;
		 	
				dryFloatTemp = (float) buf[i] * dry;
				wetFloatTemp = (float) bufTemp[i] * wet;
				buf[i] = (uint16_t) dryFloatTemp + (uint16_t) wetFloatTemp ;
		 	} 
			break;
		*/
		case 7 :
			for(i=0;i<size;i++)
				bufTemp[i] = bufTemp[i] << 5 ;		
			break;
		 case 8 : // Trash !!!
			 
			for(i=0;i<size;i++)
				bufTemp[i] = bufTemp[i] << 7 ;		
			break;
		case 9 : // reverse all bit 1 becomes 0 :-)
			for(i=0;i<size;i++)
			{
			
				for (j=0;j<16;j++)
					temp = bit_flip(bufTemp[i],BIT(j));
				bufTemp[i] = temp;
			}			
			break;
		case 10 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= (0x0000F000);	
		 	break;
		case 11 : // bof
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x0000E7FF; 
			break;
		case 12 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x00000087; 
			break;
		/*case 12 : // pas mal a voir
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x00003D5F; 
			break; */
		case 13 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x0000F7FF; 
			break;
		case 14 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x01009009; 
			break;
		case 15 : // rien !! a virer
			for(i=0;i<size;i++)
				bufTemp[i] &= 0xFF7F; 
			break;
		
		default: 
			maskType = 0;
			break;
		break;	
	}	

// We merge the Dry and Wet Signal
if (maskType!=0)
	for(i=0;i<size;i++)
	{
		dryFloatTemp = (float) (buf[i]) * dry;
		wetFloatTemp = (float) (bufTemp[i]) * wet;
		buf[i] = wetFloatTemp + dryFloatTemp ;
	}
 	
}




// rstephane : OTO biscuit
void calcOTOFxBlockOLD(uint8_t maskType, int16_t* buf,const uint8_t size)
{
	uint8_t i,j;
	uint16_t temp;
	int16_t* bufTemp;
	int16_t* bufTemp2;
	
	switch(maskType)
	{
		case 1 : 
			for(i=0;i<size;i++)
				buf[i] &= 0x0100100F; // ou 7F avce en plus COA à 0 !!!
			break;
		case 2 :
			for(i=0;i<size;i++)
				buf[i] &= 0x00000FFF; // remove 12 TOP BIts 16 bits to 8 bitmap 
			break;
		case 3 : 
			for(i=0;i<size;i++)
				buf[i] &= 0x000007FF; // remove 8 TOP BIts 16 bits to 8 bitmap 
			break;
		case 4 : 
			
			for(i=0;i<size;i++)
			{
				buf[i] = buf[i] << 1 ;
		 	}	
			break;
		
		case 5 : 
			
			for(i=0;i<size;i++)
			{
				buf[i] = buf[i] << 2 ;
		 	}	
			break;
		case 6 : 
			for(i=0;i<size;i++)
			{
				buf[i] = buf[i] << 3 ;
		 	}	
			break;
		case 7 :
			for(i=0;i<size;i++)
			{
				buf[i] = buf[i] << 5 ;
		 	}	
			break;
		 case 8 : // Trash !!!
			 
			for(i=0;i<size;i++)
			{
				buf[i] = buf[i] << 7 ;
		 	}	
			break;
		case 9 : // reverse all bit 1 becomes 0 :-)
			for(i=0;i<size;i++)
			{
			
				bufTemp[i] = buf[i];
				for (j=0;j<16;j++)
					temp = bit_flip(bufTemp[i],BIT(j));
				buf[i] = temp;
			}			
			break;
		case 10 : 
			for(i=0;i<size;i++)
				buf[i] &= (0x0000F000);
			break;
		case 11 : // bof
			for(i=0;i<size;i++)
				buf[i] &= 0x0000E7FF; 
			break;
		case 12 : 
			for(i=0;i<size;i++)
				buf[i] &= 0x00000087; 
			break;
		/*case 12 : // pas mal a voir
			for(i=0;i<size;i++)
				buf[i] &= 0x00003D5F; 
			break; */
		case 13 : 
			for(i=0;i<size;i++)
				buf[i] &= 0x0000F7FF; 
			break;
		case 14 : 
			for(i=0;i<size;i++)
				buf[i] &= 0x01009009; 
			break;
		case 15 : // rien !! a virer
			for(i=0;i<size;i++)
				buf[i] &= 0xFF7F; 
			break;
		
		default: 
			break;
		break;	
	}	
   /*ASR    R7, R8, #9  ; Arithmetic shift right by 9 bits
    LSLS   R1, R2, #3  ; Logical shift left by 3 bits with flag update
    LSR    R4, R5, #6  ; Logical shift right by 6 bits */
}

