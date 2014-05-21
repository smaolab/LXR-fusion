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
#include <complex.h>
#include <fcntl.h>
#include <sys/stat.h>

#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define BIT(x) (0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))

#define samplerate 44100     
#define lfoskipsamples 25 // How many samples are processed before compute the lfo value again

// table to fixup Q in order to remain constant for various pole frequencies, from Tim Stilson's code @ CCRMA (also in CLM distribution)

static float gaintable[199] = { 0.999969, 0.990082, 0.980347, 0.970764, 0.961304, 0.951996, 0.94281, 0.933777, 0.924866, 0.916077, 0.90741, 0.898865, 0.890442, 0.882141 , 0.873962, 0.865906, 0.857941, 0.850067, 0.842346, 0.834686, 0.827148, 0.819733, 0.812378, 0.805145, 0.798004, 0.790955, 0.783997, 0.77713, 0.770355, 0.763672, 0.75708 , 0.75058, 0.744141, 0.737793, 0.731537, 0.725342, 0.719238, 0.713196, 0.707245, 0.701355, 0.695557, 0.689819, 0.684174, 0.678558, 0.673035, 0.667572, 0.66217, 0.65686, 0.651581, 0.646393, 0.641235, 0.636169, 0.631134, 0.62619, 0.621277, 0.616425, 0.611633, 0.606903, 0.602234, 0.597626, 0.593048, 0.588531, 0.584045, 0.579651, 0.575287 , 0.570953, 0.566681, 0.562469, 0.558289, 0.554169, 0.550079, 0.546051, 0.542053, 0.538116, 0.53421, 0.530334,0.52652, 0.522736, 0.518982, 0.515289, 0.511627, 0.507996 , 0.504425, 0.500885, 0.497375, 0.493896, 0.490448, 0.487061, 0.483704, 0.480377, 0.477081, 0.473816, 0.470581, 0.467377, 0.464203, 0.46109, 0.457977, 0.454926, 0.451874, 0.448883, 0.445892, 0.442932, 0.440033, 0.437134, 0.434265, 0.431427, 0.428619, 0.425842, 0.423096, 0.42038, 0.417664, 0.415009, 0.412354, 0.409729, 0.407135, 0.404572, 0.402008, 0.399506, 0.397003, 0.394501, 0.392059, 0.389618, 0.387207, 0.384827, 0.382477, 0.380127, 0.377808, 0.375488, 0.37323, 0.370972, 0.368713, 0.366516, 0.364319, 0.362122, 0.359985, 0.357849, 0.355713, 0.353607, 0.351532,
0.349457, 0.347412, 0.345398, 0.343384, 0.34137, 0.339417, 0.337463, 0.33551, 0.333588, 0.331665, 0.329773, 0.327911, 0.32605, 0.324188, 0.322357, 0.320557,0.318756, 0.316986, 0.315216, 0.313446, 0.311707, 0.309998, 0.308289, 0.30658, 0.304901, 0.303223, 0.301575, 0.299927, 0.298309, 0.296692, 0.295074, 0.293488, 0.291931, 0.290375, 0.288818, 0.287262, 0.285736, 0.284241, 0.282715, 0.28125, 0.279755, 0.27829, 0.276825, 0.275391, 0.273956, 0.272552, 0.271118, 0.269745, 0.268341, 0.266968, 0.265594, 0.264252, 0.262909, 0.261566, 0.260223, 0.258911, 0.257599, 0.256317, 0.255035, 0.25375 };

typedef struct
{
  // Parameters
	float x_resonance;
	float x_cutoff;
  // Internals
	float p, Q;
	float lowpass, highpass, bandpass;

  int srate;
  float *output; //pointer to the filter output, based on type
} f_moog;

float state[4], output; //for MOOG filter : should be global scope / preserved between calls

//End rstephane


struct params
{
   float freq,startphase,fb;
   int delay;
} awparams;
//alien wah internal parameters

struct alienwahinternals
{
 _Complex *delaybuf;
 float lfoskip;
 long int t;
 _Complex c;
 int k;
} awint;
// end rstephane



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
void calcDelayBlock(float delay, int16_t* buf, const uint8_t size)
{

	uint8_t j,i;
	int16_t SampleRate=44000;
	int16_t DlyBuffer[size];
	
	for(i=0;i<size;i++)
	{
		DlyBuffer[ i ] = buf[ i ];
		j = i - ((delay) * SampleRate); // delay 0 to 1

		if( j < 0 )
		    j = SampleRate + j;
		buf[i] = DlyBuffer[ i++ ] = buf[i] + (DlyBuffer[ j ] * 0.5); // fFeedback);

	}



}

// rstephane : OTO biscuit
void calcOTOFxBlock(uint8_t maskType, int16_t* buf,const uint8_t size, uint8_t otoAmount)
{
	
		
	uint8_t i,j;
	uint16_t temp;
	int16_t bufTemp[size];
	
	// for a strange effect derived from moog filter from musicdsp.org
	float c , r ,v0,v1;
	
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
		case 13 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x0000F7FF; 
			break;
		case 14 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x01009009; 
			break;
		case 15 : // rien !! a virer
			//for(i=0;i<size;i++)
			//	bufTemp[i] &= 0x00003D5F; 
			//calcAlienWahFxBlock(maskType, bufTemp, size);

			//Parameter calculation
			//cutoff and resonance are from 0 to 127
			//   c = pow(0.5, (128-cutOff)   / 16.0);
			// r = pow(0.5, (resonance+24) / 16.0);

			 c = pow(0.5, (128-35)   / 16.0);
			 r = pow(0.5, (40+24) / 16.0);
			
			//Loop:
			for ( i=0; i < size; i++ ) {
			   v0 =  (1-r*c)*v0  -  (c)*v1  + (c)*bufTemp[i];
			   v1 =  (1-r*c)*v1  +  (c)*v0;

			   bufTemp[i] = v1; // Low pass
			}			
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


// rstephane : Alien Wah from music dsp . org
/*
 Alien-Wah by Nasca Octavian Paul from Tg. Mures, Romania
 e-mail:  <paulnasca@email.ro> or <paulnasca@yahoo.com>.
*/

/*
 The algorithm was found by me by mistake(I was looking for something else);
 I called this effect "Alien Wah" because sounds a bit like wahwah, but more strange.
 The ideea of this effect is very simple: It is a feedback delay who uses complex numbers.
 If x[] represents the input and y[] is the output, so a simple feedback delay looks like this:
 y[n]=y[n-delay]*fb+x[n]*(1-fb)
 
 'fb' is a real number between 0 and 1.
 If you change the fb with a complex number who has the MODULUS smaller than 1, it will look like this.

 fb=R*(cos(alpha)+i*sin(alpha));  i^2=-1; R<1;
 y[n]=y[n-delay]*R*(cos(alpha)+i*sin(alpha))+x[n]*(1-R);

 alpha is the phase of the number and is controlled by the LFO(Low Frequency Oscillator).
 If the 'delay' parameter is low, the effect sounds more like wah-wah,
 but if it is big, the effect will sound very interesting.
 The input x[n] has the real part of the samples from the wavefile and the imaginary part is zero.
 The output of this effect is the real part of y[n].

 Here it is a simple and unoptimised implementation of the effect. All parameters should be changed at compile time.
 It was tested only with Borland C++ 3.1.

 Please send me your opinions about this effect.
 Hope you like it (especially if you are play to guitar).
 Paul.
*/

/*
Alien Wah Parameters

 freq       - "Alien Wah" LFO frequency
 startphase - "Alien Wah" LFO startphase (radians), needed for stereo
 fb         - "Alien Wah" FeedBack (0.0 - low feedback, 1.0 = 100% high feedback)
 delay      -  delay in samples at 44100 KHz (recomanded from 5 to 50...)
*/


//effect initialisation
void init_phase(float freq,float startphase,float fb,int delay)
{  
  awparams.freq=freq;
  awparams.startphase=startphase;
  awparams.fb=fb/4+0.74;
  awparams.delay=(int)(delay/44100.0*samplerate);
  
  if (delay<1) delay=1;
  
  awint.delaybuf=awparams.delay;
  //complex[awparams.delay];
  
  int i;
  for (i=0;i<delay;i++) 
  	awint.delaybuf[i]=0,0;
  awint.lfoskip=freq*2*3.141592653589/samplerate;
  awint.t=0;
}

//-----------------
// rstephane : ALien Wah from Musicdsp.org

void calcAlienWahFxBlock(uint8_t maskType, int16_t* buf,const uint8_t size)
{
  
	uint16_t i;
	float lfo,out;
	_Complex outc;

	// WE copy the Sounds before manipulating it 
	int16_t bufTemp[size];
	for(i=0;i<size;i++)
		bufTemp[i] = buf[i] ;
	
	//init_phase(0.6,0,0.5,20);  //effects parameters
	init_phase(0.4,0,0.8,40);  //effects parameters
	
 	for(i=0;i<size;i++)
 	{
   		if (awint.t++%lfoskipsamples==0)
   		{
      			lfo=(1+cos(awint.t*awint.lfoskip+awparams.startphase));
      			awint.c= cos(lfo)*awparams.fb,sin(lfo)*awparams.fb;
   		};
   	
   		outc=awint.c*awint.delaybuf[awint.k]+(1-awparams.fb)*bufTemp[i];
   		awint.delaybuf[awint.k]=outc;
	   	if ((++awint.k)>=awparams.delay)
      			awint.k=0;
   
   		out=creal(outc)*3;  //take real part of outc
   		if (out<-32768) out=-32768;
   		else if (out>32767) out=32767; //Prevents clipping
   
   		bufTemp[i]=out;
	}; 
	
	for(i=0;i<size;i++)
		buf[i] = bufTemp[i] ;
	
	
}

// -------------------
// Moog filter 4poles

 float saturate( float input ) { 
//clamp without branching
#define _limit 0.95
  float x1 = fabsf( input + _limit );
  float x2 = fabsf( input - _limit );
  return 0.5 * (x1 - x2);
}

float crossfade( float amount, float a, float b ) {
  return (1-amount)*a + amount*b;
}


void moog_perform(uint8_t moogFilterType,int8_t cutOff,int8_t resonance, int16_t* buf,const uint8_t size)
{ 
   int i,pole;
  float temp, input;
   f_moog* x;
   
   // update the value of the Cutoff and Reso
   moog_init( x );
   moog_cutoff( x, cutOff );
   //moog_resonance( x, resonance ); not needed as implicit call within the previous function
   
  for ( i=0; i < size; i++ ) {
          input = (buf[i]/((float)0x7fff));
          output = 0.25 * ( input - output ); //negative feedback
          
         /* for( pole = 0; pole < 4; pole++) {
                  temp = state[pole];
                  output = saturate( output + x->p * (output - temp));
                  state[pole] = output;
                  output = saturate( output + temp );
          }
          float lowpass = output;
          float highpass = input - output;
          float bandpass = 3 * state[2] - x->lowpass; //got this one from paul kellet
          */
          //*output++ = lowpass;
	  //*output = lowpass;
          output *= x->Q;  //scale the feedback
          buf[i] = output * FILTER_GAIN; // Low pass
	}

}
	

float MAX( float a, float b) {
  a -= b;
  a += fabsf( a );
  a *= 0.5;
  a += b;
  return a;
}

void moog_resonance( f_moog *x, float resonance )
{
	resonance = MAX( resonance, 1);
	x->x_resonance = resonance;

	float ix, ixfrac;
	int ixint;
	ix = x->p * 99;
	ixint = floor( ix );
	ixfrac = ix - ixint;
   	
	x->Q = resonance * crossfade( ixfrac, gaintable[ ixint + 99 ], gaintable[ ixint + 100 ] );

}

void moog_cutoff( f_moog *x, float frequency )
{
	x->x_cutoff = frequency;

	float fc = 2 * frequency / x->srate;
    	float x2 = fc*fc;
	float x3 = fc*x2;
    
    	x->p = -0.69346 * x3 - 0.59515 * x2 + 3.2937 * fc - 1.0072; //cubic fit
	moog_resonance( x, x->x_resonance);
}

void moog_init( f_moog *x )
{
	x->srate = 10; //(int)sys_getsr();
	moog_set_lowpass(x); //default

}

void moog_set_lowpass( f_moog *x )
{
	x->output = &(x->lowpass);
}

void moog_set_highpass( f_moog *x )
{
	x->output = &(x->highpass);
}

 void moog_set_bandpass( f_moog *x )
{
	x->output = &(x->bandpass);
}

/*
float MIN( float a, float b) {
  a = b - a;
  a += fabsf( a );
  a *= 0.5;
  a = b - a;
  return a;
}*/


