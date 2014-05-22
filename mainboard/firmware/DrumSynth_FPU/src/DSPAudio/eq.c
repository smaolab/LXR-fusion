//----------------------------------------------------------------------------
//
// 3 Band EQ :)
//
// EQ.C - Main Source file for 3 band EQ
//
// (c) Neil C / Etanza Systems / 2K6
//
// Shouts / Loves / Moans = etanza at lycos dot co dot uk
//
// This work is hereby placed in the public domain for all purposes, including
// use in commercial applications.
//
// The author assumes NO RESPONSIBILITY for any problems caused by the use of
// this software.
//
//----------------------------------------------------------------------------

// NOTES :
//
// - Original filter code by Paul Kellet (musicdsp.pdf)
//
// - Uses 4 first order filters in series, should give 24dB per octave
//
// - Now with P4 Denormal fix :)


//----------------------------------------------------------------------------

// ----------
//| Includes |
// ----------

#include <math.h>
#include "eq.h"

// -----------
//| Constants |
// -----------

static double vsa = (1.0 / 4294967295.0); // Very small amount (Denormal Fix)

// ---------------
//| Initialise EQ |
// ---------------

// Recommended frequencies are ...
//
// lowfreq = 880 Hz
// highfreq = 5000 Hz
//
// Set mixfreq to whatever rate your system is using (eg 48Khz)

void init_3band_state(EQSTATE* eq, int lowfreq, int highfreq, int mixfreq,const uint8_t size)
{
// Clear state
uint16_t i;
for(i=0;i<size;i++)
 	{
		eq->f1p0 = 0.0;
		eq->f1p1 = 0.0;
		eq->f1p2 = 0.0;
		eq->f1p3 = 0.0;
		eq->f2p0 = 0.0;
		eq->f2p1 = 0.0;
		eq->f2p2 = 0.0;
		eq->f2p3 = 0.0;
		eq->sdm3 = 0.0;
		eq->sdm2 = 0.0;
		eq->sdm1 = 0.0;
	

		// Set Low/Mid/High gains to unity

		eq->lg = 1.0;
		eq->mg = 1.0;
		eq->hg = 1.0;

		// Calculate filter cutoff frequencies

		eq->lf = 2 * sin(M_PI * ((double)lowfreq / (double)mixfreq));
		eq->hf = 2 * sin(M_PI * ((double)highfreq / (double)mixfreq));

	}

}


// ---------------
//| EQ one sample |
// ---------------

// - sample can be any range you like :)
//
// Note that the output will depend on the gain settings for each band
// (especially the bass) so may require clipping before output, but you
// knew that anyway :)


void calc3BandEqBlock(uint8_t lowFreq, uint8_t midFreq,uint8_t  highFreq, int16_t* buf, const uint8_t size)
{
EQSTATE* eq;
int16_t bufTemp[size];
double l,m,h; // Low / Mid / High - Sample Values
uint16_t i;
int16_t out;
float input;

// WE copy the Sounds before manipulating it 
for(i=0;i<size;i++)
	bufTemp[i] = buf[i] ;
  		

init_3band_state(eq,880,5000,44100,size);

// gain of the three bands, we need to manipoulate this to make the EQ sounds changes ! 
eq->lg = 1.5; // Boost bass by 50%
eq->mg = 0.75; // Cut mid by 25%
eq->hg = 1.0; // Leave high band alone

	
	for(i=0;i<size;i++)
 	{
		// Filter #1 (lowpass)
		input = (bufTemp[i])/((float)0x7fff);
		eq->f1p0 += (eq->lf * (input - eq->f1p0)) + vsa;
		eq->f1p1 += (eq->lf * (eq->f1p0 - eq->f1p1));
		eq->f1p2 += (eq->lf * (eq->f1p1 - eq->f1p2));
		eq->f1p3 += (eq->lf * (eq->f1p2 - eq->f1p3));

		l = eq->f1p3;

		// Filter #2 (highpass)

		eq->f2p0 += (eq->hf * (input - eq->f2p0)) + vsa;
		eq->f2p1 += (eq->hf * (eq->f2p0 - eq->f2p1));
		eq->f2p2 += (eq->hf * (eq->f2p1 - eq->f2p2));
		eq->f2p3 += (eq->hf * (eq->f2p2 - eq->f2p3));

		h = eq->sdm3 - eq->f2p3;

		// Calculate midrange (signal - (low + high))

		m = eq->sdm3 - (h + l);

		// Scale, Combine and store

		l *= eq->lg;
		m *= eq->mg;
		h *= eq->hg;

		// Shuffle history buffer

		eq->sdm3 = eq->sdm2;
		eq->sdm2 = eq->sdm1;
		eq->sdm1 = bufTemp[i];
 
		// result
		out = (l + m + h);
   		//if (out<-32768) out=-32768;
   		//else if (out>32767) out=32767; //Prevents clipping
   		bufTemp[i]=out *((float)0x7fff);
	}; 
	
// We copy back the results :-)	
for(i=0;i<size;i++)
	buf[i] = bufTemp[i] ;
	

}
  


//----------------------------------------------------------------------------
