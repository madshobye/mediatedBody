//**************************************************************************
//*   Simple AVR wavetable synthesizer V1.0                                *
//*                                                                        *
//*   Implements 4 voices using selectable waveform and envelope tables    *
//*   Uses 8-bit PWM @ 62.5 kHz for audio output                           *
//*                                                                        *
//*   (C) DZL 2008                                                         *
//**************************************************************************

#define SET(x,y) (x |=(1<<y))		        //-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))       	// |
#define CHK(x,y) (x & (1<<y))           	// |
#define TOG(x,y) (x^=(1<<y))            	//-+

#include "avr/interrupt.h"
#include "avr/pgmspace.h"
#include "sin256.h"
#include "ramp256.h"
#include "saw256.h"
#include "square256.h"
#include "noise256.h"
#include "tria256.h"
#include "env0.h"
#include "env1.h"
#include "env2.h"
#include "env3.h"

//*********************************************************************
//	Audio interrupt
//*********************************************************************

volatile unsigned int PCW[4]={0,0,0,0};				//-Wave phase accumolators
volatile unsigned int FTW[4]={1000,200,300,400};              	//-Wave frequency tuning words
volatile unsigned char AMP[4]={255,255,255,255};                //-Wave amplitudes [0-255]
volatile unsigned int PITCH[4]={500,500,500,500};               //-Voice pitch
volatile int MOD[4]={20,0,64,127};                             	//-Voice envelope modulation [0-1023 512=no mod. <512 pitch down >512 pitch up]
volatile unsigned int wavs[4];                                  //-Wave table selector [address of wave in memory]
volatile unsigned int envs[4];                                  //-Envelopte selector [address of envelope in memory]
volatile unsigned int EPCW[4]={0x8000,0x8000,0x8000,0x8000};    //-Envelope phase accumolator
volatile unsigned int EFTW[4]={10,10,10,10};                    //-Envelope speed tuning word
volatile unsigned int tim=0;                                    //-Sample counter eg. for sequencer
volatile unsigned char divider=4;                               //-Sample rate decimator for envelope

#define FS 16000.0                                              //-Sample rate

float cap_fast,cap_dif;
SIGNAL(TIMER2_COMPA_vect)
{
  OCR2A+=250;					                //-16kHz.

	//SET(DDRB,0); // test togler leg 8
	//TOG(PORTB,0);

  if(divider)
  {
    divider--;
  }
  else
  {

//-------------------------------
// Volume envelope generator
//-------------------------------

     divider=4;
  /* if(!(EPCW[0]&0x8000))
    {
      AMP[0]=255;//pgm_read_byte_near(envs[0]+ ((EPCW[0]+=EFTW[0])>>7) );
      if(EPCW[0]&0x8000)
        AMP[0]=255;
    }
    else
      AMP[0]=255;

    if(!(EPCW[1]&0x8000))
    {
      AMP[1]=255;//pgm_read_byte_near(envs[1]+ ((EPCW[1]+=EFTW[1])>>7) );
      if(EPCW[1]&0x8000)
        AMP[1]=255;
    }
    else
      AMP[1]=255;

    if(!(EPCW[2]&0x8000))
    {
      AMP[2]=pgm_read_byte_near(envs[2]+ ((EPCW[2]+=EFTW[2])>>7) );
      if(EPCW[2]&0x8000)
        AMP[2]=255;
    }
    else
      AMP[2]=255;

    if(!(EPCW[3]&0x8000))
    {
      AMP[3]=pgm_read_byte_near(envs[3]+ ((EPCW[3]+=EFTW[3])>>7) );
      if(EPCW[3]&0x8000)
        AMP[3]=255;
    }
    else
      AMP[3]=255;*/
  }
//-------------------------------
//  Synthesizer/audio mixer
//-------------------------------

  OCR0A=127+
	((
	(((signed char)pgm_read_byte_near(wavs[0]+((PCW[0]+=FTW[0])>>8))*AMP[0])>>8)+
	(((signed char)pgm_read_byte_near(wavs[1]+((PCW[1]+=FTW[1])>>8))*AMP[1])>>8)+
	(((signed char)pgm_read_byte_near(wavs[2]+((PCW[2]+=FTW[2])>>8))*AMP[2])>>8)+
	(((signed char)pgm_read_byte_near(wavs[3]+((PCW[3]+=FTW[3])>>8))*AMP[3])>>8)
	)>>2);

   tim++;
}

//*********************************************************************
//  Setup all voice parameters
//*********************************************************************

void setup_voice(unsigned char voice,unsigned int waveform, float pitch, unsigned int envelope, float length, unsigned int mod)
{
  wavs[voice]=waveform;//[address in program memory]
  envs[voice]=envelope;//[address in program memory]
  EFTW[voice]=(1.0/length)/(FS/(32767.5*10.0));//[s];
  PITCH[voice]=pitch/(FS/65535.0); //[Hz]
  MOD[voice]=mod;//0-1023 512=no mod
}

//*********************************************************************
//  Midi trigger
//*********************************************************************

void mtrigger(unsigned char voice,unsigned char note)
{
  PITCH[voice]=(440. * exp(.057762265 * (note - 69.)))/(FS/65535.0); //[MIDI note]
  EPCW[voice]=0;
}

//*********************************************************************
//  Simple trigger
//*********************************************************************

void trigger(unsigned char voice)
{
  EPCW[voice]=0;
}

//*********************************************************************
//  Make Arduino happy
//*********************************************************************
void setup(void)

{

}
//*********************************************************************
//  Main
//*********************************************************************
void loop(void)
{
 //TCCR1B=0b11011001;
  TCCR2B=0x02;                                    //-Start audio interrupt
  SET(TIMSK2,OCIE2A);                             // |
  sei();                                          //-+
  SET(DDRD,6);				          //-PWM pin
 
  TCCR0A=0xA3;                                    //-8 bit audio PWM
  TCCR0B=0x01;                                    // |
  OCR0A=127;                                      //-+
  OCR0B=255;
    pinMode(5,OUTPUT);
/*
unsigned char pattern[4][32]=
{
	{1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0},
	{0,0,48,0,0,0,48,0,0,0,48,0,0,0,48,0,0,0,51,0,0,0,51,0,0,0,50,0,0,0,50,0},
	{60,0,0,60,0,0,72,0,60,0,60,0,67,0,67,0,60,0,0,60,0,0,74,0,63,0,63,0,67,0,74,0},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0}
};
*/
unsigned char pattern[4][32]=
{
	{1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0},
	{48,0,0,41,0,0,44,0,44,0,0,46,0,0,41,0,48,0,0,41,0,0,44,0,0,44,0,46,0,0,51,0},
	{0,0,0,50,0,0,0,0,0,0,0,50,0,0,0,0,0,0,0,50,0,0,0,0,0,0,0,50,0,0,0,0},
	{0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0}
};

unsigned char song[4][32]=
{
	{0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0},
	{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
	{0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

/* uncomment if defaults are needed
   wavs[0]=(unsigned int)SinTable;
   wavs[1]=(unsigned int)RampTable;
   wavs[2]=(unsigned int)SawTable;
   wavs[3]=(unsigned int)NoiseTable;

   envs[0]=(unsigned int)Env3;
   envs[1]=(unsigned int)Env2;
   envs[2]=(unsigned int)Env1;
   envs[3]=(unsigned int)Env0;
*/
  setup_voice(0,(unsigned int)SinTable,200.0,(unsigned int)Env1,1.0,1500);
  setup_voice(1,(unsigned int)SinTable,100.0,(unsigned int)Env1,1.0,1500);
  setup_voice(2,(unsigned int)TriangleTable,100.0,(unsigned int)Env2 ,.5,1500);
  setup_voice(3,(unsigned int)SinTable,1200.0,(unsigned int)Env3,.02,1500);


  unsigned int counter=0;
  unsigned char bar;
  unsigned int  demo;
  Serial.begin(9600);

  float color;
   float colorSpeed = 1;
  setupPole();
  float lastPoleValue = 0;
  float change = 0;
  while(1)
  {
    
    
     float poleValue =getPoleValue();
     float tmpChange =  abs(poleValue-lastPoleValue) ;
     if( change < tmpChange && poleValue > 0.20)
     {
       change =change *0.5+ tmpChange * 0.5;
     }
     else
     {
       change = change * 0.99 +tmpChange * 0.01; 
     } 
       
     lastPoleValue = poleValue;
     float poleValue_sensitive = sqrt(poleValue);
  //Serial.println(change);
  //  delay(2); ///????
color = color+colorSpeed;
      PITCH[0]= PITCH[0] * 0.99 + 0.01 *colorSpeed*40.0f; // slow-fast;
      PITCH[1]= PITCH[1] * 0.9 + 0.1 * (1500.0f*change+ colorSpeed*40.0f) ;
      PITCH[2]= 200;  //*poleValue*350.0f * (sin( poleValue*10)+1.0f)/2 ;
      PITCH[3]= PITCH[3] * 0.95 + 0.05 *(poleValue_sensitive*1000.0+change*60000.0f); ;// PITCH[3] * 0.99f + poleValue* 0.01f ;
      //  Serial.println(colorSpeed);
      AMP[0]= colorSpeed * 255.0f;  // rising it is all good
      AMP[1]= min(200,change*555.0f); //hummm
      AMP[2]=min(200,poleValue_sensitive*change * 2000); //du dom
       AMP[3] =155;//  (sin( millis()/10000.0f)+1)*88;

     // analogWrite(5,0);
        
     analogWrite(10,round((sin(color/400 + poleValue)+1.0f) *60.50f) );
      analogWrite(9,round(poleValue*255.0f));
       //analogWrite(11,round((sin(color/100 + poleValue*1.4f)+1.0f) *127.50f) );
       readPole();
       
        if(poleValue> 0.50)
      {
             colorSpeed = colorSpeed * 0.9999f + poleValue*300.0f  * 0.0001f;
      }
      else
      {
             colorSpeed = colorSpeed * 0.995f;
      }
       colorSpeed = max(0.3,min(colorSpeed,100));
    
     if(tim>1000*(100-colorSpeed)/50.0)
    {
     
      bar=counter&0x1f;
      demo=counter>>5;
  
   
 /*     switch(demo && false)
      {
        case 4:
        {
          setup_voice(3,(unsigned int)TriangleTable,1500.0,(unsigned int)Env3,.03,100);
        };break;      
        
        case 8:
        {
          setup_voice(3,(unsigned int)NoiseTable,1500.0,(unsigned int)Env3,.03,300);
        };break;      
      
        case 12:
        {
          setup_voice(2,(unsigned int)TriangleTable,100.0,(unsigned int)Env1,2.0,512);
        };break;      
      
        case 16:
        {
          setup_voice(2,(unsigned int)RampTable,100.0,(unsigned int)Env1,1.0,512);
          counter=0;
        };break;      
      }

      if(song[0][demo])
        if(pattern[0][bar])
          trigger(0);
      if(song[1][demo])
      if(pattern[1][bar])   
        mtrigger(1,pattern[1][bar]);
      if(song[2][demo])
      if(pattern[2][bar])
        mtrigger(2,pattern[2][bar]);
      if(song[3][demo])
      if(pattern[3][bar])
        trigger(3);*/
    
    
    
      if(pattern[1][bar])
        trigger(3);
      tim=0;
      //counter++;
      //counter&=0x3ff;
   }   

//************************************************
//  Modulation engine
//************************************************
   
   FTW[0]=PITCH[0]+(PITCH[0]*(EPCW[0]/(32767.5*128.0  ))*((int)MOD[0]-512));
   FTW[1]=PITCH[1]+(PITCH[1]*(EPCW[1]/(32767.5*128.0  ))*((int)MOD[1]-512));
   FTW[2]=PITCH[2]+(PITCH[2]*(EPCW[2]/(32767.5*128.0  ))*((int)MOD[2]-512));
   FTW[3]=PITCH[3]+(PITCH[3]*(EPCW[3]/(32767.5*128.0  ))*((int)MOD[3]-512));
  }
}

//*****************************************************************************************************************
//  
//*****************************************************************************************************************

 // CapSense   cs_4_2 = CapSense(A5,2); 

// POLE SYSTEM
void setupPole()
{
  //cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF); 
  
 pinMode(2,OUTPUT);
 //digitalWrite(14,HIGH  );
 
}
  int high=0, low=0;
   float sum2=0;
void readPole()
{
    digitalWrite(2,HIGH);
   
     high = analogRead(5);

    digitalWrite(2,LOW);

    low = analogRead(5);

 sum2 = sum2*0.9f + (high - low)* 0.1f;

}

float getPoleValue()
{
 
  Serial.println(sum2); 
     return  min(1,max(0,(sum2-350)/400));
     
     // return  sqrt(sqrt(min(1,max(0,(sum2-240)/800))));
 
 // Serial.print("  ");
  float vavlue = max(min(1,1-(log(sum2/5000.0f)+1)),0);
 // Serial.println(vavlue);
  
   return vavlue; // sqrt(sqrt(min(1,max(0,min(1100,sum2-100)/1100))));
// return sin(min(1,max(0,min(1900,sum2-200)/1800))*2.5f);
   // float value =min(1,max(0,min(1900,sum2-200)/1800));
   // return 1+(log(value)*0.4);
}

