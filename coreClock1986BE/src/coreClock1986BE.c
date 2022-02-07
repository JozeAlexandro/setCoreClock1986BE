/** ************************************************************
 * \brief Source code for processor clock frequency setting 
					module 1986BE 
 * \file coreClock1986BE.h
 * \author Petrov A.S.
 * ************************************************************/

#include "../h/coreClock1986BE.h" // module header
#include "coreClock1986BEP.h" // module private header

/** ************************************************************
 * API definition
 * ************************************************************/

MGz_t freqHsiCpuSet( MGz_t freq )
{
	const static MGz_t HSI_INTNL_FREQ = MIN_FREQ;
	
	if( freq < HSI_INTNL_FREQ || freq > MAX_FREQ )
		return PARAM_ERR;
	
	/// Frequency should be a multiple of 8
	MGz_t realFreq = freq - freq % HSI_INTNL_FREQ;
	
	/// BKP clocking. Be able to turn on the HSI 
	/// and install the regulator
	RST_CLK_PCLKcmd( RST_CLK_PCLK_BKP, ENABLE );
	
	/// regulator mode and resist 
	regulatorSettings( realFreq );

	/// HSI frequency tuning up to clear 8 MHz
	const static int TRIVAL_8_MHz = 25;
	RST_CLK_HSIadjust( TRIVAL_8_MHz );


	/// HSI clocking
	RST_CLK_HSIcmd( ENABLE );
	
	/// Is the HSI (8MHz) generator on. ( Must be switched on when power is supplied ) 
	if( SUCCESS != RST_CLK_HSIstatus() ) 
		return FREQ_CPU_ERR;
	
	/// Multiplier for HSI frequency. 
	int multiplier = realFreq / HSI_INTNL_FREQ; 
	if( 0 != multiplier ) multiplier -= 1;
	
	/// CPU PLL ON, set multiplier 
	turnCpuAndSetMult( multiplier );
	
	/// Checking CPU PLL turning on 
	if( SUCCESS != RST_CLK_CPU_PLLstatus() )
		return FREQ_CPU_ERR;
	
	/// Muxing PLL
	setHCLKMux();
		
	return realFreq;
}

/** ************************************************************
 * Private definition
 * ************************************************************/

static unsigned int countRegulatorValue( MGz_t freq )
{
	if( freq < 10 )
	{
		return 0x0; /// 0b000
	}
	else if( freq < 40 )
	{
		return 0x5; /// 0b101
	}
	else if( freq < 80 )
	{
		return 0x6; /// 0b110;
	}
	else /// freq >= 80
	{
		return 0x7; /// 0b111
	}
	
}