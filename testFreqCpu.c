#include "MDR32F9Qx_rst_clk.h"          // Keil::Drivers:RST_CLK


/// warning reset registers

/// api 


typedef unsigned int MGz_t;

/// \todo naming
enum 
	{ 
		FREQ_CPU_ERR = 0,
		PARAM_ERR = -1
	};

		const static MGz_t MIN_FREQ = 8;
	const static MGz_t MAX_FREQ = 128;




MGz_t freqHsiCpuSet( MGz_t freq );


/// private
/// \todo typedef param
unsigned int countRegulatorValue( MGz_t freq );



int main()
{	
	/// settings frequency
	freqHsiCpuSet( 128 );
	
	/// For looking meandr 
	RST_CLK_PCLKcmd( RST_CLK_PCLK_PORTA, ENABLE );
	
	MDR_PORTA->OE |= 1;
	MDR_PORTA->ANALOG |= 1;
	MDR_PORTA->PULL |= 1; // pull down for first pin
	MDR_PORTA->PWR |= 3; // fastest
	

	/// blink
	while(1)
	{
		MDR_PORTA->RXTX ^= 1;
	}
	
	return 0;
}



MGz_t freqHsiCpuSet( MGz_t freq )
{
	const static MGz_t HSI_INTNL_FREQ = MIN_FREQ;
	
	if( freq < HSI_INTNL_FREQ || freq > MAX_FREQ )
		return PARAM_ERR;
	
	
	/// \todo 
	MGz_t realFreq = freq - freq % HSI_INTNL_FREQ; 
	if( 0 == realFreq ) realFreq = 1;
	
	
	/// BKP enable \todo
	RST_CLK_PCLKcmd( RST_CLK_PCLK_BKP, ENABLE );
	
	/// regulator mode and resist \todo 
	const unsigned int REGULATOR_SELECT_RI_AND_LOW_VAL = countRegulatorValue( realFreq );
	MDR_BKP->REG_0E = ( MDR_BKP->REG_0E & 0xFFFFFFC0 ) | (REGULATOR_SELECT_RI_AND_LOW_VAL << 3) | REGULATOR_SELECT_RI_AND_LOW_VAL;
	
	/// \todo
	RST_CLK_HSIcmd( ENABLE );
	
	/// \todo 
	/// Is the HSI (8MHz) generator on. ( Must be switched on when power is supplied ) \todo
	if( SUCCESS != RST_CLK_HSIstatus() ) 
	{
		return FREQ_CPU_ERR;
	}
	
	
	/// Multiplier for HSI frequency. 
	/// \todo -1  !! 128/8
	/// \todo If equal 0 then freq (user's) = 1
	unsigned int multiplier = realFreq / HSI_INTNL_FREQ; 
	if( 0 != multiplier ) multiplier -= 1; /// \todo  why -1 and 0 \todo if del
	
	const static unsigned int BIT_PLL_CPU_ON_MASK = ( 1 << 2 );
	const static unsigned int MULTUPLIER_BIT_SHIFT = 8;
	
	MDR_RST_CLK->PLL_CONTROL = ( BIT_PLL_CPU_ON_MASK | 
															 ( multiplier << MULTUPLIER_BIT_SHIFT ) );
	
	/// \todo
	if( SUCCESS != RST_CLK_CPU_PLLstatus() )
	{
		return FREQ_CPU_ERR;
	}
	
	MDR_RST_CLK->PER_CLOCK|=0x08;					//EEPROM_CTRL Clock enable
	MDR_EEPROM->CMD=1;
	
	MDR_RST_CLK->CPU_CLOCK = ( 1 << 2 ) | ( 1 << 8 );
	
	return realFreq;
}

unsigned int countRegulatorValue( MGz_t freq )
{
	/// \todo fairy values
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



