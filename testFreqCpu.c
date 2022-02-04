#include "MDR32F9Qx_rst_clk.h" // Keil::Drivers:RST_CLK
#include "coreClock1986BE/h/coreClock1986BE.h" // core clock module

int main()
{	
	/// settings frequency
	freqHsiCpuSet( 60 );
	
	/// to view the meander
	RST_CLK_PCLKcmd( RST_CLK_PCLK_PORTA, ENABLE );
	
	/// all for port's A first pin 
	MDR_PORTA->OE |= 1;
	MDR_PORTA->ANALOG |= 1;
	MDR_PORTA->PULL |= 1; // pull down 
	MDR_PORTA->PWR |= 3; // fastest
	
	/// "blink"
	while(1)
	{
		MDR_PORTA->RXTX ^= 1;
	}
	
	return 0;
}







