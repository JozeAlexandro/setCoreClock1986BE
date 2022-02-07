/** ************************************************************
 * \brief Private header for processor clock frequency 
 *				setting module 1986BE 
 * \file coreClock1986BE.h
 * \author Petrov A.S.
 * ************************************************************/
 
#ifndef CORE_CLOCK_1986_BE_P_H
#define CORE_CLOCK_1986_BE_P_H


#include "MDR32F9Qx_rst_clk.h" // Keil::Drivers:RST_CLK
#include "../h/coreClock1986BE.h" // module header

/// \brief /// Counting value for BKP_REG_0E->SelectRI/LOW
///
/// \param [in] freq CPU core frequency
///	
/// \return A number with the least significant three bits
/// to write it to BKP_REG_0E->SelectRI/LOW
static unsigned int countRegulatorValue( MGz_t freq );

/// \brief Apply settings applying ti regulator
/// \param [in] realFreq CPU core frequency
static inline void regulatorSettings( MGz_t realFreq )
{	
	const static unsigned int SELECTRI_BIT_SHIFT = 3;
	const unsigned int REGULATOR_SELECT_RI_AND_LOW_VAL = countRegulatorValue( realFreq );
	/// setting
	MDR_BKP->REG_0E |= (REGULATOR_SELECT_RI_AND_LOW_VAL << SELECTRI_BIT_SHIFT ) | REGULATOR_SELECT_RI_AND_LOW_VAL;
}
	
/// \brief Enabling PLL CPU and setting the multiplier
static inline void turnCpuAndSetMult( int multiplier )
{
	const static int BIT_PLL_CPU_ON_MASK = ( 1 << 2 );
	const static int MULTUPLIER_BIT_SHIFT = 8;
	
	MDR_RST_CLK->PLL_CONTROL = ( uint32_t ) ( ( BIT_PLL_CPU_ON_MASK | 
																						( multiplier << MULTUPLIER_BIT_SHIFT ) ) );
}

/// \brief PLL source selection HSI + PLLCPUo
static inline void setHCLKMux()
{
	const static int CPUC2SEL_IS_PLLCPUo_BIT = ( 1 << 2 );
	const static int HCLKSEL_IS_CPUC3_BIT = ( 1 << 8 );
	MDR_RST_CLK->CPU_CLOCK = CPUC2SEL_IS_PLLCPUo_BIT | HCLKSEL_IS_CPUC3_BIT;
}

#endif // CORE_CLOCK_1986_BE_P_H
