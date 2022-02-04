/** ************************************************************
 * \brief Processor clock frequency setting module 1986BE 
 * \file coreClock1986BE.h
 * \author Petrov A.S.
 * ************************************************************/

#ifndef CORE_CLOCK_1986_BE_H
#define CORE_CLOCK_1986_BE_H

/** ************************************************************
 * Constants and aliases
 * ************************************************************/

/// \brief Frequency in megahertz 
typedef int MGz_t;

/// \todo naming
typedef enum 
{ 
		FREQ_CPU_ERR = 0,
		PARAM_ERR = -1
} eReturnCodeValue;

enum
{
	MIN_FREQ = 8, ///< minimum available frequency
	MAX_FREQ = 128 ///< maximum available frequency
};
	
/** ************************************************************
 * Functions 
 * ************************************************************/

/// \brief The function of setting the frequency of the 
/// processor core 
///
/// \details Source - HSI internal generator. 
///
/// \warning freq should be MIN_FREQ <= freq <= MAX_FREQ
///
/// \warning The actual set frequency will be a multiple of 8
///
/// \param [in] freq Desired frequency
///
/// \return FREQ_CPU_ERR - HSI or CPU PLL can't be turned on
/// PARAM_ERR - User freq not in MIN_FREQ <= freq <= MAX_FREQ
/// <other value> - The actual set frequency
MGz_t freqHsiCpuSet( MGz_t freq );


#endif // CORE_CLOCK_1986_BE_H