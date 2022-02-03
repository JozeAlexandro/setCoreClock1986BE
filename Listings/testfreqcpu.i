# 1 "testFreqCpu.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 352 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "testFreqCpu.c" 2
# 1 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h" 1
# 32 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
# 1 "./RTE/Device/MDR1986BE1T\\MDR32F9Qx_config.h" 1
# 54 "./RTE/Device/MDR1986BE1T\\MDR32F9Qx_config.h"
# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdint.h" 1 3
# 56 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdint.h" 3
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int int64_t;


typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef signed short int int_least16_t;
typedef signed int int_least32_t;
typedef signed long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned short int uint_least16_t;
typedef unsigned int uint_least32_t;
typedef unsigned long long int uint_least64_t;




typedef signed int int_fast8_t;
typedef signed int int_fast16_t;
typedef signed int int_fast32_t;
typedef signed long long int int_fast64_t;


typedef unsigned int uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
typedef unsigned long long int uint_fast64_t;






typedef signed int intptr_t;
typedef unsigned int uintptr_t;



typedef signed long long intmax_t;
typedef unsigned long long uintmax_t;
# 55 "./RTE/Device/MDR1986BE1T\\MDR32F9Qx_config.h" 2
# 1 "./RTE/_Target_1\\RTE_Components.h" 1
# 56 "./RTE/Device/MDR1986BE1T\\MDR32F9Qx_config.h" 2
# 84 "./RTE/Device/MDR1986BE1T\\MDR32F9Qx_config.h"
# 1 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h" 1
# 32 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef enum IRQn
{

  NonMaskableInt_IRQn = -14,
  HardFault_IRQn = -13,
  SVCall_IRQn = -5,
  PendSV_IRQn = -2,
  SysTick_IRQn = -1,


  MIL_STD_1553B2_IRQn = 0,
  MIL_STD_1553B1_IRQn = 1,
  USB_IRQn = 2,
  CAN1_IRQn = 3,
  CAN2_IRQn = 4,
  DMA_IRQn = 5,
  UART1_IRQn = 6,
  UART2_IRQn = 7,
  SSP1_IRQn = 8,
  BUSY_IRQn = 9,
  ARINC429R_IRQn = 10,
  POWER_IRQn = 11,
  WWDG_IRQn = 12,
  TIMER4_IRQn = 13,
  TIMER1_IRQn = 14,
  TIMER2_IRQn = 15,
  TIMER3_IRQn = 16,
  ADC_IRQn = 17,
  ETHERNET_IRQn = 18,
  SSP3_IRQn = 19,
  SSP2_IRQn = 20,
  ARINC429T1_IRQn = 21,
  ARINC429T2_IRQn = 22,
  ARINC429T3_IRQn = 23,
  ARINC429T4_IRQn = 24,
  BKP_IRQn = 27,
  EXT_INT1_IRQn = 28,
  EXT_INT2_IRQn = 29,
  EXT_INT3_IRQn = 30,
  EXT_INT4_IRQn = 31
}IRQn_Type;
# 86 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
# 1 "./RTE/Device/MDR1986BE1T\\MDR32F9Qx_config.h" 1
# 87 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h" 2
# 1 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h" 1
# 130 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
# 1 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmInstr.h" 1
# 325 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __NOP(void)
{
  __asm volatile ("nop");
}







__attribute__( ( always_inline ) ) static inline void __WFI(void)
{
  __asm volatile ("wfi");
}







__attribute__( ( always_inline ) ) static inline void __WFE(void)
{
  __asm volatile ("wfe");
}






__attribute__( ( always_inline ) ) static inline void __SEV(void)
{
  __asm volatile ("sev");
}
# 369 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __ISB(void)
{
  __asm volatile ("isb");
}







__attribute__( ( always_inline ) ) static inline void __DSB(void)
{
  __asm volatile ("dsb");
}







__attribute__( ( always_inline ) ) static inline void __DMB(void)
{
  __asm volatile ("dmb");
}
# 404 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV(uint32_t value)
{



  uint32_t result;

  __asm volatile ("rev %0, %1" : "=l" (result) : "l" (value) );
  return(result);

}
# 424 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV16(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev16 %0, %1" : "=l" (result) : "l" (value) );
  return(result);
}
# 440 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline int32_t __REVSH(int32_t value)
{



  uint32_t result;

  __asm volatile ("revsh %0, %1" : "=l" (result) : "l" (value) );
  return(result);

}
# 461 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{
  return (op1 >> op2) | (op1 << (32 - op2));
}
# 131 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h" 2
# 1 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h" 1
# 329 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}







__attribute__( ( always_inline ) ) static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}
# 352 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 367 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
}
# 379 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}
# 394 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}
# 409 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}
# 424 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, psp\n" : "=r" (result) );
  return(result);
}
# 439 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0\n" : : "r" (topOfProcStack) : "sp");
}
# 451 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_MSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, msp\n" : "=r" (result) );
  return(result);
}
# 466 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0\n" : : "r" (topOfMainStack) : "sp");
}
# 478 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}
# 493 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 132 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h" 2
# 198 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
typedef union
{
  struct
  {

    uint32_t _reserved0:27;





    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;

    uint32_t _reserved0:15;





    uint32_t T:1;
    uint32_t IT:2;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;




typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t FPCA:1;
    uint32_t _reserved0:29;
  } b;
  uint32_t w;
} CONTROL_Type;
# 283 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
typedef struct
{
  volatile uint32_t ISER[1];
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];
} NVIC_Type;
# 308 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];
  volatile uint32_t SHCSR;
} SCB_Type;
# 413 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 518 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 530 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
static inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 546 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
static inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}
# 558 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
static inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 570 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
static inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 585 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}
# 607 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
static inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2))); }
  else {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2))); }
}






static inline void NVIC_SystemReset(void)
{
  __DSB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = ((0x5FA << 16) |
                 (1UL << 2));
  __DSB();
  while(1);
}
# 659 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/CoreSupport/CM1\\core_cm1.h"
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0)) return (1);

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = ticks - 1;
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) |
                   (1UL << 1) |
                   (1UL << 0);
  return (0);
}
# 88 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h" 2
# 1 "./RTE/Device/MDR1986BE1T\\system_MDR1986VE1T.h" 1
# 31 "./RTE/Device/MDR1986BE1T\\system_MDR1986VE1T.h"
extern uint32_t SystemCoreClock;







extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
# 89 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h" 2




typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus;



typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;
# 117 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
  volatile uint32_t ID;
  volatile uint32_t DLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
}MDR_CAN_BUF_TypeDef;


typedef struct
{
  volatile uint32_t MASK;
  volatile uint32_t FILTER;
}MDR_CAN_BUF_FILTER_TypeDef;


typedef struct
{
  volatile uint32_t CONTROL;
  volatile uint32_t STATUS;
  volatile uint32_t BITTMNG;
       uint32_t RESERVED0;
  volatile uint32_t INT_EN;
       uint32_t RESERVED1[2];
  volatile uint32_t OVER;
  volatile uint32_t RXID;
  volatile uint32_t RXDLC;
  volatile uint32_t RXDATAL;
  volatile uint32_t RXDATAH;
  volatile uint32_t TXID;
  volatile uint32_t TXDLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
  volatile uint32_t BUF_CON[32];
  volatile uint32_t INT_RX;
  volatile uint32_t RX;
  volatile uint32_t INT_TX;
  volatile uint32_t TX;
       uint32_t RESERVED2[76];
    MDR_CAN_BUF_TypeDef CAN_BUF[32];
       uint32_t RESERVED3[64];
    MDR_CAN_BUF_FILTER_TypeDef CAN_BUF_FILTER[32];
}MDR_CAN_TypeDef;
# 395 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t STS;
  volatile uint32_t TS;
  volatile uint32_t NTS;
}MDR_USB_SEP_TypeDef;


typedef struct
{
  volatile uint32_t RXFD;
       uint32_t RESERVED0;
  volatile uint32_t RXFDC_L;
  volatile uint32_t RXFDC_H;
  volatile uint32_t RXFC;
       uint32_t RESERVED1[11];
  volatile uint32_t TXFD;
       uint32_t RESERVED2[3];
  volatile uint32_t TXFDC;
       uint32_t RESERVED3[11];
}MDR_USB_SEP_FIFO_TypeDef;


typedef struct
{
  volatile uint32_t HTXC;
  volatile uint32_t HTXT;
  volatile uint32_t HTXLC;
  volatile uint32_t HTXSE;
  volatile uint32_t HTXA;
  volatile uint32_t HTXE;
  volatile uint32_t HFN_L;
  volatile uint32_t HFN_H;
  volatile uint32_t HIS;
  volatile uint32_t HIM;
  volatile uint32_t HRXS;
  volatile uint32_t HRXP;
  volatile uint32_t HRXA;
  volatile uint32_t HRXE;
  volatile uint32_t HRXCS;
  volatile uint32_t HSTM;
       uint32_t RESERVED0[16];
  volatile uint32_t HRXFD;
       uint32_t RESERVED1;
  volatile uint32_t HRXFDC_L;
  volatile uint32_t HRXFDC_H;
  volatile uint32_t HRXFC;
       uint32_t RESERVED2[11];
  volatile uint32_t HTXFD;
       uint32_t RESERVED3[3];
  volatile uint32_t HTXFC;
       uint32_t RESERVED4[11];
    MDR_USB_SEP_TypeDef USB_SEP[4];
  volatile uint32_t SC;
  volatile uint32_t SLS;
  volatile uint32_t SIS;
  volatile uint32_t SIM;
  volatile uint32_t SA;
  volatile uint32_t SFN_L;
  volatile uint32_t SFN_H;
       uint32_t RESERVED5[9];
    MDR_USB_SEP_FIFO_TypeDef USB_SEP_FIFO[4];
  volatile uint32_t HSCR;
  volatile uint32_t HSVR;
}MDR_USB_TypeDef;
# 738 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
  volatile uint32_t CMD;
  volatile uint32_t ADR;
  volatile uint32_t DI;
  volatile uint32_t DO;
  volatile uint32_t KEY;
}MDR_EEPROM_TypeDef;
# 797 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct {
  volatile uint32_t CLOCK_STATUS;
  volatile uint32_t PLL_CONTROL;
  volatile uint32_t HS_CONTROL;
  volatile uint32_t CPU_CLOCK;
  volatile uint32_t USB_CLOCK;
  volatile uint32_t ADC_MCO_CLOCK;
  volatile uint32_t RTC_CLOCK;
  volatile uint32_t PER_CLOCK;
  volatile uint32_t CAN_CLOCK;
  volatile uint32_t TIM_CLOCK;
  volatile uint32_t UART_CLOCK;
  volatile uint32_t SSP_CLOCK;
       uint32_t RESERVED;
  volatile uint32_t ETH_CLOCK;
} MDR_RST_CLK_TypeDef;
# 1109 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
  volatile uint32_t STATUS;
  volatile uint32_t CFG;
  volatile uint32_t CTRL_BASE_PTR;
  volatile uint32_t ALT_CTRL_BASE_PTR;
  volatile uint32_t WAITONREQ_STATUS;
  volatile uint32_t CHNL_SW_REQUEST;
  volatile uint32_t CHNL_USEBURST_SET;
  volatile uint32_t CHNL_USEBURST_CLR;
  volatile uint32_t CHNL_REQ_MASK_SET;
  volatile uint32_t CHNL_REQ_MASK_CLR;
  volatile uint32_t CHNL_ENABLE_SET;
  volatile uint32_t CHNL_ENABLE_CLR;
  volatile uint32_t CHNL_PRI_ALT_SET;
  volatile uint32_t CHNL_PRI_ALT_CLR;
  volatile uint32_t CHNL_PRIORITY_SET;
  volatile uint32_t CHNL_PRIORITY_CLR;
       uint32_t RESERVED0[3];
  volatile uint32_t ERR_CLR;
}MDR_DMA_TypeDef;
# 1188 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct {
  volatile uint32_t DR;
  volatile uint32_t RSR_ECR;
       uint32_t RESERVED0[4];
  volatile uint32_t FR;
       uint32_t RESERVED1;
  volatile uint32_t ILPR;
  volatile uint32_t IBRD;
  volatile uint32_t FBRD;
  volatile uint32_t LCR_H;
  volatile uint32_t CR;
  volatile uint32_t IFLS;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
  volatile uint32_t UARTTCR;
} MDR_UART_TypeDef;
# 1545 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
  volatile uint32_t CR0;
  volatile uint32_t CR1;
  volatile uint32_t DR;
  volatile uint32_t SR;
  volatile uint32_t CPSR;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
}MDR_SSP_TypeDef;
# 1794 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct {
 volatile uint32_t Data[32];
} MDR_MIL_STD_1553_DataBuffer;


typedef struct {
 MDR_MIL_STD_1553_DataBuffer SubAddr[32];
 volatile uint32_t CONTROL;
 volatile uint32_t STATUS;
 volatile uint32_t ERROR;
 volatile uint32_t CommandWord1;
 volatile uint32_t CommandWord2;
 volatile uint32_t ModeData;
 volatile uint32_t StatusWord1;
 volatile uint32_t StatusWord2;
 volatile uint32_t INTEN;
 volatile uint32_t MSG;
} MDR_MIL_STD_1553_TypeDef;
# 1939 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
  volatile uint32_t PVDCS;
}MDR_POWER_TypeDef;
# 1994 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
}MDR_WWDG_TypeDef;
# 2052 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
}MDR_IWDG_TypeDef;
# 2098 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct {
  volatile uint32_t CNT;
  volatile uint32_t PSG;
  volatile uint32_t ARR;
  volatile uint32_t CNTRL;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  volatile uint32_t CH1_CNTRL;
  volatile uint32_t CH2_CNTRL;
  volatile uint32_t CH3_CNTRL;
  volatile uint32_t CH4_CNTRL;
  volatile uint32_t CH1_CNTRL1;
  volatile uint32_t CH2_CNTRL1;
  volatile uint32_t CH3_CNTRL1;
  volatile uint32_t CH4_CNTRL1;
  volatile uint32_t CH1_DTG;
  volatile uint32_t CH2_DTG;
  volatile uint32_t CH3_DTG;
  volatile uint32_t CH4_DTG;
  volatile uint32_t BRKETR_CNTRL;
  volatile uint32_t STATUS;
  volatile uint32_t IE;
  volatile uint32_t DMA_RE;
  volatile uint32_t CH1_CNTRL2;
  volatile uint32_t CH2_CNTRL2;
  volatile uint32_t CH3_CNTRL2;
  volatile uint32_t CH4_CNTRL2;
  volatile uint32_t CCR11;
  volatile uint32_t CCR21;
  volatile uint32_t CCR31;
  volatile uint32_t CCR41;
  volatile uint32_t DMA_REChx[4];
} MDR_TIMER_TypeDef;
# 2370 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct {
  volatile uint32_t ADC1_CFG;
  volatile uint32_t ADC2_CFG;
  volatile uint32_t ADC1_H_LEVEL;
  volatile uint32_t RESERVED0;
  volatile uint32_t ADC1_L_LEVEL;
  volatile uint32_t RESERVED1;
  volatile uint32_t ADC1_RESULT;
  volatile uint32_t RESERVED2;
  volatile uint32_t ADC1_STATUS;
  volatile uint32_t RESERVED3;
  volatile uint32_t ADC1_CHSEL;
  volatile uint32_t RESERVED4;
  volatile uint32_t ADC1_TRIM;
} MDR_ADC_TypeDef;
# 2520 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct {
  volatile uint32_t CFG;
  volatile uint32_t DAC1_DATA;
  volatile uint32_t DAC2_DATA;
} MDR_DAC_TypeDef;
# 2618 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct {
  volatile uint32_t RXTX;
  volatile uint32_t OE;
  volatile uint32_t FUNC;
  volatile uint32_t ANALOG;
  volatile uint32_t PULL;
  volatile uint32_t PD;
  volatile uint32_t PWR;
  volatile uint32_t GFEN;
  volatile uint32_t SETTX;
  volatile uint32_t CLRTX;
  volatile uint32_t RDTX;
} MDR_PORT_TypeDef;
# 2770 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
 volatile uint32_t CONTROL1;
 volatile uint32_t CONTROL2;
 volatile uint32_t CONTROL3;
 volatile uint32_t STATUS1;
 volatile uint32_t STATUS2;
 volatile uint32_t CONTROL4;
 volatile uint32_t CONTROL5;
 volatile uint32_t CHANNEL;
 volatile uint32_t LABEL;
 volatile uint32_t DATA_R;
   uint32_t RESERVED1[2];
 volatile uint32_t DATA_R_Direct[8];
   uint32_t RESERVED2[6];
 volatile uint32_t INTMASK;
 volatile uint32_t RESERVED3;
 volatile uint32_t CONTROL8;
 volatile uint32_t CONTROL9;
}MDR_ARINC429R_TypeDef;
# 3151 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
 volatile uint32_t CONTROL1;
 volatile uint32_t CONTROL2;
 volatile uint32_t STATUS;
 volatile uint32_t DATA1_T;
 volatile uint32_t DATA2_T;
 volatile uint32_t DATA3_T;
 volatile uint32_t DATA4_T;
 volatile uint32_t CONTROL3;
 volatile uint32_t CONTROL4;
}MDR_ARINC429T_TypeDef;
# 3317 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
  volatile uint32_t REG_00;
  volatile uint32_t REG_01;
  volatile uint32_t REG_02;
  volatile uint32_t REG_03;
  volatile uint32_t REG_04;
  volatile uint32_t REG_05;
  volatile uint32_t REG_06;
  volatile uint32_t REG_07;
  volatile uint32_t REG_08;
  volatile uint32_t REG_09;
  volatile uint32_t REG_0A;
  volatile uint32_t REG_0B;
  volatile uint32_t REG_0C;
  volatile uint32_t REG_0D;
  volatile uint32_t REG_0E;
  volatile uint32_t REG_0F;
  volatile uint32_t RTC_CNT;
  volatile uint32_t RTC_DIV;
  volatile uint32_t RTC_PRL;
  volatile uint32_t RTC_ALRM;
  volatile uint32_t RTC_CS;
}MDR_BKP_TypeDef;
# 3453 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct
{
       uint32_t RESERVED0[20];
  volatile uint32_t NAND_CYCLES;
  volatile uint32_t CONTROL;
  volatile uint32_t MEM_REGION[4];
}MDR_EBC_TypeDef;
# 3554 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/CMSIS/MDR32Fx/DeviceSupport/MDR1986VE1T/inc\\MDR1986VE1T.h"
typedef struct {
  volatile uint16_t ETH_Dilimiter;
  volatile uint16_t ETH_MAC_T;
  volatile uint16_t ETH_MAC_M;
  volatile uint16_t ETH_MAC_H;
  volatile uint16_t ETH_HASH0;
  volatile uint16_t ETH_HASH1;
  volatile uint16_t ETH_HASH2;
  volatile uint16_t ETH_HASH3;
  volatile uint16_t ETH_IPG;
  volatile uint16_t ETH_PSC;
  volatile uint16_t ETH_BAG;
  volatile uint16_t ETH_JitterWnd;
  volatile uint16_t ETH_R_CFG;
  volatile uint16_t ETH_X_CFG;
  volatile uint16_t ETH_G_CFGl;
  volatile uint16_t ETH_G_CFGh;
  volatile uint16_t ETH_IMR;
  volatile uint16_t ETH_IFR;
  volatile uint16_t ETH_MDIO_CTRL;
  volatile uint16_t ETH_MDIO_DATA;
  volatile uint16_t ETH_R_Head;
  volatile uint16_t ETH_X_Tail;
  volatile uint16_t ETH_R_Tail;
  volatile uint16_t ETH_X_Head;
  volatile uint16_t ETH_STAT;
  volatile uint16_t Reserved;
  volatile uint16_t PHY_Control;
  volatile uint16_t PHY_Status;
} MDR_ETHERNET_TypeDef;
# 85 "./RTE/Device/MDR1986BE1T\\MDR32F9Qx_config.h" 2
# 33 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h" 2
# 49 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef struct
{
    uint32_t CPU_CLK_Frequency;
    uint32_t USB_CLK_Frequency;
    uint32_t ADC_CLK_Frequency;
    uint32_t RTCHSI_Frequency;
    uint32_t RTCHSE_Frequency;
} RST_CLK_FreqTypeDef;




typedef struct
{
    uint32_t REG_0F;
} Init_NonVolatile_RST_CLK_TypeDef;




typedef enum
{
    RST_CLK_HSE_OFF = ((uint32_t)0x00),
    RST_CLK_HSE_ON = ((uint32_t)0x01),
    RST_CLK_HSE_Bypass = ((uint32_t)0x02)
} RST_CLK_HSE_Mode;
# 84 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_HSE2_OFF = ((uint32_t)0x00),
    RST_CLK_HSE2_ON = ((uint32_t)0x04),
    RST_CLK_HSE2_Bypass = ((uint32_t)0x08)
} RST_CLK_HSE2_Mode;
# 99 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_LSE_OFF = ((uint32_t)0x00),
    RST_CLK_LSE_ON = ((uint32_t)0x01),
    RST_CLK_LSE_Bypass = ((uint32_t)0x02)
} RST_CLK_LSE_Mode;
# 113 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_CPU_PLLsrcHSIdiv1 = ((uint32_t)0x00),
    RST_CLK_CPU_PLLsrcHSIdiv2 = ((uint32_t)0x01),
    RST_CLK_CPU_PLLsrcHSEdiv1 = ((uint32_t)0x02),
    RST_CLK_CPU_PLLsrcHSEdiv2 = ((uint32_t)0x03)
} RST_CLK_CPU_PLL_Source;
# 129 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_CPU_C1srcHSIdiv1 = ((uint32_t)0x00),
    RST_CLK_CPU_C1srcHSIdiv2 = ((uint32_t)0x01),
    RST_CLK_CPU_C1srcHSEdiv1 = ((uint32_t)0x02),
    RST_CLK_CPU_C1srcHSEdiv2 = ((uint32_t)0x03)
} RST_CLK_CPU_C1_Source;
# 145 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_CPU_PLLmul1 = ((uint32_t)0x00),
    RST_CLK_CPU_PLLmul2 = ((uint32_t)0x01),
    RST_CLK_CPU_PLLmul3 = ((uint32_t)0x02),
    RST_CLK_CPU_PLLmul4 = ((uint32_t)0x03),
    RST_CLK_CPU_PLLmul5 = ((uint32_t)0x04),
    RST_CLK_CPU_PLLmul6 = ((uint32_t)0x05),
    RST_CLK_CPU_PLLmul7 = ((uint32_t)0x06),
    RST_CLK_CPU_PLLmul8 = ((uint32_t)0x07),
    RST_CLK_CPU_PLLmul9 = ((uint32_t)0x08),
    RST_CLK_CPU_PLLmul10 = ((uint32_t)0x09),
    RST_CLK_CPU_PLLmul11 = ((uint32_t)0x0A),
    RST_CLK_CPU_PLLmul12 = ((uint32_t)0x0B),
    RST_CLK_CPU_PLLmul13 = ((uint32_t)0x0C),
    RST_CLK_CPU_PLLmul14 = ((uint32_t)0x0D),
    RST_CLK_CPU_PLLmul15 = ((uint32_t)0x0E),
    RST_CLK_CPU_PLLmul16 = ((uint32_t)0x0F)
} RST_CLK_CPU_PLL_Multiplier;






typedef enum
{
    RST_CLK_USB_PLLsrcHSIdiv1 = ((uint32_t)0x00),
    RST_CLK_USB_PLLsrcHSIdiv2 = ((uint32_t)0x01),
    RST_CLK_USB_PLLsrcHSEdiv1 = ((uint32_t)0x02),
    RST_CLK_USB_PLLsrcHSEdiv2 = ((uint32_t)0x03)
} RST_CLK_USB_PLL_Source;
# 186 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_USB_PLLmul1 = ((uint32_t)0x00),
    RST_CLK_USB_PLLmul2 = ((uint32_t)0x01),
    RST_CLK_USB_PLLmul3 = ((uint32_t)0x02),
    RST_CLK_USB_PLLmul4 = ((uint32_t)0x03),
    RST_CLK_USB_PLLmul5 = ((uint32_t)0x04),
    RST_CLK_USB_PLLmul6 = ((uint32_t)0x05),
    RST_CLK_USB_PLLmul7 = ((uint32_t)0x06),
    RST_CLK_USB_PLLmul8 = ((uint32_t)0x07),
    RST_CLK_USB_PLLmul9 = ((uint32_t)0x08),
    RST_CLK_USB_PLLmul10 = ((uint32_t)0x09),
    RST_CLK_USB_PLLmul11 = ((uint32_t)0x0A),
    RST_CLK_USB_PLLmul12 = ((uint32_t)0x0B),
    RST_CLK_USB_PLLmul13 = ((uint32_t)0x0C),
    RST_CLK_USB_PLLmul14 = ((uint32_t)0x0D),
    RST_CLK_USB_PLLmul15 = ((uint32_t)0x0E),
    RST_CLK_USB_PLLmul16 = ((uint32_t)0x0F)
} RST_CLK_USB_PLL_Multiplier;






typedef enum
{
    RST_CLK_CPUclkDIV1 = ((uint32_t)0x00),
    RST_CLK_CPUclkDIV2 = ((uint32_t)0x08),
    RST_CLK_CPUclkDIV4 = ((uint32_t)0x09),
    RST_CLK_CPUclkDIV8 = ((uint32_t)0x0A),
    RST_CLK_CPUclkDIV16 = ((uint32_t)0x0B),
    RST_CLK_CPUclkDIV32 = ((uint32_t)0x0C),
    RST_CLK_CPUclkDIV64 = ((uint32_t)0x0D),
    RST_CLK_CPUclkDIV128 = ((uint32_t)0x0E),
    RST_CLK_CPUclkDIV256 = ((uint32_t)0x0F)
} RST_CLK_CPU_C3_Divisor;
# 237 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_CPUclkHSI = ((uint32_t)0x0000),
    RST_CLK_CPUclkCPU_C3 = ((uint32_t)0x0100),
    RST_CLK_CPUclkLSE = ((uint32_t)0x0200),
    RST_CLK_CPUclkLSI = ((uint32_t)0x0300)
} RST_CLK_HCLK_Source;
# 253 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_ADCclkCPU_C1 = ((uint32_t)0x0020),
    RST_CLK_ADCclkUSB_C1 = ((uint32_t)0x0021),
    RST_CLK_ADCclkCPU_C2 = ((uint32_t)0x0022),
    RST_CLK_ADCclkUSB_C2 = ((uint32_t)0x0023),
    RST_CLK_ADCclkLSE = ((uint32_t)0x0000),
    RST_CLK_ADCclkLSI = ((uint32_t)0x0010),
    RST_CLK_ADCclkHSI_C1 = ((uint32_t)0x0030)
} RST_CLK_ADC_Source;
# 275 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_ADCclkDIV1 = ((uint32_t)0x00),
    RST_CLK_ADCclkDIV2 = ((uint32_t)0x08),
    RST_CLK_ADCclkDIV4 = ((uint32_t)0x09),
    RST_CLK_ADCclkDIV8 = ((uint32_t)0x0A),
    RST_CLK_ADCclkDIV16 = ((uint32_t)0x0B),
    RST_CLK_ADCclkDIV32 = ((uint32_t)0x0C),
    RST_CLK_ADCclkDIV64 = ((uint32_t)0x0D),
    RST_CLK_ADCclkDIV128 = ((uint32_t)0x0E),
    RST_CLK_ADCclkDIV256 = ((uint32_t)0x0F)
} RST_CLK_ADC_C3_Divisor;
# 349 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_FLAG_HSIRDY = ((uint32_t)(0x00 | 23)),
    RST_CLK_FLAG_LSIRDY = ((uint32_t)(0x00 | 21)),
    RST_CLK_FLAG_HSERDY = ((uint32_t)(0x20 | 2)),
    RST_CLK_FLAG_HSE2RDY = ((uint32_t)(0x20 | 3)),
    RST_CLK_FLAG_LSERDY = ((uint32_t)(0x00 | 13)),
    RST_CLK_FLAG_PLLCPURDY = ((uint32_t)(0x20 | 1)),
    RST_CLK_FLAG_PLLUSBRDY = ((uint32_t)(0x20 | 0)),
    RST_CLK_FLAG_PLLDSPRDY = ((uint32_t)(0x20 | 3))
} RST_CLK_Flags;
# 373 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_HSIclkDIV1 = ((uint32_t)0x00),
    RST_CLK_HSIclkDIV2 = ((uint32_t)0x08),
    RST_CLK_HSIclkDIV4 = ((uint32_t)0x09),
    RST_CLK_HSIclkDIV8 = ((uint32_t)0x0A),
    RST_CLK_HSIclkDIV16 = ((uint32_t)0x0B),
    RST_CLK_HSIclkDIV32 = ((uint32_t)0x0C),
    RST_CLK_HSIclkDIV64 = ((uint32_t)0x0D),
    RST_CLK_HSIclkDIV128 = ((uint32_t)0x0E),
    RST_CLK_HSIclkDIV256 = ((uint32_t)0x0F)
} RST_CLK_HSI_C1_Divisor;
# 399 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_HSEclkDIV1 = ((uint32_t)0x00),
    RST_CLK_HSEclkDIV2 = ((uint32_t)0x08),
    RST_CLK_HSEclkDIV4 = ((uint32_t)0x09),
    RST_CLK_HSEclkDIV8 = ((uint32_t)0x0A),
    RST_CLK_HSEclkDIV16 = ((uint32_t)0x0B),
    RST_CLK_HSEclkDIV32 = ((uint32_t)0x0C),
    RST_CLK_HSEclkDIV64 = ((uint32_t)0x0D),
    RST_CLK_HSEclkDIV128 = ((uint32_t)0x0E),
    RST_CLK_HSEclkDIV256 = ((uint32_t)0x0F)
} RST_CLK_HSE_C1_Divisor;
# 683 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
void RST_CLK_DeInit(void);
void RST_CLK_WarmDeInit(void);

void RST_CLK_HSEconfig(RST_CLK_HSE_Mode RST_CLK_HSE);
ErrorStatus RST_CLK_HSEstatus(void);

    void RST_CLK_HSE2config(RST_CLK_HSE2_Mode RST_CLK_HSE2);
    ErrorStatus RST_CLK_HSE2status(void);


void RST_CLK_LSEconfig(RST_CLK_LSE_Mode RST_CLK_LSE);
ErrorStatus RST_CLK_LSEstatus(void);

void RST_CLK_HSIcmd(FunctionalState NewState);
void RST_CLK_HSIadjust(uint32_t HSItrimValue);
ErrorStatus RST_CLK_HSIstatus(void);

void RST_CLK_LSIcmd(FunctionalState NewState);
void RST_CLK_LSIadjust(uint32_t LSItrimValue);
ErrorStatus RST_CLK_LSIstatus(void);

void RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLL_Source RST_CLK_CPU_PLLsource, uint32_t RST_CLK_CPU_PLLmul);
void RST_CLK_CPU_PLLuse(FunctionalState UsePLL);
void RST_CLK_CPU_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_CPU_PLLstatus(void);

void RST_CLK_CPUclkPrescaler(RST_CLK_CPU_C3_Divisor CPUclkDivValue);
void RST_CLK_CPUclkSelection(RST_CLK_HCLK_Source CPU_CLK);

void RST_CLK_USB_PLLconfig(RST_CLK_USB_PLL_Source RST_CLK_USB_PLLsource, uint32_t RST_CLK_USB_PLLmul);
void RST_CLK_USB_PLLuse(FunctionalState UsePLL);
void RST_CLK_USB_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_USB_PLLstatus(void);

void RST_CLK_USBclkPrescaler(FunctionalState NewState);
void RST_CLK_USBclkEnable(FunctionalState NewState);

void RST_CLK_ADCclkSelection(RST_CLK_ADC_Source ADC_CLK);
void RST_CLK_ADCclkPrescaler(RST_CLK_ADC_C3_Divisor ADCclkDivValue);
void RST_CLK_ADCclkEnable(FunctionalState NewState);

void RST_CLK_HSIclkPrescaler(RST_CLK_HSI_C1_Divisor HSIclkDivValue);
void RST_CLK_RTC_HSIclkEnable(FunctionalState NewState);

void RST_CLK_HSEclkPrescaler(RST_CLK_HSE_C1_Divisor HSEclkDivValue);
void RST_CLK_RTC_HSEclkEnable(FunctionalState NewState);

void RST_CLK_CPUclkSelectionC1(RST_CLK_CPU_C1_Source CPU_CLK);

void RST_CLK_PCLKcmd(uint32_t RST_CLK_PCLK, FunctionalState NewState);
# 744 "C:/Users/ASePetrov/AppData/Local/Arm/Packs/Keil/MDR1986BExx/2.0.3/Libraries/SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
void RST_CLK_GetClocksFreq(RST_CLK_FreqTypeDef* RST_CLK_Clocks);

FlagStatus RST_CLK_GetFlagStatus(RST_CLK_Flags RST_CLK_FLAG);
# 2 "testFreqCpu.c" 2


int freqCpuSet( int freq );

int main()
{
 SystemInit();


 RST_CLK_PCLKcmd( ((uint32_t)(1U << ((((uint32_t)(0x400A8000)) >> 15) & 0x1F))), ENABLE );


 ((MDR_PORT_TypeDef *)(0x400A8000) )->OE |= 1;
 ((MDR_PORT_TypeDef *)(0x400A8000) )->ANALOG |= 1;
 ((MDR_PORT_TypeDef *)(0x400A8000) )->PULL |= 1;
 ((MDR_PORT_TypeDef *)(0x400A8000) )->PWR |= 3;


 RST_CLK_PCLKcmd( ((uint32_t)(1U << ((((uint32_t)(0x400C0000)) >> 15) & 0x1F))), ENABLE );
 ((MDR_PORT_TypeDef *)(0x400C0000) )->FUNC |= 0x00000000;
 ((MDR_PORT_TypeDef *)(0x400C0000) )->OE |= (1 << 0) | (1 << 7) | (1 << 8) | (1 << 11);
 ((MDR_PORT_TypeDef *)(0x400C0000) )->ANALOG |= (1 << 0) | (1 << 7) | (1 << 8) | (1 << 11);
 ((MDR_PORT_TypeDef *)(0x400C0000) )->PWR |= (0x3 << (0 * 2)) | (0x03 << (7 * 2)) | (0x03 << (8 * 2)) | (0x03 << (11 * 2));





 ((MDR_PORT_TypeDef *)(0x400C0000) )->RXTX = 0;


 freqCpuSet(100);



 while(1)
 {
  ((MDR_PORT_TypeDef *)(0x400A8000) )->RXTX ^= 1;
 }

 return 0;
}


int freqCpuSet( int freq )
{
 const int realFreq = freq;


 RST_CLK_PCLKcmd( ((uint32_t)(1U << ((((uint32_t)(0x400D8000)) >> 15) & 0x1F))), ENABLE );


 ((MDR_BKP_TypeDef *)(0x400D8000) )->REG_0E = ( ((MDR_BKP_TypeDef *)(0x400D8000) )->REG_0E & 0xFFFFFFC0 ) | (7 << 3) | 7;

 RST_CLK_HSIcmd( ENABLE );


 if( SUCCESS != RST_CLK_HSIstatus() )
 {
  ((MDR_PORT_TypeDef *)(0x400C0000) )->RXTX |= (1 << 0);
  return 0;
 }





 ((MDR_RST_CLK_TypeDef *)(0x40020000) )->PLL_CONTROL = ( 1 << 2 ) | ( 15 << 8 );

 if( SUCCESS != RST_CLK_CPU_PLLstatus() )
 {
  ((MDR_PORT_TypeDef *)(0x400C0000) )->RXTX |= (1 << 7);
  return 0;
 }



 ((MDR_RST_CLK_TypeDef *)(0x40020000) )->CPU_CLOCK = ( 1 << 2 ) | ( 1 << 8 );
# 94 "testFreqCpu.c"
 return realFreq;
}
