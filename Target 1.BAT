SET PATH=C:\Keil_v5\ARM\ARMCLANG\Bin;C:\Program Files\Common Files\Oracle\Java\javapath;c:\qt\qt5.12.9\tools\mingw730_32\bin;c:\qt\qt5.12.9\tools\mingw730_32\libexec\gcc\x86_64-w64-mingw32\11.1.0;C:\Programms\SQLITE\sqlite-tools-win32-x86-3340100;C;C:\Program Files (x86)\dotnet\;C:\Programms\Qt\5.11.1\mingw53_32\binC:\Program Files\Common Files\Oracle\Java\javapath;c:\qt\qt5.12.9\tools\mingw730_32\bin;c:\qt\qt5.12.9\tools\mingw730_32\libexec\gcc\x86_64-w64-mingw32\11.1.0;C:\Programms\SQLITE\sqlite-tools-win32-x86-3340100;C;C:\Program Files (x86)\dotnet\;C:\Windows\system32;C:\Windows;C:\Windows\system32\WBEM;C:\Qt\Qt5.12.9\Tools\mingw730_32\bin;C:\Windows\System32;
SET CPU_TYPE=MDR1986BE1T
SET CPU_VENDOR=Milandr
SET UV2_TARGET=Target 1
SET CPU_CLOCK=0x00B71B00
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\testfreqcpu.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\startup_mdr1986ve1t._ac"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\system_mdr1986ve1t.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\mdr32f9qx_rst_clk.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmLink" --Via ".\Objects\testFreqCpu.lnp"
"C:\Keil_v5\ARM\ARMCLANG\Bin\fromelf.exe" ".\Objects\testFreqCpu.axf" --i32combined --output ".\Objects\testFreqCpu.hex"
