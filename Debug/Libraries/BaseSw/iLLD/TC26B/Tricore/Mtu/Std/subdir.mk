################################################################################
# Automatically-generated file. Do not edit!
################################################################################

C_FILES += "..\Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.c"
OBJ_FILES += "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.o"
"Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.o" : "..\Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.c" "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\.IfxMtu.o.opt"
	@echo Compiling ${<F}
	@"${PRODDIR}\bin\cctc" -f "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\.IfxMtu.o.opt"

"Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\.IfxMtu.o.opt" : .refresh
	@argfile "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\.IfxMtu.o.opt" -o "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.o" "..\Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.c" -Ctc26x --lsl-core=vtc -t -I"D:\smartcar\smartcar" -Wa-H"sfr/regtc26x.def" -Wa-gAHLs --emit-locals=-equs,-symbols -Wa-Ogs -Wa--error-limit=42 -I"D:\smartcar\smartcar\CODE" -I"D:\smartcar\smartcar\Libraries" -I"D:\smartcar\smartcar\Libraries\BaseSw" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\_Build" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\_Impl" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\_Lib" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\_Lib\DataHandling" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\_Lib\InternalMux" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\_PinMap" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin\Asc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin\Lin" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin\Spi" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\Icu" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmBc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\Timer" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\TimerWithTrigger" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\TPwm" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Cif" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Cif\Cam" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Cif\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu\CStart" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu\Irq" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu\Trap" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dma" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dma\Dma" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dma\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dsadc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dsadc\Dsadc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dsadc\Rdc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dsadc\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dts" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dts\Dts" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Dts\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Emem" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Emem\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Eray" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Eray\Eray" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Eray\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Eth" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Eth\Phy_Pef7071" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Eth\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Fce" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Fce\Crc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Fce\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Fft" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Fft\Fft" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Fft\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Flash" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Flash\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gpt12" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gpt12\IncrEnc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gpt12\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\Pwm" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\PwmHl" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\Timer" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tim" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tim\In" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\Pwm" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\PwmHl" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\Timer" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Trig" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Hssl" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\I2c" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\I2c\I2c" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\I2c\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Iom" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Iom\Driver" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Iom\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Msc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Msc\Msc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Msc\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Multican" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Multican\Can" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Multican\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Port" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Port\Io" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Port\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5\Psi5" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5s" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5s\Psi5s" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5s\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Qspi" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Qspi\SpiMaster" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Qspi\SpiSlave" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Qspi\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Scu" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Scu\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Sent" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Sent\Sent" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Sent\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Smu" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Smu\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Src" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Src\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Stm" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Stm\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Stm\Timer" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Vadc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Vadc\Adc" -I"D:\smartcar\smartcar\Libraries\BaseSw\iLLD\TC26B\Tricore\Vadc\Std" -I"D:\smartcar\smartcar\Libraries\BaseSw\Infra" -I"D:\smartcar\smartcar\Libraries\BaseSw\Infra\Platform" -I"D:\smartcar\smartcar\Libraries\BaseSw\Infra\Platform\Tricore" -I"D:\smartcar\smartcar\Libraries\BaseSw\Infra\Platform\Tricore\Compilers" -I"D:\smartcar\smartcar\Libraries\BaseSw\Infra\Sfr" -I"D:\smartcar\smartcar\Libraries\BaseSw\Infra\Sfr\TC26B" -I"D:\smartcar\smartcar\Libraries\BaseSw\Infra\Sfr\TC26B\_Reg" -I"D:\smartcar\smartcar\Libraries\BaseSw\Service" -I"D:\smartcar\smartcar\Libraries\BaseSw\Service\CpuGeneric" -I"D:\smartcar\smartcar\Libraries\seekfree_libraries" -I"D:\smartcar\smartcar\Libraries\seekfree_libraries\common" -I"D:\smartcar\smartcar\Libraries\seekfree_peripheral" -I"D:\smartcar\smartcar\USER" --iso=99 --language=-gcc,-volatile,+strings,-kanji --fp-model=3 --switch=auto --align=0 --default-near-size=0 --default-a0-size=0 --default-a1-size=0 -O0 --tradeoff=0 -g --error-limit=42 --source -c --dep-file="Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\.IfxMtu.o.d" -Wc--make-target="Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.o"
DEPENDENCY_FILES += "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\.IfxMtu.o.d"


GENERATED_FILES += "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.o" "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\.IfxMtu.o.opt" "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\.IfxMtu.o.d" "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.src" "Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std\IfxMtu.lst"
