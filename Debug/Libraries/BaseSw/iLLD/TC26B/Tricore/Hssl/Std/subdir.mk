################################################################################
# Automatically-generated file. Do not edit!
################################################################################

C_FILES += "..\Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.c"
OBJ_FILES += "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.o"
"Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.o" : "..\Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.c" "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\.IfxHssl.o.opt"
	@echo Compiling ${<F}
	@"${PRODDIR}\bin\cctc" -f "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\.IfxHssl.o.opt"

"Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\.IfxHssl.o.opt" : .refresh
	@argfile "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\.IfxHssl.o.opt" -o "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.o" "..\Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.c" -Ctc26x --lsl-core=vtc -t -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0" -Wa-H"sfr/regtc26x.def" -Wa-gAHLs --emit-locals=-equs,-symbols -Wa-Ogs -Wa--error-limit=42 -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\CODE" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\_Build" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\_Impl" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\_Lib" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\_Lib\DataHandling" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\_Lib\InternalMux" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\_PinMap" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin\Asc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin\Lin" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin\Spi" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Asclin\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\Icu" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmBc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\Timer" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\TimerWithTrigger" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Ccu6\TPwm" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Cif" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Cif\Cam" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Cif\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu\CStart" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu\Irq" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Cpu\Trap" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dma" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dma\Dma" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dma\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dsadc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dsadc\Dsadc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dsadc\Rdc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dsadc\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dts" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dts\Dts" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Dts\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Emem" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Emem\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Eray" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Eray\Eray" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Eray\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Eth" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Eth\Phy_Pef7071" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Eth\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Fce" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Fce\Crc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Fce\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Fft" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Fft\Fft" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Fft\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Flash" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Flash\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gpt12" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gpt12\IncrEnc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gpt12\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\Pwm" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\PwmHl" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\Timer" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tim" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tim\In" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\Pwm" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\PwmHl" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\Timer" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Gtm\Trig" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Hssl" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\I2c" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\I2c\I2c" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\I2c\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Iom" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Iom\Driver" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Iom\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Msc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Msc\Msc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Msc\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Mtu\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Multican" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Multican\Can" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Multican\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Port" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Port\Io" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Port\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5\Psi5" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5s" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5s\Psi5s" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Psi5s\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Qspi" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Qspi\SpiMaster" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Qspi\SpiSlave" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Qspi\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Scu" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Scu\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Sent" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Sent\Sent" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Sent\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Smu" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Smu\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Src" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Src\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Stm" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Stm\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Stm\Timer" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Vadc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Vadc\Adc" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\iLLD\TC26B\Tricore\Vadc\Std" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\Infra" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\Infra\Platform" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\Infra\Platform\Tricore" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\Infra\Platform\Tricore\Compilers" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\Infra\Sfr" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\Infra\Sfr\TC26B" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\Infra\Sfr\TC26B\_Reg" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\Service" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\BaseSw\Service\CpuGeneric" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\seekfree_libraries" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\seekfree_libraries\common" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\Libraries\seekfree_peripheral" -I"C:\Users\zxy\Desktop\new_car_v1.8\new_car_v1.4\image_processing1.0\USER" --iso=99 --language=-gcc,-volatile,+strings,-kanji --fp-model=3 --switch=auto --align=0 --default-near-size=0 --default-a0-size=0 --default-a1-size=0 -O0 --tradeoff=0 -g --error-limit=42 --source -c --dep-file="Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\.IfxHssl.o.d" -Wc--make-target="Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.o"
DEPENDENCY_FILES += "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\.IfxHssl.o.d"


GENERATED_FILES += "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.o" "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\.IfxHssl.o.opt" "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\.IfxHssl.o.d" "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.src" "Libraries\BaseSw\iLLD\TC26B\Tricore\Hssl\Std\IfxHssl.lst"
