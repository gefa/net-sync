################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
DebugTools.obj: ../DebugTools.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c6000_7.4.12/bin/cl6x" -mv6713 --abi=coffabi -g --include_path="C:/ti/ccsv6/tools/compiler/c6000_7.4.12/include" --include_path="C:/Program Files/C6xCSL/include" --include_path="C:/TI_DSK/dsk6713revc_files/CCStudio/c6000/dsk6713/include" --define=CHIP_6713 --display_error_number --diag_warning=225 --diag_wrap=off --mem_model:const=far --mem_model:data=far --preproc_with_compile --preproc_dependency="DebugTools.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

MathCalculations.obj: ../MathCalculations.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c6000_7.4.12/bin/cl6x" -mv6713 --abi=coffabi -g --include_path="C:/ti/ccsv6/tools/compiler/c6000_7.4.12/include" --include_path="C:/Program Files/C6xCSL/include" --include_path="C:/TI_DSK/dsk6713revc_files/CCStudio/c6000/dsk6713/include" --define=CHIP_6713 --display_error_number --diag_warning=225 --diag_wrap=off --mem_model:const=far --mem_model:data=far --preproc_with_compile --preproc_dependency="MathCalculations.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

time_stamper_master.obj: ../time_stamper_master.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c6000_7.4.12/bin/cl6x" -mv6713 --abi=coffabi -g --include_path="C:/ti/ccsv6/tools/compiler/c6000_7.4.12/include" --include_path="C:/Program Files/C6xCSL/include" --include_path="C:/TI_DSK/dsk6713revc_files/CCStudio/c6000/dsk6713/include" --define=CHIP_6713 --display_error_number --diag_warning=225 --diag_wrap=off --mem_model:const=far --mem_model:data=far --preproc_with_compile --preproc_dependency="time_stamper_master.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

vectors.obj: ../vectors.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c6000_7.4.12/bin/cl6x" -mv6713 --abi=coffabi -g --include_path="C:/ti/ccsv6/tools/compiler/c6000_7.4.12/include" --include_path="C:/Program Files/C6xCSL/include" --include_path="C:/TI_DSK/dsk6713revc_files/CCStudio/c6000/dsk6713/include" --define=CHIP_6713 --display_error_number --diag_warning=225 --diag_wrap=off --mem_model:const=far --mem_model:data=far --preproc_with_compile --preproc_dependency="vectors.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


