################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/Src/stm32f411re_gpio_driver.c \
../driver/Src/stm32f411re_usart_driver.c 

OBJS += \
./driver/Src/stm32f411re_gpio_driver.o \
./driver/Src/stm32f411re_usart_driver.o 

C_DEPS += \
./driver/Src/stm32f411re_gpio_driver.d \
./driver/Src/stm32f411re_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/Src/%.o driver/Src/%.su driver/Src/%.cyclo: ../driver/Src/%.c driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -I"C:/Users/Admin/STM32CubeIDE/workspace_1.13.2/STM32F411RE_by_Belt/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-driver-2f-Src

clean-driver-2f-Src:
	-$(RM) ./driver/Src/stm32f411re_gpio_driver.cyclo ./driver/Src/stm32f411re_gpio_driver.d ./driver/Src/stm32f411re_gpio_driver.o ./driver/Src/stm32f411re_gpio_driver.su ./driver/Src/stm32f411re_usart_driver.cyclo ./driver/Src/stm32f411re_usart_driver.d ./driver/Src/stm32f411re_usart_driver.o ./driver/Src/stm32f411re_usart_driver.su

.PHONY: clean-driver-2f-Src

