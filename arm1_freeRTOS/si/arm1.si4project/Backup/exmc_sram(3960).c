/*!
    \file    exmc_sram.c
    \brief   EXMC SRAM(ISSI IS61LV51216) driver

    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2022-03-09, V3.0.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "exmc_sram.h"
#include "FmcAdapter.h"
#include "drv_gpio.h"
#include "pin_numbers_def.h"


#define Bank0_BASE_ADDR            ((uint32_t)0x60000000)
#define Bank0_ADDR_OFFSET          ((uint32_t)0x04000000)

#define	EXMC_NE1_PIN	GD32F450_PD7_PIN_NUM
#define	EXMC_NE2_PIN	GD32F450_PG9_PIN_NUM


/*!
    \brief      SRAM peripheral initialize
    \param[in]  nor_region: specify the nor flash region
    \param[out] none
    \retval     none
*/
void exmc_sram_init(void)
#if 0
{
	
    exmc_norsram_parameter_struct nor_init_struct;
    exmc_norsram_timing_parameter_struct nor_timing_init_struct;

    /* EXMC clock enable */
    rcu_periph_clock_enable(RCU_EXMC);

    /* EXMC enable */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOG);

	
    /* configure GPIO D[0-15] */
    //gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
    //gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |GPIO_PIN_7  | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
    //gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1  | GPIO_PIN_4 | GPIO_PIN_5  |GPIO_PIN_7  | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
    
    gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5  | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1  | GPIO_PIN_4 | GPIO_PIN_5   | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
    
    gpio_af_set(GPIOE, GPIO_AF_12, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                  GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                            GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    /* configure GPIO A[0-18] */
    gpio_af_set(GPIOF, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                  GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                            GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    gpio_af_set(GPIOG, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                GPIO_PIN_4 | GPIO_PIN_5);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                  GPIO_PIN_4 | GPIO_PIN_5);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                            GPIO_PIN_4 | GPIO_PIN_5);

	/* configure EXMC NE1 */
	gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_7);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    /* configure EXMC NE2 */
//    gd32_pin_mode(EXMC_NE2_PIN, PIN_MODE_OUTPUT);
//	gd32_pin_write(EXMC_NE2_PIN, PIN_LOW);
    gpio_af_set(GPIOG, GPIO_AF_12, GPIO_PIN_9);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* configure timing parameter */
    nor_timing_init_struct.asyn_access_mode = EXMC_ACCESS_MODE_A;
    nor_timing_init_struct.syn_data_latency = EXMC_DATALAT_2_CLK;
    nor_timing_init_struct.syn_clk_division = EXMC_SYN_CLOCK_RATIO_5_CLK;
    nor_timing_init_struct.bus_latency = 0;
    nor_timing_init_struct.asyn_data_setuptime = 30;
    nor_timing_init_struct.asyn_address_holdtime = 15;
    nor_timing_init_struct.asyn_address_setuptime = 10;

    /* configure EXMC bus parameters */
    nor_init_struct.norsram_region = EXMC_BANK0_NORSRAM_REGION0;
    nor_init_struct.write_mode = EXMC_ASYN_WRITE;
    nor_init_struct.extended_mode = DISABLE;
    nor_init_struct.asyn_wait = DISABLE;
    nor_init_struct.nwait_signal = DISABLE;
    nor_init_struct.memory_write = ENABLE;
    nor_init_struct.nwait_config = EXMC_NWAIT_CONFIG_BEFORE;
    nor_init_struct.wrap_burst_mode = DISABLE;
    nor_init_struct.nwait_polarity = EXMC_NWAIT_POLARITY_LOW;
    nor_init_struct.burst_mode = DISABLE;
    nor_init_struct.databus_width = EXMC_NOR_DATABUS_WIDTH_16B;
    nor_init_struct.memory_type = EXMC_MEMORY_TYPE_SRAM;
    nor_init_struct.address_data_mux = DISABLE;
    nor_init_struct.read_write_timing = &nor_timing_init_struct;
    nor_init_struct.write_timing = &nor_timing_init_struct;
    exmc_norsram_init(&nor_init_struct);

    /* enable the EXMC bank0 NORSRAM */
    exmc_norsram_enable(EXMC_BANK0_NORSRAM_REGION0);
    /* enable the EXMC bank1 NORSRAM */
    nor_init_struct.norsram_region = EXMC_BANK0_NORSRAM_REGION1;
    exmc_norsram_enable(EXMC_BANK0_NORSRAM_REGION1);

    /* Read FPGA version */
    //while (FMC_ReadWord(SPACE_COM, 0x220 >> 1) != 0x5555);
}
#else
{
	exmc_norsram_parameter_struct		sram_init_struct;
	exmc_norsram_timing_parameter_struct read_timing;
//	exmc_norsram_timing_parameter_struct write_timing;

	/* enable EXMC clock*/
	rcu_periph_clock_enable(RCU_EXMC);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);
	rcu_periph_clock_enable(RCU_GPIOE);
	rcu_periph_clock_enable(RCU_GPIOF);
	rcu_periph_clock_enable(RCU_GPIOG);
	rcu_periph_clock_enable(RCU_GPIOH);

	/* common GPIO configuration */
	gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);
	gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);
	
	
	gpio_af_set(GPIOE, GPIO_AF_12, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	
	gpio_af_set(GPIOF, GPIO_AF_12, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	
	gpio_af_set(GPIOG, GPIO_AF_12, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9);
	gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9);
	
	
	//读时序配置
	read_timing.asyn_access_mode		= EXMC_ACCESS_MODE_A;	//模式A，异步访问SRAM
	read_timing.asyn_address_setuptime	= 10;					//异步访问地址建立时间
	read_timing.asyn_address_holdtime	= 2;					//异步访问地址保持时间
	read_timing.asyn_data_setuptime		= 40;					//异步访问数据建立时间
	read_timing.bus_latency				= 0;					//同步/异步访问总线延时时间
	read_timing.syn_clk_division		= EXMC_SYN_CLOCK_RATIO_2_CLK;  //同步访问时钟分频系数（从HCLK中分频）
	read_timing.syn_data_latency		= EXMC_DATALAT_2_CLK;	//同步访问中获得第1个数据所需要的等待延迟

	//写时序配置
//	write_timing.asyn_access_mode			= EXMC_ACCESS_MODE_A;	//模式A，异步访问SRAM
//	write_timing.asyn_address_setuptime		= 0;					//异步访问地址建立时间
//	write_timing.asyn_address_holdtime		= 15;					//异步访问地址保持时间
//	write_timing.asyn_data_setuptime		= 40;					//异步访问数据建立时间
//	write_timing.bus_latency				= 0;					//同步/异步访问总线延时时间
//	write_timing.syn_clk_division			= EXMC_SYN_CLOCK_RATIO_15_CLK; //同步访问时钟分频系数（从HCLK中分频）
//	write_timing.syn_data_latency			= EXMC_DATALAT_2_CLK;	//同步访问中获得第1个数据所需要的等待延迟


	/* step 2 : configure SDRAM control registers */
	//Region1配置

	sram_init_struct.norsram_region		= EXMC_BANK0_NORSRAM_REGION0;	//Region0
	sram_init_struct.address_data_mux	= DISABLE;						//地址、数据总线多路复用
	sram_init_struct.memory_type		= EXMC_MEMORY_TYPE_SRAM;		//储存器类型为SRAM
	sram_init_struct.databus_width		= EXMC_NOR_DATABUS_WIDTH_16B;	//数据宽度16位
	sram_init_struct.burst_mode			= DISABLE;						//禁用突发访问
	sram_init_struct.nwait_config		= EXMC_NWAIT_CONFIG_BEFORE;		//等待输入配置
	sram_init_struct.nwait_polarity		= EXMC_NWAIT_POLARITY_HIGH;		//等待输入信号低电平有效
	sram_init_struct.wrap_burst_mode	= DISABLE;						//禁用包突发访问
	sram_init_struct.asyn_wait			= DISABLE;						//禁用异步等待
	sram_init_struct.extended_mode		= DISABLE;						//使能扩展模式
	sram_init_struct.memory_write		= ENABLE;						//使能写入外部存储器
	sram_init_struct.nwait_signal		= DISABLE;						//禁用等待输入信号
	sram_init_struct.write_mode			= EXMC_ASYN_WRITE;				//写入模式为异步写入
	sram_init_struct.read_write_timing	= &read_timing;					//读时序配置
	sram_init_struct.write_timing		= &read_timing;					//写时序配置
	//初始化Region1
	exmc_norsram_init(&sram_init_struct);

	//使能Region1
	exmc_norsram_enable(EXMC_BANK0_NORSRAM_REGION0);
	
	exmc_norsram_enable(EXMC_BANK0_NORSRAM_REGION1);
	//delay_us(200);
//	while (FMC_ReadWord(SPACE_COM, 0x220 >> 1) != 0x5555);
}
#endif

/*!
    \brief      write a Half-word buffer(data is 16 bits) to the EXMC SRAM memory
    \param[in]  pbuffer : pointer to buffer
    \param[in]  writeaddr : SRAM memory internal address from which the data will be written
    \param[in]  num_halfword_write : number of half-words to write
    \param[out] none
    \retval     none
*/
void exmc_sram_writebuffer_16(uint16_t *pbuffer, uint32_t writeaddr, uint32_t num_halfword_write, uint8_t region)
{
    uint32_t sram_addr;

    if(region > BANK0_REGION_MAX)return;

    sram_addr = Bank0_BASE_ADDR + (Bank0_ADDR_OFFSET * region);

    /* while there is data to write */
    for(; num_halfword_write != 0; num_halfword_write--)
    {
        /* transfer data to the memory */
        *(uint16_t *)(sram_addr + writeaddr) = *pbuffer++;

        /* increment the address */
        writeaddr += 2;
    }
}

/*!
    \brief      read a block of 16-bit data from the EXMC SRAM memory
    \param[in]  pbuffer : pointer to the buffer that receives the data read from the SRAM memory
    \param[in]  readaddr : SRAM memory internal address to read from
    \param[in]  num_halfword_read : number of half-words to read
    \param[out] none
    \retval     none
*/
void exmc_sram_readbuffer_16(uint16_t *pbuffer, uint32_t readaddr, uint32_t num_halfword_read, uint8_t region)
{
    uint32_t sram_addr;

    if(region > BANK0_REGION_MAX)return;

    sram_addr = Bank0_BASE_ADDR + (Bank0_ADDR_OFFSET * region);

    /* while there is data to read */
    for(; num_halfword_read != 0; num_halfword_read--)
    {
        /* read a half-word from the memory */
        *pbuffer++ = *(uint16_t *)(sram_addr + readaddr);

        /* increment the address */
        readaddr += 2;
    }
}

/*!
    \brief      write a word buffer(data is 32 bits) to the EXMC SRAM memory
    \param[in]  pbuffer : pointer to buffer
    \param[in]  writeaddr : SRAM memory internal address from which the data will be written
    \param[in]  num_word_write : number of words to write
    \param[out] none
    \retval     none
*/
void exmc_sram_writebuffer_32(uint32_t *pbuffer, uint32_t writeaddr, uint32_t num_word_write, uint8_t region)
{
    uint32_t sram_addr;

    if(region > BANK0_REGION_MAX)return;

    sram_addr = Bank0_BASE_ADDR + (Bank0_ADDR_OFFSET * region);

    /* while there is data to write */
    for(; num_word_write != 0; num_word_write--)
    {
        /* transfer data to the memory */
        *(uint32_t *)(sram_addr + writeaddr) = *pbuffer++;

        /* increment the address */
        writeaddr += 4;
    }
}

/*!
    \brief      read a block of 32-bit data from the EXMC SRAM memory
    \param[in]  pbuffer : pointer to the buffer that receives the data read from the SRAM memory
    \param[in]  readaddr : SRAM memory internal address to read from
    \param[in]  num_word_read : number of words to read
    \param[out] none
    \retval     none
*/
void exmc_sram_readbuffer_32(uint32_t *pbuffer, uint32_t readaddr, uint32_t num_word_read, uint8_t region)
{
    uint32_t sram_addr;

    if(region > BANK0_REGION_MAX)return;

    sram_addr = Bank0_BASE_ADDR + (Bank0_ADDR_OFFSET * region);

    /* while there is data to read */
    for(; num_word_read != 0; num_word_read--)
    {
        /* read a word from the memory */
        *pbuffer++ = *(uint32_t *)(sram_addr + readaddr);

        /* increment the address */
        readaddr += 4;
    }
}

/*!
    \brief      write a Byte buffer(data is 8 bits ) to the EXMC SRAM memory
    \param[in]  pbuffer : pointer to buffer
    \param[in]  writeaddr : SRAM memory internal address from which the data will be written
    \param[in]  num_byte_write : number of bytes to write
    \param[out] none
    \retval     none
*/
void exmc_sram_writebuffer_8(uint8_t *pbuffer, uint32_t writeaddr, uint32_t num_byte_write, uint8_t region)
{
    uint32_t sram_addr;

    if(region > BANK0_REGION_MAX)return;

    sram_addr = Bank0_BASE_ADDR + (Bank0_ADDR_OFFSET * region);

    /* while there is data to write */
    for(; num_byte_write != 0; num_byte_write--)
    {
        /* transfer data to the memory */
        *(uint8_t *)(sram_addr + writeaddr) = *pbuffer++;

        /* increment the address*/
        writeaddr += 1;
    }
}

/*!
    \brief      read a block of 8-bit data from the EXMC SRAM memory
    \param[in]  pbuffer : pointer to the buffer that receives the data read from the SRAM memory
    \param[in]  readaddr : SRAM memory internal address to read from
    \param[in]  num_byte_read : number of bytes to write
    \param[out] none
    \retval     none
*/
void exmc_sram_readbuffer_8(uint8_t *pbuffer, uint32_t readaddr, uint32_t num_byte_read, uint8_t region)
{
    uint32_t sram_addr;

    if(region > BANK0_REGION_MAX)return;

    sram_addr = Bank0_BASE_ADDR + (Bank0_ADDR_OFFSET * region);

    /* while there is data to read */
    for(; num_byte_read != 0; num_byte_read--)
    {
        /* read a byte from the memory */
        *pbuffer++ = *(uint8_t *)(sram_addr + readaddr);

        /* increment the address */
        readaddr += 1;
    }
}

/*!
    \brief      fill the 16-bit buffer with specified value 
    \param[in]  pbuffer: pointer on the buffer to fill
    \param[in]  buffersize: size of the buffer to fill
    \param[in]  offset: first value to fill on the buffer
    \param[out] none
    \retval     none
*/
void fill_buffer_16(uint16_t *pbuffer, uint16_t buffer_lenght, uint16_t offset)
{
    uint16_t index = 0;

    for(index = 0; index < buffer_lenght; index++)
    {
        pbuffer[index] = index + offset;
    }
}

/* SRAM */
uint16_t txbuffer[BUFFER_SIZE];
uint16_t rxbuffer[BUFFER_SIZE];
void exmc_selftest(void)
{
	exmc_sram_init();
	fill_buffer_16(txbuffer, BUFFER_SIZE, 0x1215);
    /* write data to SRAM  */
    exmc_sram_writebuffer_8((uint8_t*)txbuffer, WRITE_READ_ADDR, BUFFER_SIZE,0);
    /* read data from SRAM */
    exmc_sram_readbuffer_8((uint8_t*)rxbuffer, WRITE_READ_ADDR, BUFFER_SIZE,0);
}




