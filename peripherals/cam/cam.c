include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "utils.h"
#include "udma.h"
#include "udma_cpi_v1.h"
#include "rgb565_f0.h"

//This test receives 32*32*2 bytes from the VIP, change the VIP to receive bigger data
#define HRES 32
#define VRES 32

#define BUFFER_SIZE 10
#define BUFFER_SIZE_READ 12
#define N_CAM 1


#define OUT 1
#define IN  0

uint32_t configure_gpio(uint32_t number, uint32_t direction){
  uint32_t address;
  uint32_t dir;
  uint32_t gpioen;

  //--- set GPIO
  if(number < 32)
  {
    if (direction == IN) 
    {

      address = ARCHI_GPIO_ADDR + GPIO_GPIOEN_OFFSET;

      gpioen = pulp_read32(address);
      //--- enable GPIO
      //printf("GPIOEN RD: %x\n",gpioen);
      gpioen |= (1 << number);
      //printf("GPIOEN WR: %x\n",gpioen); 
      pulp_write32(address, gpioen);
      //--- set direction
      address = ARCHI_GPIO_ADDR + GPIO_PADDIR_OFFSET;
      dir = pulp_read32(address);
      //printf("GPIODIR RD: %x\n",dir);
      dir |= (0 << number);
      //printf("GPIODIR WR: %x\n",dir);
      pulp_write32(address, dir);

    }else if (direction == OUT){ 
      //--- enable GPIO
      address = ARCHI_GPIO_ADDR + GPIO_GPIOEN_OFFSET;
      gpioen = pulp_read32(address);
      gpioen |= (1 << number);
      pulp_write32(address, gpioen);
      //--- set direction
      dir=gpioen;
      address = ARCHI_GPIO_ADDR + GPIO_PADDIR_OFFSET;
      pulp_write32(address, dir);
    }
  }else{
    if (direction == IN)
    {
      address = ARCHI_GPIO_ADDR + GPIO_GPIOEN_32_63_OFFSET;
      gpioen = pulp_read32(address);
      //--- enable GPIO
      //printf("GPIOEN RD: %x\n",gpioen);
      gpioen |= (1 << (number-32));
      //printf("GPIOEN WR: %x\n",gpioen); 
      pulp_write32(address, gpioen);
      //--- set direction
      address = ARCHI_GPIO_ADDR + GPIO_PADDIR_32_63_OFFSET;
      dir = pulp_read32(address);
      //printf("GPIODIR RD: %x\n",dir);
      dir |= (0 << (number-32));
      //printf("GPIODIR WR: %x\n",dir);
      pulp_write32(address, dir);
    }else if (direction == OUT){
      //--- enable GPIO
      address = ARCHI_GPIO_ADDR + GPIO_GPIOEN_32_63_OFFSET;
      gpioen = pulp_read32(address);
      gpioen |= (1 << (number-32));
      pulp_write32(address, gpioen);
      //--- set direction
      dir=gpioen;
      address = ARCHI_GPIO_ADDR + GPIO_PADDIR_32_63_OFFSET;
      pulp_write32(address, dir);
    }
  }

  while(pulp_read32(address) != dir);

}

void set_gpio(uint32_t number, uint32_t value){
  uint32_t value_wr;
  uint32_t address;
  if (number < 32)
  {
    address = ARCHI_GPIO_ADDR + GPIO_PADOUT_OFFSET;
    value_wr = pulp_read32(address);
    if (value == 1)
    {
      value_wr |= (1 << (number));
    }else{
      value_wr &= ~(1 << (number));
    }
    pulp_write32(address, value_wr);
  }else{
    address = ARCHI_GPIO_ADDR + GPIO_PADOUT_32_63_OFFSET;
    value_wr = pulp_read32(address);
    if (value == 1)
    {
      value_wr |= (1 << (number % 32));
    }else{
      value_wr &= ~(1 << (number % 32));
    }
    pulp_write32(address, value_wr);
  }

  while(pulp_read32(address) != value_wr);
}


uint32_t get_gpio(uint32_t number){
  uint32_t value_rd;
  uint32_t address;
  uint32_t bit;
  if (number < 32)
  {
    address = ARCHI_GPIO_ADDR + GPIO_PADIN_OFFSET;
    value_rd = pulp_read32(address);
    bit= 0x1 & (value_rd>>number);
  }else{
    address = ARCHI_GPIO_ADDR + GPIO_PADIN_32_63_OFFSET;
    value_rd = pulp_read32(address);
    bit= 0x1 & (value_rd>>(number%32));
  }  
  //printf("GPIO %d: HEX:%x Bit:%d \n",number,value_rd,bit);
  return bit;
}


  
int main(){
  int error=0;

  //config registers
  uint32_t reg=0;
  uint16_t concat=0;
  uint32_t address;
  uint32_t val_wr = 0x00000000;

  uint16_t *rx_addr= (uint16_t*) 0x1C001000;

  int j;
  //config pad_gpio as GPIO
  configure_gpio(1, 1);
  // need to config padframe on cam
  set_gpio(9,1);
  set_gpio(10,1);
    set_gpio(11,1);
	 set_gpio(12,1);
	  set_gpio(13,1); 
	  set_gpio(14,1);
	  set_gpio(15,1);
	  set_gpio(16,1);
	  set_gpio(17,1);
	  set_gpio(18,1);
	  set_gpio(19,1);
	  
  
  
  
  
  #ifdef FPGA_EMULATION
  int baud_rate = 115200;
  int test_freq = 50000000;
  #else
  set_flls();
  int baud_rate = 115200;
  int test_freq = 100000000;
  #endif  
  uart_set_cfg(0,(test_freq/baud_rate)>>4);
  
  
    #ifdef PRINTF_ON
    printf("Camera Vip Disabled\n");
    uart_wait_tx_done();
  #endif

  //clear rx buffer
  for(int i=0; i< HRES * VRES; i++){
    rx_addr[i]=0x00;
  }

  uint32_t udma_cam_channel_base = hal_udma_channel_base(UDMA_CHANNEL_ID(ARCHI_UDMA_CAM_ID(0))); //select the camera ID=0
  barrier();

  #ifdef PRINTF_ON
    printf("Channel base: %x\n", udma_cam_channel_base);
    uart_wait_tx_done();
  #endif
  

  plp_udma_cg_set(plp_udma_cg_get() | (0xffffffff));
  #ifdef PRINTF_ON
    printf("Enable all CG\n");
    uart_wait_tx_done();
  #endif
  barrier();

  //write RX_SADDR register: it sets the L2 start address 
  udma_cpi_cam_rx_saddr_set(udma_cam_channel_base, 0x1C001000);
  barrier();
     
  //write RX_SIZE register: it sets the buffer syze in bytes
  udma_cpi_cam_rx_size_set(udma_cam_channel_base, N_PIXEL);

  reg|= 1<<UDMA_CPI_CAM_CFG_FILTER_R_COEFF_BIT | 1<<UDMA_CPI_CAM_CFG_FILTER_G_COEFF_BIT | 1<<UDMA_CPI_CAM_CFG_FILTER_B_COEFF_BIT ;
  udma_cpi_cam_cfg_filter_set(udma_cam_channel_base, reg);
  barrier();

  reg=0;
  reg|= 1<<UDMA_CPI_CAM_VSYNC_POLARITY_VSYNC_POLARITY_BIT;
  udma_cpi_cam_vsync_polarity_set(udma_cam_channel_base, reg);

  reg=0;
  reg|= 1<<UDMA_CPI_CAM_CFG_GLOB_EN_BIT | 4<< UDMA_CPI_CAM_CFG_GLOB_FORMAT_BIT;
  udma_cpi_cam_cfg_glob_set(udma_cam_channel_base, reg);
  barrier();

  reg=0;
  reg|= 1<<UDMA_CPI_CAM_RX_CFG_EN_BIT | 1<<UDMA_CPI_CAM_RX_CFG_DATASIZE_BIT | 0<<UDMA_CPI_CAM_RX_CFG_CONTINOUS_BIT;
  udma_cpi_cam_rx_cfg_set(udma_cam_channel_base,reg);
  barrier();

  #ifdef PRINTF_ON
    printf("End Of Config\n");
    uart_wait_tx_done();
  #endif

  //Enable Camera VIP by GPIO 0 -> 1
  address = ARCHI_GPIO_ADDR + GPIO_PADOUT_0_31_OFFSET;
  val_wr = 0x1;
  pulp_write32(address, val_wr);
  while(pulp_read32(address) != val_wr);

  #ifdef PRINTF_ON
    printf("Camera Vip Enabled\n");
    uart_wait_tx_done();
  #endif
  

  //wait_cycles(70000);
  do{
    #ifdef PRINTF_ON
      printf("Still writing...\n");
      uart_wait_tx_done();
    #endif
  }while(udma_cpi_cam_rx_size_get(udma_cam_channel_base)!=0);

  #ifdef PRINTF_ON
      printf("End Transaction\n");
      uart_wait_tx_done();
    #endif

  for (int i=0; i<N_PIXEL; i+=2){
    concat= frame_0[i]<<8 | frame_0[i+1];
    if(rx_addr[j]!=concat)
      error++;
    //printf("L2[%d]: %x - Pixel[%d]: %x\n", j, rx_addr[j], j, concat);
    j++;
  }

  if(error!=0)
    printf("Test FAILED with :%d\n",error);
  else
    printf("Test PASSED\n");

  uart_wait_tx_done();

  return error;
}
