#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pulp.h"
#include "rgb565_f0.h"

#define REG_PADFUN0_OFFSET 0x10
#define REG_PADFUN1_OFFSET 0x14
#define REG_PADFUN2_OFFSET 0x18
#define REG_PADFUN3_OFFSET 0x1C


//This test receives 32*32*2 bytes from the VIP, change the VIP to receive bigger data
// *2  defined in vip
#define HRES 32
#define VRES 32

#define BUFFER_SIZE 10
#define BUFFER_SIZE_READ 12
#define N_CAM 1


#define OUT 1
#define IN  0

#define PRINTF_ON


uint16_t rx_addr[N_PIXEL];
// N_PIXEL is defined in vip

int pad_fun_offset[4] = {REG_PADFUN0_OFFSET,REG_PADFUN1_OFFSET,REG_PADFUN2_OFFSET,REG_PADFUN3_OFFSET};



uint32_t set_alternate(uint32_t number){
  uint32_t which_reg_fun = number / 16; //select the correct register 
  uint32_t address = ARCHI_APB_SOC_CTRL_ADDR + pad_fun_offset[which_reg_fun]; // the corresponding reg addr

  //--- set alternate 1 on GPIO
  uint32_t value_wr = pulp_read32(address);
  value_wr |= (1 << ((number - which_reg_fun*16)*2));
  pulp_write32(address, value_wr);
}


uint32_t configure_gpio(uint32_t number, uint32_t direction, uint32_t alternate){
  uint32_t which_reg_fun = number / 16; //select the correct register
  uint32_t address = ARCHI_APB_SOC_CTRL_ADDR + pad_fun_offset[which_reg_fun];

  //--- set alternate 1/2/3 on GPIO
  uint32_t value_wr = pulp_read32(address);
  value_wr |= ((alternate & 0x00000003) << ((number - which_reg_fun*16)*2));
  pulp_write32(address, value_wr);

  //--- set GPIO
  if(number < 32)
  {
    if (direction == IN) 
    {
      //--- enable GPIO
      address = ARCHI_GPIO_ADDR + GPIO_GPIOEN_OFFSET;
      value_wr = pulp_read32(address);
      value_wr &= ~(1 << number);
      pulp_write32(address, value_wr);
      //--- set direction
      address = ARCHI_GPIO_ADDR + GPIO_PADDIR_OFFSET;
      pulp_write32(address, value_wr);
    }else if (direction == OUT){ 
      //--- enable GPIO
      address = ARCHI_GPIO_ADDR + GPIO_GPIOEN_OFFSET;
      value_wr = pulp_read32(address);
      value_wr |= (1 << number);
      pulp_write32(address, value_wr);
      //--- set direction
      address = ARCHI_GPIO_ADDR + GPIO_PADDIR_OFFSET;
      pulp_write32(address, value_wr);
    }
  }else{
    if (direction == IN)
    {
      //--- enable GPIO
      address = ARCHI_GPIO_ADDR + GPIO_GPIOEN_32_63_OFFSET;
      value_wr = pulp_read32(address);
      value_wr &= ~(1 << (number-32));
      pulp_write32(address, value_wr);
      //--- set direction
      address = ARCHI_GPIO_ADDR + GPIO_PADDIR_32_63_OFFSET;
      pulp_write32(address, value_wr);
    }else if (direction == OUT){
      //--- enable GPIO
      address = ARCHI_GPIO_ADDR + GPIO_GPIOEN_32_63_OFFSET;
      value_wr = pulp_read32(address);
      value_wr |= (1 << (number-32));
      pulp_write32(address, value_wr);
      //--- set direction
      address = ARCHI_GPIO_ADDR + GPIO_PADDIR_32_63_OFFSET;
      pulp_write32(address, value_wr);
    }
  }

  while(pulp_read32(address) != value_wr);

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
  if (number < 32)
  {
    address = ARCHI_GPIO_ADDR + GPIO_PADIN_OFFSET;
    value_rd = pulp_read32(address);
  }else{
    address = ARCHI_GPIO_ADDR + GPIO_PADIN_32_63_OFFSET;
    value_rd = pulp_read32(address);
  }
  //printf("value reg: %d \n",value_rd);
  return (value_rd & (1 << (number % 32)))>>number;
}


int main(){
  int error=0;

  //config registers
  uint32_t reg=0;
  uint16_t concat=0;
  uint32_t address;
  uint32_t value_wr = 0x00000000;

 // uint16_t *rx_addr= (uint16_t*) 0x1A101000;

  int j;

  //clear rx buffer
  /*for( int i=0; i< N_PIXEL; i++){
    rx_addr[i]=0;
  }

  #ifdef PRINTF_ON
    printf("rx buffer resetted\n");
    //uart_wait_tx_done();
  #endif*/

  //enable gpio and set direction as output 
  configure_gpio(0, OUT,1);


  set_gpio(0,0);

  #ifdef PRINTF_ON
    printf("Camera Vip Disabled\n");
    //uart_wait_tx_done();
  #endif

  uint32_t udma_cam_channel_base = hal_udma_channel_base(UDMA_CHANNEL_ID(ARCHI_UDMA_CAM_ID(0))); //select the camera ID=0
  //barrier();

  #ifdef PRINTF_ON
    printf("Channel base: %x\n", udma_cam_channel_base);
   // uart_wait_tx_done();
  #endif
  

  plp_udma_cg_set(plp_udma_cg_get() | (0xffffffff));
  #ifdef PRINTF_ON
    printf("Enable all CG\n");
   // uart_wait_tx_done();
  #endif
  //barrier();

  //write RX_SADDR register: it sets the L2 start address 
  //udma_cpi_cam_rx_saddr_set(udma_cam_channel_base, 0x1C001000);
  udma_cpi_cam_rx_saddr_set(udma_cam_channel_base, (int)rx_addr); //change
  //barrier();
     
  //write RX_SIZE register: it sets the buffer syze in bytes
  udma_cpi_cam_rx_size_set(udma_cam_channel_base, N_PIXEL);

  reg|= 1<<UDMA_CPI_CAM_CFG_FILTER_R_COEFF_BIT | 1<<UDMA_CPI_CAM_CFG_FILTER_G_COEFF_BIT | 1<<UDMA_CPI_CAM_CFG_FILTER_B_COEFF_BIT ;
  udma_cpi_cam_cfg_filter_set(udma_cam_channel_base, reg);
  //barrier();

  reg=0;
  reg|= 1<<UDMA_CPI_CAM_VSYNC_POLARITY_VSYNC_POLARITY_BIT;
  udma_cpi_cam_vsync_polarity_set(udma_cam_channel_base, reg);

  reg=0;
  reg|= 1<<UDMA_CPI_CAM_CFG_GLOB_EN_BIT | 4<< UDMA_CPI_CAM_CFG_GLOB_FORMAT_BIT;
  udma_cpi_cam_cfg_glob_set(udma_cam_channel_base, reg);
  //barrier();

  reg=0;
  reg|= 1<<UDMA_CPI_CAM_RX_CFG_EN_BIT | 1<<UDMA_CPI_CAM_RX_CFG_DATASIZE_BIT | 0<<UDMA_CPI_CAM_RX_CFG_CONTINOUS_BIT;
  udma_cpi_cam_rx_cfg_set(udma_cam_channel_base,reg);
  //barrier();

  #ifdef PRINTF_ON
    printf("End Of Config\n");
  //  uart_wait_tx_done();
  #endif

  //Enable Camera VIP by GPIO 0 -> 1
  set_gpio(0,1);

  #ifdef PRINTF_ON
    printf("Camera Vip Enabled\n");
   // uart_wait_tx_done();
  #endif
  

  //wait_cycles(70000);
  do{
    #ifdef PRINTF_ON
      printf("Still writing...\n");
    //  uart_wait_tx_done();
    #endif
  }while(udma_cpi_cam_rx_size_get(udma_cam_channel_base)!=0);

  #ifdef PRINTF_ON
      printf("End Transaction\n");
    //  uart_wait_tx_done();
    #endif


  for ( int i=0; i<N_PIXEL; i+=2){
    concat= frame_0[i]<<8 | frame_0[i+1];
    if(rx_addr[j]!=concat)
      error++;
      printf("L2[%d]: %x - Pixel[%d]: %x\n", j, rx_addr[j], j, concat);
    j++;
  }

  if(error!=0)
    printf("Test FAILED with :%d\n",error);
  else
    printf("Test PASSED\n");

  //uart_wait_tx_done();

  return error;
}
