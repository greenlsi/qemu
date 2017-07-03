
#include "hw/gpio/bcm2835_gpio.h"

//-----------------------------------------------------------------------------------------------
static void
rx_message (SoupWebsocketConnection *self,
            gint                     type,
            GBytes                  *message,
            gpointer                 user_data)
{

	struct bcm2835_gpio_s *s = &((bcm2835_gpif_s *) user_data)->bcm2835_1;


	guint i;
  guint gpio_in = 0; //0 -> entrada / 1 -> salida
	GByteArray *msg = g_bytes_unref_to_array (message);

	//g_print("GPIO Message received (%d bytes): ", msg->len);

	for (i=0 ; i<msg->len; i++) {
		if(msg->data[i]< 0x40){msg->data[i] = msg->data[i] - 0x30;}
		else if(msg->data[i]> 0x60){msg->data[i] = msg->data[i] - 0x57;}

		//g_print(" %02X ", msg->data[i]);
	}
	//g_print("\n");

	msg->data[0] = (msg->data[0]*0X10)+ msg->data[1];
	//g_print(" %d ,", msg->data[0]);

	uint32_t bit = 1 << msg->data[0];
  uint32_t old_value;

  //Detección de GPIO de entrada o salida
  uint32_t val_GPFSEL = 0x00;
  int despl = ((int)(msg->data[0]%10))*3;
  uint32_t cmp = 0x00;
  int reg_GPSELX = msg->data[0]/10;
  switch (reg_GPSELX){
    case 0:
      val_GPFSEL = s->registers[ID(GPFSEL0)];
      break;
    case 1:
      val_GPFSEL = s->registers[ID(GPFSEL1)];
      break;
    case 2:
      val_GPFSEL = s->registers[ID(GPFSEL2)];
      break;
    case 3:
      val_GPFSEL = s->registers[ID(GPFSEL3)];
      break;
    case 4:
      val_GPFSEL = s->registers[ID(GPFSEL4)];
      break;
    case 5:
      val_GPFSEL = s->registers[ID(GPFSEL5)];
      break;
  }
  val_GPFSEL &= 0b111 << despl; 
  if (val_GPFSEL == cmp){
    gpio_in = 1;
  }


	//g_print("bit: %d ,", bit);
  if(gpio_in){
  	if(msg->data[2]==0x01 ){	 
  		if(msg->data[0]<0x20) {//32 en decimal
  			//g_print(" bit: 0x%08X | GPLEV0: 0x%08X\n ",  bit, s->registers[ID(GPLEV0)]);
        old_value = s->registers[ID(GPLEV0)];
  			s->registers[ID(GPLEV0)] |= (bit); 
        if ((s->registers[ID(GPREN0)] & bit) && !(old_value & bit)){
          s->registers[ID(GPEDS0)] |= bit;
          qemu_set_irq (s->irq[0], 1);
          //g_print("GPEDS0: 0x%08X \n", (uint32_t)s->registers[ID(GPEDS0)]);
        }
  			//g_print("Status Register 0 - GPLEV0 = 0x%08X\n ",  s->registers[ID(GPLEV0)]);
  		} else {
  			//g_print(" bit(32-52): 0x%08X | GPLEV1: 0x%08X\n ",  bit, s->registers[ID(GPLEV1)]);
  			old_value = s->registers[ID(GPLEV1)];
        s->registers[ID(GPLEV1)] |= (bit); 
        if ((s->registers[ID(GPREN1)] & bit) && !(old_value & bit)){
          s->registers[ID(GPEDS1)] |= bit;
          qemu_set_irq (s->irq[1], 1);
          //g_print("GPEDS1: 0x%08X \n", (uint32_t)s->registers[ID(GPEDS1)]);
        }
  			//g_print("Status Register 1 - GPLEV1 = 0x%08X\n ",  s->registers[ID(GPLEV1)]);
  		}
  	} else {
  		if(msg->data[0]<0x20) {
  			//g_print(" bit: 0x%08X & GPLEV0: 0x%08X\n ",  bit, s->registers[ID(GPLEV0)]);
        old_value = s->registers[ID(GPLEV0)];
  			s->registers[ID(GPLEV0)] &= (~bit);
        if ((s->registers[ID(GPFEN0)] & bit) && (old_value & bit)){
          s->registers[ID(GPEDS0)] |= bit;
          qemu_set_irq (s->irq[0], 1);
          //g_print("GPEDS0: 0x%08X \n", (uint32_t)s->registers[ID(GPEDS0)]);
        }
  			//g_print("Status Register 0 - GPLEV0 = 0x%08X\n ",  s->registers[ID(GPLEV0)]);
  		} else {
  			//g_print(" bit: 0x%08X & GPLEV1: 0x%08X\n ",  bit, s->registers[ID(GPLEV1)]);
        old_value = s->registers[ID(GPLEV1)];
        s->registers[ID(GPLEV1)] &= (~bit);
        if ((s->registers[ID(GPFEN1)] & bit) && (old_value & bit)){
          s->registers[ID(GPEDS1)] |= bit;
          qemu_set_irq (s->irq[0], 1);
          //g_print("GPEDS0: 0x%08X \n", (uint32_t)s->registers[ID(GPEDS1)]);
        }			//g_print("Status Register 1 - GPLEV1 = 0x%08X\n ",  s->registers[ID(GPLEV1)]);
  		}
  	}
  }
}

//---------------------------------------------------------------------------------------
/* General-Purpose I/O of bcm2835 */
#if 0
#endif
//------------------------------------------------------------------------------------------------
static void
error_message (SoupWebsocketConnection *self,
               GError                  *error,
               gpointer                 user_data)
{

  //g_print("Error message: %s\n", error->message);
}
//--------------------------------------------------------------------------------------------------
static void
close_message (SoupWebsocketConnection *self,
               gpointer                 user_data)
{
  //bcm2835_gpif_s* d = BCM2835_GPIO(user_data);
  struct bcm2835_gpio_s *s = &((bcm2835_gpif_s *) user_data)->bcm2835_1;

  if (s->connection) {
    g_object_unref(s->connection); 
    s->connection = 0;
  }
  //g_print("Closed message\n");
}
//-------------------------------------------------------------------------------------------------
static void
server_websocket_callback (SoupServer               *server,
                           SoupWebsocketConnection  *connection,
                           const char               *path,
                           SoupClientContext        *client,
                           gpointer                  user_data) 
{
  
  bcm2835_gpif_s* d = BCM2835_GPIO(user_data);
  struct bcm2835_gpio_s *s = &d->bcm2835_1;
  ////g_print("Dir mem puntero s en socket: %08lX\n", (unsigned long)(d));

  //g_print("Gpio Received Connection from client %s!\n", soup_websocket_connection_get_origin(connection));

  if (s->connection) {
    g_object_unref(s->connection); 
  }

  s->connection = connection;
  g_object_ref(s->connection);

  g_signal_connect (connection,
                    "message",
                    G_CALLBACK (rx_message),
                    user_data);

  g_signal_connect (connection,
                    "error",
                    G_CALLBACK (error_message),
                    user_data);

  g_signal_connect (connection,
                    "closed",
                    G_CALLBACK (close_message),
                    user_data);


  int comando;
  if (s->connection) {
    gchar message[26];
    comando = 0;
    sprintf(message, "%X%08X%08X%08X", comando,s->registers[ID(GPFSEL2)],s->registers[ID(GPFSEL1)],s->registers[ID(GPFSEL0)]);
    soup_websocket_connection_send_text (s->connection, (const char*)message);
    //printf("Enviando configuracion actual del HW GPIO al Simulador\n"); 
    gchar message2[18];
    comando = 2;
    sprintf(message2, "%X%08X%08X", comando,s->registers[ID(GPLEV1)],s->registers[ID(GPLEV0)]);
    soup_websocket_connection_send_text (s->connection, (const char*)message2);
    } 
}


//----------------------------------------------------------------------------------------
static uint64_t bcm2835_gpio_read(void *opaque, hwaddr addr, unsigned size) {

  struct bcm2835_gpio_s *s = (struct bcm2835_gpio_s *) opaque;
     

  if (addr > GPPUDCLK1) {//si la drireccion es mayor
    BCM2835_BAD_REG(addr);
  }

  if (addr % 4) { 
    BCM2835_BAD_REG(addr);
  }
    
  switch ((unsigned int)(addr & 0xFFFFFFFFUL)) {
    case GPSET0:
    case GPSET1:
    case GPCLR0:
    case GPCLR1:
      return 0;
    default: 
      return s->registers[ID(addr)];
  }
    
 return 0;
}

//------------------------------------------------------------------------------------------
static void bcm2835_gpio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {

	struct bcm2835_gpio_s *s = (struct bcm2835_gpio_s *) opaque;

	unsigned old_value;
  //uint32_t value_n;

	static int comando;

  switch((unsigned int)(addr & 0xFFFFFFFFUL)){
  	
  	case GPFSEL0:
      old_value = s->registers[ID(GPFSEL0)]; 
  	  s->registers[ID(GPFSEL0)] = value;   
  		//g_print("old value: 0x%08X value: 0x%08X\n",  old_value, (uint32_t)value);

      break;
    //****************************************************************************************
  	case GPFSEL1:
      old_value = s->registers[ID(GPFSEL1)];
      s->registers[ID(GPFSEL1)] = value;
      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, (uint32_t)value);
              
      break;
    //**************************************************************************************
  	case GPFSEL2:
      old_value = s->registers[ID(GPFSEL2)]; 
      s->registers[ID(GPFSEL2)] = value;   
      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, (uint32_t)value);

      break;
    //**************************************************************************************
  	case GPFSEL3:
      old_value = s->registers[ID(GPFSEL3)]; 
      s->registers[ID(GPFSEL3)] = value;   
      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, (uint32_t)value);

      break;
    //**************************************************************************************
  	case GPFSEL4:
      old_value = s->registers[ID(GPFSEL4)]; 
      s->registers[ID(GPFSEL4)] = value;   
      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, (uint32_t)value);

      break;
    //**************************************************************************************
  	case GPFSEL5:
      old_value = s->registers[ID(GPFSEL5)]; 
      s->registers[ID(GPFSEL5)] = value;   
      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, (uint32_t)value);

      break;
    //**************************************************************************************
  	case GPSET0:
      old_value = s->registers[ID(GPLEV0)];
  		s->registers[ID(GPLEV0)] |= value;
      if ((s->registers[ID(GPREN0)] & value) && !(old_value & value)){
        s->registers[ID(GPEDS0)] |= value;
        qemu_set_irq (s->irq[0], 1);
        //g_print("GPEDS0: 0x%08X \n", (uint32_t)s->registers[ID(GPEDS0)]);
      }
  	
      break;
    //***************************************************************************************
  	case GPSET1:
	  	old_value = s->registers[ID(GPLEV1)];
      s->registers[ID(GPLEV1)] |= value;
      if ((s->registers[ID(GPREN1)] & value) && !(old_value & value)){
        s->registers[ID(GPEDS1)] |= value;
        qemu_set_irq (s->irq[1], 1);
      }
  	
      break;
    //**************************************************************************************
  	case GPCLR0:
      old_value = s->registers[ID(GPLEV0)];
  		s->registers[ID(GPLEV0)] &= (~value);
      if ((s->registers[ID(GPFEN0)] & value) && (old_value & value)){
        s->registers[ID(GPEDS0)] |= value;
        qemu_set_irq (s->irq[0], 1);
        //g_print("GPEDS0: 0x%08X \n", (uint32_t)s->registers[ID(GPEDS0)]);
      }
      
  	  break;
    //***************************************************************************************
  	case GPCLR1:
  		old_value = s->registers[ID(GPLEV1)];
      s->registers[ID(GPLEV1)] &= (~value);
      if ((s->registers[ID(GPFEN1)] & value) && (old_value & value)){
        s->registers[ID(GPEDS1)] |= value;
        qemu_set_irq (s->irq[1], 1);
      }
  	
      break;
    //***************************************************************************************
    case GPEDS0: 
      old_value = s->registers[ID(GPEDS0)]; 
      //value_n = value;
      //g_print("DEBUG valor cuando se llama a GPEDS0 value: 0x%08X\n",value_n);
      s->registers[ID(GPEDS0)] &= (~value);  
      qemu_set_irq(s->irq[0], 0);
      qemu_set_irq(s->irq[1], 0);

      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, s->registers[ID(GPEDS0)]);  

      break;
    //***************************************************************************************
    case GPEDS1: 
      old_value = s->registers[ID(GPEDS1)]; 
      //value_n = value;
      //g_print("DEBUG valor cuando se llama a GPEDS1 value: 0x%08X\n",value_n);
      s->registers[ID(GPEDS1)] &= (~value);  
      qemu_set_irq(s->irq[0], 0);
      qemu_set_irq(s->irq[1], 0);

      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, s->registers[ID(GPEDS1)]);

      break;
    //***************************************************************************************
    case GPREN0: 
      old_value = s->registers[ID(GPREN0)]; 
      s->registers[ID(GPREN0)] |= value;  
      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, s->registers[ID(GPREN0)]);

      break;
    //***************************************************************************************
    case GPREN1:
      old_value = s->registers[ID(GPREN1)]; 
      s->registers[ID(GPREN1)] |= value; 
      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, s->registers[ID(GPREN1)]);

      break;
    //***************************************************************************************
		case GPFEN0:
			old_value = s->registers[ID(GPFEN0)]; 
			s->registers[ID(GPFEN0)] |= value;   
			//g_print("old value: 0x%08X value: 0x%08X\n",  old_value, s->registers[ID(GPFEN0)]);

			break;
    //***************************************************************************************
    case GPFEN1:
      old_value = s->registers[ID(GPFEN1)]; 
      s->registers[ID(GPFEN1)] |= value; 
      //g_print("old value: 0x%08X value: 0x%08X\n",  old_value, s->registers[ID(GPFEN1)]);

      break;
    //***************************************************************************************
  	default:
      //g_print("Incorrect addr: 0x%08X, val: %ld, size: %d\n", (unsigned int)(addr & 0xFFFFFFFFUL), value, size);
      break;
  }

  if (s->connection) {
    gchar message[26];
    comando = 0;
    sprintf(message, "%X%08X%08X%08X", comando,s->registers[ID(GPFSEL2)],s->registers[ID(GPFSEL1)],s->registers[ID(GPFSEL0)]);
    soup_websocket_connection_send_text (s->connection, (const char*)message);
    //printf("Enviando configuracion actual del HW GPIO al Simulador\n"); 
    gchar message2[18];
    comando = 2;
    sprintf(message2, "%X%08X%08X", comando,s->registers[ID(GPLEV1)],s->registers[ID(GPLEV0)]);
    soup_websocket_connection_send_text (s->connection, (const char*)message2);
  } 

}

// *Some* sources say the memory region is 32-bit.  */
static const MemoryRegionOps bcm2835_gpio_ops = {
  .read = bcm2835_gpio_read,
  .write = bcm2835_gpio_write,
  .endianness = DEVICE_NATIVE_ENDIAN,
};

//-------------------------------------------------------------------------------
static void bcm2835_gpio_reset(struct bcm2835_gpio_s *s) {

  //Fill the registers default value
  printf("Gpio reset\n");
  bzero(s->registers, GPIO_REGISTERS_LEN*4);
}

//-------------------------------------------------------------------------------
static void bcm2835_gpio_init(Object *obj) {

  bcm2835_gpif_s *d = BCM2835_GPIO(obj);
  SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
  GError* error;

  //qdev_init_gpio_in(DEVICE(sbd), bcm2835_gpio_set_irq, 54);
  int i;
  for (i = 0; i < 4; i++){
  	sysbus_init_irq(sbd, &d->bcm2835_1.irq[i]);
  }

  memory_region_init_io(&d->iomem, obj, &bcm2835_gpio_ops, &d->bcm2835_1, "bcm2835.gpio", 0x1000);
  sysbus_init_mmio(sbd, &d->iomem);
  
  d->server = soup_server_new (SOUP_SERVER_SERVER_HEADER, "bcm2835_gpio", NULL);
  soup_server_listen_all (d->server, 1700, 0, &error);
  soup_server_add_websocket_handler (d->server, NULL, NULL, NULL, server_websocket_callback, d, NULL);


  printf("GPIO loaded\n");
  
}

//-------------------------------------------------------------------------------
static void bcm2835_gpio_realize(DeviceState *dev, Error **errp) {

}

//------------------------------------------------------------------------------
static void bcm2835_gpif_reset(DeviceState *dev) {
  
  bcm2835_gpif_s *s = BCM2835_GPIO(dev);
  //g_print("Dir mem puntero s en reset: %08lX\n", (unsigned long)(s));
 
  bcm2835_gpio_reset(&s->bcm2835_1);
}

//------------------------------------------------------------------------------
static Property bcm2835_gpio_properties[] = {
  /*   DEFINE_PROP_INT32("mpu_model", struct bcm2835_gpif_s, mpu_model, 0),
      DEFINE_PROP_PTR("clk", struct bcm2835_gpif_s, clk),
      DEFINE_PROP_END_OF_LIST(),*/
};

//--------------------------------------------------------------------------------  
static void bcm2835_gpio_class_init(ObjectClass *klass, void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);
  //GPIODeviceClass *k = GPIO_DEVICE_CLASS(klass);
  dc->realize = bcm2835_gpio_realize;
  dc->reset = bcm2835_gpif_reset;
  dc->props = bcm2835_gpio_properties;
  /* Reason: pointer property "clk" */
  dc->cannot_instantiate_with_device_add_yet = true;

  printf("GPIO init\n");
}
//---------------------------------------------------------------------------------- 
static const TypeInfo bcm2835_gpio_info = {
  .name          = TYPE_BCM2835_GPIO,
  .parent        = TYPE_SYS_BUS_DEVICE,
  .instance_size = sizeof(struct bcm2835_gpif_s),
  .instance_init = bcm2835_gpio_init,
  .class_init    = bcm2835_gpio_class_init,
};
 
static void bcm2835_gpio_register_types(void) {
  type_register_static(&bcm2835_gpio_info);
  printf("GPIO register\n");
}
 
type_init(bcm2835_gpio_register_types)
