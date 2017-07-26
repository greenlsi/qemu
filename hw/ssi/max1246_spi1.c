#include "hw/ssi/max1246_spi.h"
#include <math.h>
#include <stdio.h>

#define DBGprintf(a) printf("%s:%d:%s:", __FILE__, __LINE__, __func__); printf(a)

 //max1246_spi_s* s_g = 0;

static void
print_chx (struct max1246_spi_s *s)
{

  guint i;
  for (i=0 ; i<4; i++) {
    //g_print("%d: %f\n", i, s->CHX[i]);
  }
}

static void
rx_message (SoupWebsocketConnection *self,
            gint                     type,
            GBytes                  *message,
            gpointer                 user_data)
{

  struct max1246_spi_s *s = (struct max1246_spi_s *) user_data;


  guint i;
  GByteArray *msg = g_bytes_unref_to_array (message);

  //g_print("\nADC Message received (%d bytes): ", msg->len);

  for (i=0 ; i<msg->len; i++) {
    //g_print(" %02X ", msg->data[i]);
  }
  msg->data[msg->len] = 0;
  //g_print("\n");

  float channel = 0;
  sscanf((const char*)(msg->data+1), "%f", &channel);

  if (msg->data[0] < '0' || msg->data[0] > '3') {
    //g_prints("CHANNEL incorrect: %c\n", msg->data[0]);
    return;
  }
  s->ch = msg->data[0]-0x30;
  s->CHX[s->ch] = channel;
  //g_print("CHANNEL%i = %f\n", s->ch, s->CHX[s->ch]);

  gchar data[12];
  guint k;
  for(k=0; k<4; k++){
    if(s->CHX[k] < 0){
      sprintf(data, "%d%d%f", 1, k, (s->CHX[k]*-1));
    } else {
     sprintf(data, "%d%d%f", 0, k, s->CHX[k]); 
    }
    soup_websocket_connection_send_text (s->connection, (const char*)data);
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
  struct max1246_spi_s *s = (struct max1246_spi_s *) user_data;

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
  
  struct max1246_spi_s* s = (struct max1246_spi_s *) user_data;

  //g_print("\nADC Received Connection from client %s!\n", soup_websocket_connection_get_origin(connection));

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

  gchar data[12];
  guint k;
  for(k=0; k<4; k++){
    if(s->CHX[k] < 0){
      sprintf(data, "%d%d%f", 1, k, (s->CHX[k]*-1));
    } else {
     sprintf(data, "%d%d%f", 0, k, s->CHX[k]); 
    }
    soup_websocket_connection_send_text (s->connection, (const char*)data);
  }
}

static uint64_t max1246_spi_read(void *opaque, hwaddr addr, unsigned size) {

  struct max1246_spi_s *s = (struct max1246_spi_s *) opaque;

  if (addr > DC) {//si la drireccion es mayor
    BCM2835_BAD_REG(addr);
  }

  if (addr % 4) { 
    BCM2835_BAD_REG(addr);
  }

  switch ((unsigned int)(addr & 0xFFFFFFFFUL)) {
    case FIFO:
      if (s->index_out == 2){
          s->registers[ID(CS)] |= CS_DONE;
      }
      return (s->data_out >> ((s->index_out++)*8)) & 0x00FF;
    default:
      return s->registers[ID(addr)];
    }

  return 0;
}


static void max1246_spi_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {

  struct max1246_spi_s *s = (struct max1246_spi_s *) opaque;

  float Vref_uni = 5.000;
  float Vref_bip = 2.5;
  int LSBmax_uni = 4095;
  int LSBmin_bip = -2048;
  int LSBmax_bip = 2047;
  int var = 0;
  float adc_data=0;

  switch((unsigned int)(addr & 0xFFFFFFFFUL)){
    
    case CS:
      s->registers[ID(CS)] &= value;
      if (value & CS_CLEAR){
        s->registers[ID(FIFO)] &= 0X00000000;
        s->data_in = 0;
        s->data_out = 0;
        s->index_in = 0;
        s->index_out = 0;
        s->registers[ID(CS)] &= ~CS_CLEAR;
      }
      if (value & CS_TA){
        s->registers[ID(CS)] |= CS_TXD;
        s->registers[ID(CS)] |= CS_RXD;
      }
      break;
    case FIFO:
      if (s->index_in == 0){
        // First bit high
        if (! (value & START)) {
          break;
        }
        // External clock mode
        if ((value & PD_MASK) != PD_MASK ) {
          break;
        }

        if (value & SGL_DIF) {     // Single ended mode
          switch (value & SEL_MASK) {
          case CH0:
            s->ch = 0;
            break;
          case CH1:
            s->ch = 1;
            break;
          case CH2:
            s->ch = 2;
            break;
          case CH3:
            s->ch = 3;
            break;
          }
          adc_data = s->CHX[s->ch];
          //adc_data -= s->CHX[s->ch ^ 1];
        } else {
          switch (value & SEL_MASK) {
          case DIFF1:
            adc_data = s->CHX[0] - s->CHX[1];
            break;
          case DIFF2:
            adc_data = s->CHX[2] - s->CHX[3];
            break;
          case DIFF3:
            adc_data = s->CHX[1] - s->CHX[0];
            break;
          case DIFF4:
            adc_data = s->CHX[3] - s->CHX[2];
            break;
          }
        }
        if (s->data_in & UNI_BIP){    // Unipolar mode
          var = ((LSBmax_uni + 1) * adc_data)/Vref_uni;
          var = min(var, LSBmax_uni);
          var = max(var, 0);
        } else {                      // Bipolar mode
          var = ((LSBmax_bip + 1) * adc_data)/Vref_bip;
          var = min(var, LSBmax_bip);
          var = max(var, LSBmin_bip);
        }

        s->data_out |= ((var & 0x0FF0) << 4);
        s->data_out |= ((var & 0x000F) << 20);
      }
      s->index_in++;
      break;
    case CLK:
      s->registers[ID(CLK)] = value;

    default:
      //g_print("Incorrect addr: 0x%08X, val: %ld, size: %d\n", (unsigned int)(addr & 0xFFFFFFFFUL), value, size);
      break;
  }
}


static const MemoryRegionOps max1246_spi_ops = {
  .read = max1246_spi_read,
  .write = max1246_spi_write,
  .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_bcm2835_adc = {
    .name = TYPE_MAX1246_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void max1246_spi_init(Object *obj) {
  max1246_spi_s *d = MAX1246_SPI(obj);
  SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
  GError* error;
  DBGprintf(("\n"));


  //sysbus_init_irq(sbd, &d->irq);

  memory_region_init_io(&d->iomem, obj, &max1246_spi_ops, d, "bcm2835.adc", 0x1000);
  sysbus_init_mmio(sbd, &d->iomem);
  DBGprintf(("\n"));

  d->server = soup_server_new (SOUP_SERVER_SERVER_HEADER, "max1246_adc", NULL);
  soup_server_listen_all (d->server, 1600, 0, &error);
  soup_server_add_websocket_handler (d->server, NULL, NULL, NULL, server_websocket_callback, d, NULL);
  
}

static void max1246_spi_realize(DeviceState *dev, Error **errp) {
}

static void max1246_spi_reset(DeviceState *dev) {
  
  DBGprintf(("\n"));
  max1246_spi_s *s = MAX1246_SPI(dev);
  //Fill the registers default value
  printf("MAX1246 reset\n");
  bzero(s->registers, SPI0_REGISTERS_LEN*4);
  s->registers[ID(CS)] &= 0x41000; 
  s->registers[ID(LTOH)] &= 0x1; 
  s->registers[ID(DC)] &= 0x30201020; 
  s->ch = 0;
  int i;
  for (i=0; i<4; i++){
    s->CHX[i] = 0;
  }

  print_chx(s);

  DBGprintf(("\n"));
}


static Property max1246_spi_properties[] = {
};

static void max1246_spi_class_init(ObjectClass *klass, void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = max1246_spi_realize;
  dc->reset = max1246_spi_reset;
  dc->props = max1246_spi_properties;
  dc->vmsd = &vmstate_bcm2835_adc;

  dc->cannot_instantiate_with_device_add_yet = true;
  DBGprintf(("\n"));
}

static const TypeInfo max1246_spi_info = {
  .name          = TYPE_MAX1246_SPI,
  .parent        = TYPE_SYS_BUS_DEVICE,
  .instance_size = sizeof(struct max1246_spi_s),
  .instance_init = max1246_spi_init,
  .class_init    = max1246_spi_class_init,
};
 
static void max1246_spi_register_types(void) {
  type_register_static(&max1246_spi_info);
  printf("MAX1246 register\n");
}
 
type_init(max1246_spi_register_types)
