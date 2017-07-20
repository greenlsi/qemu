#include "hw/ssi/mfc522_adc_spi.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define DBGprintf(a) printf("%s:%d:%s:", __FILE__, __LINE__, __func__); printf(a)


/////////////////////////////////////////////////////////////////////////////////////
// Websocket communication functions
/////////////////////////////////////////////////////////////////////////////////////

/*static void
print_chx (struct mfrc522_spi_s *s)
{

  guint i;
  for (i=0 ; i<4; i++) {
    //g_print("%d: %f\n", i, s->CHX[i]);
  }
}*/

static void
rx_message (SoupWebsocketConnection *self,
            gint                     type,
            GBytes                  *message,
            gpointer                 user_data)
{

  struct mfrc522_spi_s *s = (struct mfrc522_spi_s *) user_data;
  struct uid *u = &s->uid_1;

  guint i;
  GByteArray *msg = g_bytes_unref_to_array (message);

  //g_print("\nRFID Message received (%d bytes): ", msg->len);

  for (i=0 ; i<msg->len; i++) {
    if(msg->data[i]< 0x40){
      msg->data[i] = msg->data[i] - 0x30;
    }
    else if(msg->data[i]> 0x40){
      msg->data[i] = msg->data[i] - 0x37;
    }

    //g_print("%01X ", msg->data[i]);
  }

  for (i=0 ; i<msg->len; i++) {
    //g_print(" %02X ", msg->data[i]);
  }

  //g_print("\n");

  //g_print("CHANNEL%i = %f\n", s->ch, s->CHX[s->ch]);

  //Reconocer si ya hay tarjeta, si la hay ver si es la que se quiere retirar y si no la hay asignala al 
  uint8_t data_uid[4];
  data_uid[0] = (msg->data[0] << 4) + msg->data[1];
  data_uid[1] = (msg->data[2] << 4) + msg->data[3];
  data_uid[2] = (msg->data[4] << 4) + msg->data[5];
  data_uid[3] = (msg->data[6] << 4) + msg->data[7];  

  u->size = 4;

  guint eq = 0;
  s->read_done = 0;

  if (s->card_present){
    for (i=0; i<4; i++){
      if (data_uid[i] == u->uidByte[i]){
        eq++;
      }
    }
    if (eq == 4 && msg->data[8] == 0){ //La tarjeta a la que accedo es la que está colocada
      for (i=0; i<4; i++){
        u->uidByte[i] = 0x00;
      }
      s->card_present = 0;
    }
  } else {
    if (msg->data[8] == 1){
      for (i=0; i<4; i++){
          u->uidByte[i] = data_uid[i];
      }
      s->card_present = 1;
    }
  }

  //g_print("Card present: %i \nUID: %02X%02X%02X%02X", s->card_present, u->uidByte[0], u->uidByte[1], u->uidByte[2], u->uidByte[3]);

  gchar data_send[10];
  sprintf(data_send, "%i%02X%02X%02X%02X", s->card_present, u->uidByte[0], u->uidByte[1], u->uidByte[2], u->uidByte[3]);
  soup_websocket_connection_send_text (s->connection, (const char*)data_send);
  
}

static void
error_message (SoupWebsocketConnection *self,
               GError                  *error,
               gpointer                 user_data)
{

  //g_print("Error message: %s\n", error->message);
}

static void
close_message (SoupWebsocketConnection *self,
               gpointer                 user_data)
{
  struct mfrc522_spi_s *s = (struct mfrc522_spi_s *) user_data;

  if (s->connection) {
    g_object_unref(s->connection); 
    s->connection = 0;
  }
  //g_print("Closed message\n");
}

static void
server_websocket_callback (SoupServer               *server,
                           SoupWebsocketConnection  *connection,
                           const char               *path,
                           SoupClientContext        *client,
                           gpointer                  user_data) 
{
  
  struct mfrc522_spi_s* s = (struct mfrc522_spi_s *) user_data;
  struct uid *u = &s->uid_1;

  //g_print("\nRFID Received Connection from client %s!\n", soup_websocket_connection_get_origin(connection));

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

  gchar data[10];
  sprintf(data, "%i%02X%02X%02X%02X", s->card_present, u->uidByte[0], u->uidByte[1], u->uidByte[2], u->uidByte[3]);
  soup_websocket_connection_send_text (s->connection, (const char*)data);
}

/////////////////////////////////////////////////////////////////////////////////////
// Mealy State Machine
/////////////////////////////////////////////////////////////////////////////////////

/*
fsm_t*
fsm_new (int state, fsm_trans_t* tt, void* user_data)
{
  fsm_t* this = (fsm_t*) malloc (sizeof (fsm_t));
  fsm_init (this, state, tt, user_data);
  return this;
}

void
fsm_init (fsm_t* this, int state, fsm_trans_t* tt, void* user_data)
{
  this->current_state = state;
  this->tt = tt;
  this->user_data = user_data;
}

void
fsm_destroy (fsm_t* this)
{
  free(this);
}

void
fsm_fire (fsm_t* this)
{
  fsm_trans_t* t;
  for (t = this->tt; t->orig_state >= 0; ++t) {
    if ((this->current_state == t->orig_state) && t->in(this)) {
      this->current_state = t->dest_state;
      if (t->out)
        t->out(this);
      break;
    }
  }
}

enum fsm_state {
  IDLE 
};

int event_idle(fsm_t* this) {
}

fsm_trans_t fsm_tt[] = {
  //{ IDLE, event_idle, IDLE, StopComand },
  { -1, NULL, -1, NULL } 
};
*/

/////////////////////////////////////////////////////////////////////////////////////
// FIFO Buffer Handler
/////////////////////////////////////////////////////////////////////////////////////
static int
fifo_full(mfrc522_spi_s* s)
{
  return ((s->registers_rfid[FIFOLevelReg] & 0x7F) == 64);
}

static int
fifo_empty(mfrc522_spi_s* s)
{
  return ((s->registers_rfid[FIFOLevelReg] & 0x7F) == 0);
}

static void 
fifo_write(mfrc522_spi_s* s)
{
  if (fifo_full(s)){
    return;
  }
  s->fifo_rfid[s->i_fifo_rfid_w] = s->registers_rfid[FIFODataReg];
  s->i_fifo_rfid_w++;
  s->i_fifo_rfid_w %= 64;
  s->registers_rfid[FIFOLevelReg]++;
}

static void 
fifo_read(mfrc522_spi_s* s)
{
  if (fifo_empty(s)){
    return;
  }
  s->registers_rfid[FIFODataReg] = s->fifo_rfid[s->i_fifo_rfid_r];
  s->i_fifo_rfid_r++;
  s->i_fifo_rfid_r %= 64;
  s->registers_rfid[FIFOLevelReg]--;
}

static void 
fifo_clear(mfrc522_spi_s* s)
{
  int m;
  s->i_fifo_rfid_w = 0;
  s->i_fifo_rfid_r = 0;
  for(m=0; m<64; m++){
    s->fifo_rfid[m] = 0;
  } 
  s->registers_rfid[FIFOLevelReg] = 0x00;
}



/////////////////////////////////////////////////////////////////////////////////////
// RFID MFRC522 Device Funtions
/////////////////////////////////////////////////////////////////////////////////////

static uint64_t mfrc522_spi_read(void *opaque, hwaddr addr, unsigned size) {

  struct mfrc522_spi_s *s = (struct mfrc522_spi_s *) opaque;

  if (addr > DC) {//si la drireccion es mayor
    BCM2835_BAD_REG(addr);
  }

  if (addr % 4) { 
    BCM2835_BAD_REG(addr);
  }

  switch ((unsigned int)(addr & 0xFFFFFFFFUL)) {
    case FIFO:
      if (s->index_out_w != s->index_out_r ){
        return s->data_out[s->index_out_r++];
      }
    default:
      return s->registers_spi[ID(addr)];
    }

  return 0;
}


static void mfrc522_spi_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {

  struct mfrc522_spi_s *s = (struct mfrc522_spi_s *) opaque;
  struct uid *u = &s->uid_1;

  int i,j = 0;
  //uint8_t buffer[4];

  switch((unsigned int)(addr & 0xFFFFFFFFUL)){
    case CS:
      if (s->rw && s->reg == FIFODataReg && !(s->finish)){
        return;
      }
      s->registers_spi[ID(CS)] &= value;
      if (value & CS_CLEAR){
        s->registers_spi[ID(FIFO)] &= 0X00000000;
        for(i=0; i<4; i++){
          s->data_in[i] = 0;
          s->data_out[i] = 0;
        }
        s->index_in = 0;
        s->index_out_w = 0;
        s->index_out_r = 0;
        s->registers_spi[ID(CS)] &= ~CS_CLEAR;
      }
      if (value & CS_TA){
        s->registers_spi[ID(CS)] |= CS_TXD;
        s->registers_spi[ID(CS)] |= CS_RXD;
      }
      break;
    case FIFO:
      if (s->index_in == 0){  //First Byte Received -> Command
        s->reg = (value & 0x7E) >> 1;
        s->rw = (value & 0x80) >> 7;
        s->data_out[s->index_out_w] = 0;
        s->index_out_w++;
        s->index_in++;
        //g_print("Primer dato escrito en FIFO: registro %02X -- r/w %02X \n", s->reg, s->rw);
        //g_print("Value %02X\n", value);
      } 
      else {
        if (s->rw) {          //Read operation
          //g_print("Lectura \n");
          //g_print("registro %02X -- r/w %02X \n", s->reg, s->rw);
          if (s->reg == FIFODataReg){
            //printf("HAGO LECTURAS DE FIFO\n");
            fifo_read(s); 
          }
          s->data_out[s->index_out_w++] = s->registers_rfid[s->reg];
          if (value != 0){
            s->reg = (value & 0x7E) >> 1;
            s->rw = (value & 0x80) >> 7;
            s->index_in++;
            s->finish = 0;
          } else {
            if (s->rw && s->reg == FIFODataReg){
              s->finish = 1;
            }
          }       
        } 
        else {                //Write operation
          //g_print("Escritura: ");
          //g_print("registro %02X -- r/w %02X \n", s->reg, s->rw);
          s->registers_rfid[s->reg] = value;
          if (s->reg == FIFODataReg){
            fifo_write(s); 
          }
          if (s->reg == FIFOLevelReg && s->registers_rfid[FIFOLevelReg] & 0x80){
            fifo_clear(s);
          }

          switch (s->registers_rfid[CommandReg]){
            case PCD_Idle:
              // Nothing to do, only stop any active command
              s->registers_rfid[DivIrqReg] &= 0xFB;
              break;

            case PCD_Mem:
              for (j=0; j<25; j++){
              fifo_read(s);
              }
              break;
            case PCD_GenerateRandomID:
            case PCD_CalcCRC:
              while (!fifo_empty(s)){
                fifo_read(s);
              }
              s->registers_rfid[CRCResultRegL] = 0xAA; //****
              s->registers_rfid[CRCResultRegH] = 0xAA; //****
              s->registers_rfid[DivIrqReg] |= 0x04;
              break;

            case PCD_Transmit:
            case PCD_NoCmdChange:
            case PCD_Receive:
              break;
            case PCD_Transceive:
              //printf("BitFramingReg FIFO: %02X\n", s->registers_rfid[BitFramingReg]);
              if (s->registers_rfid[BitFramingReg] & 0x80){
                if (!fifo_empty(s)){
                  fifo_read(s);
                  //return;
                }
                s->registers_rfid[ComIrqReg] = 0x20;
                //printf("DataReg FIFO: %02X\n", s->registers_rfid[FIFODataReg]);
                switch (s->registers_rfid[FIFODataReg]){
                  case PICC_CMD_REQA:
                    if (s->card_present && !(s->read_done)){
                      s->registers_rfid[FIFODataReg] = 0xFF; //****
                      fifo_write(s);
                      s->registers_rfid[FIFODataReg] = 0xFF; //****
                      fifo_write(s);
                    } else {
                      //Wait to launch STATUS_TIMEOUT
                    }
                    break;
                  case PICC_CMD_SEL_CL1:
                    fifo_read(s);
                    if (s->registers_rfid[FIFODataReg] == 0x20){
                      for (j=0; j<4; j++){
                        s->registers_rfid[FIFODataReg] = u->uidByte[j];
                        fifo_write(s);
                      }
                      s->registers_rfid[FIFODataReg] = 0xFF;
                    } 
                    else if (s->registers_rfid[FIFODataReg] == 0x70){
                      for (j=0; j<4; j++){
                        fifo_read(s);
                        //buffer[j] = s->registers_rfid[FIFODataReg];
                      }
                      fifo_read(s);
                      if(u->size > 4){
                        s->registers_rfid[FIFODataReg] = 0x04;
                      } else {
                        s->registers_rfid[FIFODataReg] = 0x00; 
                      }
                      fifo_write(s);
                      fifo_read(s);
                      //s->registers_rfid[FIFODataReg] = 0xAA;  //****
                      fifo_write(s);
                      fifo_read(s);
                      //s->registers_rfid[FIFODataReg] = 0xAA;  //****
                      fifo_write(s);
                      s->read_done = 1;
                    }
                    else {
                    }
                    break;
                  case PICC_CMD_SEL_CL2:
                    break;
                  default:
                    break;
                }
              } 
              s->registers_rfid[ControlReg] &= 0xF8;
              s->registers_rfid[ErrorReg] = 0x00;
              s->registers_rfid[BitFramingReg] &= 0x7F;
              break;

            case PCD_MFAuthent:
            case PCD_SoftReset:
            default:
              break;
          }

          s->data_out[s->index_out_w] = 0;
          s->index_out_w++;
          s->index_in++;
          //fsm_fire(fsm);
          //printf("LevelReg FIFO: %02X\n", s->registers_rfid[FIFOLevelReg] & 0x7F);
          //printf("Valores de indices fifo: write->%i read->%i\n", s->i_fifo_rfid_w, s->i_fifo_rfid_r);
        }
      }
      s->registers_spi[ID(CS)] |= CS_DONE;
      break;
    case CLK:
      s->registers_spi[ID(CLK)] = value;
      break;
    default:
      //g_print("Incorrect addr: 0x%08X, val: %ld, size: %d\n", (unsigned int)(addr & 0xFFFFFFFFUL), value, size);
      break;
  }
}


static const MemoryRegionOps mfrc522_spi_ops = {
  .read = mfrc522_spi_read,
  .write = mfrc522_spi_write,
  .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_rfid = {
    .name = TYPE_MFRC522_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};


static void mfrc522_spi_init(Object *obj) {
  mfrc522_spi_s *d = MFRC522_SPI(obj);
  SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
  GError* error;
  DBGprintf(("\n"));

  //s->fsm = fsm_new(IDLE, fsm_tt, s->reg);


  //sysbus_init_irq(sbd, &d->irq);

  memory_region_init_io(&d->iomem, obj, &mfrc522_spi_ops, d, "bcm2835.rfid", 0x1000);
  sysbus_init_mmio(sbd, &d->iomem);
  DBGprintf(("\n"));

  d->server = soup_server_new (SOUP_SERVER_SERVER_HEADER, "mfrc522_adc", NULL);
  soup_server_listen_all (d->server, 1800, 0, &error);
  soup_server_add_websocket_handler (d->server, NULL, NULL, NULL, server_websocket_callback, d, NULL);
  
}

static void mfrc522_spi_realize(DeviceState *dev, Error **errp) {
}

static void mfrc522_spi_reset(DeviceState *dev) {
  
  DBGprintf(("\n"));
  mfrc522_spi_s *s = MFRC522_SPI(dev);
  struct uid *u = &s->uid_1;
  //Fill the registers default value
  printf("mfrc522 reset\n");
  bzero(s->registers_spi, SPI_REGISTERS_LEN*4);
  bzero(s->registers_spi, RFID_REGISTERS_LEN);
  s->registers_spi[ID(CS)] &= 0x41000; 
  s->registers_spi[ID(LTOH)] &= 0x1; 
  s->registers_spi[ID(DC)] &= 0x30201020; 

  s->card_present = 0;
  int i;
  for(i=0; i<10; i++){
    u->uidByte[i] = 0;
  }
  u->size = 4;
  s->finish = 0;
  s->read_done = 0;

  fifo_clear(s);

  DBGprintf(("\n"));
}


static Property mfrc522_spi_properties[] = {
};

static void mfrc522_spi_class_init(ObjectClass *klass, void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = mfrc522_spi_realize;
  dc->reset = mfrc522_spi_reset;
  dc->props = mfrc522_spi_properties;
  dc->vmsd = &vmstate_rfid;

  dc->cannot_instantiate_with_device_add_yet = true;
  DBGprintf(("\n"));
}

static const TypeInfo mfrc522_spi_info = {
  .name          = TYPE_MFRC522_SPI,
  .parent        = TYPE_SYS_BUS_DEVICE,
  .instance_size = sizeof(struct mfrc522_spi_s),
  .instance_init = mfrc522_spi_init,
  .class_init    = mfrc522_spi_class_init,
};
 
static void mfrc522_spi_register_types(void) {
  type_register_static(&mfrc522_spi_info);
  printf("mfrc522 register\n");
}
 
type_init(mfrc522_spi_register_types)
