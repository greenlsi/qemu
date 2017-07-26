#include "hw/ssi/mfc522_adc_spi.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define DBGprintf(a) printf("%s:%d:%s:", __FILE__, __LINE__, __func__); printf(a)


/////////////////////////////////////////////////////////////////////////////////////
// RFID Websocket communication functions
/////////////////////////////////////////////////////////////////////////////////////

static void
rx_message_rfid   (SoupWebsocketConnection *self,
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
      s->read_done = 0;
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
error_message_rfid  (SoupWebsocketConnection *self,
                    GError                  *error,
                    gpointer                 user_data)
{

  //g_print("Error message: %s\n", error->message);
}

static void
close_message_rfid  (SoupWebsocketConnection *self,
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
server_websocket_callback_rfid  (SoupServer               *server,
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
                    G_CALLBACK (rx_message_rfid),
                    user_data);

  g_signal_connect (connection,
                    "error",
                    G_CALLBACK (error_message_rfid),
                    user_data);

  g_signal_connect (connection,
                    "closed",
                    G_CALLBACK (close_message_rfid),
                    user_data);

  gchar data[10];
  sprintf(data, "%i%02X%02X%02X%02X", s->card_present, u->uidByte[0], u->uidByte[1], u->uidByte[2], u->uidByte[3]);
  soup_websocket_connection_send_text (s->connection, (const char*)data);
}

/////////////////////////////////////////////////////////////////////////////////////
// ADC Websocket communication functions
/////////////////////////////////////////////////////////////////////////////////////
static void
rx_message_adc  (SoupWebsocketConnection *self,
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

static void
error_message_adc (SoupWebsocketConnection *self,
                  GError                  *error,
                  gpointer                 user_data)
{

  //g_print("Error message: %s\n", error->message);
}

static void
close_message_adc (SoupWebsocketConnection *self,
                  gpointer                 user_data)
{
  struct max1246_spi_s *s = (struct max1246_spi_s *) user_data;

  if (s->connection) {
    g_object_unref(s->connection); 
    s->connection = 0;
  }
  //g_print("Closed message\n");
}

static void
server_websocket_callback_adc   (SoupServer               *server,
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
                    G_CALLBACK (rx_message_adc),
                    user_data);

  g_signal_connect (connection,
                    "error",
                    G_CALLBACK (error_message_adc),
                    user_data);

  g_signal_connect (connection,
                    "closed",
                    G_CALLBACK (close_message_adc),
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


/////////////////////////////////////////////////////////////////////////////////////
// RFID FIFO Buffer Handler
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
// SPI Interface Funtions
/////////////////////////////////////////////////////////////////////////////////////

static uint64_t raspi_spi_read(void *opaque, hwaddr addr, unsigned size) {

  struct raspi_spi_s *s = (struct raspi_spi_s *) opaque;

  if (addr > DC) {//si la drireccion es mayor
    BCM2835_BAD_REG(addr);
  }

  if (addr % 4) { 
    BCM2835_BAD_REG(addr);
  }

  switch ((unsigned int)(addr & 0xFFFFFFFFUL)) {
    case FIFO:
      //Add functions
    int cs = s->registers_spi[ID(CS)] & CS_CS;
      if (cs < 2 && s->ch[cs]) {
        s->ch[cs]->read(s->ch[cs].opaque, size);
      }
    default:
      return s->registers_spi[ID(addr)];
    }

  return 0;
}


static void raspi_spi_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {

  struct raspi_spi_s *s = (struct raspi_spi_s *) opaque;
  int cs = s->registers_spi[ID(CS)] & CS_CS;

  int i,j = 0;
  //uint8_t buffer[4];

  switch((unsigned int)(addr & 0xFFFFFFFFUL)){
    case CS:
      s->registers_spi[ID(CS)] &= value;
      if (value & CS_CLEAR){
        s->registers_spi[ID(FIFO)] &= 0X00000000;
        if (cs < 2 && s->ch[cs]) {
          s->ch[cs]->cs_write(s->ch[cs].opaque, value, size);
        }
        s->registers_spi[ID(CS)] &= ~CS_CLEAR;
      }
      if (value & CS_TA){
        s->registers_spi[ID(CS)] |= CS_TXD;
        s->registers_spi[ID(CS)] |= CS_RXD;
      }
      break;
    case FIFO: 
      //Add functions
      if (cs < 2 && s->ch[cs]) {
        s->ch[cs]->write(s->ch[cs].opaque, value, size);
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

/////////////////////////////////////////////////////////////////////////////////////
// RFID MFRC522 Device Funtions
/////////////////////////////////////////////////////////////////////////////////////

static uint64_t mfrc522_read(void *opaque, unsigned size) {

  struct mfrc522_spi_s *s = (struct mfrc522_spi_s *) opaque;

  if (s->index_out_w != s->index_out_r ){
    return s->data_out[s->index_out_r++];
  }
  return 0;
}

static void mfc522_cs_write(void *opaque, uint64_t value, unsigned size) {

  struct mfrc522_spi_s *s = (struct mfrc522_spi_s *) opaque;
  struct uid *u = &s->uid_1;

  int i,j = 0;

  if (s->rw && s->reg == FIFODataReg && !(s->finish)){
        return;
  }
  for(i=0; i<4; i++){
    s->data_in[i] = 0;
    s->data_out[i] = 0;
  }
  s->index_in = 0;
  s->index_out_w = 0;
  s->index_out_r = 0;

}

static void mfc522_write(void *opaque, uint64_t value, unsigned size) {

  struct mfrc522_spi_s *s = (struct mfrc522_spi_s *) opaque;
  struct uid *u = &s->uid_1;

  int i,j = 0;
 
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

}

static void mfrc522_init(void *opaque) {

  mfrc522_spi_s *d = (struct mfrc522_spi_s *) opaque;

  d->server = soup_server_new (SOUP_SERVER_SERVER_HEADER, "mfrc522_adc", NULL);
  soup_server_listen_all (d->server, 1800, 0, &error);
  soup_server_add_websocket_handler (d->server, NULL, NULL, NULL, server_websocket_callback_rfid, d, NULL);
  
}

static void mfrc522_reset(void *opaque) {
  
  DBGprintf(("\n"));
  struct mfrc522_spi_s *s = (struct mfrc522_spi_s *) opaque;
  struct uid *u = &s->uid_1;
  //Fill the registers default value
  printf("mfrc522 reset\n");
  bzero(s->registers_spi, RFID_REGISTERS_LEN);

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

struct mfrc522_spi_s mfrc522_s;

static spi_channel_s mfrc522_spi_ch = {
  .init = mfrc522_init,
  .reset = mfrc522_reset,
  .read = mfrc522_read,
  .write = mfrc522_write,
  .cs_write = mfc522_cs_write,
  .opaque = &mfrc522_s
};

/////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////
// ADC MAX1246 Device Funtions
/////////////////////////////////////////////////////////////////////////////////////

static uint64_t mfrc522_read(void *opaque, unsigned size) {

  struct max1246_spi_s *s = (struct max1246_spi_s *) opaque;

  if (s->index_out == 2){
      s->registers[ID(CS)] |= CS_DONE;
  }
  return (s->data_out >> ((s->index_out++)*8)) & 0x00FF;
}

static void mfc522_cs_write(void *opaque, uint64_t value, unsigned size) {

  struct max1246_spi_s *s = (struct max1246_spi_s *) opaque;

  s->data_in = 0;
  s->data_out = 0;
  s->index_in = 0;
  s->index_out = 0;

}

static void mfc522_write(void *opaque, uint64_t value, unsigned size) {

  struct max1246_spi_s *s = (struct max1246_spi_s *) opaque;

  float Vref_uni = 5.000;
  float Vref_bip = 2.5;
  int LSBmax_uni = 4095;
  int LSBmin_bip = -2048;
  int LSBmax_bip = 2047;
  int var = 0;
  float adc_data=0;

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
}

static void mfrc522_init(void *opaque) {

  max1246_spi_s *d = (struct max1246_spi_s *) opaque;

  d->server = soup_server_new (SOUP_SERVER_SERVER_HEADER, "max1246_adc", NULL);
  soup_server_listen_all (d->server, 1600, 0, &error);
  soup_server_add_websocket_handler (d->server, NULL, NULL, NULL, server_websocket_callback_adc, d, NULL);
  
}

static void mfrc522_reset(void *opaque) {

  struct max1246_spi_s *s = (struct max1246_spi_s *) opaque;

  s->ch = 0;
  int i;
  for (i=0; i<4; i++){
    s->CHX[i] = 0;
  }
}

struct max1246_spi_s max1246_s;

static spi_channel_s mfrc522_spi_ch = {
  .init = max1246_init,
  .reset = max1246_reset,
  .read = max1246_read,
  .write = max1246_write,
  .cs_write = max1246_cs_write,
  .opaque = &max1246_s
};

/////////////////////////////////////////////////////////////////////////////////////



static const MemoryRegionOps raspi_spi_ops = {
  .read = raspi_spi_read,
  .write = raspi_spi_write,
  .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_spi = {
    .name = TYPE_SPI_DEVICE,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void raspi_spi_init(Object *obj) {
  raspi_spi_s *d = SPI_DEVICE(obj);
  SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
  GError* error;
  DBGprintf(("\n"));

  //s->fsm = fsm_new(IDLE, fsm_tt, s->reg);


  //sysbus_init_irq(sbd, &d->irq);

  memory_region_init_io(&d->iomem, obj, &raspi_spi_ops, d, "bcm2835.spi", 0x1000);
  sysbus_init_mmio(sbd, &d->iomem);
  DBGprintf(("\n"));

  d->ch[0] = &mfrc522_spi_ch;
  d->ch[1] = &max1246_spi_ch;

  d->ch[0]->init(s->ch[0].opaque);
  d->ch[1]->init(s->ch[1].opaque);
}

static void raspi_spi_realize(DeviceState *dev, Error **errp) {
}

static void raspi_spi_reset(DeviceState *dev) {
  
  DBGprintf(("\n"));
  raspi_spi_s *s = SPI_DEVICE(dev);
  bzero(s->registers_spi, SPI_REGISTERS_LEN*4);
  s->registers_spi[ID(CS)] &= 0x41000; 
  s->registers_spi[ID(LTOH)] &= 0x1; 
  s->registers_spi[ID(DC)] &= 0x30201020; 

  s->ch[0]->reset(s->ch[0].opaque);
}


static Property mfrc522_spi_properties[] = {
};

static void mfrc522_spi_class_init(ObjectClass *klass, void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = raspi_spi_realize;
  dc->reset = raspi_spi_reset;
  dc->props = raspi_spi_properties;
  dc->vmsd = &vmstate_spi;

  dc->cannot_instantiate_with_device_add_yet = true;
  DBGprintf(("\n"));
}

static const TypeInfo spi_info = {
  .name          = TYPE_SPI_DEVICE,
  .parent        = TYPE_SYS_BUS_DEVICE,
  .instance_size = sizeof(struct raspi_spi_s),
  .instance_init = raspi_spi_init,
  .class_init    = raspi_spi_class_init,
};
 
static void raspi_spi_register_types(void) {
  type_register_static(&spi_info);
  printf("mfrc522 register\n");
}
 
type_init(raspi_spi_register_types)
