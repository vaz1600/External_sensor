/*
 * RF24_mesh.c
 *
 *  Created on: 20 янв. 2025 г.
 *      Author: bryki
 */
#include "RF24_mesh.h"
#include "RF24.h"
#include "main.h"

uint16_t node_address = 0x0924;

uint8_t _multicast_level = 0;
uint8_t parent_pipe = 0;
uint16_t node_mask = 0;
uint16_t parent_node;

uint8_t frame_buffer[32];
uint8_t frame_size = 32;

static uint16_t next_id = 0;

typedef struct
{
    uint16_t from_node;
    uint16_t to_node;
    uint16_t id;
    unsigned char type;
    unsigned char reserved;
} RF24NetworkHeader;

uint64_t pipe_address(uint16_t node, uint8_t pipe);
void setup_address(void);
uint8_t write_to_pipe(uint16_t node, uint8_t pipe, uint8_t multicast);
uint8_t mesh_SendFrame(RF24NetworkHeader *h, uint8_t *buf, uint8_t size, uint8_t multicast);

void mesh_Init(void)
{
    if( NRF_Init() == 0)
    {
        while(1)
        {
            LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);
            HAL_Delay(200);
            LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);
            HAL_Delay(200);
        }
    }

    stopListening();

    setChannel(97);

    //mesh addr 0x0924
    setAutoAck(1);
    setAutoAckPipe(0, 0);
    enableDynamicPayloads();


    mesh_Begin(0x0924);
    parent_pipe = 0;
}

void mesh_Begin(uint16_t addr)
{
    node_address = addr;

    // Use different retry periods to reduce data collisions
    uint8_t retryVar = (((node_address % 6) + 1) * 2) + 3;
    setRetries(retryVar, 5); // max about 85ms per attempt
    //txTimeout = 25;
    //routeTimeout = txTimeout * 3; // Adjust for max delay per node within a single chain

    // Setup our address helper cache
    setup_address();

    // Open up all listening pipes
    uint8_t i = 6;
    while (i--)
    openReadingPipe(i, pipe_address(node_address, i));

    startListening();
}

uint8_t mesh_AddressRequest(void)
{
    RF24NetworkHeader h;
    RF24NetworkHeader *hptr;
    uint8_t poll;
    uint16_t contact_node = 0;

    powerUp();

    // poll network
    for(poll = 0; poll < 3; poll++)
    {
        h.from_node = node_address;
        h.to_node = 0x0040;
        h.id = next_id++;
        h.type = 0xC2;
        h.reserved = 0;

        if(mesh_SendFrame(&h, 0, 0, 1))
        {
            hptr = (RF24NetworkHeader *)frame_buffer;
            if(hptr->type == 0xC2)
            {
                contact_node = hptr->from_node;
                break;
            }
        }
    }

    if (poll == 3)
        return 0;

    for(poll = 0; poll < 4; poll++)
    {
        h.from_node = node_address;
        h.to_node = contact_node;
        h.id = next_id++;
        h.type = 0xC3;          //address request
        h.reserved = 0x01; // node id

        if(mesh_SendFrame(&h, 0, 0, 1))
        {
            hptr = (RF24NetworkHeader *)frame_buffer;
            if(hptr->type == 0x80) //network address response
            {
                if(hptr->reserved == 0x01)//==node id
                {
                    memcpy(&node_address, &frame_buffer[8], 2);

                    parent_pipe = 5;
                    mesh_Begin(node_address);

                    return 1;
                }
            }
        }
    }

    return 0;
}

uint8_t mesh_Lookup(void)
{
    RF24NetworkHeader h;
    RF24NetworkHeader *hptr;
    uint8_t poll;
    uint8_t buf[2];

    //mesh lookup
    for(poll = 0; poll < 3; poll++)
    {
        h.from_node = node_address;
        h.to_node = 0x0000;
        h.id = next_id++;
        h.type = 0xC6;
        h.reserved = 0;

        memcpy(buf, &node_address, 2);

        if(mesh_SendFrame(&h, buf, 2, 0))
        {
            hptr = (RF24NetworkHeader *)frame_buffer;
            if(hptr->type == 0xC6)
            {
                //contact_node = hptr->from_node;
                break;
            }
        }
    }

    if (poll == 3)
        return 0;
    else
        return 1;
}

uint8_t mesh_SendFrame(RF24NetworkHeader *h, uint8_t *buf, uint8_t size, uint8_t multicast)
{
    uint8_t ok;

    LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);

    memcpy(frame_buffer, h, 8);

    if(buf)
        memcpy(frame_buffer + 8, buf, size);

    frame_size = 8 + size;

    ok = write_to_pipe(0, parent_pipe, multicast);

    uint32_t timeout = HAL_GetTick() + 120;

    startListening();

    // timeout
    while(HAL_GetTick() < timeout)
    {
        if(availableMy())
        {
            frame_size = getDynamicPayloadSize();
            read(frame_buffer, frame_size);

            LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);
            return 1;
        }
    }

    LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);
    return 0;
}

uint8_t mesh_Write(uint8_t type, uint8_t *buf, uint8_t size)
{
    RF24NetworkHeader h;

    h.from_node = node_address;
    h.to_node = 0x0000;
    h.id = next_id++;
    h.type = type;
    h.reserved = 0x3F;

    uint8_t ok;

    LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_11);

    powerUp();

    memcpy(frame_buffer, &h, 8);

    if(buf)
        memcpy(frame_buffer + 8, buf, size);

    frame_size = 8 + size;

    ok = write_to_pipe(0, parent_pipe, 0);

    startListening();

    LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_11);

    return ok;
}

uint8_t mesh_Read(uint8_t *type, uint8_t *buf, uint8_t size)
{
    startListening();

    //availableMy
    return 0;
}
#if 0
   // address request
   hdr = (RF24NetworkHeader *)frame_buffer;

   hdr->from_node = 0x0924;
   hdr->to_node = 0x0000;
   hdr->id = next_id++;
   hdr->type = 0xC3;
   hdr->reserved = 0x01; // node address

   frame_size = 8;

   write_to_pipe(0, 0, 1);

   startListening();

   while(availableMy() == 0);

   frame_size = getDynamicPayloadSize();
   read(frame_buffer, frame_size);

   memcpy(&node_address, &frame_buffer[8], 2);

   //begin(node_address)
   // Use different retry periods to reduce data collisions
   retryVar = (((node_address % 6) + 1) * 2) + 3;
   setRetries(retryVar, 5); // max about 85ms per attempt
   //txTimeout = 25;
   //routeTimeout = txTimeout * 3; // Adjust for max delay per node within a single chain

   // Setup our address helper cache
   setup_address();

   // Open up all listening pipes
   i = 6;
   while (i--)
     openReadingPipe(i, pipe_address(node_address, i));

   startListening();


   HAL_Delay(10);
#endif



uint64_t pipe_address(uint16_t node, uint8_t pipe)
{

   static uint8_t address_translation[] = { 0xc3,
                                            0x3c,
                                            0x33,
                                            0xce,
                                            0x3e,
                                            0xe3,
                                            0xec
   };
   uint64_t result = 0xCCCCCCCCCCLL;
   uint8_t* out = (uint8_t*)(&result);

   // Translate the address to use our optimally chosen radio address bytes
   uint8_t count = 1;
   uint16_t dec = node;

   while (dec) {

       if (pipe != 0 || !node)

           out[count] = address_translation[(dec % 8)]; // Convert our decimal values to octal, translate them to address bytes, and set our address

       dec /= 8;
       count++;
   }


   if (pipe != 0 || !node)
       out[0] = address_translation[pipe];
   else
       out[1] = address_translation[count - 1];

   return result;
}


void setup_address(void)
{
   // First, establish the node_mask
   uint16_t node_mask_check = 0xFFFF;

   uint8_t count = 0;


   while (node_address & node_mask_check) {
       node_mask_check <<= 3;

       count++;
   }
   _multicast_level = count;


   node_mask = ~node_mask_check;

   // parent mask is the next level down
   uint16_t parent_mask = node_mask >> 3;

   // parent node is the part IN the mask
   parent_node = node_address & parent_mask;

   // parent pipe is the part OUT of the mask
   uint16_t i = node_address;
   uint16_t m = parent_mask;
   while (m) {
       i >>= 3;
       m >>= 3;
   }
   parent_pipe = i;
}


uint8_t write_to_pipe(uint16_t node, uint8_t pipe, uint8_t multicast)
{
   uint8_t ok = false;

   stopListening();

   setAutoAckPipe(0, !multicast);
   openWritingPipe(pipe_address(node, pipe));
#if 0
   ok = write(frame_buffer, frame_size);

   setAutoAckPipe(0, 0);
#else
   ok = writeFast(frame_buffer, frame_size, 1);

   ok = txStandBy(85, 0);
   setAutoAckPipe(0, 0);

   if(!ok)
   {
       ok = txStandBy(85, 0);
   }
   /*
   ok = writeFast(frame_buffer, frame_size, 0);

   ok = txStandBy(85);
   setAutoAckPipe(0, 0);
*/
//    if (!(networkFlags & FLAG_FAST_FRAG)) {
//        ok = txStandBy(txTimeout);
//                setAutoAck(0, 0);
//    }
//    else if (!ok) {
//        ok = txStandBy(txTimeout);
//    }
#endif
   return ok;
}
