#ifndef __CONTORL_MESSAGES_H
#define __CONTORL_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

// Special Character Definitions
#define HEADING_START (0x01)
#define TRANSMISSION_END (0x04)

// Enum Definitions for fields
typedef enum {
    MTR_MVMNT_NONE = 0,
    MTR_MVMNT_FWD  = 1,
    MTR_MVMNT_REV  = 2,
} MTR_MVMNT;

typedef enum {
    SOL_MVMNT_NONE = 0,
    SOL_MVMNT_IN   = 1,
    SOL_MVMNT_OUT  = 2,
} SOL_MVMNT;

typedef enum {
    PCKT_TYPE_SETUP = 0,
    PCKT_TYPE_INDEX,
    PCKT_TYPE_MVMNT_FIN,
    PCKT_TYPE_ERROR,
    PCKT_TYPE_STOP // Keep as last item
} PCKT_TYPE;

// Packet structures
typedef struct {
    char      StartOfHeader;
    uint32_t  PacketSize;
    uint8_t   PacketType;
    char      EndOfHeader;
} PACKET_HEADER;

typedef struct {
    char      StartOfPacket;
    uint8_t   Motor;
    MTR_MVMNT Mvmnt;
    char      EndOfPacket;
} PACKET_MTR;

typedef struct {
    char      StartOfPacket;
    uint8_t   Solenoid;
    SOL_MVMNT Mvmnt;
    char      EndOfPacket;
} PACKET_SOL;

typedef struct {
    char Tail;
    char NullTerm;
} PACKET_TAIL;

// Public Functions
// Packet Formation Functions

/**
    Creates a packet given a number of motor/solenoid movements.
    Setup such that the index
**/

bool CreateIndexPacket(uint8_t* dataOut, int* dataSize, 
                       PACKET_MTR* mtrPckt, int numMtrPckt, 
                       PACKET_SOL* solPckt, int numSolPckt);
PACKET_MTR CreateMtrPacket(uint8_t mtr, MTR_MVMNT mvmnt);
PACKET_SOL CreateSolPacket(uint8_t sol, SOL_MVMNT mvmnt);

// Packet Parsing Functions
bool ParseMtrPacket(PACKET_MTR, uint8_t* mtr, MTR_MVMNT* mvmnt);
bool ParseSolPacket(PACKET_SOL, uint8_t* sol, SOL_MVMNT* mvmnt);

/**
  * Series of functions to put packet data into a buffer.
  * \param dataStart[In] - Start of buffer 
  * \param dataPtr[Out] - Pointer in buffer after newly written data
  * \
**/
bool PacketStart(uint8_t* dataStart, uint8_t *dataPtr);
bool PacketAddMtrMvmnt(uint8_t* dataStart, uint8_t *dataPtr, MTR_MVMNT Mvmnt, uint8_t mtr);
bool PacketAddSolMvmnt(uint8_t* dataStart, uint8_t *dataPtr, SOL_MVMNT Mvmnt, uint8_t sol);
bool PacketTerm(uint8_t* dataStart, uint8_t *dataPtr);
    
#endif
