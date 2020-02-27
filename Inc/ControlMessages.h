#ifndef __CONTORL_MESSAGES_H
#define __CONTORL_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

#define PCKT_TX_START     ('{')
#define PCKT_TX_END       ('}')
#define PCKT_HEADER_START ('<')
#define PCKT_HEADER_END   ('>')
#define PCKT_MTR_START    ('(')
#define PCKT_MTR_END      (')')
#define PCKT_SOL_START    ('[')
#define PCKT_SOL_END      (']')

#define PCKT_SIZE_HEX_MAX (4)

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
    PCKT_TYPE_INVALID = 0,
    PCKT_TYPE_SETUP,
    PCKT_TYPE_INDEX,
    PCKT_TYPE_MVMNT_FIN,
    PCKT_TYPE_ERROR,
    PCKT_TYPE_STOP // Keep as last item
} PCKT_TYPE;

typedef enum {
    PCKT_INDEX_TYPE_INVALID,
    PCKT_INDEX_TYPE_MTR,
    PCKT_INDEX_TYPE_SOL,
} PCKT_INDEX_TYPE;

// Packet structures
typedef struct __attribute__((__packed__)){
    char      StartOfHeader;
    char      PacketSize[PCKT_SIZE_HEX_MAX + 2];
    uint8_t   PacketType;
    char      EndOfHeader;
} PACKET_HEADER;

typedef struct __attribute__((__packed__)){
    char      StartOfPacket;
    uint8_t   Motor;
    uint8_t   Mvmnt;
    char      EndOfPacket;
} PACKET_MTR;

typedef struct __attribute__((__packed__)){
    char      StartOfPacket;
    uint8_t   Solenoid;
    uint8_t   Mvmnt;
    char      EndOfPacket;
} PACKET_SOL;

typedef struct __attribute__((__packed__)){
    char Tail;
    char NullTerm;
} PACKET_TAIL;

// Public Functions

/**
  * Series of functions to put packet data into a buffer.
  * \param dataStart[In] - Start of buffer 
  * \param dataPtr[In/Out] - Reference to index into buffer
  * \
**/
bool PacketStart(uint8_t* dataStart, uint8_t* dataPtr);
bool PacketAddMtrMvmnt(uint8_t* dataStart, uint8_t* dataPtr, MTR_MVMNT Mvmnt, uint8_t mtr);
bool PacketAddSolMvmnt(uint8_t* dataStart, uint8_t* dataPtr, SOL_MVMNT Mvmnt, uint8_t sol);
bool PacketTerm(uint8_t* dataStart, uint8_t* dataPtr);
    
// Packet Parsing Functions
bool ParsePacketHeader(uint8_t* dataStart, uint8_t* dataPtr, PCKT_TYPE *packetType, uint16_t* size);
PCKT_INDEX_TYPE ParseIndexType(uint8_t* dataStart);
bool ParseMtrPacket(PACKET_MTR *pckt, uint8_t* mtr, MTR_MVMNT* mvmnt);
bool ParseSolPacket(PACKET_SOL *pckt, uint8_t* sol, SOL_MVMNT* mvmnt);

    
#endif
