#include "ControlMessages.h"

#include <string.h> // For memory operations
#include <stdio.h> // sprintf

uint16_t hex2int(char *hex);

bool PacketStart(uint8_t* dataStart, uint16_t* dataPtr){
    
    *dataPtr = 0;
    
    dataStart[*dataPtr] = PCKT_TX_START;
    (*dataPtr)++;

    PACKET_HEADER header;
    header.StartOfHeader = PCKT_HEADER_START;
    // Calculate size. Header+MtrSize*NumPackets+SolSize*NumPackets+Tail
    //header.PacketSize = 0;
    header.PacketSize[0] = '0';
    header.PacketSize[1] = '0';
    header.PacketSize[2] = '0';
    header.PacketSize[3] = '0';
    header.PacketSize[4] = '0';
    header.PacketSize[5] = '0';
    header.PacketType = '0' + PCKT_TYPE_INDEX;
    header.EndOfHeader = PCKT_HEADER_END;

    // Copy header into data array
    memcpy(&dataStart[*dataPtr], &header, sizeof(PACKET_HEADER));
    *dataPtr += sizeof(PACKET_HEADER);

    return true;
}

bool PacketAddMtrMvmnt(uint8_t* dataStart, uint16_t* dataPtr, MTR_MVMNT mvmnt, uint8_t mtr){
    
    PACKET_MTR pckt;
    pckt.StartOfPacket = PCKT_MTR_START;
    pckt.Motor = '0' + mtr;
    pckt.Mvmnt = '0' + mvmnt;
    pckt.EndOfPacket = PCKT_MTR_END;
    
    memcpy(&dataStart[*dataPtr], &pckt, sizeof(PACKET_MTR));
    *dataPtr += sizeof(PACKET_MTR);
    
    return true;
}

bool PacketAddSolMvmnt(uint8_t* dataStart, uint16_t* dataPtr, SOL_MVMNT mvmnt, uint8_t sol){
    
    PACKET_SOL pckt;
    pckt.StartOfPacket = PCKT_SOL_START;
    pckt.Solenoid = '0' + sol;
    pckt.Mvmnt = '0' + mvmnt;
    pckt.EndOfPacket = PCKT_SOL_END;
    
    memcpy(&dataStart[*dataPtr], &pckt, sizeof(PACKET_SOL));
    *dataPtr += sizeof(PACKET_SOL);
    
    return true;
}

bool PacketTerm(uint8_t* dataStart, uint16_t* dataPtr){
    // Add Tailing character
    dataStart[*dataPtr] = PCKT_TX_END;
    (*dataPtr)++;
    // Null Terminate
    dataStart[*dataPtr] = 0;
    
    // Add the size of the data to the header
    PACKET_HEADER *header = (PACKET_HEADER*) (dataStart + 1);
    uint16_t pcktSize = *dataPtr;
    
    snprintf(header->PacketSize, PCKT_SIZE_HEX_MAX+2+1, "0x%04X", pcktSize) ;
    // Sprintf adds a null terminator, so restore the packet type
    header->PacketType = PCKT_TYPE_INDEX;
    
    
    
    return true;
}

bool ParsePacketHeader(uint8_t* dataStart, uint16_t* dataPtr, PCKT_TYPE *packetType, uint16_t* size){
    *dataPtr = 0;
    *packetType = PCKT_TYPE_INVALID;
    *size = 0;
    // Check transmission start character
    if (dataStart[*dataPtr] != PCKT_TX_START){
        return false;
    }
    // Move onto header
    (*dataPtr)++;
    PACKET_HEADER *header = (PACKET_HEADER*) (&dataStart[*dataPtr]);
    // Check Header start character
    if (header->StartOfHeader != PCKT_HEADER_START){
        return false;
    }
    // retrieve values
    (*dataPtr)+= sizeof(PACKET_HEADER);
    (*size) = hex2int( &(header->PacketSize[2]));
    (*packetType) = (PCKT_TYPE) (header->PacketType - '0');
    return true;
}

PCKT_INDEX_TYPE ParseIndexType(uint8_t* dataStart){
    if (dataStart[0] == PCKT_MTR_START){
        return PCKT_INDEX_TYPE_MTR;
    } else if (dataStart[0] == PCKT_SOL_START){
        return PCKT_INDEX_TYPE_SOL;
    }
    return PCKT_INDEX_TYPE_INVALID;
}

bool ParseMtrPacket(PACKET_MTR *pckt, uint8_t* mtr, MTR_MVMNT* mvmnt){
    // Check to see if head and tail characters are what they should be
    if (pckt->StartOfPacket != PCKT_MTR_START){
        return false;
    }
    if (pckt->EndOfPacket != PCKT_MTR_END){
        return false;
    }
    
    *mtr = pckt->Motor - '0';
    *mvmnt = (MTR_MVMNT) (pckt->Mvmnt - '0');
 
    return true;
}

bool ParseSolPacket(PACKET_SOL *pckt, uint8_t* sol, SOL_MVMNT* mvmnt){
    if (pckt->StartOfPacket != PCKT_SOL_START){
        return false;
    }
    if (pckt->EndOfPacket != PCKT_SOL_END){
        return false;
    }
    
    *sol = pckt->Solenoid - '0';
    *mvmnt = (SOL_MVMNT)(pckt->Mvmnt - '0');
 
    return true;
}


uint16_t hex2int(char *hex) {
    uint16_t val = 0;
    char* limit = hex+4;
    while (*hex && (hex < limit)) {
        // get current character then increment
        uint8_t byte = *hex++; 
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;    
        // shift 4 to make space for new digit, and add the 4 bits of the new digit 
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}
