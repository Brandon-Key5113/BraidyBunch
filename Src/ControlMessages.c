#include "ControlMessages.h"

#include "string.h" // For memory operations

bool CreateIndexPacket(uint8_t* dataOut, int* dataSize, 
                       PACKET_MTR* mtrPckt, int numMtrPckt, 
                       PACKET_SOL* solPckt, int numSolPckt){
    uint8_t *dataPtr = dataOut;
                           
    PACKET_HEADER header;
    header.StartOfHeader = '{';
    header.PacketType = PCKT_TYPE_INDEX;
    // Calculate size. Header+MtrSize*NumPackets+SolSize*NumPackets+Tail
    header.PacketSize = sizeof(PACKET_HEADER)+ sizeof(PACKET_MTR)*numMtrPckt +
                        sizeof(PACKET_SOL)*numSolPckt + 1;
    header.EndOfHeader = '}';
    
    // Copy header into the data array
    memcpy(dataPtr, &header, sizeof(PACKET_HEADER));
    dataPtr += sizeof(PACKET_HEADER);
    
    // Add 
    
    // Add Tailing newline
}

PACKET_MTR CreateMtrPacket(uint8_t mtr, MTR_MVMNT mvmnt){
    PACKET_MTR ret;
    ret.StartOfPacket = '(';
    ret.Motor = mtr;
    ret.Mvmnt = mvmnt;
    ret.EndOfPacket = ')';
    return ret;
}

PACKET_SOL CreateSolPacket(uint8_t sol, SOL_MVMNT mvmnt){
    PACKET_SOL ret;
    ret.StartOfPacket = '[';
    ret.Solenoid = sol;
    ret.Mvmnt = mvmnt;
    ret.EndOfPacket = ']';
    return ret;
}