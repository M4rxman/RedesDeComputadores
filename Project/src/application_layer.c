// Application layer protocol implementation

#include "application_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include "application_layer.h"
#include <link_layer.h>

#define DATA_SIZE 1024

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int retryLimit, int timeout, const char *fileName) {
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort, serialPort);
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = retryLimit;
    linkLayer.timeout = timeout;
    linkLayer.role = strcmp(role, "tx") == 0 ? LlTx : LlRx;

    int fd = llopen(linkLayer);
    if (fd < 0) {
        printf("Error opening connection\n");
        return;
    }

    if (linkLayer.role == LlTx) {
        transmitFile(fd, fileName);
    } else if (linkLayer.role == LlRx) {
        receiveFile(fd);
    }
}

int generateControlPacket(unsigned char c, unsigned char** controlPacket, int fileLength, const char* fileName) {
    int nameLength = strlen(fileName) + 1;
    int fileSizeBits = sizeof(int) * 8 - __builtin_clz(fileLength);
    unsigned char fileLengthBytes = (fileSizeBits + 7) / 8;

    unsigned char* packet = (unsigned char*)malloc(nameLength + fileLengthBytes + 5);
    packet[0] = c;
    packet[1] = 0x00;
    packet[2] = nameLength & 0xFF;
    memcpy(packet + 3, fileName, nameLength);
    packet[3 + nameLength] = 0x01;
    packet[4 + nameLength] = fileLengthBytes;

    for (int i = 0; i < fileLengthBytes; i++) {
        packet[4 + fileLengthBytes + nameLength - i] = (fileLength >> (i * 8)) & 0xFF;
    }

    *controlPacket = packet;
    return nameLength + fileLengthBytes + 5;
}

int generateDataPacket(unsigned char** dataPtr, unsigned char* packetBuffer, int dataSize) {
    packetBuffer[0] = 1;
    packetBuffer[1] = 2;
    packetBuffer[2] = (dataSize >> 8) & 0xFF;
    packetBuffer[3] = dataSize & 0xFF;
    memcpy(packetBuffer + 4, *dataPtr, DATA_SIZE);
    *dataPtr += DATA_SIZE;
    return 0;
}

int loadFileData(unsigned char* dataBuffer, FILE* file, int dataSize) {
    if (fread(dataBuffer, 1, dataSize, file) != dataSize) {
        printf("Error reading file\n");
        return -1;
    }
    return 0;
}

int interpretControlPacket(unsigned char* packet, char** extractedFileName, int* extractedSize) {
    unsigned char nameLength = packet[2];
    unsigned char fileSizeBytes = packet[4 + nameLength];

    char* nameBuffer = (char*)malloc(nameLength);
    unsigned char sizeBuffer[fileSizeBytes];
    memcpy(sizeBuffer, packet + 5 + nameLength, fileSizeBytes);

    for (int i = 0; i < fileSizeBytes; i++) {
        *extractedSize |= (sizeBuffer[fileSizeBytes - i - 1] << (i * 8));
    }
    
    memcpy(nameBuffer, packet + 3, nameLength);
    *extractedFileName = nameBuffer;
    return 0;
}

int interpretDataPacket(unsigned char* packet, unsigned char* dataBuffer, int dataSize) {
    memcpy(dataBuffer, packet + 4, dataSize - 4);
    return 0;
}

int transmitFile(int fd, const char* fileName) {
    FILE* file = fopen(fileName, "rb");
    if (file == NULL) {
        printf("Error opening file\n");
        return -1;
    }

    fseek(file, 0, SEEK_END);
    int fileSize = ftell(file);
    rewind(file);

    int controlPacketSize;
    unsigned char* startControlPacket;
    controlPacketSize = generateControlPacket(2, &startControlPacket, fileSize, fileName);

    if (llwrite(startControlPacket, controlPacketSize) == -1) {
        printf("Error sending control packet\n");
        return -1;
    }

    unsigned char* dataBuffer = (unsigned char*)malloc(fileSize);
    loadFileData(dataBuffer, file, fileSize);
    int remainingData = fileSize % DATA_SIZE;
    int chunkSize;
    int packetSize;

    while (fileSize != 0) {
        chunkSize = fileSize > DATA_SIZE ? DATA_SIZE : remainingData;
        packetSize = chunkSize + 4;
        unsigned char* dataPacket = (unsigned char*)malloc(packetSize);
        generateDataPacket(&dataBuffer, dataPacket, DATA_SIZE);

        if (llwrite(dataPacket, packetSize) == -1) {
            printf("Error sending data packet\n");
            return -1;
        }

        fileSize -= chunkSize;
    }

    unsigned char* endControlPacket;
    controlPacketSize = generateControlPacket(3, &endControlPacket, fileSize, fileName);

    if (llwrite(endControlPacket, controlPacketSize) != 0) {
        printf("Error sending control packet\n");
        return -1;
    }

    llclose(fd);
    return 0;
}

int receiveFile(int fd) {
    unsigned char* buffer[DATA_SIZE];
    unsigned char* extractedFileName;
    int fileSize;
    int totalChunks;
    int receivedChunks = 0;
    int chunkSize;

    llread(buffer);
    interpretControlPacket(buffer, (char**)&extractedFileName, &fileSize);

    totalChunks = (fileSize + DATA_SIZE - 1) / DATA_SIZE;
    FILE* outputFile = fopen((char*)extractedFileName, "wb+");

    while (receivedChunks < totalChunks) {
        if ((chunkSize = llread(buffer)) == -1) {
            continue;
        }

        unsigned char* dataSegment[chunkSize];
        interpretDataPacket(buffer, dataSegment, chunkSize);
        fwrite(dataSegment, sizeof(unsigned char), chunkSize - 4, outputFile);
        receivedChunks++;
    }

    while (llread(buffer) == -1);
    interpretControlPacket(buffer, (char**)&extractedFileName, &fileSize);
    llclose(0);
    fclose(outputFile);
    return 0;
}
