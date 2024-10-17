// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <termios.h>
#include <signal.h>
#include <fcntl.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define BUF_SIZE 2048
#define CONTROL_BYTE_SIZE 5

#define FLAG 0x7E
#define A_tx 0x03 
#define C_SET 0x03
#define C_UA 0x07


#define FALSE 0
#define TRUE 1

int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{ 

    int state = 0;
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    
    if (fd < 0)
    {
        return -1;
    }

    unsigned char set[CONTROL_BYTE_SIZE];
    unsigned char ua[CONTROL_BYTE_SIZE];
    

    set[0] = FLAG;
    set[1] = A_tx;
    set[2] = C_SET;
    set[3] = set[1] ^ set[2];
    set[4] = FLAG;

    ua[0] = FLAG;
    ua[1] = A_tx;
    ua[2] = C_UA;
    ua[3] = ua[1] ^ ua[2];
    ua[4] = FLAG;

    // Create string to send
    unsigned char buf[CONTROL_BYTE_SIZE] = {0};

    if (connectionParameters.role == LlTx) {
    (void)signal(SIGALRM, alarmHandler);
    
        while (alarmCount < connectionParameters.nRetransmissions) {
        
            if (alarmEnabled == FALSE) {
                write(fd, set, CONTROL_BYTE_SIZE);
                alarmEnabled = TRUE;
                alarm(connectionParameters.timeout); 
                state = 0;
            }
            if (alarmCount == connectionParameters.nRetransmissions)
                break;

            read(fd, buf, 1);

            //printf("var = 0x%02X state:%d\n", (unsigned int)(buf[0] & 0xFF), state);
            changeOpenState(buf[0], &state, connectionParameters.role);

            if (state == 5) {
                alarm(0);
                break;
            }
        }

        printf("llopen\n");
        return 1;
    }


    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
