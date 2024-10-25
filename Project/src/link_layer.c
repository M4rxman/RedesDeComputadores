// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FLAG 0x7E
#define ATrRr 0x03
#define ARrTr 0x01
#define BYTESIZE 5
#define SET_I 0x03
#define UA_I 0x07

int alarmEnabled;
int alarmCount;

volatile Stop;
int StateT;//the state will be connected 1 or disconnected 0
int StateR;
int succeed_state;
int fd;
int retransmitons;
int timeout;
int BUFFER_SIZE=5;
int stage;
#define StartStage 0
#define FlagStage 1
#define AdressStage 2
#define controlStage 3
#define bccStage 4
#define flagendStage 5


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int changeFase(LinkLayerRole role ,unsigned char buf,int* state){
    if(role==LlTx){
        if(state==StartStage && buf==FLAG){ 
            state=FlagStage;
            return 1;
            }
        else if( state== FlagStage && buf==ATrRr){
            state=AdressStage;
            return 1;
            }
        else if (state== AdressStage && buf==SET_I){
            state=controlStage;
            return 1;
            }
        else if (state==controlStage && buf==(ATrRr^SET_I)){
            state=bccStage;
            return 1;
            }
        else if (state==bccStage && buf==FLAG){
            state=flagendStage;
            return 1;
            }
        else if (buf==FLAG){ 
            state=FlagStage;
            return 1;
            }
        else{
            state=StartStage;
            return 1;
            }
    }

    /*
    ua[0]=FLAG;
    ua[1]=ATrRr;
    ua[2]=UA_I;
    ua[3]=ATrRr^UA_I;
    ua[4]=FLAG;
    */ 
    else if(role==LlRx){
        if      (state==StartStage && buf==FLAG){
            state=FlagStage;
            return 1;}
        else if (state== FlagStage && buf==ATrRr){
            state=AdressStage;
            return 1;}
        else if (state== AdressStage && buf==UA_I){
            state=controlStage;
            return 1;}
        else if (state==controlStage && buf==(ATrRr^UA_I)){
            state=bccStage;
            return 1;}
        else if (state==bccStage && buf==FLAG){
            state=flagendStage;
            return 1;}
        else if (buf==FLAG){
            state=FlagStage;
            return 1;}
        else    {
            state=StartStage;
            return 1;}
    }
    return 0;
}


//basecally to make a connection between the Transmiter and the Receiver we need to send signals that are messages in a certain order
//so we will send a part of the set at the time and depending on what we receive we can see in what fase is the comunication going
// the variable state is to know if the connection is done or not
int llopen(LinkLayer connectionParameters)
{
    Stop=FALSE;
    alarmEnabled=FALSE;
    StateT=0;
    if (fd=openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;        
    }
    
    unsigned char set[BUFFER_SIZE];
    unsigned char ua[BUFFER_SIZE];
    unsigned char buf[BUFFER_SIZE];
    alarmEnabled = FALSE;
    alarmCount = 0;
    StateT=FALSE;
    StateR=FALSE;
    timeout=connectionParameters.timeout;
    retransmitons=connectionParameters.nRetransmissions;

    set[0]=FLAG;
    set[1]=ATrRr;
    set[2]=SET_I;
    set[3]=ATrRr^SET_I;
    set[4]=FLAG;

    ua[0]=FLAG;
    ua[1]=ATrRr;
    ua[2]=UA_I;
    ua[3]=ATrRr^UA_I;
    ua[4]=FLAG;
    
    //strig that we will read

    
    if(connectionParameters.role==LlTx){
        (void)signal(SIGALRM, alarmHandler);

        while (alarmCount!=connectionParameters.nRetransmissions){
            
            if(alarmEnabled==FALSE){
                write(fd,set,BUFFER_SIZE);
                alarmEnabled=TRUE;
                alarm(timeout);
                stage=StartStage;

            }
            if (alarmCount==retransmitons){ break;}

            read(fd,buf, 1);
            int i=changeFase(connectionParameters.role,buf[0],&stage);
            if(stage==flagendStage){
                alarm(0);
                break;
            }
        }
        return 1;

    }
    else if (connectionParameters.role==LlRx){

        while (Stop!=FALSE){
            read(fd,buf, 1);
            int i=changeFase(connectionParameters.role,buf[0],&stage);
            if(stage==flagendStage){
                Stop=TRUE;
            }
        }
        write(fd,ua,BUFFER_SIZE);
    }
        return 1;

    }


void alarmHandler(int Signal){
    alarmCount++;
    alarmEnabled=FALSE;
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
