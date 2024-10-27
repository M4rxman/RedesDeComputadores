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
#define ESC 0x7D
#define RR0 0xAA
#define RR1 0xAB
#define REJ0 0x54
#define REJ1 0x55
#define DISC 0x0B

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
int state;
#define StartState 0
#define FlagState 1
#define AdressState 2
#define controlState 3
#define bccState 4
#define flagendState 5


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int changeFase(LinkLayerRole role ,unsigned char buf,int* state){
    if(role==LlTx){
        if(state==StartState && buf==FLAG){ 
            state=FlagState;
            return 1;
            }
        else if( state== FlagState && buf==ATrRr){
            state=AdressState;
            return 1;
            }
        else if (state== AdressState && buf==SET_I){
            state=controlState;
            return 1;
            }
        else if (state==controlState && buf==(ATrRr^SET_I)){
            state=bccState;
            return 1;
            }
        else if (state==bccState && buf==FLAG){
            state=flagendState;
            return 1;
            }
        else if (buf==FLAG){ 
            state=FlagState;
            return 1;
            }
        else{
            state=StartState;
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
        if      (state==StartState && buf==FLAG){
            state=FlagState;
            return 1;}
        else if (state== FlagState && buf==ATrRr){
            state=AdressState;
            return 1;}
        else if (state== AdressState && buf==UA_I){
            state=controlState;
            return 1;}
        else if (state==controlState && buf==(ATrRr^UA_I)){
            state=bccState;
            return 1;}
        else if (state==bccState && buf==FLAG){
            state=flagendState;
            return 1;}
        else if (buf==FLAG){
            state=FlagState;
            return 1;}
        else    {
            state=StartState;
            return 1;}
    }
    return 0;
}


//basecally to make a connection between the Transmiter and the Receiver we need to send signals that are messages in a certain order
//so we will send a part of the set at the time and depending on what we receive we can see in what fase is the comunication going
// the variable state is to know if the connection is done or not
int llopen(LinkLayer connectionParameters)
{
    if (fd=openSerialPort(connectionParameters.serialPort,connectionParameters.baudRate) < 0){
        return -1;        
    }

    Stop=FALSE;
    state=StartState;
    alarmEnabled = FALSE;
    alarmCount = 0;
    unsigned char set[BUFFER_SIZE];
    unsigned char ua[BUFFER_SIZE];
    unsigned char buf[BUFFER_SIZE];
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
                state=StartState;
            }
            if (alarmCount==retransmitons){ break;}

            if (read(fd,buf, 1)==0){continue;}

            int i=changeFase(connectionParameters.role,buf[0],&state);
            if(state==flagendState){
                alarm(0);
                break;
            }
        }
        return 1;

    }
    else if (connectionParameters.role==LlRx){

        while (Stop!=FALSE){
            read(fd,buf, 1);
            int i=changeFase(connectionParameters.role,buf[0],&state);
            if(state==flagendState){
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

int calculate_bcc2( const unsigned char *buf, int bufSize, unsigned char *data_bbc2 ){
    // the information will be processed  so we will return a buffer data_bbc2 that has the information of the data and the bbc2 
    int bbc2;
    data_bbc2[0]=buf[0];
    bbc2=buf[0];
    for(int i=1; i<bufSize; i++){
        bbc2^=buf[i];
        data_bbc2[i]=buf[i];
    }
    data_bbc2[bufSize+1];
    return 0;
}

int payload_stuffing( unsigned char *data_bbc2,const unsigned char *buf, int bufSize ){
    int j=0;
    for(int i=0;i<bufSize;i++ ){
        if(buf[i]==FLAG){
            data_bbc2[j]=ESC;
            data_bbc2[j++]=FLAG^0x20;
        }
        else if( buf[i]==ESC){
            data_bbc2[j]=ESC;
            data_bbc2[j++]=ESC^0x20;
        }
        else{
            data_bbc2[j]=buf[i];
            j++;
        }
    }
    return 0;
}
int stuffing_size(unsigned char *buf, int bufSizedata){
    int j=0;
    int n=0;
    for(int i=0; i< bufSizedata; i++){
        if(buf[i]==FLAG||buf[i]==ESC){
            j++;
        }
        n++;
    }
    return n+j;
}


int changeControlpacket(unsigned char buf,int* state, unsigned char* ol ){

    if(buf==FLAG && state==StartState){
        state=FlagState;
        return 1;
    }
    else if( state== FlagState && buf==ATrRr){
        state=AdressState;
        return 1;

    }
    else if (state== AdressState && (buf == RR0 || buf== RR1 || buf==REJ0 || buf== REJ1)){
        state=controlState;
        ol= buf;
        return 1;
    }
    else if( state==controlState&& buf==(ATrRr ^ol)){
        state = bccState ;
        return 1;
    }
    else if( state==bccState && buf==FLAG){
        state=flagendState;
        return 1;
    }
    else if (buf==FLAG){ 
        state=FlagState;
        return 1;
    }
    else{
        state=StartState;
        return 1;
        }

}
int llwrite(const unsigned char *buf, int bufSize)
{   
    int trframei=0;
    int rrframei=0;
    //1.Calculate BBC2
    //buf is the Data
    unsigned char data_bbc2[bufSize+1];
    calculate_bcc2(buf, bufSize, data_bbc2);
    buf=data_bbc2;

    //2. Stuff payload  BCC2
    int size_payload=stuffing_size(buf, bufSize);
    unsigned char data_payload[size_payload+1];
    payload_stuffing(data_payload, buf, bufSize+1);

    //3. F, A, C, BCC1, D1...Dn, BCC2, F
    // F,A,C,BBC1,F =5 
    int frame_size= 5+ bufSize+1;

    unsigned char *frame[frame_size];
    frame[0]=FLAG;
    frame[1]=ATrRr;
    frame[2]=trframei;
    frame[3]=ATrRr ^ trframei;
    for( int i=0; i<size_payload; i++){
        frame[4+i]=data_payload[i];
    }
    frame[frame_size-1]=FLAG;

    //4. Write Bytes(...)

    Stop=FALSE;
    state=StartState;
    alarmEnabled = FALSE;
    alarmCount = 0;
    unsigned char ol= 0x00;
    (void)signal(SIGALRM, alarmHandler);

        while (alarmCount!=3){
            
            if(alarmEnabled==FALSE){
                write(fd,frame,frame_size);
                alarmEnabled=TRUE;
                alarm(timeout);
            }
            if (read(fd,buf, 1)==0){continue;}

            changeControlpacket(buf[0], &state, ol);

        }
        return 0;
    }
    //5. Receive Confirmation (State Machine)


    //make bcc2 XOR
    //0x7D is for stuffing
    //stuffing array  Flag^0x20 Esc^x20
    //esc before
    //made stuffing of the bcc2
    //define frame
    //handle of the frame frameindex%2 



////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    //1. Read Frame (STM)   
    //2. Destuffing
    //3. Calculate BCC2
    //4. Validate BCC2
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
