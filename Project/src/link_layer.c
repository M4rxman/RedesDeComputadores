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
#define FLAG_COVER 0X5E
#define ESC_COVER 0X5D


#define StartState 0
#define FlagState 1
#define AdressState 2
#define controlState 3
#define bccState 4
#define flagendState 5
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
int trframei=0;
int rrframei=0;

LinkLayerRole cRole;//stands for communication role

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int changeFase(LinkLayerRole role ,unsigned char buf,int* state){
    cRole=role;
    if(cRole==LlTx){
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
    else if(cRole==LlRx){
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
            data_bbc2[j++]=FLAG_COVER;
        }
        else if( buf[i]==ESC){
            data_bbc2[j]=ESC;
            data_bbc2[j++]=ESC_COVER;
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


int changeControlpacket(unsigned char buf, int* state, unsigned char* byte) {
    if(buf == FLAG && *state == StartState) {
        *state = FlagState;
    } 
    else if (*state == FlagState && buf == ATrRr) {
        *state = AdressState;
    } 
    else if (*state == AdressState && (buf == RR0 || buf == RR1 || buf == REJ0 || buf == REJ1)) {
        *state = controlState;
        *byte = buf;
    } 
    else if (*state == controlState && buf == (ATrRr ^ *byte)) {
        *state = bccState;
    } 
    else if (*state == bccState && buf == FLAG) {
        *state = flagendState;
    } 
    else if (buf == FLAG) { 
        *state = FlagState;
    } 
    else {
        *state = StartState;
    }
    return 0;
}

int llwrite(const unsigned char *buf, int bufSize)
{   
    
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
    frame[3]=*frame[1] ^ *frame[2];
    for( int i=0; i<size_payload; i++){
        frame[4+i]=data_payload[i];
    }
    frame[frame_size-1]=FLAG;

    //4. Write Bytes(...)

    Stop=FALSE;
    state=StartState;
    alarmEnabled = FALSE;
    alarmCount = 0;
    unsigned char byte= 0x00;
    (void)signal(SIGALRM, alarmHandler);

        while (alarmCount!=3){
            
            if(alarmEnabled == FALSE){
                write(fd,frame,frame_size);
                alarmEnabled=TRUE;
                alarm(timeout);
            }
            if (read(fd,buf,1)==0){continue;}

            int bufsize_1= length(buf);

            changeControlpacket(buf[0], &state, &byte);
            //5. Receive Confirmation (State Machine)

            if(byte==REJ0 || byte==REJ1){
                printf(" Information rejected.");
                return -1;
            }

            if(byte==RR0 || byte==RR1){
                printf(" Information can be received.");
                trframei++;
                trframei=trframei%2;
            }
            
        }
        return frame_size;
    }
    

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


int send_RR(unsigned char frameindex){
    unsigned char rr[5];

    rr[0]=FLAG;
    rr[1]=ATrRr;
    rr[2]=RR0;//has to be changed
    rr[3]=ATrRr^rr[2];
    rr[4]=FLAG;

    write(fd,rr,5);
    return 0;
}
int send_REJ(unsigned char frameindex){
    unsigned char rej[5];

    rej[0]=FLAG;
    rej[1]=ATrRr;
    rej[2]=REJ0;//has to be changed
    rej[3]=ATrRr^rej[2];
    rej[4]=FLAG;

    write(fd,rej,5);
    return 0;
}

int send_rec_DISC(){
    unsigned char disc[5];

    disc[0]=FLAG;
    disc[1]=ARrTr;
    disc[2]=DISC;
    disc[3]=ATrRr^disc[2];
    disc[4]=FLAG;

    write(fd,disc,5);
    return 0;
}

int changeReadState(unsigned char buf,int* state, unsigned char* packet, int* index){
    
    if(*state==StartState){
        *index=0;
        if(buf==FLAG){
            *state=FlagState;
        }
    }
    else if(*state==FlagState){
        *index=0;
        if(buf==ATrRr){
            state=AdressState;
        }
        else if(buf != FLAG) {
            *state = StartState;
        } 
        else { 
            *state = FlagState;
        }
    }
    else if(*state==AdressState){
        if(buf==rrframei){
            state=controlState;
        }
        else if(buf==(rrframei+1)%2){
            send_RR((rrframei+1)%2);
            return 0;
        }
        else if(buf != FLAG) {
            *state = StartState;

        } else {
            *state = FlagState;
        }
    }
    else if(*state==controlState && buf==(ATrRr^ARrTr)){
            *state=bccState;
        }
    else if(*state==bccState){

        if(buf==ESC){
            *state=flagendState;
        }
    //2. Destuffing

        else if (buf == FLAG) {
            
                *state = 6;
                //3. Calculate BCC2

                unsigned char bcc2 = packet[*index];
                unsigned char xor = packet[0];
                for (int j = 1; j< *index; j++) {
                    xor ^= packet[j];
                }
                if (bcc2 == xor) {  // Valid data
                    *state = 6;
                    send_RR((rrframei + 1) % 2);
                    printf("SENT RR\n");
                    rrframei = (rrframei + 1) % 2;
                    return *index;
                }
                else  {
                    send_REJ(rrframei);
                    printf("SENT REJ\n");
                    return -1;
                }
            }
        else {
            packet[(*index)++] = buf;
            
        }
            
        }
    else if(state==flagendState){
        if (buf == ESC_COVER) {
                packet[(*index)++] = ESC;
            }
            else if (buf == FLAG_COVER) {
                packet[(*index)++] = FLAG;
            }
            else if (buf == FLAG) {
                packet[(*index)++] = ESC;
                *state = 6;
            }
            else {
                packet[(*index)++] = ESC;
                packet[(*index)++] = buf;
            }
    }
    else{
        state=StartState;
    }
    return index;
}
int llread(unsigned char *packet){
    int state=StartState;
    int i=0;
    unsigned char buf[1];
    int size=0;
    while(state != 6 ){
        if(read(fd,buf,1)==0){continue;}
        changeReadState(buf[0],&state, &size, packet);
        if (size == -1){
            return -1;
        }
    }
    
    return size;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int changeCloseStateTr(unsigned char buf,int* state){
    if(*state==StartState){
        if(buf==FLAG){
            *state=FlagState;
        }
    }
    else if(*state==FlagState){
        if(buf==ATrRr){
            *state=AdressState;
        }
        else if(buf != FLAG) {
            *state = StartState;
        } 
        else { 
            *state = FlagState;
        }
    }
    else if(*state==AdressState){
        if(buf==DISC){
            *state=controlState;
        }
        else if(buf != FLAG) {
            *state = StartState;

        } else {
            *state = FlagState;
        }
    }
    else if(*state==controlState){
        if(buf==(ARrTr^DISC)){
            *state=bccState;
        }
        else if (buf!=FLAG){
            *state=StartState;
        }
        else{
            *state=FlagState;
        }
    }
    else if(*state==bccState){
        if( buf==FLAG){
            *state=flagendState;
        }
        else{
            *state=StartState;
        }
    }
    else{
        *state=StartState;
    }
    return 0;
}

int changeCloseStateRrDISC(unsigned char buf,int* state){
    if(*state==StartState){
        if(buf==FLAG){
            *state=FlagState;
        }
    }
    else if(*state==FlagState){
        if(buf==ATrRr){
            *state=AdressState;
        }
        else if(buf != FLAG) {
            *state = StartState;
        } 
        else { 
            *state = FlagState;
        }
    }
    else if(*state==AdressState){
        if(buf==DISC){
            *state=controlState;
        }
        else if(buf != FLAG) {
            *state = StartState;

        } else {
            *state = FlagState;
        }
    }
    else if(*state==controlState){
        if( buf==(ATrRr^DISC)){
            *state=bccState;
        }
        else if (buf!=FLAG){
            *state=StartState;
        }
        else{
            *state=FlagState;
        }
    }
    else if(*state==bccState){
        if( buf==FLAG){
            *state=flagendState;
        }
        else{
            *state=StartState;
        }
    }
    else{
        *state=StartState;
    }
    return 0;
}

int changeCloseStateRrUA(unsigned char buf,int* state){
    if(*state==StartState){
        if(buf==FLAG){
            *state=FlagState;
        }
    }
    else if(*state==FlagState){
        if(buf==ATrRr){
            *state=AdressState;
        }
        else if(buf != FLAG) {
            *state = StartState;
        } 
        else { 
            *state = FlagState;
        }
    }
    else if(*state==AdressState){
        if(buf==UA_I){
            *state=controlState;
        }
        else if(buf != FLAG) {
            *state = StartState;

        } else {
            *state = FlagState;
        }
    }
    else if(*state==controlState){
        if( buf==(ARrTr^UA_I)){
            *state=bccState;
        }
        else if (buf!=FLAG){
            *state=StartState;
        }
        else{
            *state=FlagState;
        }
    }
    else if(*state==bccState){
        if( buf==FLAG){
            *state=flagendState;
        }
        else{
            *state=StartState;
        }
    }

    return 0;   
}

int llclose(int showStatistics)
{
    state=StartState;
    unsigned char tr_disc[5];
    unsigned char ua[5];

    tr_disc[0]=FLAG;
    tr_disc[1]=ATrRr;
    tr_disc[2]=DISC;
    tr_disc[3]=ATrRr^tr_disc[2];
    tr_disc[4]=FLAG;

    ua[0]=FLAG;
    ua[1]=ATrRr;
    ua[2]=UA_I;
    ua[3]=ATrRr^UA_I;
    ua[4]=FLAG;

    alarmCount = 0;
    alarmEnabled = FALSE;

    unsigned char buf[5] = {0};
    (void)signal(SIGALRM, alarmHandler);

    if(cRole==LlTx){
        while (alarmCount!=3){
            if(alarmEnabled==FALSE){
                write(fd,tr_disc,5);
                alarmEnabled=TRUE;
                alarm(3);
                state=StartState;
            }
            if (alarmCount==3)
            {break;}
            read(fd,buf,1);


            if(state==flagendState){
                alarm(0);
                break;
            }
        }
        while (alarmCount!=3){
            if(alarmEnabled==FALSE){
                write(fd,ua,5);
                alarmEnabled=TRUE;
                alarm(timeout);
            }
            if (read(fd,buf,1)==0){continue;}

            int i=changeCloseStateTr(buf[0],&state);

            if(state==flagendState){
                alarm(0);
                write(fd,ua,5);
                break;
            }
        }
    }
    else if(cRole==LlRx){
        Stop = FALSE;
        state = 0;
        while (Stop == FALSE) {
            if (read(fd, buf, 1) == 0){
                continue;
            }
            changeCloseStateRrDISC(buf[0], &state);

            if (state == flagendState) {
                break;
            }
        }
        alarmCount = 0;
        alarmEnabled = FALSE;

        while (alarmCount<3){
            if(alarmEnabled==FALSE){
                send_rec_DISC();
                alarmEnabled=TRUE;
                alarm(3);
                state=StartState;
            }
            if(alarmCount==3){
                break;
            }

            read(fd,buf, 1);
            
            changeCloseStateRrUA(buf[0],&state);

            if(state==flagendState){
                alarm(0);
            }
        }
    }
    close(fd);
    if(state==flagendState){
        int clstat = closeSerialPort();
        return clstat;
    }

    return -1;
}
