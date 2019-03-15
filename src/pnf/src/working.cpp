#include<iostream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <time.h>   // time calls
#include <iomanip>
#include<string.h>
#include <sstream>

using namespace std;


void isKthBitSet(int n, int k)
{
    if (n & (1 << (k - 1)))
        cout << "NOT SET\n";
    else
        cout << "SET\n";
}


int main(void)
{
    int fd;
    int X_value=0;
    int Y_value=0;
    int Ang_value=0;
    int marker=0;

    fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);
    if(fd == 1)
    {
        printf("\n  Error! in Opening ttyUSB0\n");
    }
    else
    {
        printf("\n  ttyUSB0 Opened Successfully\n");
    }
    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);

    cfsetispeed(&SerialPortSettings,B38400);
    cfsetospeed(&SerialPortSettings,B38400);
    SerialPortSettings.c_cflag |=  PARENB; /*SET   Parity Bit PARENB*/
    SerialPortSettings.c_cflag &= ~PARODD;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;//enable receiver
    SerialPortSettings.c_lflag &= ~ICANON; /* canonical mode */
    SerialPortSettings.c_iflag |= IGNBRK;
    SerialPortSettings.c_oflag &= ~OPOST; /* raw output */
    SerialPortSettings.c_cc[VMIN]  =21; /* Read 10 characters */
    SerialPortSettings.c_cc[VTIME] = 5;  /* Wait indefinitely   */
    tcsetattr(fd,TCSANOW,&SerialPortSettings); /* apply the settings */
    //tcflush(fd, TCOFLUSH);

    unsigned char initiate_bytes[] = {0xEC,0x13};
    unsigned char send_bytes[] = {0xC8,0x37};
    unsigned char read_buffer[25];
    int  bytes_read = 0;
    int tag_id=0;

    write(fd, initiate_bytes, sizeof(initiate_bytes));
    usleep(3000000);
    printf("Initiated\n");


    while(1)
    {



        write(fd, send_bytes, sizeof(send_bytes));  //Send data
        bytes_read = read(fd,&read_buffer,25);
        fcntl(fd, F_SETFL, FNDELAY);
        if (bytes_read > 20)
        {
            read_buffer[bytes_read] = 0;
            char   *p;
            cout << "number :  "<<bytes_read << endl;
            X_value  = read_buffer[5]-read_buffer[4];
            Y_value  = read_buffer[7]-read_buffer[6];
            Ang_value= read_buffer[10]*128 + read_buffer[11];
            tag_id   =read_buffer[17] - read_buffer[16];
            marker   =read_buffer[0];
            isKthBitSet(marker, 2);
            printf("X  : %d   Y : %d  Ang  : %d   Tag : %d   \n",X_value,Y_value,Ang_value,tag_id);
            printf("\n");
        }
        else if (bytes_read < 0)
        {
            printf("Error from read: %d: %s\n", bytes_read, strerror(errno));
        }
        usleep(100000);
    ///s    tcflush(fd, TCIFLUSH); /* Discards old data in the rx buffer            */
    }
    close(fd);
    return 0;
}
