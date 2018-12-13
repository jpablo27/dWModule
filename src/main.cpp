#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>


int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);



int main(int argc, char const *argv[]){

    ssize_t wret;
    std::string bufStr;
    char const *portname = "/dev/ttyACM0";
    char buf [230];
    int n=0;
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0){ 
        std::cout << "Error abriendo puerto " << std::endl;
        return -1;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

    wret=write (fd, "\r\r",2);           // send 2 character greetin
    usleep (300000);

    while(n<100){
        n = read (fd, buf, sizeof buf);
        usleep (300000);
    }

    for (int i = 0; i < n; ++i)
    {
        bufStr = buf[i];
        if (bufStr == "\n")
        {
            std::cout << "ENTER"<< std::endl; 
        }
        std::cout << buf[i]; 
    }


    /*
    int n = read (fd, buf, sizeof buf); 


    usleep (100000);

*/

    //usleep (100000);             // sleep enough to transmit the 7 plus


   // 

   // usleep (100000);             // sleep enough to transmit the 7 plus

/*    wret=write(fd,"la",2); 
                                         // receive 25:  approx 100 uS per char transmit

    usleep (100000);

    wret=write(fd,"\n",1); 
                                         // receive 25:  approx 100 uS per char transmit

    usleep (100000);*/
    

/*


    n = read (fd, buf, sizeof buf); 

    usleep (100000);

    for (int i = 0; i < n; ++i)
    {
        if (false)
        {
            std::cout << "ENTER"<< std::endl; 

        }
        std::cout <<i<< ": "<<  buf[i] << std::endl; 

    }

*/
    return 0;
}



int set_interface_attribs (int fd, int speed, int parity){
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        std::cout << "error from tggetattr"<< errno << std::endl;
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 10;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
            std::cout << "error from tggetattr"<< errno << std::endl;
            return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block){
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        std::cout << "error from tggetattr"<< errno << std::endl;
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        std::cout << "error setting term attributes" <<  errno << std::endl;
}