#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <map>

#include "ros/ros.h"
#include "dwmodule/distances.h"

int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);


int main(int argc, char *argv[]){

    dwmodule::distances T1_ds;

    ros::init(argc, argv, "T1_Distances");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<dwmodule::distances>("chatter", 1);
    ros::Rate loop_rate(60);
    //MAPA----DATOS PARA ORDENAR LOS VALORES DE DISTANCIAS EN LAS VARIABLES CORRECTAS
    float d1,d2,d3,d4;

    std::map<std::string , float*> mapa;
    // hay que poner los IDs correctos
    mapa["HT54"]= &d1;
    mapa["JG54"]= &d2;
    mapa["PTM4"]= &d3;
    mapa["DCI4"]= &d4;
    //MAPA----------------------------------------------------------------------------

    bool in=false;
    bool stb=false;
    bool out=false;
    bool eol=false;

    std::ostringstream oss,idss;
    ssize_t wret;
    std::string bufStr;
    std::vector<std::string> idStr;
    std::vector<float> val_v;
    char const *portname = "/dev/ttyACM0";
    char buf [230];
    int n=0;
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    int id_cnt=0;
    int tag_cnt=0;

    if (fd < 0){ 
        std::cout << "Error abriendo puerto " << std::endl;
        return -1;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

    wret=write (fd, "\r\r",2);           // send 2 character greetin
    usleep (500000);
    n = read (fd, buf, sizeof buf);
    usleep (500000);
    wret=write (fd, "les\r",4);           // send 2 character greetin
    n = read (fd, buf, sizeof buf);
    usleep (500000);
    n = read (fd, buf, sizeof buf);
    usleep (500000);
    while(ros::ok()){
        n = read (fd, buf, sizeof buf);
        for (int i = 0; i < n; ++i)
        {   
            
            bufStr=buf[i];


            if((bufStr=="\n")&&!eol){
               eol = true;
               continue; 
            } 


            if(eol){//Esta bandera es para sincronizar, se leerá después del primer 'enter'

            //std::cout <<bufStr ;


                if(bufStr=="="&&!in){
                    oss.str("");
                    oss.clear();
                    idss.str("");
                    idss.clear();
                    in = true;
                   
                    continue;
                }else if(bufStr==" "&&in){
                    val_v.push_back(std::stof(oss.str()));
                    in = false;
                    id_cnt=0;
                    continue;

                }else if(bufStr=="\r"){
                    for (int i = 0; i < idStr.size(); ++i)
                    {
                        *mapa[idStr[i]]=val_v[i]; //Aqui se le asigna su valor a d1,d2,d3,d4 según corresponde.
                        T1_ds.d1=d1;
                        T1_ds.d2=d2;
                        T1_ds.d3=d3;
                        T1_ds.d4=d4;
                    }
                    chatter_pub.publish(T1_ds);
                    ros::spinOnce();
                    loop_rate.sleep();
                    std::cout <<std::endl;
                    idStr.clear();
                    val_v.clear();
                    continue;

                }else if(bufStr=="\n"){
                    continue;
                }else{

                    if(in) oss<<bufStr;

                    if ((id_cnt<4)&&(!stb))
                    {
                        idss<<bufStr;                       
                        id_cnt++;
                    }else if(id_cnt==4){
                        idStr.push_back( idss.str());
                        id_cnt++;
                    }
                }
            }
        }        
    }

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
    tty.c_lflag &=~(ICANON);
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
