#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <iostream>

char *ip_s = "192.168.0.100";
char *socket_s = "65432";

using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

char konek(char robot, int perintah){
	int sockfd, portno, n, dd;
	struct sockaddr_in serv_addr;
    struct hostent *server;
	char ewa[255];
	char se[255];
    portno = atoi(socket_s);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    struct timeval tv;
    tv.tv_sec = 0;//5 Secs Timeout
    tv.tv_usec = 1;//Not init'ing this can cause strange errors
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(struct timeval));
    if (sockfd < 0) 
        cout << "eror gan gak bisa buka socket\n";
    server = gethostbyname(ip_s);
    if (server == NULL) {
        fprintf(stderr,"erorr gan, gak ada host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);

	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		cout << "Ga nyambung server gann!!\n";	
		return 0;
		}
	sprintf(se,"%c%d", robot, perintah);
    n = write(sockfd, se, strlen(se));
	bzero(ewa,255);
	dd = read(sockfd,ewa,255);
    cout << ewa << endl;
    cout << perintah << endl;
//	sleep(1);
//close(sockfd);
return ewa[0];
}

int main()
{
	while(1){
	char abj_robot = konek( 'b', 30); //char konek(argc1, argv1, char robot, int perintah)
		cout << "robot " << abj_robot << endl;
if(abj_robot == 0)cout<<"sa\n";
		//break;
	}
	return 0;
}

