#include "NetCamera.h"

NetCamera::NetCamera()
{
	ConnectSocket = INVALID_SOCKET;
}

bool NetCamera::initNetCamera(std::string address, std::string port)
{
	WSADATA wsaData;
	int iResult;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		std::cerr << "WSAStartup failed! "<< std::endl;
		return false;
	}

	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = PF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(address.c_str(), port.c_str(), &hints, &result);
	if (iResult != 0) {
		std::cerr << "getaddrinfo failed! " << std::endl;
		WSACleanup();
		return false;
	}

	// Attempt to connect to the first address returned by
	// the call to getaddrinfo
	ptr = result;

	// Create a SOCKET for connecting to server
	ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
		ptr->ai_protocol);
	if (ConnectSocket == INVALID_SOCKET) {
		std::cerr << "Error at socket()! " << std::endl;
		freeaddrinfo(result);
		WSACleanup();
		return false;
	}

	// set recieve buff size
	int optVal = 120000;
	int optLen = sizeof(int);
	setsockopt(ConnectSocket,
		SOL_SOCKET,
		SO_RCVBUF,
		(char*)&optVal,
		optLen);

	// Connect to server.
	iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
		closesocket(ConnectSocket);
		ConnectSocket = INVALID_SOCKET;
	}


	// Should really try the next address returned by getaddrinfo
	// if the connect call failed
	// But for this simple example we just free the resources
	// returned by getaddrinfo and print an error message

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		std::cerr <<  "Unable to connect to server!" << std::endl;
		WSACleanup();
		return false;
	}
	return true;
}
void NetCamera::grabImage(cv::Mat &left, cv::Mat &right)
{
	char imgSize_buff[4];
	int imgSize = 0;
	int iResult = 0;
	int iLen = 0;
	int received = 0;
	cv::Mat rawData;
	char * longbuff = nullptr;

	// recevice left image
	while (recv(ConnectSocket, imgSize_buff, 4, 0) != 4);
	imgSize = ((int *)imgSize_buff)[0];
	iLen = imgSize;
	received = 0;
	longbuff = new char[imgSize];
	while ((iResult = recv(ConnectSocket, longbuff + received, iLen, 0)) > 0)
	{
		iLen -= iResult;
		received += iResult;
		if (iLen <= 0)
			break;
	}
	rawData = cv::Mat(1, imgSize, CV_8UC1, longbuff);
	left = imdecode(rawData, CV_LOAD_IMAGE_COLOR);
	std::cerr << "Left Ok" << std::endl;
	free(longbuff);

	// recevice right image
	recv(ConnectSocket, imgSize_buff, 4, 0);
	imgSize = ((int *)imgSize_buff)[0];
	iLen = imgSize;
	received = 0;
	longbuff = new char[imgSize];
	while ((iResult = recv(ConnectSocket, longbuff + received, iLen, 0)) > 0)
	{
		iLen -= iResult;
		received += iResult;
		if (iLen <= 0)
			break;
	}
	rawData = cv::Mat(1, imgSize, CV_8UC1, longbuff);
	right = imdecode(rawData, CV_LOAD_IMAGE_COLOR);
	std::cerr << "Right Ok" << std::endl;
	free(longbuff);
}
NetCamera::~NetCamera()
{
	// shutdown the send half of the connection since no more data will be sent
	if (shutdown(ConnectSocket, SD_SEND) == SOCKET_ERROR) {
		printf("shutdown failed: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
	}
	// cleanup
	closesocket(ConnectSocket);
	WSACleanup();
}
