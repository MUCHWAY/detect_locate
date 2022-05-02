// Definition of the ClientSocket class

#ifndef ClientSocket_class
#define ClientSocket_class

#include "gambal_control/Socket.h"


class ClientSocket : private Socket
{
public:

    ClientSocket ( std::string host, int port );
    virtual ~ClientSocket(){};

    const ClientSocket& operator << ( const std::string& ) const;
    const ClientSocket& operator >> ( std::string& ) const;
    const ClientSocket& operator << ( const cv::Mat& ) const;
    const ClientSocket& operator >> ( cv::Mat& ) const;
    const void sendBuffer ( const uint8_t* , int ) const;



};


#endif

