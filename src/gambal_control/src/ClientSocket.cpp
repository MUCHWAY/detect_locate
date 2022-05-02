// Implementation of the ClientSocket class

#include "gambal_control/ClientSocket.h"
#include "gambal_control/SocketException.h"

ClientSocket::ClientSocket(std::string host, int port)
{
  if (!Socket::create())
  {
    throw SocketException("Could not create client socket.");
  }

  /*if ( ! Socket::connect ( host, port ) )
    {
      throw SocketException ( "Could not bind to port." );
    }*/
  while (!Socket::connect(host, port))
  {
    std::cout << "Could not bind to port" << std::endl;
  }
  std::cout << "succeed to bind to port" << std::endl;
}

const ClientSocket &ClientSocket::operator<<(const std::string &s) const
{
  if (!Socket::send(s))
  {
    throw SocketException("Could not write to socket.");
  }

  return *this;
}
const ClientSocket &ClientSocket::operator<<(const cv::Mat &mat) const
{
  if (!Socket::send(mat))
  {
    throw SocketException("Could not write to socket.");
  }

  return *this;
}
const void ClientSocket::sendBuffer(const uint8_t* s, int length) const
{
  if (!Socket::send(s,length))
  {
    throw SocketException("Could not write to socket.");
  }
}

const ClientSocket &ClientSocket::operator>>(std::string &s) const
{
  if (!Socket::recv(s))
  {
    throw SocketException("Could not read from socket.");
  }

  return *this;
}

const ClientSocket &ClientSocket::operator>>(cv::Mat &mat) const
{
  if (!Socket::recv(mat))
  {
    throw SocketException("Could not read from socket.");
  }

  return *this;
}
