#include "urc_util/ethernet_socket.hpp"

namespace ip = boost::asio::ip;

EthernetSocket::EthernetSocket(std::string ip_addr, int port)
{
  // resolve all possible endpoints
  ip::udp::resolver resolver(io_service_);
  ip::udp::resolver::query query(ip_addr, std::to_string(port));
  ip::udp::resolver::iterator endpoint_iterator = resolver.resolve(query);

  // look through endpoints and hit socket's connect() member function until
  // a successful UDP connection is established
  this->sock_ = std::make_unique<ip::udp::socket>(io_service_);
  boost::asio::connect(*sock_, endpoint_iterator);
}

EthernetSocket::~EthernetSocket()
{
  // shut down the UDP connection
  this->sock_->shutdown(ip::udp::socket::shutdown_send);
}

void EthernetSocket::sendMessage(char * message, size_t len)
{
  boost::system::error_code error;
  sock_->send(boost::asio::buffer(message, len), 0, error);
}

size_t EthernetSocket::readMessage(unsigned char (& buffer)[256])
{
  // read data from UDP connection
  boost::system::error_code error;
  size_t len = sock_->receive(boost::asio::buffer(buffer, sizeof(buffer) - 1), 0, error);

  if (error == boost::asio::error::eof) {
    len = 0;
  } else if (error) {
    throw boost::system::system_error(error);
  }

  return len;
}

std::string EthernetSocket::getIP()
{
  return sock_->remote_endpoint().address().to_string();
}

int EthernetSocket::getPort()
{
  return static_cast<int>(sock_->remote_endpoint().port());
}

std::string EthernetSocket::getBoostVersion()
{
  std::stringstream version;
  version << BOOST_VERSION / 100000 << "." <<  // major version
    BOOST_VERSION / 100 % 1000 << "." <<       // minor version
    BOOST_VERSION % 100;                       // patch level

  return version.str();
}
