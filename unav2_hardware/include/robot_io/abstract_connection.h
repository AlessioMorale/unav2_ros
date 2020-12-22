#ifndef ABSTRACT_CONNECTION_H
#define ABSTRACT_CONNECTION_H
#include <functional>
#include <map>
#include <vector>
namespace robot_io::connection {

class AbstractConnection {
protected:
  AbstractConnection(){};

public:
  virtual ~AbstractConnection() {};
  virtual void set_receive_callback(std::function<void(const std::vector<uint8_t>&)> callback) = 0;
  virtual void transmit(std::vector<uint8_t>&) = 0;
  virtual bool is_connected() = 0;
};
};     // namespace robot_io::connection
#endif /* ABSTRACT_CONNECTION_H */
