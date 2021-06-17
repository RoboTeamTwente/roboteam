//
// Created by rolf on 10-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_INCLUDE_ROBOTEAM_OBSERVER_ROBOCUPRECEIVER_H_
#define RTT_ROBOTEAM_WORLD_INCLUDE_ROBOTEAM_OBSERVER_ROBOCUPRECEIVER_H_
#include <QUdpSocket>
#include <QNetworkProxy>
#include <QNetworkInterface>

template<typename ProtoMessageType>
class RobocupReceiver {
 public:
  RobocupReceiver(const QHostAddress& groupAddress, quint16 port) :
  group_address{groupAddress},
  port{port},
  socket{nullptr}{}
  void disconnect(){
    delete socket;
    socket = nullptr;
  }
  ~RobocupReceiver(){
    disconnect();
  }
  //we cannot copy because this class has unique control over the connection (moving is legal, however)
  RobocupReceiver(const RobocupReceiver& ) = delete;
  RobocupReceiver operator =(const RobocupReceiver&)  = delete;

  bool connect(){
    //first disconnect if necessary
    if(socket != nullptr){
      disconnect();
    }

    socket = new QUdpSocket();

    //proxying won't work according to er-force, no clue what this actually does
    socket->setProxy(QNetworkProxy::NoProxy);

    socket->bind(QHostAddress::AnyIPv4,port,QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    if(socket->state() != QAbstractSocket::BoundState){
      return false; // the socket failed to bind for whatever reason
    }
    if(!group_address.isNull()){
      for(const auto& net_interface : QNetworkInterface::allInterfaces()){
        socket->joinMulticastGroup(group_address,net_interface);
      }
    }
    return true;
  }
  bool receive(std::vector<ProtoMessageType>& packets){
    bool no_error = true;
    while(socket->hasPendingDatagrams()){
      QByteArray data;
      data.resize(socket->pendingDatagramSize());
      qint64 status = socket->readDatagram(data.data(),data.size());
      if(status == -1 ){
        no_error = false;
      }else{
        ProtoMessageType proto_message;
        bool success = proto_message.ParseFromArray(data.data(),data.size());
        if(!success){
          no_error = false;
        }
        packets.push_back(proto_message);
      }
    }
    return no_error;
  }

 private:
  QHostAddress group_address;
  quint16 port;
  QUdpSocket * socket;
};

#endif //RTT_ROBOTEAM_WORLD_INCLUDE_ROBOTEAM_OBSERVER_ROBOCUPRECEIVER_H_
