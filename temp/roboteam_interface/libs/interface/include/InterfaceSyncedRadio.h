////
//// Created by Dawid Kulikowski on 04/10/2021.
////
//
//#ifndef RTT_INTERFACESYNCEDRADIO_H
//#define RTT_INTERFACESYNCEDRADIO_H
//#include <QRadioButton>
//#include <QButtonGroup>
//#include "roboteam_interface_utils/InterfaceDeclaration.h"
//#include "InterfaceControllerClient.h"
//#include "MainWindow.h"
//
//namespace rtt::Interface {
//    class InterfaceSyncedRadio: public QButtonGroup {
//        Q_OBJECT
//    public:
//        InterfaceSyncedRadio(std::weak_ptr<InterfaceControllerClient>, std::string, QWidget* = nullptr);
//    private:
//        std::string identity;
//        int dpi;
//
//        std::weak_ptr<InterfaceControllerClient> ctrl;
//
//        void updateProps(const InterfaceDeclaration&);
//    protected slots:
//        void updateDeclaration();
//        void updateValue();
//
//        void notifyChangedValue(int id, bool enabled);
//
//    };
//}
//
//
//#endif  // RTT_INTERFACESYNCEDRADIO_H
