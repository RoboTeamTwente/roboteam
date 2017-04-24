import rospy

from python_qt_binding import QtWidgets, QtGui, QtCore
from python_qt_binding.QtWidgets import QMessageBox
from python_qt_binding.QtCore import pyqtSlot


class RobotTypeWidget(QtWidgets.QFrame):

    def __init__(self, input_id):
        super(RobotTypeWidget, self).__init__()

        self._id = input_id

        self.setLayout(QtWidgets.QHBoxLayout())

        self._id_label = QtWidgets.QLabel("ID: " + str(self._id))
        self.layout().addWidget(self._id_label)

        # ---- Type selector ----

        self._type_selector = QtWidgets.QComboBox()
        self.layout().addWidget(self._type_selector)

        self._type_selector.insertItem(0, "arduino")
        self._type_selector.insertItem(0, "proto")
        self._type_selector.insertItem(0, "grsim")
        self._type_selector.insertItem(0, "unset")

        self.update_robot_type()

        # ---- Timer callback ----
        
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.update_robot_type)
        self._timer.start(200)

        # ---- /Timer callback ----

    def get_param_base(self):
        return "robot" + str(self._id) + "/"


    def change_robot_type(self, type_index):
        robot_type = self._type_selector.itemText(type_index)
        param_name = self.get_param_base() + "robotType"

        if robot_type == "unset":
            rospy.delete_param(param_name)
        else:
            rospy.set_param(param_name, robot_type)

    def update_robot_type(self):
        # Disconnect the signal so we can change the checkbox
        if self._type_selector.receivers(self._type_selector.currentIndexChanged) > 0:
            self._type_selector.currentIndexChanged.disconnect(self.change_robot_type)
        
        param_name = self.get_param_base() + "robotType"

        if rospy.has_param(param_name):
            currentRobotType = rospy.get_param(self.get_param_base() + "robotType");
            currentRobotTypeIndex = self._type_selector.findText(currentRobotType)

            if currentRobotTypeIndex != -1 and not currentRobotType == "unset":
                # If the param is there and is not equal to "unset" we set the selector to it
                self._type_selector.setCurrentIndex(currentRobotTypeIndex)
            elif currentRobotTypeIndex == -1 or currentRobotType == "unset":
                # If the type is unknown or is equal to "unset"
                # We show a messagebox because something is wrong
                # And we delete the param

                # Show warning message box
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Warning)
                msg.setText("Robot " 
                        + str(self._id) 
                        + " had an invalid output type ("
                        + currentRobotType
                        + "). The param will now be deleted."
                        )
                msg.setWindowTitle("Invalid robot type in param")
                msg.setStandardButtons(QMessageBox.Ok)

                # Execute it
                retval = msg.exec_() 

                # Delete the param and set the selector to unset
                rospy.delete_param(param_name)
                self._type_selector.setCurrentIndex(self._type_selector.findText("unset"))
        else:
            # If We cannot find the param it's unset
            self._type_selector.setCurrentIndex(self._type_selector.findText("unset"))

        # Hook up the signal again to receive changed events from the selector
        self._type_selector.currentIndexChanged.connect(self.change_robot_type)

    def shutdown_widget(self):
        self._timer.stop()

