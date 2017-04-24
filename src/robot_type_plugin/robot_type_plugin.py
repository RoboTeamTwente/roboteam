import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtWidgets

from widgets import robot_type_widget

class RobotTypePlugin(Plugin):

    def __init__(self, context):
        super(RobotTypePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RobotTypePlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QtWidgets.QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('roboteam_utils'), 'resource', 'RobotTypePlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RobotTypePlugin')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self.robot_type_selectors = []

        global_robot_type = robot_type_widget.RobotTypeWidget(-1, True)
        self.robot_type_selectors.append(global_robot_type)
        self._widget.layout().addWidget(global_robot_type)

        for i in range(0, 16):
            new_robot_type = robot_type_widget.RobotTypeWidget(i, False)
            self.robot_type_selectors.append(new_robot_type)
            self._widget.layout().addWidget(new_robot_type)

        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here

        for robot_type_widget in self.robot_type_selectors:
            robot_type_widget.shutdown_widget()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
