import rospy
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QTableWidgetItem
 # Resolve path to the .ui file in the package
from rospkg import RosPack
from mpc_planner_msgs.msg import MPCMetrics


class MPCPlannerDashboard(Plugin):

    def __init__(self, context):
        # Initialize the base Plugin class
        super().__init__(context)
        # Set a unique object name for this plugin instance
        self.setObjectName('MPCPlannerDashboard')

        # Create QWidget and load UI
        self._widget = QWidget()
        # Get the UI file name from ROS parameters, default to 'mpc_planner_dashboard.ui'
        ui_file = rospy.get_param('~ui_file', 'mpc_planner_dashboard.ui')

        # Find the package path and construct full path to the UI file
        rp = RosPack()
        pkg_path = rp.get_path('mpc_planner_dashboard')
        ui_full_path = pkg_path + '/resource/' + ui_file

        # Load the UI file into the widget (this creates all GUI elements defined in the .ui file)
        loadUi(ui_full_path, self._widget)
        self._widget.setObjectName('MPCPlannerDashboardUi')

        # If multiple instances of this plugin are opened, append a number to the title
        # to distinguish them (e.g., "MPC Planner Dashboard (2)")
        if context.serial_number() > 1:
            current_title = self._widget.windowTitle()
            self._widget.setWindowTitle(f"{current_title} ({context.serial_number()})")
        
        # Register this widget with the rqt_context so it appears in the GUI
        context.add_widget(self._widget)

        # --- GUI initialization ---
        # Populate the robot dropdown with available robot names
        self._widget.robotComboBox.addItems(['jackal1', 'jackal2', 'jackal3'])
        # Connect the dropdown change event to our handler
        self._widget.robotComboBox.currentTextChanged.connect(self.onRobotChanged)

        # Store the currently selected robot name
        self.current_robot = self._widget.robotComboBox.currentText()

        # Set initial status label
        self._widget.statusLabel.setText('● UNKNOWN')
        self._widget.statusLabel.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        # Initialize ROS subscriber (will subscribe to /<robot>/mpc_metrics)
        self._sub = None
        self.subscribeToRobot(self.current_robot)
        
        # Cache for distribution data to avoid unnecessary table updates
        self._last_topology_data = None
        self._last_comm_trigger_data = None

    # ---------------------------------------------------------------------
    # ROS subscription management
    # ---------------------------------------------------------------------
    def onRobotChanged(self, robot_name):
        """Called when user selects a different robot from the dropdown"""
        self.current_robot = robot_name
        # Re-subscribe to the new robot's topic
        self.subscribeToRobot(robot_name)

    def subscribeToRobot(self, robot_name):
        """Subscribe to the MPC metrics topic for the specified robot"""
        # Unsubscribe from previous robot's topic if it exists
        if self._sub is not None:
            self._sub.unregister()
            self._sub = None

        # Construct the topic name: /<robot_name>/mpc_metrics
        topic = f"/{robot_name}/mpc_metrics"
        rospy.loginfo(f"MPCPlannerDashboard subscribing to {topic}")
        # Create new subscriber that calls metricsCallback when messages arrive
        self._sub = rospy.Subscriber(topic, MPCMetrics,
                                     self.metricsCallback,
                                     queue_size=10)

    # ---------------------------------------------------------------------
    # Callback: update GUI
    # ---------------------------------------------------------------------
    def metricsCallback(self, msg):
        """Called whenever a new MPCMetrics message is received from the selected robot"""
        
        try:
            # Update status indicator to show robot is running
            self._widget.statusLabel.setText('● RUNNING')

            # --- Solver Performance Section ---
            # Display solve time in the progress bar with text overlay
            self._widget.solveTimeBar.setFormat(f'{msg.solve_time_ms:.1f} ms')
            # Set bar range and clamp value to prevent overflow
            max_solve = 200.0
            value = max(0.0, min(msg.solve_time_ms, max_solve))
            self._widget.solveTimeBar.setMaximum(max_solve)
            self._widget.solveTimeBar.setValue(value)

            # Display other solver metrics
            # Handle -1 values for unavailable data
            success_text = f'{msg.success_rate:.1f} %' if msg.success_rate >= 0 else 'N/A'
            self._widget.successRateValue.setText(success_text)
            
            iterations_text = str(msg.iterations) if msg.iterations >= 0 else 'N/A'
            self._widget.iterationsValue.setText(iterations_text)
            
            self._widget.exitCodeValue.setText(f'{msg.exit_code:d}')
            self._widget.objectiveValue.setText(f'{msg.objective_value:.3f}')

            # --- Topology Selection Section ---
            # Show which topology (homotopy class) was selected
            self._widget.currentTopologyValue.setText(str(msg.current_topology_id))
            self._widget.previousTopologyValue.setText(str(msg.previous_topology_id))
            self._widget.topologySwitchValue.setText('YES' if msg.topology_switch else 'NO')
            self._widget.usedGuidanceValue.setText('YES' if msg.used_guidance else 'NO')
            self._widget.plannerIndexValue.setText(str(msg.selected_planner_index))
            
            # Display number of guidance trajectories found
            num_guidance = msg.num_of_guidance_found if msg.num_of_guidance_found >= 0 else 'N/A'
            if hasattr(self._widget, 'numGuidanceValue'):
                self._widget.numGuidanceValue.setText(str(num_guidance))
            
            # --- State Information Section ---
            # Display current and previous planner states
            if hasattr(self._widget, 'currentStateValue'):
                self._widget.currentStateValue.setText(msg.current_state)
            if hasattr(self._widget, 'previousStateValue'):
                self._widget.previousStateValue.setText(msg.previous_state)
            
            # Display robot position and velocities
            if hasattr(self._widget, 'positionValue'):
                if len(msg.current_position) >= 2:
                    pos_text = f'({msg.current_position[0]:.2f}, {msg.current_position[1]:.2f})'
                    self._widget.positionValue.setText(pos_text)
                else:
                    self._widget.positionValue.setText('N/A')
            
            if hasattr(self._widget, 'linearVelValue'):
                self._widget.linearVelValue.setText(f'{msg.current_linear_x:.2f} m/s')
            if hasattr(self._widget, 'angularVelValue'):
                self._widget.angularVelValue.setText(f'{msg.current_angular_vel:.2f} rad/s')

            # --- Communication Metrics Section ---
            # Display what triggered the last communication and cumulative statistics
            self._widget.lastTriggerValue.setText(msg.last_communication_trigger)
            self._widget.messagesSentValue.setText(str(msg.messages_sent_total))
            self._widget.messagesSavedValue.setText(str(msg.messages_saved_total))
            self._widget.commSavingsValue.setText(f'{msg.communication_savings_percent:.1f} %')

            # --- Distribution Tables ---
            # Only update if data is available AND has changed (to avoid recursive repaints)
            if hasattr(self._widget, 'topologyTable'):
                topology_data = (tuple(msg.topology_labels), tuple(msg.topology_selection_counts))
                if len(msg.topology_labels) > 0 and topology_data != self._last_topology_data:
                    self._last_topology_data = topology_data
                    self.updateDistributionTable(
                        table=self._widget.topologyTable,
                        labels=msg.topology_labels,
                        counts=msg.topology_selection_counts
                    )

            # Update table showing distribution of communication triggers
            if hasattr(self._widget, 'commTable'):
                comm_data = (tuple(msg.communication_trigger_labels), tuple(msg.communication_trigger_counts))
                if len(msg.communication_trigger_labels) > 0 and comm_data != self._last_comm_trigger_data:
                    self._last_comm_trigger_data = comm_data
                    self.updateDistributionTable(
                        table=self._widget.commTable,
                        labels=msg.communication_trigger_labels,
                        counts=msg.communication_trigger_counts
                    )
            
        except Exception as e:
            rospy.logerr(f"Error in metricsCallback: {e}")
            import traceback
            traceback.print_exc()


    def resetCallback(self, msg):
        if msg.data is True:
            return
        
    def resetAllValues(self):
        return
    
    def updateDistributionTable(self, table, labels, counts):
        """Helper function to populate a table with label-count pairs"""
        # Block signals during update to prevent recursive repaints
        table.blockSignals(True)
        try:
            # Clear existing content
            table.clear()
            # Set table dimensions
            table.setRowCount(len(labels))
            table.setColumnCount(2)
            
            # Set column headers
            table.setHorizontalHeaderLabels(['Label', 'Count'])

            # Populate table rows with data
            for i, (label, count) in enumerate(zip(labels, counts)):
                table.setItem(i, 0, QTableWidgetItem(label))
                table.setItem(i, 1, QTableWidgetItem(str(count)))

            # Auto-resize columns to fit content
            table.resizeColumnsToContents()
        finally:
            # Always re-enable signals
            table.blockSignals(False)

    # ---------------------------------------------------------------------
    # Standard rqt plugin lifecycle
    # ---------------------------------------------------------------------
    def shutdownPlugin(self):
        """Called when the plugin is being closed - cleanup resources"""
        # Unsubscribe from ROS topic to prevent memory leaks
        if self._sub is not None:
            self._sub.unregister()
            self._sub = None

    def saveSettings(self, plugin_settings, instance_settings):
        """Save plugin state when rqt closes (persists across sessions)"""
        # Save the currently selected robot so it's restored next time
        instance_settings.set_value('current_robot', self.current_robot)

    def restoreSettings(self, plugin_settings, instance_settings):
        """Restore plugin state when rqt starts (loads saved settings)"""
        # Restore the previously selected robot from saved settings
        robot = instance_settings.value('current_robot', 'jackal1')
        # Find the robot in the dropdown and select it
        index = self._widget.robotComboBox.findText(robot)
        if index >= 0:
            self._widget.robotComboBox.setCurrentIndex(index)
