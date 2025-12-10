import rospy
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (
    QWidget, QTableWidgetItem, QProgressBar, QLabel, 
    QHBoxLayout, QVBoxLayout, QGroupBox, QSizePolicy
)
from python_qt_binding.QtCore import Qt, Signal, QObject
from python_qt_binding.QtGui import QPalette, QColor

 # Resolve path to the .ui file in the package
from rospkg import RosPack
from mpc_planner_msgs.msg import MPCMetrics


class MetricsSignalBridge(QObject):
    """
    Bridge class to safely pass ROS messages to the Qt main thread.
    
    ROS callbacks run in a separate spinner thread, but Qt widgets can only
    be safely updated from the main GUI thread. This bridge uses Qt's signal-slot
    mechanism with QueuedConnection to marshal data between threads.
    """
    metrics_received = Signal(object)  # Signal that carries the MPCMetrics message


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
        
        # Flag to track if plugin is shutting down (prevents GUI updates after widget destruction)
        self._is_shutdown = False
        
        # Create signal bridge for thread-safe GUI updates
        # ROS callbacks run in a separate spinner thread, but Qt widgets can only
        # be safely modified from the main GUI thread. This bridge marshals data.
        self._signal_bridge = MetricsSignalBridge()
        self._signal_bridge.metrics_received.connect(
            self._handle_metrics_in_main_thread, 
            Qt.QueuedConnection  # Ensures slot runs in receiver's thread (main thread)
        )
        
        self.subscribeToRobot(self.current_robot)
        
        # Cache for distribution data to avoid unnecessary table updates
        self._last_topology_data = None
        self._last_comm_trigger_data = None
        
        # Storage for dynamically created objective cost widgets
        self._objective_bars = {}  # Dict[str, QProgressBar] - maps planner name to progress bar
        self._objective_labels = {}  # Dict[str, QLabel] - maps planner name to label
        
        # Reference the UI-defined container for objective cost bars
        self._initializeObjectiveCostsFromUI()

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
        # queue_size=1 prevents message buildup if GUI updates are slow
        self._sub = rospy.Subscriber(topic, MPCMetrics,
                                     self.metricsCallback,
                                     queue_size=1)

    # ---------------------------------------------------------------------
    # Objective Costs UI Initialization
    # ---------------------------------------------------------------------
    def _initializeObjectiveCostsFromUI(self):
        """
        Initialize references to the objective costs container defined in the .ui file.
        The group box and layout are defined in mpc_planner_dashboard.ui, 
        and progress bars will be added dynamically as data arrives.
        """
        # Get references to UI-defined widgets
        self._objective_costs_group = self._widget.objectiveCostsGroupBox
        self._objective_costs_layout = self._widget.objectiveCostsLayout
        self._objective_costs_placeholder = self._widget.objectiveCostsPlaceholder

    def _getOrCreateObjectiveBar(self, planner_name):
        """
        Get an existing progress bar for the planner, or create a new one.
        Returns the progress bar widget.
        """
        if planner_name in self._objective_bars:
            return self._objective_bars[planner_name]
        
        # Hide placeholder if this is the first bar
        if self._objective_costs_placeholder.isVisible():
            self._objective_costs_placeholder.hide()
        
        # Create a horizontal layout for this planner's row
        row_layout = QHBoxLayout()
        row_layout.setSpacing(8)
        
        # Create label for planner name (fixed width for alignment)
        label = QLabel(planner_name)
        label.setMinimumWidth(100)
        label.setMaximumWidth(150)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row_layout.addWidget(label)
        
        # Create progress bar for objective value
        bar = QProgressBar()
        bar.setMinimum(-100)  # Scaled: -1.0 maps to -100
        bar.setMaximum(3000)  # Scaled: 30.0 maps to 3000
        bar.setValue(0)
        bar.setFormat("%.2f")  # Will be overwritten with actual value
        bar.setTextVisible(True)
        bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        bar.setMinimumHeight(20)
        
        # Style the bar with color coding
        self._styleObjectiveBar(bar, 0.0)
        
        row_layout.addWidget(bar)
        
        # Store references
        self._objective_bars[planner_name] = bar
        self._objective_labels[planner_name] = label
        
        # Add row to the main layout
        self._objective_costs_layout.addLayout(row_layout)
        
        return bar
    
    def _styleObjectiveBar(self, bar, value):
        """
        Apply color styling to progress bar based on value.
        - value < 0: Red (infeasible/no guidance)
        - value 0-10: Green (good)
        - value 10-20: Yellow (moderate)
        - value > 20: Orange/Red (high cost)
        """
        if value < 0:
            # Red for infeasible
            bar.setStyleSheet("""
                QProgressBar {
                    border: 1px solid grey;
                    border-radius: 3px;
                    text-align: center;
                    background-color: #3a3a3a;
                }
                QProgressBar::chunk {
                    background-color: #cc4444;
                }
            """)
        elif value < 10:
            # Green for good
            bar.setStyleSheet("""
                QProgressBar {
                    border: 1px solid grey;
                    border-radius: 3px;
                    text-align: center;
                    background-color: #3a3a3a;
                }
                QProgressBar::chunk {
                    background-color: #44aa44;
                }
            """)
        elif value < 20:
            # Yellow for moderate
            bar.setStyleSheet("""
                QProgressBar {
                    border: 1px solid grey;
                    border-radius: 3px;
                    text-align: center;
                    background-color: #3a3a3a;
                }
                QProgressBar::chunk {
                    background-color: #aaaa44;
                }
            """)
        else:
            # Orange for high cost
            bar.setStyleSheet("""
                QProgressBar {
                    border: 1px solid grey;
                    border-radius: 3px;
                    text-align: center;
                    background-color: #3a3a3a;
                }
                QProgressBar::chunk {
                    background-color: #cc8844;
                }
            """)
    
    def _updateObjectiveCostBars(self, planner_names, planner_objective_values):
        """
        Update the objective cost progress bars with new data.
        Creates new bars as needed for new planner names.
        
        Args:
            planner_names: List of planner name strings
            planner_objective_values: List of corresponding objective values
        """
        # Safety check: arrays must be same length
        if len(planner_names) != len(planner_objective_values):
            rospy.logwarn_throttle(5.0, 
                f"Planner names ({len(planner_names)}) and values ({len(planner_objective_values)}) length mismatch")
            return
        
        # Update each planner's bar
        for name, value in zip(planner_names, planner_objective_values):
            bar = self._getOrCreateObjectiveBar(name)
            
            # Scale value for the progress bar (multiply by 100 for precision)
            # Range: -1.0 to 30.0 maps to -100 to 3000
            scaled_value = int(value * 100)
            scaled_value = max(-100, min(3000, scaled_value))  # Clamp to range
            bar.setValue(scaled_value)
            
            # Update the display format with actual value
            if value < 0:
                bar.setFormat(f"{value:.2f} (N/A)")
            else:
                bar.setFormat(f"{value:.2f}")
            
            # Update styling based on value
            self._styleObjectiveBar(bar, value)

    # ---------------------------------------------------------------------
    # Callback: update GUI (thread-safe via signal bridge)
    # ---------------------------------------------------------------------
    def metricsCallback(self, msg):
        """
        Called in ROS spinner thread when a new MPCMetrics message arrives.
        
        IMPORTANT: This callback runs in a background thread (ROS spinner),
        NOT the Qt main thread. Directly updating Qt widgets here causes
        "QWidget::repaint: Recursive repaint detected" crashes.
        
        Instead, we emit a signal that will be delivered to _handle_metrics_in_main_thread
        via QueuedConnection, ensuring the GUI update runs in the main thread.
        """
        # Don't emit signals if plugin is shutting down
        if self._is_shutdown:
            return
        # Emit signal to transfer message to main thread for safe GUI updates
        self._signal_bridge.metrics_received.emit(msg)
    
    def _handle_metrics_in_main_thread(self, msg):
        """
        Slot that handles MPCMetrics in the Qt main thread.
        
        This method is connected to MetricsSignalBridge.metrics_received with
        Qt.QueuedConnection, ensuring it runs in the main GUI thread even though
        the signal was emitted from the ROS spinner thread.
        """
        # Check if plugin is shutting down or widget was destroyed
        if self._is_shutdown:
            return
        
        try:
            # Additional check: verify widget still exists
            if self._widget is None:
                return
            
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
            
            # --- Planner Objective Costs Section ---
            # Update progress bars showing objective cost for each planner
            # Safely extract planner_names and planner_objective_values arrays
            if hasattr(msg, 'planner_names') and hasattr(msg, 'planner_objective_values'):
                if len(msg.planner_names) > 0:
                    self._updateObjectiveCostBars(
                        planner_names=list(msg.planner_names),
                        planner_objective_values=list(msg.planner_objective_values)
                    )
            
        except Exception as e:
            rospy.logerr(f"Error in _handle_metrics_in_main_thread: {e}")
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
        # Set shutdown flag FIRST to stop processing new messages
        self._is_shutdown = True
        
        # Disconnect signal to prevent queued messages from being processed
        try:
            self._signal_bridge.metrics_received.disconnect(self._handle_metrics_in_main_thread)
        except (TypeError, RuntimeError):
            pass  # Signal was already disconnected
        
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
