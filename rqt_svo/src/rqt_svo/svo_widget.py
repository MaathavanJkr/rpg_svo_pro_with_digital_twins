#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer, Slot
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from svo_msgs.msg import Info
from std_msgs.msg import String

class SvoWidget(QWidget):
  _last_info_msg = Info()
  _publisher = None
  _subscriber = None
  _num_received_msgs = 0
  _svo_namespace = None
  _param_client = None
  def __init__(self, node, svo_namespace='svo'):

    # Init QWidget
    super(SvoWidget, self).__init__()
    self.setObjectName('SvoWidget')
    self._node = node

    # load UI
    ui_file = os.path.join(get_package_share_directory('rqt_svo'), 'resource', 'widget.ui')
    loadUi(ui_file, self)

    # init and start update timer for data, the timer calls the function update_info all 40ms
    self._update_info_timer = QTimer(self)
    self._update_info_timer.timeout.connect(self.update_info)
    self._update_info_timer.start(40)

    # set the functions that are called when a button is pressed
    self.button_start.pressed.connect(self.on_start_button_pressed)
    self.button_reset.pressed.connect(self.on_reset_button_pressed)
    self.button_quit.pressed.connect(self.on_quit_button_pressed)

    # set callback for changed topic
    self._svo_namespace = svo_namespace
    self.topic_line_edit.textChanged.connect(self._on_topic_changed)
    self.register(svo_namespace)
    self.topic_line_edit.setText(svo_namespace)

    # TODO: set a timer when the last message was received and give a warning if it is too long ago!

  @Slot(str)
  def _on_topic_changed(self, topic):
    self._svo_namespace = str(topic)
    self.unregister()
    self.register(self._svo_namespace)

  def register(self, svo_namespace):
    # Load parameters
    if not svo_namespace:
        svo_namespace = 'svo'

    # Feature bar (the maximum is updated once the svo node answers)
    self.num_tracked_bar.setMaximum(120)
    self._request_max_num_features(svo_namespace)

    # Subscribe to ROS Info topic and register callback
    self._subscriber = self._node.create_subscription(Info, svo_namespace+'/info', self.info_cb, 10)

    # Initialize Publisher
    self._publisher = self._node.create_publisher(String, svo_namespace+'/remote_key', 1)

  def _request_max_num_features(self, svo_namespace):
    # ROS 2 has no global parameter server: ask the svo node itself.
    self._param_client = self._node.create_client(GetParameters, svo_namespace+'/get_parameters')
    if not self._param_client.service_is_ready():
      return
    request = GetParameters.Request(names=['max_fts'])
    self._param_client.call_async(request).add_done_callback(self._on_max_num_features)

  def _on_max_num_features(self, future):
    result = future.result()
    if result is None or not result.values:
      return
    value = result.values[0]
    if value.type == ParameterType.PARAMETER_INTEGER:
      self.num_tracked_bar.setMaximum(value.integer_value)
      print('set maximum number of features to '+str(value.integer_value))

  def unregister(self):
    if self._publisher is not None:
      self._node.destroy_publisher(self._publisher)
      self._publisher = None

    if self._subscriber is not None:
      self._node.destroy_subscription(self._subscriber)
      self._subscriber = None

    if self._param_client is not None:
      self._node.destroy_client(self._param_client)
      self._param_client = None

  def shutdown(self):
    self._update_info_timer.stop()
    self.unregister()


  def info_cb(self, msg):
    self._last_info_msg = msg
    self._num_received_msgs += 1

  def update_info(self):
    info_text = 'Not Connected'
    if self._num_received_msgs > 0:
      fps = 0
      if self._last_info_msg.processing_time > 0:
        fps = 1.0/self._last_info_msg.processing_time
      info_text = 'fps = %.2f' % fps
      info_text += '\t #Features = ' + str(self._last_info_msg.num_matches)
      info_text += '\t'
      if self._last_info_msg.stage == 0:
        info_text += '\t PAUSED'
      elif self._last_info_msg.stage == 1:
        info_text += '\t FIRST_FRAME'
      elif self._last_info_msg.stage == 2:
        info_text += '\t SECOND_FRAME'
      elif self._last_info_msg.stage == 3:
        info_text += '\t RUNNING'
      if self._last_info_msg.tracking_quality == 0:
        info_text += '\t CRITICAL'
      elif self._last_info_msg.tracking_quality == 1:
        info_text += '\t BAD TRACKING'
      elif self._last_info_msg.tracking_quality == 2:
        info_text += '\t GOOD TRACKING'

    # set info text
    self.svo_info_label.setText(info_text)

    # set progress bar
    self.num_tracked_bar.setValue(self._last_info_msg.num_matches)

  def on_start_button_pressed(self):
    print('START SLAM')
    self.send_command('s')

  def on_reset_button_pressed(self):
    print('RESET SLAM')
    self.send_command('r')

  def on_quit_button_pressed(self):
    print('QUIT SLAM')
    self.send_command('q')

  def send_command(self, cmd):
    if self._publisher is None:
      return
    self._publisher.publish(String(data=cmd))
