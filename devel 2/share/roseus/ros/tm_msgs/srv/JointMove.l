;; Auto-generated. Do not edit!


(when (boundp 'tm_msgs::JointMove)
  (if (not (find-package "TM_MSGS"))
    (make-package "TM_MSGS"))
  (shadow 'JointMove (find-package "TM_MSGS")))
(unless (find-package "TM_MSGS::JOINTMOVE")
  (make-package "TM_MSGS::JOINTMOVE"))
(unless (find-package "TM_MSGS::JOINTMOVEREQUEST")
  (make-package "TM_MSGS::JOINTMOVEREQUEST"))
(unless (find-package "TM_MSGS::JOINTMOVERESPONSE")
  (make-package "TM_MSGS::JOINTMOVERESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass tm_msgs::JointMoveRequest
  :super ros::object
  :slots (_jointTarget ))

(defmethod tm_msgs::JointMoveRequest
  (:init
   (&key
    ((:jointTarget __jointTarget) (instance sensor_msgs::JointState :init))
    )
   (send-super :init)
   (setq _jointTarget __jointTarget)
   self)
  (:jointTarget
   (&rest __jointTarget)
   (if (keywordp (car __jointTarget))
       (send* _jointTarget __jointTarget)
     (progn
       (if __jointTarget (setq _jointTarget (car __jointTarget)))
       _jointTarget)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/JointState _jointTarget
    (send _jointTarget :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/JointState _jointTarget
       (send _jointTarget :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/JointState _jointTarget
     (send _jointTarget :deserialize buf ptr-) (incf ptr- (send _jointTarget :serialization-length))
   ;;
   self)
  )

(defclass tm_msgs::JointMoveResponse
  :super ros::object
  :slots (_isOK ))

(defmethod tm_msgs::JointMoveResponse
  (:init
   (&key
    ((:isOK __isOK) nil)
    )
   (send-super :init)
   (setq _isOK __isOK)
   self)
  (:isOK
   (&optional (__isOK :null))
   (if (not (eq __isOK :null)) (setq _isOK __isOK)) _isOK)
  (:serialization-length
   ()
   (+
    ;; bool _isOK
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isOK
       (if _isOK (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isOK
     (setq _isOK (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tm_msgs::JointMove
  :super ros::object
  :slots ())

(setf (get tm_msgs::JointMove :md5sum-) "07f94568e46947f8fdd61bbf89ecd59c")
(setf (get tm_msgs::JointMove :datatype-) "tm_msgs/JointMove")
(setf (get tm_msgs::JointMove :request) tm_msgs::JointMoveRequest)
(setf (get tm_msgs::JointMove :response) tm_msgs::JointMoveResponse)

(defmethod tm_msgs::JointMoveRequest
  (:response () (instance tm_msgs::JointMoveResponse :init)))

(setf (get tm_msgs::JointMoveRequest :md5sum-) "07f94568e46947f8fdd61bbf89ecd59c")
(setf (get tm_msgs::JointMoveRequest :datatype-) "tm_msgs/JointMoveRequest")
(setf (get tm_msgs::JointMoveRequest :definition-)
      "sensor_msgs/JointState jointTarget

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
bool isOK
")

(setf (get tm_msgs::JointMoveResponse :md5sum-) "07f94568e46947f8fdd61bbf89ecd59c")
(setf (get tm_msgs::JointMoveResponse :datatype-) "tm_msgs/JointMoveResponse")
(setf (get tm_msgs::JointMoveResponse :definition-)
      "sensor_msgs/JointState jointTarget

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
bool isOK
")



(provide :tm_msgs/JointMove "07f94568e46947f8fdd61bbf89ecd59c")


