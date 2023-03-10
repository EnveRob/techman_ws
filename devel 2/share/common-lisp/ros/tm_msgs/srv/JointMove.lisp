; Auto-generated. Do not edit!


(cl:in-package tm_msgs-srv)


;//! \htmlinclude JointMove-request.msg.html

(cl:defclass <JointMove-request> (roslisp-msg-protocol:ros-message)
  ((jointTarget
    :reader jointTarget
    :initarg :jointTarget
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass JointMove-request (<JointMove-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointMove-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointMove-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tm_msgs-srv:<JointMove-request> is deprecated: use tm_msgs-srv:JointMove-request instead.")))

(cl:ensure-generic-function 'jointTarget-val :lambda-list '(m))
(cl:defmethod jointTarget-val ((m <JointMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tm_msgs-srv:jointTarget-val is deprecated.  Use tm_msgs-srv:jointTarget instead.")
  (jointTarget m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointMove-request>) ostream)
  "Serializes a message object of type '<JointMove-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'jointTarget) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointMove-request>) istream)
  "Deserializes a message object of type '<JointMove-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'jointTarget) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointMove-request>)))
  "Returns string type for a service object of type '<JointMove-request>"
  "tm_msgs/JointMoveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointMove-request)))
  "Returns string type for a service object of type 'JointMove-request"
  "tm_msgs/JointMoveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointMove-request>)))
  "Returns md5sum for a message object of type '<JointMove-request>"
  "07f94568e46947f8fdd61bbf89ecd59c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointMove-request)))
  "Returns md5sum for a message object of type 'JointMove-request"
  "07f94568e46947f8fdd61bbf89ecd59c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointMove-request>)))
  "Returns full string definition for message of type '<JointMove-request>"
  (cl:format cl:nil "sensor_msgs/JointState jointTarget~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointMove-request)))
  "Returns full string definition for message of type 'JointMove-request"
  (cl:format cl:nil "sensor_msgs/JointState jointTarget~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointMove-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'jointTarget))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointMove-request>))
  "Converts a ROS message object to a list"
  (cl:list 'JointMove-request
    (cl:cons ':jointTarget (jointTarget msg))
))
;//! \htmlinclude JointMove-response.msg.html

(cl:defclass <JointMove-response> (roslisp-msg-protocol:ros-message)
  ((isOK
    :reader isOK
    :initarg :isOK
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass JointMove-response (<JointMove-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointMove-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointMove-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tm_msgs-srv:<JointMove-response> is deprecated: use tm_msgs-srv:JointMove-response instead.")))

(cl:ensure-generic-function 'isOK-val :lambda-list '(m))
(cl:defmethod isOK-val ((m <JointMove-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tm_msgs-srv:isOK-val is deprecated.  Use tm_msgs-srv:isOK instead.")
  (isOK m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointMove-response>) ostream)
  "Serializes a message object of type '<JointMove-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isOK) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointMove-response>) istream)
  "Deserializes a message object of type '<JointMove-response>"
    (cl:setf (cl:slot-value msg 'isOK) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointMove-response>)))
  "Returns string type for a service object of type '<JointMove-response>"
  "tm_msgs/JointMoveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointMove-response)))
  "Returns string type for a service object of type 'JointMove-response"
  "tm_msgs/JointMoveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointMove-response>)))
  "Returns md5sum for a message object of type '<JointMove-response>"
  "07f94568e46947f8fdd61bbf89ecd59c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointMove-response)))
  "Returns md5sum for a message object of type 'JointMove-response"
  "07f94568e46947f8fdd61bbf89ecd59c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointMove-response>)))
  "Returns full string definition for message of type '<JointMove-response>"
  (cl:format cl:nil "bool isOK~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointMove-response)))
  "Returns full string definition for message of type 'JointMove-response"
  (cl:format cl:nil "bool isOK~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointMove-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointMove-response>))
  "Converts a ROS message object to a list"
  (cl:list 'JointMove-response
    (cl:cons ':isOK (isOK msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'JointMove)))
  'JointMove-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'JointMove)))
  'JointMove-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointMove)))
  "Returns string type for a service object of type '<JointMove>"
  "tm_msgs/JointMove")