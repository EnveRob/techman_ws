; Auto-generated. Do not edit!


(cl:in-package tm_msgs-srv)


;//! \htmlinclude CartesianMove-request.msg.html

(cl:defclass <CartesianMove-request> (roslisp-msg-protocol:ros-message)
  ((poseTarget
    :reader poseTarget
    :initarg :poseTarget
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass CartesianMove-request (<CartesianMove-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianMove-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianMove-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tm_msgs-srv:<CartesianMove-request> is deprecated: use tm_msgs-srv:CartesianMove-request instead.")))

(cl:ensure-generic-function 'poseTarget-val :lambda-list '(m))
(cl:defmethod poseTarget-val ((m <CartesianMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tm_msgs-srv:poseTarget-val is deprecated.  Use tm_msgs-srv:poseTarget instead.")
  (poseTarget m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianMove-request>) ostream)
  "Serializes a message object of type '<CartesianMove-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'poseTarget) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianMove-request>) istream)
  "Deserializes a message object of type '<CartesianMove-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'poseTarget) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianMove-request>)))
  "Returns string type for a service object of type '<CartesianMove-request>"
  "tm_msgs/CartesianMoveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianMove-request)))
  "Returns string type for a service object of type 'CartesianMove-request"
  "tm_msgs/CartesianMoveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianMove-request>)))
  "Returns md5sum for a message object of type '<CartesianMove-request>"
  "7c6bc7eb9aa36ece17e84459bdba640a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianMove-request)))
  "Returns md5sum for a message object of type 'CartesianMove-request"
  "7c6bc7eb9aa36ece17e84459bdba640a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianMove-request>)))
  "Returns full string definition for message of type '<CartesianMove-request>"
  (cl:format cl:nil "geometry_msgs/Pose poseTarget~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianMove-request)))
  "Returns full string definition for message of type 'CartesianMove-request"
  (cl:format cl:nil "geometry_msgs/Pose poseTarget~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianMove-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'poseTarget))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianMove-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianMove-request
    (cl:cons ':poseTarget (poseTarget msg))
))
;//! \htmlinclude CartesianMove-response.msg.html

(cl:defclass <CartesianMove-response> (roslisp-msg-protocol:ros-message)
  ((isOK
    :reader isOK
    :initarg :isOK
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CartesianMove-response (<CartesianMove-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianMove-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianMove-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tm_msgs-srv:<CartesianMove-response> is deprecated: use tm_msgs-srv:CartesianMove-response instead.")))

(cl:ensure-generic-function 'isOK-val :lambda-list '(m))
(cl:defmethod isOK-val ((m <CartesianMove-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tm_msgs-srv:isOK-val is deprecated.  Use tm_msgs-srv:isOK instead.")
  (isOK m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianMove-response>) ostream)
  "Serializes a message object of type '<CartesianMove-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isOK) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianMove-response>) istream)
  "Deserializes a message object of type '<CartesianMove-response>"
    (cl:setf (cl:slot-value msg 'isOK) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianMove-response>)))
  "Returns string type for a service object of type '<CartesianMove-response>"
  "tm_msgs/CartesianMoveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianMove-response)))
  "Returns string type for a service object of type 'CartesianMove-response"
  "tm_msgs/CartesianMoveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianMove-response>)))
  "Returns md5sum for a message object of type '<CartesianMove-response>"
  "7c6bc7eb9aa36ece17e84459bdba640a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianMove-response)))
  "Returns md5sum for a message object of type 'CartesianMove-response"
  "7c6bc7eb9aa36ece17e84459bdba640a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianMove-response>)))
  "Returns full string definition for message of type '<CartesianMove-response>"
  (cl:format cl:nil "bool isOK~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianMove-response)))
  "Returns full string definition for message of type 'CartesianMove-response"
  (cl:format cl:nil "bool isOK~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianMove-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianMove-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianMove-response
    (cl:cons ':isOK (isOK msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CartesianMove)))
  'CartesianMove-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CartesianMove)))
  'CartesianMove-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianMove)))
  "Returns string type for a service object of type '<CartesianMove>"
  "tm_msgs/CartesianMove")