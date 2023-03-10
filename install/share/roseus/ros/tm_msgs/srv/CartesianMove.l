;; Auto-generated. Do not edit!


(when (boundp 'tm_msgs::CartesianMove)
  (if (not (find-package "TM_MSGS"))
    (make-package "TM_MSGS"))
  (shadow 'CartesianMove (find-package "TM_MSGS")))
(unless (find-package "TM_MSGS::CARTESIANMOVE")
  (make-package "TM_MSGS::CARTESIANMOVE"))
(unless (find-package "TM_MSGS::CARTESIANMOVEREQUEST")
  (make-package "TM_MSGS::CARTESIANMOVEREQUEST"))
(unless (find-package "TM_MSGS::CARTESIANMOVERESPONSE")
  (make-package "TM_MSGS::CARTESIANMOVERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass tm_msgs::CartesianMoveRequest
  :super ros::object
  :slots (_poseTarget ))

(defmethod tm_msgs::CartesianMoveRequest
  (:init
   (&key
    ((:poseTarget __poseTarget) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _poseTarget __poseTarget)
   self)
  (:poseTarget
   (&rest __poseTarget)
   (if (keywordp (car __poseTarget))
       (send* _poseTarget __poseTarget)
     (progn
       (if __poseTarget (setq _poseTarget (car __poseTarget)))
       _poseTarget)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _poseTarget
    (send _poseTarget :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _poseTarget
       (send _poseTarget :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _poseTarget
     (send _poseTarget :deserialize buf ptr-) (incf ptr- (send _poseTarget :serialization-length))
   ;;
   self)
  )

(defclass tm_msgs::CartesianMoveResponse
  :super ros::object
  :slots (_isOK ))

(defmethod tm_msgs::CartesianMoveResponse
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

(defclass tm_msgs::CartesianMove
  :super ros::object
  :slots ())

(setf (get tm_msgs::CartesianMove :md5sum-) "7c6bc7eb9aa36ece17e84459bdba640a")
(setf (get tm_msgs::CartesianMove :datatype-) "tm_msgs/CartesianMove")
(setf (get tm_msgs::CartesianMove :request) tm_msgs::CartesianMoveRequest)
(setf (get tm_msgs::CartesianMove :response) tm_msgs::CartesianMoveResponse)

(defmethod tm_msgs::CartesianMoveRequest
  (:response () (instance tm_msgs::CartesianMoveResponse :init)))

(setf (get tm_msgs::CartesianMoveRequest :md5sum-) "7c6bc7eb9aa36ece17e84459bdba640a")
(setf (get tm_msgs::CartesianMoveRequest :datatype-) "tm_msgs/CartesianMoveRequest")
(setf (get tm_msgs::CartesianMoveRequest :definition-)
      "geometry_msgs/Pose poseTarget

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool isOK
")

(setf (get tm_msgs::CartesianMoveResponse :md5sum-) "7c6bc7eb9aa36ece17e84459bdba640a")
(setf (get tm_msgs::CartesianMoveResponse :datatype-) "tm_msgs/CartesianMoveResponse")
(setf (get tm_msgs::CartesianMoveResponse :definition-)
      "geometry_msgs/Pose poseTarget

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool isOK
")



(provide :tm_msgs/CartesianMove "7c6bc7eb9aa36ece17e84459bdba640a")


