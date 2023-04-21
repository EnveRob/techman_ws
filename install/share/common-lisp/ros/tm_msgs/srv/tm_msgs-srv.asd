
(cl:in-package :asdf)

(defsystem "tm_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "AskItem" :depends-on ("_package_AskItem"))
    (:file "_package_AskItem" :depends-on ("_package"))
    (:file "AskSta" :depends-on ("_package_AskSta"))
    (:file "_package_AskSta" :depends-on ("_package"))
    (:file "CartesianMove" :depends-on ("_package_CartesianMove"))
    (:file "_package_CartesianMove" :depends-on ("_package"))
    (:file "ConnectTM" :depends-on ("_package_ConnectTM"))
    (:file "_package_ConnectTM" :depends-on ("_package"))
    (:file "JointMove" :depends-on ("_package_JointMove"))
    (:file "_package_JointMove" :depends-on ("_package"))
    (:file "SendScript" :depends-on ("_package_SendScript"))
    (:file "_package_SendScript" :depends-on ("_package"))
    (:file "SetEvent" :depends-on ("_package_SetEvent"))
    (:file "_package_SetEvent" :depends-on ("_package"))
    (:file "SetIO" :depends-on ("_package_SetIO"))
    (:file "_package_SetIO" :depends-on ("_package"))
    (:file "SetPositions" :depends-on ("_package_SetPositions"))
    (:file "_package_SetPositions" :depends-on ("_package"))
    (:file "WriteItem" :depends-on ("_package_WriteItem"))
    (:file "_package_WriteItem" :depends-on ("_package"))
  ))