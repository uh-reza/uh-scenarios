
(in-package :asdf)

(defsystem "uh_simple_scenario1-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg :sensor_msgs-msg)
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "FacePose" :depends-on ("_package"))
    (:file "_package_FacePose" :depends-on ("_package"))
    ))
