
(cl:in-package :asdf)

(defsystem "uh_study-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "FacePoseSrv" :depends-on ("_package_FacePoseSrv"))
    (:file "_package_FacePoseSrv" :depends-on ("_package"))
  ))