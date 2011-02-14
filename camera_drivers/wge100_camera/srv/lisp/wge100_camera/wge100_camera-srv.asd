
(in-package :asdf)

(defsystem "wge100_camera-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "BoardConfig" :depends-on ("_package"))
    (:file "_package_BoardConfig" :depends-on ("_package"))
    ))
