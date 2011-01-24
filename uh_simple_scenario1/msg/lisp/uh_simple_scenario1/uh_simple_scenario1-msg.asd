
(in-package :asdf)

(defsystem "uh_simple_scenario1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "Num" :depends-on ("_package"))
    (:file "_package_Num" :depends-on ("_package"))
    ))
