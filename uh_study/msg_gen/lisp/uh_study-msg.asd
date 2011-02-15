
(cl:in-package :asdf)

(defsystem "uh_study-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FacePose" :depends-on ("_package_FacePose"))
    (:file "_package_FacePose" :depends-on ("_package"))
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
  ))