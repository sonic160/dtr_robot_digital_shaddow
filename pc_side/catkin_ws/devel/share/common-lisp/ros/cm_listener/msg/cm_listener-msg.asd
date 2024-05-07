
(cl:in-package :asdf)

(defsystem "cm_listener-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "msg_cm" :depends-on ("_package_msg_cm"))
    (:file "_package_msg_cm" :depends-on ("_package"))
  ))