
(cl:in-package :asdf)

(defsystem "serl_franka_controllers-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ZeroJacobian" :depends-on ("_package_ZeroJacobian"))
    (:file "_package_ZeroJacobian" :depends-on ("_package"))
  ))