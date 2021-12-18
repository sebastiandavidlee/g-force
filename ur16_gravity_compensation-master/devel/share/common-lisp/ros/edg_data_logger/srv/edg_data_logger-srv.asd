
(cl:in-package :asdf)

(defsystem "edg_data_logger-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Enable" :depends-on ("_package_Enable"))
    (:file "_package_Enable" :depends-on ("_package"))
  ))