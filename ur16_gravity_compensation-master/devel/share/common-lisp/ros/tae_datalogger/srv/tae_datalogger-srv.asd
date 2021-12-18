
(cl:in-package :asdf)

(defsystem "tae_datalogger-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Enable" :depends-on ("_package_Enable"))
    (:file "_package_Enable" :depends-on ("_package"))
  ))