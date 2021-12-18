;; Auto-generated. Do not edit!


(when (boundp 'tae_datalogger::Enable)
  (if (not (find-package "TAE_DATALOGGER"))
    (make-package "TAE_DATALOGGER"))
  (shadow 'Enable (find-package "TAE_DATALOGGER")))
(unless (find-package "TAE_DATALOGGER::ENABLE")
  (make-package "TAE_DATALOGGER::ENABLE"))
(unless (find-package "TAE_DATALOGGER::ENABLEREQUEST")
  (make-package "TAE_DATALOGGER::ENABLEREQUEST"))
(unless (find-package "TAE_DATALOGGER::ENABLERESPONSE")
  (make-package "TAE_DATALOGGER::ENABLERESPONSE"))

(in-package "ROS")





(defclass tae_datalogger::EnableRequest
  :super ros::object
  :slots (_EnableDataLogging ))

(defmethod tae_datalogger::EnableRequest
  (:init
   (&key
    ((:EnableDataLogging __EnableDataLogging) nil)
    )
   (send-super :init)
   (setq _EnableDataLogging __EnableDataLogging)
   self)
  (:EnableDataLogging
   (&optional __EnableDataLogging)
   (if __EnableDataLogging (setq _EnableDataLogging __EnableDataLogging)) _EnableDataLogging)
  (:serialization-length
   ()
   (+
    ;; bool _EnableDataLogging
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _EnableDataLogging
       (if _EnableDataLogging (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _EnableDataLogging
     (setq _EnableDataLogging (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tae_datalogger::EnableResponse
  :super ros::object
  :slots (_OutputFileName ))

(defmethod tae_datalogger::EnableResponse
  (:init
   (&key
    ((:OutputFileName __OutputFileName) "")
    )
   (send-super :init)
   (setq _OutputFileName (string __OutputFileName))
   self)
  (:OutputFileName
   (&optional __OutputFileName)
   (if __OutputFileName (setq _OutputFileName __OutputFileName)) _OutputFileName)
  (:serialization-length
   ()
   (+
    ;; string _OutputFileName
    4 (length _OutputFileName)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _OutputFileName
       (write-long (length _OutputFileName) s) (princ _OutputFileName s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _OutputFileName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _OutputFileName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tae_datalogger::Enable
  :super ros::object
  :slots ())

(setf (get tae_datalogger::Enable :md5sum-) "f3f19e69803c97782b2fdad054f37b22")
(setf (get tae_datalogger::Enable :datatype-) "tae_datalogger/Enable")
(setf (get tae_datalogger::Enable :request) tae_datalogger::EnableRequest)
(setf (get tae_datalogger::Enable :response) tae_datalogger::EnableResponse)

(defmethod tae_datalogger::EnableRequest
  (:response () (instance tae_datalogger::EnableResponse :init)))

(setf (get tae_datalogger::EnableRequest :md5sum-) "f3f19e69803c97782b2fdad054f37b22")
(setf (get tae_datalogger::EnableRequest :datatype-) "tae_datalogger/EnableRequest")
(setf (get tae_datalogger::EnableRequest :definition-)
      "bool EnableDataLogging
---
string OutputFileName

")

(setf (get tae_datalogger::EnableResponse :md5sum-) "f3f19e69803c97782b2fdad054f37b22")
(setf (get tae_datalogger::EnableResponse :datatype-) "tae_datalogger/EnableResponse")
(setf (get tae_datalogger::EnableResponse :definition-)
      "bool EnableDataLogging
---
string OutputFileName

")



(provide :tae_datalogger/Enable "f3f19e69803c97782b2fdad054f37b22")


