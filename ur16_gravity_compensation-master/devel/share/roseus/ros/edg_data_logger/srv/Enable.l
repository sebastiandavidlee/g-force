;; Auto-generated. Do not edit!


(when (boundp 'edg_data_logger::Enable)
  (if (not (find-package "EDG_DATA_LOGGER"))
    (make-package "EDG_DATA_LOGGER"))
  (shadow 'Enable (find-package "EDG_DATA_LOGGER")))
(unless (find-package "EDG_DATA_LOGGER::ENABLE")
  (make-package "EDG_DATA_LOGGER::ENABLE"))
(unless (find-package "EDG_DATA_LOGGER::ENABLEREQUEST")
  (make-package "EDG_DATA_LOGGER::ENABLEREQUEST"))
(unless (find-package "EDG_DATA_LOGGER::ENABLERESPONSE")
  (make-package "EDG_DATA_LOGGER::ENABLERESPONSE"))

(in-package "ROS")





(defclass edg_data_logger::EnableRequest
  :super ros::object
  :slots (_EnableDataLogging ))

(defmethod edg_data_logger::EnableRequest
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

(defclass edg_data_logger::EnableResponse
  :super ros::object
  :slots (_OutputFileName ))

(defmethod edg_data_logger::EnableResponse
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

(defclass edg_data_logger::Enable
  :super ros::object
  :slots ())

(setf (get edg_data_logger::Enable :md5sum-) "f3f19e69803c97782b2fdad054f37b22")
(setf (get edg_data_logger::Enable :datatype-) "edg_data_logger/Enable")
(setf (get edg_data_logger::Enable :request) edg_data_logger::EnableRequest)
(setf (get edg_data_logger::Enable :response) edg_data_logger::EnableResponse)

(defmethod edg_data_logger::EnableRequest
  (:response () (instance edg_data_logger::EnableResponse :init)))

(setf (get edg_data_logger::EnableRequest :md5sum-) "f3f19e69803c97782b2fdad054f37b22")
(setf (get edg_data_logger::EnableRequest :datatype-) "edg_data_logger/EnableRequest")
(setf (get edg_data_logger::EnableRequest :definition-)
      "bool EnableDataLogging
---
string OutputFileName

")

(setf (get edg_data_logger::EnableResponse :md5sum-) "f3f19e69803c97782b2fdad054f37b22")
(setf (get edg_data_logger::EnableResponse :datatype-) "edg_data_logger/EnableResponse")
(setf (get edg_data_logger::EnableResponse :definition-)
      "bool EnableDataLogging
---
string OutputFileName

")



(provide :edg_data_logger/Enable "f3f19e69803c97782b2fdad054f37b22")


