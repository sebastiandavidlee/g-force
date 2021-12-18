; Auto-generated. Do not edit!


(cl:in-package edg_data_logger-srv)


;//! \htmlinclude Enable-request.msg.html

(cl:defclass <Enable-request> (roslisp-msg-protocol:ros-message)
  ((EnableDataLogging
    :reader EnableDataLogging
    :initarg :EnableDataLogging
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Enable-request (<Enable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Enable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Enable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name edg_data_logger-srv:<Enable-request> is deprecated: use edg_data_logger-srv:Enable-request instead.")))

(cl:ensure-generic-function 'EnableDataLogging-val :lambda-list '(m))
(cl:defmethod EnableDataLogging-val ((m <Enable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader edg_data_logger-srv:EnableDataLogging-val is deprecated.  Use edg_data_logger-srv:EnableDataLogging instead.")
  (EnableDataLogging m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Enable-request>) ostream)
  "Serializes a message object of type '<Enable-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'EnableDataLogging) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Enable-request>) istream)
  "Deserializes a message object of type '<Enable-request>"
    (cl:setf (cl:slot-value msg 'EnableDataLogging) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Enable-request>)))
  "Returns string type for a service object of type '<Enable-request>"
  "edg_data_logger/EnableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Enable-request)))
  "Returns string type for a service object of type 'Enable-request"
  "edg_data_logger/EnableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Enable-request>)))
  "Returns md5sum for a message object of type '<Enable-request>"
  "f3f19e69803c97782b2fdad054f37b22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Enable-request)))
  "Returns md5sum for a message object of type 'Enable-request"
  "f3f19e69803c97782b2fdad054f37b22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Enable-request>)))
  "Returns full string definition for message of type '<Enable-request>"
  (cl:format cl:nil "bool EnableDataLogging~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Enable-request)))
  "Returns full string definition for message of type 'Enable-request"
  (cl:format cl:nil "bool EnableDataLogging~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Enable-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Enable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Enable-request
    (cl:cons ':EnableDataLogging (EnableDataLogging msg))
))
;//! \htmlinclude Enable-response.msg.html

(cl:defclass <Enable-response> (roslisp-msg-protocol:ros-message)
  ((OutputFileName
    :reader OutputFileName
    :initarg :OutputFileName
    :type cl:string
    :initform ""))
)

(cl:defclass Enable-response (<Enable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Enable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Enable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name edg_data_logger-srv:<Enable-response> is deprecated: use edg_data_logger-srv:Enable-response instead.")))

(cl:ensure-generic-function 'OutputFileName-val :lambda-list '(m))
(cl:defmethod OutputFileName-val ((m <Enable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader edg_data_logger-srv:OutputFileName-val is deprecated.  Use edg_data_logger-srv:OutputFileName instead.")
  (OutputFileName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Enable-response>) ostream)
  "Serializes a message object of type '<Enable-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'OutputFileName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'OutputFileName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Enable-response>) istream)
  "Deserializes a message object of type '<Enable-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'OutputFileName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'OutputFileName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Enable-response>)))
  "Returns string type for a service object of type '<Enable-response>"
  "edg_data_logger/EnableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Enable-response)))
  "Returns string type for a service object of type 'Enable-response"
  "edg_data_logger/EnableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Enable-response>)))
  "Returns md5sum for a message object of type '<Enable-response>"
  "f3f19e69803c97782b2fdad054f37b22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Enable-response)))
  "Returns md5sum for a message object of type 'Enable-response"
  "f3f19e69803c97782b2fdad054f37b22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Enable-response>)))
  "Returns full string definition for message of type '<Enable-response>"
  (cl:format cl:nil "string OutputFileName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Enable-response)))
  "Returns full string definition for message of type 'Enable-response"
  (cl:format cl:nil "string OutputFileName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Enable-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'OutputFileName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Enable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Enable-response
    (cl:cons ':OutputFileName (OutputFileName msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Enable)))
  'Enable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Enable)))
  'Enable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Enable)))
  "Returns string type for a service object of type '<Enable>"
  "edg_data_logger/Enable")