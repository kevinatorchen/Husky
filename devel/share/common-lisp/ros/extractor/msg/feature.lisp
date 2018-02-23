; Auto-generated. Do not edit!


(cl:in-package extractor-msg)


;//! \htmlinclude feature.msg.html

(cl:defclass <feature> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (diameter
    :reader diameter
    :initarg :diameter
    :type cl:float
    :initform 0.0))
)

(cl:defclass feature (<feature>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <feature>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'feature)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name extractor-msg:<feature> is deprecated: use extractor-msg:feature instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <feature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader extractor-msg:header-val is deprecated.  Use extractor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <feature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader extractor-msg:position-val is deprecated.  Use extractor-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'diameter-val :lambda-list '(m))
(cl:defmethod diameter-val ((m <feature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader extractor-msg:diameter-val is deprecated.  Use extractor-msg:diameter instead.")
  (diameter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <feature>) ostream)
  "Serializes a message object of type '<feature>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'diameter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <feature>) istream)
  "Deserializes a message object of type '<feature>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'diameter) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<feature>)))
  "Returns string type for a message object of type '<feature>"
  "extractor/feature")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'feature)))
  "Returns string type for a message object of type 'feature"
  "extractor/feature")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<feature>)))
  "Returns md5sum for a message object of type '<feature>"
  "77a2cae90db3b4aa44ca6145d57ccfd6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'feature)))
  "Returns md5sum for a message object of type 'feature"
  "77a2cae90db3b4aa44ca6145d57ccfd6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<feature>)))
  "Returns full string definition for message of type '<feature>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point position~%float64 diameter~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'feature)))
  "Returns full string definition for message of type 'feature"
  (cl:format cl:nil "Header header~%geometry_msgs/Point position~%float64 diameter~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <feature>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <feature>))
  "Converts a ROS message object to a list"
  (cl:list 'feature
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':diameter (diameter msg))
))
