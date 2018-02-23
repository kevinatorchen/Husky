; Auto-generated. Do not edit!


(cl:in-package extractor-msg)


;//! \htmlinclude map.msg.html

(cl:defclass <map> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (features
    :reader features
    :initarg :features
    :type (cl:vector extractor-msg:feature)
   :initform (cl:make-array 0 :element-type 'extractor-msg:feature :initial-element (cl:make-instance 'extractor-msg:feature)))
   (robotposition
    :reader robotposition
    :initarg :robotposition
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass map (<map>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <map>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'map)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name extractor-msg:<map> is deprecated: use extractor-msg:map instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader extractor-msg:header-val is deprecated.  Use extractor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'features-val :lambda-list '(m))
(cl:defmethod features-val ((m <map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader extractor-msg:features-val is deprecated.  Use extractor-msg:features instead.")
  (features m))

(cl:ensure-generic-function 'robotposition-val :lambda-list '(m))
(cl:defmethod robotposition-val ((m <map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader extractor-msg:robotposition-val is deprecated.  Use extractor-msg:robotposition instead.")
  (robotposition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <map>) ostream)
  "Serializes a message object of type '<map>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'features))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'features))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robotposition) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <map>) istream)
  "Deserializes a message object of type '<map>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'features) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'features)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'extractor-msg:feature))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robotposition) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<map>)))
  "Returns string type for a message object of type '<map>"
  "extractor/map")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'map)))
  "Returns string type for a message object of type 'map"
  "extractor/map")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<map>)))
  "Returns md5sum for a message object of type '<map>"
  "605ba2556bdcd481e241a16199e3f3a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'map)))
  "Returns md5sum for a message object of type 'map"
  "605ba2556bdcd481e241a16199e3f3a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<map>)))
  "Returns full string definition for message of type '<map>"
  (cl:format cl:nil "Header header~%feature[] features~%geometry_msgs/Pose robotposition~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: extractor/feature~%Header header~%geometry_msgs/Point position~%float64 diameter~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'map)))
  "Returns full string definition for message of type 'map"
  (cl:format cl:nil "Header header~%feature[] features~%geometry_msgs/Pose robotposition~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: extractor/feature~%Header header~%geometry_msgs/Point position~%float64 diameter~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <map>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'features) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robotposition))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <map>))
  "Converts a ROS message object to a list"
  (cl:list 'map
    (cl:cons ':header (header msg))
    (cl:cons ':features (features msg))
    (cl:cons ':robotposition (robotposition msg))
))
