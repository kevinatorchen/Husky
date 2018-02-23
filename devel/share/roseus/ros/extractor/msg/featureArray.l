;; Auto-generated. Do not edit!


(when (boundp 'extractor::featureArray)
  (if (not (find-package "EXTRACTOR"))
    (make-package "EXTRACTOR"))
  (shadow 'featureArray (find-package "EXTRACTOR")))
(unless (find-package "EXTRACTOR::FEATUREARRAY")
  (make-package "EXTRACTOR::FEATUREARRAY"))

(in-package "ROS")
;;//! \htmlinclude featureArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass extractor::featureArray
  :super ros::object
  :slots (_header _features ))

(defmethod extractor::featureArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:features __features) (let (r) (dotimes (i 0) (push (instance extractor::feature :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _features __features)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:features
   (&rest __features)
   (if (keywordp (car __features))
       (send* _features __features)
     (progn
       (if __features (setq _features (car __features)))
       _features)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; extractor/feature[] _features
    (apply #'+ (send-all _features :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; extractor/feature[] _features
     (write-long (length _features) s)
     (dolist (elem _features)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; extractor/feature[] _features
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _features (let (r) (dotimes (i n) (push (instance extractor::feature :init) r)) r))
     (dolist (elem- _features)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get extractor::featureArray :md5sum-) "21568729fbe1966ecff36cf2cf9ffab1")
(setf (get extractor::featureArray :datatype-) "extractor/featureArray")
(setf (get extractor::featureArray :definition-)
      "Header header
extractor/feature[] features
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: extractor/feature
Header header
geometry_msgs/Point position
float64 diameter
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :extractor/featureArray "21568729fbe1966ecff36cf2cf9ffab1")


