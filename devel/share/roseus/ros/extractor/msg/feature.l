;; Auto-generated. Do not edit!


(when (boundp 'extractor::feature)
  (if (not (find-package "EXTRACTOR"))
    (make-package "EXTRACTOR"))
  (shadow 'feature (find-package "EXTRACTOR")))
(unless (find-package "EXTRACTOR::FEATURE")
  (make-package "EXTRACTOR::FEATURE"))

(in-package "ROS")
;;//! \htmlinclude feature.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass extractor::feature
  :super ros::object
  :slots (_header _position _diameter ))

(defmethod extractor::feature
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:diameter __diameter) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _diameter (float __diameter))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:diameter
   (&optional __diameter)
   (if __diameter (setq _diameter __diameter)) _diameter)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; float64 _diameter
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; float64 _diameter
       (sys::poke _diameter (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float64 _diameter
     (setq _diameter (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get extractor::feature :md5sum-) "77a2cae90db3b4aa44ca6145d57ccfd6")
(setf (get extractor::feature :datatype-) "extractor/feature")
(setf (get extractor::feature :definition-)
      "Header header
geometry_msgs/Point position
float64 diameter
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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :extractor/feature "77a2cae90db3b4aa44ca6145d57ccfd6")


