;; Auto-generated. Do not edit!


(when (boundp 'homo_msgs::HomographyResult)
  (if (not (find-package "HOMO_MSGS"))
    (make-package "HOMO_MSGS"))
  (shadow 'HomographyResult (find-package "HOMO_MSGS")))
(unless (find-package "HOMO_MSGS::HOMOGRAPHYRESULT")
  (make-package "HOMO_MSGS::HOMOGRAPHYRESULT"))

(in-package "ROS")
;;//! \htmlinclude HomographyResult.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass homo_msgs::HomographyResult
  :super ros::object
  :slots (_header _homography ))

(defmethod homo_msgs::HomographyResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:homography __homography) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _homography __homography)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:homography
   (&optional __homography)
   (if __homography (setq _homography __homography)) _homography)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[9] _homography
    (* 4    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[9] _homography
     (dotimes (i 9)
       (sys::poke (elt _homography i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[9] _homography
   (dotimes (i (length _homography))
     (setf (elt _homography i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get homo_msgs::HomographyResult :md5sum-) "18f3159e548aca86612999d64fb8f5fa")
(setf (get homo_msgs::HomographyResult :datatype-) "homo_msgs/HomographyResult")
(setf (get homo_msgs::HomographyResult :definition-)
      "Header header

float32[9] homography




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
string frame_id

")



(provide :homo_msgs/HomographyResult "18f3159e548aca86612999d64fb8f5fa")


