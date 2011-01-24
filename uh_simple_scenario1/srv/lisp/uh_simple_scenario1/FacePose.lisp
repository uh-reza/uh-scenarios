; Auto-generated. Do not edit!


(in-package uh_simple_scenario1-srv)


;//! \htmlinclude FacePose-request.msg.html

(defclass <FacePose-request> (ros-message)
  ((a
    :reader a-val
    :initarg :a
    :type integer
    :initform 0)
   (b
    :reader b-val
    :initarg :b
    :type integer
    :initform 0)
   (img_in
    :reader img_in-val
    :initarg :img_in
    :type sensor_msgs-msg:<Image>
    :initform (make-instance 'sensor_msgs-msg:<Image>)))
)
(defmethod serialize ((msg <FacePose-request>) ostream)
  "Serializes a message object of type '<FacePose-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'a)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'b)) ostream)
  (serialize (slot-value msg 'img_in) ostream)
)
(defmethod deserialize ((msg <FacePose-request>) istream)
  "Deserializes a message object of type '<FacePose-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'b)) (read-byte istream))
  (deserialize (slot-value msg 'img_in) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<FacePose-request>)))
  "Returns string type for a service object of type '<FacePose-request>"
  "uh_simple_scenario1/FacePoseRequest")
(defmethod md5sum ((type (eql '<FacePose-request>)))
  "Returns md5sum for a message object of type '<FacePose-request>"
  "8b5ab9facb6a86166a2ffe67320b1159")
(defmethod message-definition ((type (eql '<FacePose-request>)))
  "Returns full string definition for message of type '<FacePose-request>"
  (format nil "int64 a~%int64 b~%sensor_msgs/Image img_in~%~%"))
(defmethod serialization-length ((msg <FacePose-request>))
  (+ 0
     8
     8
     (serialization-length (slot-value msg 'img_in))
))
(defmethod ros-message-to-list ((msg <FacePose-request>))
  "Converts a ROS message object to a list"
  (list '<FacePose-request>
    (cons ':a (a-val msg))
    (cons ':b (b-val msg))
    (cons ':img_in (img_in-val msg))
))
;//! \htmlinclude FacePose-response.msg.html

(defclass <FacePose-response> (ros-message)
  ((sum
    :reader sum-val
    :initarg :sum
    :type integer
    :initform 0)
   (x
    :reader x-val
    :initarg :x
    :type float
    :initform 0.0)
   (y
    :reader y-val
    :initarg :y
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <FacePose-response>) ostream)
  "Serializes a message object of type '<FacePose-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'sum)) ostream)
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'x))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'y))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <FacePose-response>) istream)
  "Deserializes a message object of type '<FacePose-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'sum)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<FacePose-response>)))
  "Returns string type for a service object of type '<FacePose-response>"
  "uh_simple_scenario1/FacePoseResponse")
(defmethod md5sum ((type (eql '<FacePose-response>)))
  "Returns md5sum for a message object of type '<FacePose-response>"
  "8b5ab9facb6a86166a2ffe67320b1159")
(defmethod message-definition ((type (eql '<FacePose-response>)))
  "Returns full string definition for message of type '<FacePose-response>"
  (format nil "int64 sum~%float64 x~%float64 y~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <FacePose-response>))
  (+ 0
     8
     8
     8
))
(defmethod ros-message-to-list ((msg <FacePose-response>))
  "Converts a ROS message object to a list"
  (list '<FacePose-response>
    (cons ':sum (sum-val msg))
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
))
(defmethod service-request-type ((msg (eql 'FacePose)))
  '<FacePose-request>)
(defmethod service-response-type ((msg (eql 'FacePose)))
  '<FacePose-response>)
(defmethod ros-datatype ((msg (eql 'FacePose)))
  "Returns string type for a service object of type '<FacePose>"
  "uh_simple_scenario1/FacePose")
