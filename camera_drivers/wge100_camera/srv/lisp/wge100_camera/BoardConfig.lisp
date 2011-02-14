; Auto-generated. Do not edit!


(in-package wge100_camera-srv)


;//! \htmlinclude BoardConfig-request.msg.html

(defclass <BoardConfig-request> (ros-message)
  ((serial
    :reader serial-val
    :initarg :serial
    :type integer
    :initform 0)
   (mac
    :reader mac-val
    :initarg :mac
    :type (vector fixnum)
   :initform (make-array 0 :element-type 'fixnum :initial-element 0)))
)
(defmethod serialize ((msg <BoardConfig-request>) ostream)
  "Serializes a message object of type '<BoardConfig-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'serial)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'serial)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'serial)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'serial)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'mac))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'mac))
)
(defmethod deserialize ((msg <BoardConfig-request>) istream)
  "Deserializes a message object of type '<BoardConfig-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'serial)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'serial)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'serial)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'serial)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'mac) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'mac)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<BoardConfig-request>)))
  "Returns string type for a service object of type '<BoardConfig-request>"
  "wge100_camera/BoardConfigRequest")
(defmethod md5sum ((type (eql '<BoardConfig-request>)))
  "Returns md5sum for a message object of type '<BoardConfig-request>"
  "785c6974990d2659ed40a68807b3f3bd")
(defmethod message-definition ((type (eql '<BoardConfig-request>)))
  "Returns full string definition for message of type '<BoardConfig-request>"
  (format nil "# Used to configure a camera's MAC address and serial number. This is a~%# one-time only operation for a camera.~%uint32 serial # Serial number~%uint8[] mac # MAC address, should be 6 bytes long~%~%"))
(defmethod serialization-length ((msg <BoardConfig-request>))
  (+ 0
     4
     4 (reduce #'+ (slot-value msg 'mac) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
))
(defmethod ros-message-to-list ((msg <BoardConfig-request>))
  "Converts a ROS message object to a list"
  (list '<BoardConfig-request>
    (cons ':serial (serial-val msg))
    (cons ':mac (mac-val msg))
))
;//! \htmlinclude BoardConfig-response.msg.html

(defclass <BoardConfig-response> (ros-message)
  ((success
    :reader success-val
    :initarg :success
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <BoardConfig-response>) ostream)
  "Serializes a message object of type '<BoardConfig-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'success)) ostream)
)
(defmethod deserialize ((msg <BoardConfig-response>) istream)
  "Deserializes a message object of type '<BoardConfig-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'success)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<BoardConfig-response>)))
  "Returns string type for a service object of type '<BoardConfig-response>"
  "wge100_camera/BoardConfigResponse")
(defmethod md5sum ((type (eql '<BoardConfig-response>)))
  "Returns md5sum for a message object of type '<BoardConfig-response>"
  "785c6974990d2659ed40a68807b3f3bd")
(defmethod message-definition ((type (eql '<BoardConfig-response>)))
  "Returns full string definition for message of type '<BoardConfig-response>"
  (format nil "uint8 success # 1 means success, 0 means failure~%~%~%"))
(defmethod serialization-length ((msg <BoardConfig-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <BoardConfig-response>))
  "Converts a ROS message object to a list"
  (list '<BoardConfig-response>
    (cons ':success (success-val msg))
))
(defmethod service-request-type ((msg (eql 'BoardConfig)))
  '<BoardConfig-request>)
(defmethod service-response-type ((msg (eql 'BoardConfig)))
  '<BoardConfig-response>)
(defmethod ros-datatype ((msg (eql 'BoardConfig)))
  "Returns string type for a service object of type '<BoardConfig>"
  "wge100_camera/BoardConfig")
