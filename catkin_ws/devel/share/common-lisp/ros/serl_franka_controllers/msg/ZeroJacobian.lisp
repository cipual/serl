; Auto-generated. Do not edit!


(cl:in-package serl_franka_controllers-msg)


;//! \htmlinclude ZeroJacobian.msg.html

(cl:defclass <ZeroJacobian> (roslisp-msg-protocol:ros-message)
  ((zero_jacobian
    :reader zero_jacobian
    :initarg :zero_jacobian
    :type (cl:vector cl:float)
   :initform (cl:make-array 42 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ZeroJacobian (<ZeroJacobian>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ZeroJacobian>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ZeroJacobian)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serl_franka_controllers-msg:<ZeroJacobian> is deprecated: use serl_franka_controllers-msg:ZeroJacobian instead.")))

(cl:ensure-generic-function 'zero_jacobian-val :lambda-list '(m))
(cl:defmethod zero_jacobian-val ((m <ZeroJacobian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serl_franka_controllers-msg:zero_jacobian-val is deprecated.  Use serl_franka_controllers-msg:zero_jacobian instead.")
  (zero_jacobian m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ZeroJacobian>) ostream)
  "Serializes a message object of type '<ZeroJacobian>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'zero_jacobian))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ZeroJacobian>) istream)
  "Deserializes a message object of type '<ZeroJacobian>"
  (cl:setf (cl:slot-value msg 'zero_jacobian) (cl:make-array 42))
  (cl:let ((vals (cl:slot-value msg 'zero_jacobian)))
    (cl:dotimes (i 42)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ZeroJacobian>)))
  "Returns string type for a message object of type '<ZeroJacobian>"
  "serl_franka_controllers/ZeroJacobian")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ZeroJacobian)))
  "Returns string type for a message object of type 'ZeroJacobian"
  "serl_franka_controllers/ZeroJacobian")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ZeroJacobian>)))
  "Returns md5sum for a message object of type '<ZeroJacobian>"
  "573da0494fbe019a7da2ae31329663cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ZeroJacobian)))
  "Returns md5sum for a message object of type 'ZeroJacobian"
  "573da0494fbe019a7da2ae31329663cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ZeroJacobian>)))
  "Returns full string definition for message of type '<ZeroJacobian>"
  (cl:format cl:nil "float64[42] zero_jacobian~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ZeroJacobian)))
  "Returns full string definition for message of type 'ZeroJacobian"
  (cl:format cl:nil "float64[42] zero_jacobian~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ZeroJacobian>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'zero_jacobian) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ZeroJacobian>))
  "Converts a ROS message object to a list"
  (cl:list 'ZeroJacobian
    (cl:cons ':zero_jacobian (zero_jacobian msg))
))
