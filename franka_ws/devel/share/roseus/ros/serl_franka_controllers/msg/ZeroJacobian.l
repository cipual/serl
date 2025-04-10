;; Auto-generated. Do not edit!


(when (boundp 'serl_franka_controllers::ZeroJacobian)
  (if (not (find-package "SERL_FRANKA_CONTROLLERS"))
    (make-package "SERL_FRANKA_CONTROLLERS"))
  (shadow 'ZeroJacobian (find-package "SERL_FRANKA_CONTROLLERS")))
(unless (find-package "SERL_FRANKA_CONTROLLERS::ZEROJACOBIAN")
  (make-package "SERL_FRANKA_CONTROLLERS::ZEROJACOBIAN"))

(in-package "ROS")
;;//! \htmlinclude ZeroJacobian.msg.html


(defclass serl_franka_controllers::ZeroJacobian
  :super ros::object
  :slots (_zero_jacobian ))

(defmethod serl_franka_controllers::ZeroJacobian
  (:init
   (&key
    ((:zero_jacobian __zero_jacobian) (make-array 42 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _zero_jacobian __zero_jacobian)
   self)
  (:zero_jacobian
   (&optional __zero_jacobian)
   (if __zero_jacobian (setq _zero_jacobian __zero_jacobian)) _zero_jacobian)
  (:serialization-length
   ()
   (+
    ;; float64[42] _zero_jacobian
    (* 8    42)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[42] _zero_jacobian
     (dotimes (i 42)
       (sys::poke (elt _zero_jacobian i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[42] _zero_jacobian
   (dotimes (i (length _zero_jacobian))
     (setf (elt _zero_jacobian i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get serl_franka_controllers::ZeroJacobian :md5sum-) "573da0494fbe019a7da2ae31329663cf")
(setf (get serl_franka_controllers::ZeroJacobian :datatype-) "serl_franka_controllers/ZeroJacobian")
(setf (get serl_franka_controllers::ZeroJacobian :definition-)
      "float64[42] zero_jacobian

")



(provide :serl_franka_controllers/ZeroJacobian "573da0494fbe019a7da2ae31329663cf")


