#!/usr/bin/env roseus

;; load package
(ros::load-ros-package "std_srvs")
(ros::roseus "miraikan")

(defun call_service (service_name)
  (ros::wait-for-service service_name)
  (call-empty-service service_name)
  (ros::duration-sleep 9.0)
  )

(defun main ()
  (ros::duration-sleep 5.0)
  (call_service "episode_11")
  (call_service "episode_12")
  (call_service "episode_13")
  (call_service "episode_21")
  (call_service "episode_22")
  (call_service "episode_23")
  (call_service "episode_31")
  (call_service "episode_32")
  (call_service "episode_33")
  (call_service "episode_41")
  (call_service "episode_42")
  (call_service "episode_43")
  (call_service "episode_51")
  (call_service "episode_52")
  (call_service "episode_53")
  (call_service "episode_54") 
  )

(main)
(ros::exit)
