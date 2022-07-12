;; func in robot-behavior-server.l
(ros::roseus "miraikan")

(ros::load-ros-package "std_msgs")
;; (ros::roseus-add-msgs "miraikan_demo")
(ros::load-ros-package "miraikan_demo")
(setq *mode-srv* "/demo_mode")

(defun test_run (mode time_delay)
  (let* ((req (instance miraikan_demo::ModeRequest :init
			:mode mode :time_delay time_delay))
	 res)
    (format t "waiting service ...~%")
    (ros::wait-for-service *mode-srv*)
    (setq res (ros::service-call *mode-srv* req))
    (format t "called service~%")
    )
  )

(test_run 0 4)
(ros::duration-sleep 15)
(test_run 1 2)


