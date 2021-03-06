(ros::roseus "miraikan")

(ros::load-ros-package "std_msgs")
;; (ros::roseus-add-msgs "miraikan_demo")
(ros::load-ros-package "miraikan_demo")
(setq *mode-srv* "/demo_mode")

(defun call_demo (mode time_delay)
  (let* ((req (instance miraikan_demo::ModeRequest :init
			:mode mode :time_delay time_delay))
	 res)
    (format t "waiting service ...~%")
    (ros::wait-for-service *mode-srv*)
    (setq res (ros::service-call *mode-srv* req))
    (format t "called service~%")
    )
  )

;; memories talk mode
(defun m_episode_11 (req)
  (let ((m (send req :response)))
    (call_demo 0 4)
    m)
  )

(defun m_episode_12 (req)
  (let ((m (send req :response)))
    (call_demo 1 12)
    m)
  )

(defun m_episode_13 (req)
  (let ((m (send req :response)))
    (call_demo 2 7)
    m)
  )

(defun m_episode_21 (req)
  (let ((m (send req :response)))
    (call_demo 3 5)
    m)
  )

(defun m_episode_22 (req)
  (let ((m (send req :response)))
    (call_demo 4 10)
    m)
  )

(defun m_episode_23 (req)
  (let ((m (send req :response)))
    (call_demo 5 3)
    m)
  )

(defun m_episode_31 (req)
  (let ((m (send req :response)))
    (call_demo 6 10)
    m)
  )

(defun m_episode_32 (req)
  (let ((m (send req :response)))
    (call_demo 7 3)
    (ros::duration-sleep 8.0)
    (call_demo 8 2)
    m)
  )

(defun m_episode_33 (req)
  (let ((m (send req :response)))
    (call_demo 9 4)
    (ros::duration-sleep 6.0)
    (call_demo 10 3)
    m)
  )

(defun m_episode_41 (req)
  (let ((m (send req :response)))
    (call_demo 11 10)
    m)
  )

(defun m_episode_42 (req)
  (let ((m (send req :response)))
    (call_demo 12 4)
    (ros::duration-sleep 6.0)
    (call_demo 13 4)
    m)
  )

(defun m_episode_43 (req)
  (let ((m (send req :response)))
    (call_demo 14 4)
    m)
  )

(defun m_episode_51 (req)
  (let ((m (send req :response)))
    (call_demo 15 3)
    m)
  )

(defun m_episode_52 (req)
  (let ((m (send req :response)))
    (call_demo 16 11)
    m)
  )

(defun m_episode_53 (req)
  (let ((m (send req :response)))
    (call_demo 17 5)
    m)
  )

(defun m_episode_54 (req)
  (let ((m (send req :response)))
    (call_demo 18 4)
    (ros::duration-sleep 5.0)
    (call_demo 19 4)
    (ros::duration-sleep 5.0)
    (call_demo 20 5)
    m)
  )

;; no memories talk mode
(defun episode_11 (req)
  (let ((m (send req :response)))
    (call_demo 0 4.2)
    m)
  )

(defun episode_12 (req)
  (let ((m (send req :response)))
    (call_demo 1 4)
    m)
  )

(defun episode_13 (req)
  (let ((m (send req :response)))
    (call_demo 2 4)
    m)
  )

(defun episode_21 (req)
  (let ((m (send req :response)))
    (call_demo 3 4)
    m)
  )

(defun episode_22 (req)
  (let ((m (send req :response)))
    (call_demo 4 4)
    m)
  )

(defun episode_23 (req)
  (let ((m (send req :response)))
    (call_demo 5 4)
    m)
  )

(defun episode_31 (req)
  (let ((m (send req :response)))
    (call_demo 6 4)
    m)
  )

(defun episode_32 (req)
  (let ((m (send req :response)))
    (call_demo 7 4)
    (ros::duration-sleep 5)
    (call_demo 8 4)
    m)
  )

(defun episode_33 (req)
  (let ((m (send req :response)))
    (call_demo 9 4)
    (ros::duration-sleep 5)
    (call_demo 10 4)
    m)
  )

(defun episode_41 (req)
  (let ((m (send req :response)))
    (call_demo 11 4)
    m)
  )

(defun episode_42 (req)
  (let ((m (send req :response)))
    (call_demo 12 4)
    m)
  )

(defun episode_43 (req)
  (let ((m (send req :response)))
    (call_demo 14 4)
    m)
  )

(defun episode_51 (req)
  (let ((m (send req :response)))
    (call_demo 15 4)
    m)
  )

(defun episode_52 (req)
  (let ((m (send req :response)))
    (call_demo 16 4)
    m)
  )

(defun episode_53 (req)
  (let ((m (send req :response)))
    (call_demo 17 4)
    m)
  )

(defun episode_54 (req)
  (let ((m (send req :response)))
    (call_demo 18 4)
    (ros::duration-sleep 5)
    (call_demo 19 4)
    (ros::duration-sleep 5)
    (call_demo 20 4)
    m)
  )


;; funcs -> rossrv msg
(ros::load-ros-package "std_srvs")
(defvar *process-rate* 5)

(defun main ()
  ;; episode talk with memories ver
  (ros::advertise-service "episode_11" std_srvs::Empty #'m_episode_11)
  (ros::advertise-service "episode_12" std_srvs::Empty #'m_episode_12)
  (ros::advertise-service "episode_13" std_srvs::Empty #'m_episode_13)
  (ros::advertise-service "episode_21" std_srvs::Empty #'m_episode_21)
  (ros::advertise-service "episode_22" std_srvs::Empty #'m_episode_22)
  (ros::advertise-service "episode_23" std_srvs::Empty #'m_episode_23)
  (ros::advertise-service "episode_31" std_srvs::Empty #'m_episode_31)
  (ros::advertise-service "episode_32" std_srvs::Empty #'m_episode_32)
  (ros::advertise-service "episode_33" std_srvs::Empty #'m_episode_33)
  (ros::advertise-service "episode_41" std_srvs::Empty #'m_episode_41)
  (ros::advertise-service "episode_42" std_srvs::Empty #'m_episode_42)
  (ros::advertise-service "episode_43" std_srvs::Empty #'m_episode_43)
  (ros::advertise-service "episode_51" std_srvs::Empty #'m_episode_51)
  (ros::advertise-service "episode_52" std_srvs::Empty #'m_episode_52)
  (ros::advertise-service "episode_53" std_srvs::Empty #'m_episode_53)
  (ros::advertise-service "episode_54" std_srvs::Empty #'m_episode_54)
  ;; explanation talk ver
  (ros::advertise-service "explanation_11" std_srvs::Empty #'episode_11)
  (ros::advertise-service "explanation_12" std_srvs::Empty #'episode_12)
  (ros::advertise-service "explanation_13" std_srvs::Empty #'episode_13)
  (ros::advertise-service "explanation_21" std_srvs::Empty #'episode_21)
  (ros::advertise-service "explanation_22" std_srvs::Empty #'episode_22)
  (ros::advertise-service "explanation_23" std_srvs::Empty #'episode_23)
  (ros::advertise-service "explanation_31" std_srvs::Empty #'episode_31)
  (ros::advertise-service "explanation_32" std_srvs::Empty #'episode_32)
  (ros::advertise-service "explanation_33" std_srvs::Empty #'episode_33)
  (ros::advertise-service "explanation_41" std_srvs::Empty #'episode_41)
  (ros::advertise-service "explanation_42" std_srvs::Empty #'episode_42)
  (ros::advertise-service "explanation_43" std_srvs::Empty #'episode_43)
  (ros::advertise-service "explanation_51" std_srvs::Empty #'episode_51)
  (ros::advertise-service "explanation_52" std_srvs::Empty #'episode_52)
  (ros::advertise-service "explanation_53" std_srvs::Empty #'episode_53)
  (ros::advertise-service "explanation_54" std_srvs::Empty #'episode_54)
  (ros::rate *process-rate*)
  (format t "robot behavior server start~%")
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    )
  (format t "robot behavior server end~%")
  )

;; (main)
;; (ros::exit)

