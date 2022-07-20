#!/usr/bin/env roseus

;; load package
(ros::load-ros-package "peppereus")
(ros::load-ros-package "sensor_msgs")
(ros::load-ros-package "std_srvs")

;; load files
(require :pepper-interface "package://peppereus/pepper-interface.l")

;; var
(defvar *process-rate* 5)

;; parameter
(defparameter *continue* t)

(ros::roseus "lecture-demo")

(ros::subscribe "joy" sensor_msgs::Joy
		#'(lambda (msg)
		    (let (button)
		      ;; reset value
		      (setq *continue* t)
		      ;; update value
		      (setq button (send msg :buttons))

		      ;; A button: deai_1
		      (if (eq (elt button 0) 1)
			  (progn
			    (ros::wait-for-service "deai_1")
                            (call-empty-service "deai_1")
			    ))

		      ;; B button: deai_2
		      (if (eq (elt button 1) 1)
			  (progn
			    (ros::wait-for-service "deai_2")
                            (call-empty-service "deai_2")
			    ))
		      
		      ;; Logicool button: stop this node
		      (if (eq (elt button 8) 1)
			  (setq *continue* nil))
		      )))
		      
(defun init ()
  (unless (boundp '*ri*)
    (pepper-init))
  ;; check pepper's life
  (if (not (string= (send *ri* :get-life) "disabled"))
      (send *ri* :disable-life))
  (send *ri* :servo-on)
  (send *ri* :set-language "Japanese")
  (send *ri* :speak "\\vct=130\\ヨイショ")
  (send *ri* :set-master-volume 50) ;; with mic
  (send *ri* :stop-grasp)
  ;;(send *ri* :set-basic-awareness-enabled nil)
  ;;(send *ri* :set-background-movement-enabled nil)
  )

(defun main ()
  (ros::rate *process-rate*)
  (init)
  (print "start")
  (let ((tm nil)
	(tm2 nil))
    (while *continue*
      (if (not tm)
      	  (setq tm (ros::time-now)))
      (if (not tm2)
      	  (setq tm2 (ros::time-now)))
      ;;(print (send (ros::time- (ros::time-now) tm) :to-sec))
      (if (and tm
      	       (> (send (ros::time- (ros::time-now) tm) :to-sec) 10))
      	  (progn
      	    (send *ri* :angle-vector-sequence (list #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0) #f(2.0 -2.0 -5.0 85.0 20.0 -70.0 -20.0 -40.0 85.0 -20.0 70.0 20.0 40.0 0.0 -10.0) #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)) (list 1000 1000 1000))
      	    (setq tm nil)
      	    ))
      (if (and tm2
      	       (> (send (ros::time- (ros::time-now) tm2) :to-sec) 35))
      	  (progn
      	    (send *pepper* :head :neck-y :joint-angle -40)
      	    (send *pepper* :head :neck-p :joint-angle 10)
      	    (send *ri* :angle-vector (send *pepper* :angle-vector) 800)
      	    (send *ri* :wait-interpolation)
	    
      	    (send *pepper* :head :neck-p :joint-angle -10)
      	    (send *ri* :angle-vector (send *pepper* :angle-vector) 800)
      	    (send *ri* :wait-interpolation)
	    
      	    ;; (send *pepper* :head :neck-p :joint-angle 10)
      	    ;; (send *ri* :angle-vector (send *pepper* :angle-vector) 800)
      	    ;; (send *ri* :wait-interpolation)
	    
      	    ;; (send *pepper* :head :neck-p :joint-angle -10)
      	    ;; (send *ri* :angle-vector (send *pepper* :angle-vector) 800)
      	    ;; (send *ri* :wait-interpolation)

      	    (send *pepper* :head :neck-p :joint-angle 10)
      	    (send *ri* :angle-vector (send *pepper* :angle-vector) 800)
      	    (send *ri* :wait-interpolation)
	    
      	    (send *pepper* :head :neck-p :joint-angle 0)
      	    (send *ri* :angle-vector (send *pepper* :angle-vector) 800)
      	    (send *ri* :wait-interpolation)
	    
      	    (send *pepper* :reset-pose)
      	    (send *ri* :angle-vector (send *pepper* :angle-vector) 800)
      	    (send *ri* :wait-interpolation)
      	    (setq tm2 nil)
      	    ))
      (ros::spin-once)
      ;; (send *ri* :spin-once)
      (ros::sleep)
      ); while
    )

  (send *ri* :speak "おしまい")
  (send *ri* :hide-image)
  (send *ri* :set-language "English")
  (print "end")
  )





