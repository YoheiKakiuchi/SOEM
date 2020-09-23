;;
;; simple shm {reader/writer} for EusLisp
;;
(let ((lib (load-foreign "eus_shm_interface.so")))
  (defforeign initialize-sharedmemory lib "initialize_sharedmemory" () :integer)
  (defforeign _read-angle-shm lib "read_angle_shm" (:integer :string) :integer)
  (defforeign _read-ref-angle-shm lib "read_ref_angle_shm" (:integer :string) :integer)
  (defforeign _write-ref-angle-shm lib "write_ref_angle_shm" (:integer :string :integer) :integer)
  (defforeign _read-current-shm lib "read_current_shm" (:integer :string) :integer)
  (defforeign _read-is-servo-on-shm lib "read_is_servo_on_shm" (:integer :string) :integer)
  (defforeign _write-servo-on-shm lib "write_servo_on_shm" (:integer :string) :integer)
  (defforeign _write-servo-off-shm lib "write_servo_off_shm" (:integer :string) :integer)
  (defforeign _write-simple-seq-shm lib "write_simple_seq_shm" (:integer :string :integer) :integer)
  ;;(defforeign _read-simple-seq-shm lib "read_simple_seq_shm" (:integer :string) :integer)
  ;;(defforeign _read-force-shm lib "read_force_sensor" (:integer :string) :integer)
  ;;(defforeign _read-gyro-shm lib "read_gyro_sensor" (:integer :string) :integer)
  ;;(defforeign _read-accel-shm lib "read_accel_sensor" (:integer :string) :integer)
  ;;(defforeign _read-imu-shm lib "read_imu_posture" (:integer :string) :integer)
  (defforeign read-shm-frame lib "read_shm_frame" () :integer)
  )
#|
cd jsk_sample/elmo_shm/euslisp
irteusgl eus-shm-interface.l
|#
(defun read-is-servo-on-shm (&optional (iv (instantiate integer-vector 24)))
  (_read-is-servo-on-shm (length iv) iv)
  iv
  )
(defun write-servo-on-shm (&optional (iv (instantiate integer-vector 24)))
  (_write-servo-on-shm (length iv) iv)
  iv
  )
(defun write-servo-off-shm (&optional (iv (instantiate integer-vector 24)))
  (_write-servo-off-shm (length iv) iv)
  iv
  )
(defun read-angle-shm (&optional (fv (instantiate float-vector 24)))
;;  (let ((bytestr (dvector2float-bytestring fv)))
;;    (_read-angle-shm (length fv) bytestr)
;;    (float-bytestring2dvector bytestr)))
  (_read-angle-shm (length fv) fv)
  fv)
(defun read-ref-angle-shm (&optional (fv (instantiate float-vector 24)))
;;  (let ((bytestr (dvector2float-bytestring fv)))
;;    (_read-ref-angle-shm (length fv) bytestr)
;;    (float-bytestring2dvector bytestr)))
  (_read-ref-angle-shm (length fv) fv)
  fv)
(defun read-current-shm (&optional (fv (instantiate float-vector 24)))
;;  (let ((bytestr (dvector2float-bytestring fv)))
;;    (_read-current-shm (length fv) bytestr)
;;    (float-bytestring2dvector bytestr)))
  (_read-current-shm (length fv) fv)
  fv)
(defun write-ref-angle-shm (fv &optional (count 0)) ;; interpolation count
;;  (let ((bytestr (dvector2float-bytestring fv)))
;;    (_write-ref-angle-shm (length fv) bytestr));;
  (_write-ref-angle-shm (length fv) fv count)
  fv)
(defun read-force-sensor (&optional (id 0) (result (instantiate float-vector 6)))
  (_read-force-shm id result)
  result)
(defun read-gyro-sensor (&optional (id 0) (result (instantiate float-vector 3)))
  (_read-gyro-shm id result)
  result)
(defun read-accel-sensor (&optional (id 0) (result (instantiate float-vector 3)))
  (_read-accel-shm id result)
  result)
(defun read-imu-posture (&optional (id 0) (result (instantiate float-vector 4)))
  (_read-imu-shm id result)
  result)
;; simple sequencer
(defun write-simple-seq-shm (fv count)
  ;;(let ((bytestr (dvector2float-bytestring (scale (/ 1.0 count) fv))))
  ;;(_write-simple-seq-shm (length fv) bytestr count));;
  (_write-simple-seq-shm (length fv) fv count);;
  fv)
#|
(defun read-simple-seq-shm (&optional (fv (instantiate float-vector 24)))
  (let ((bytestr (dvector2float-bytestring fv)) ret)
    (setq ret (_read-simple-seq-shm (length fv) bytestr))
    (if (/= ret 0)
        (cons ret (float-bytestring2dvector bytestr)))
    ))
(defun move-to-seq (dest &optional (tm 2000) (vec (instantiate float-vector (length dest))))
  "move from current cylinder-vector to desired cylinder-vector using sequencer in controller
dest : cylinder-vector
(tm) : total interpolation time
(vec) : buffer of angle vector
"
  (setq tm (/ tm 4));; time -> frame count
  (let ((av (read-ref-angle-shm vec)))
    (v- dest av av)
    (write-simple-seq-shm av seq)
    ))
(defun wait-seq (&optional (vec (instantiate float-vector 24)) (cnt 0))
  (while (> (_read-simple-seq-shm (length vec) vec) cnt)
    ))
|#

(defun move-to (dest &optional (tm 2000) (stp 5))
  "move from current cylinder-vector to desired cylinder-vector
dest : cylinder-vector
(tm) : total interpolation time
(stp) : sleep 'stp' times of 1 frame(4ms)
"
  (setq tm (/ tm (* stp 4))) ;; time -> frame * stp
  (let ((av (read-ref-angle-shm (instantiate float-vector (length dest))))
        (ftm (float tm))
        tmp)
    (dotimes (n (1+ tm))
      (setq tmp (midpoint (/ n ftm) av dest))
      ;;(pprint n)
      (write-ref-angle-shm tmp)
      (unix::usleep (* 4 stp 1000)) ;; 4ms * stp
      )
    ))

;;;
(warn "
(initialize-sharedmemory)
(read-angle-shm)
(write-ref-angle-shm (instantiate float-vector 24))

(dotimes (i 2) (print (read-angle-shm)) (unix:sleep 1))
(progn
  (write-ref-angle-shm (instantiate float-vector 24))
  (unix:sleep 1)
  (write-ref-angle-shm (fill (instantiate float-vector 24) 1))
  )
")

#|
;; draw imu sensor
(initialize-sharedmemory)
(load "models/arrow-object.l")
(setq ar (arrow))
(objects (list ar))
(do-until-key
 (send ar :newcoords (make-coords :rot (quaternion2matrix (read-imu-posture 1))))
 (send *irtviewer* :draw-objects)
 (unix::usleep (* 50 1000)))

|#

#|
;; rotate robot sample
(let* ((imu (send (car (send *chidori* :imu-sensors)) :worldcoords))
       (cds (make-coords
             :pos (copy-seq (send imu :worldpos))
             :rot (quaternion2matrix (read-imu-posture))))
       )
  (pprint (list imu cds))
  (send *chidori* :move-coords cds imu))
|#

#|
(load "package://hrpsys_ros_bridge_tutorials/euslisp/chidori-interface.l")
(chidori)
(setq *robot* *chidori*)
(write-ref-angle-shm
 (scale (deg2rad 1) (send *chidori* :reset-pose))
 2000)

(defun test-chidori (&key (tm 2000) (dt 10))
  (let* ((fv
          (instantiate float-vector
                       (length (send *chidori* :angle-vector))))
         (times (/ tm dt))
         (diff-vec (float-vector 0 0 (/ 260.0 times)))
         tmp
         frame)

    (send *chidori* :angle-vector
          (scale (rad2deg 1.0) (read-ref-angle-shm fv)))

    (sys::gc)
    (setq frame (setq tmp (read-shm-frame)))
    (incf frame dt)
    (dotimes (i times)
      (send *chidori* :legs :move-end-pos diff-vec)
      (send *chidori* :fix-leg-to-coords (make-coords))
      (send *chidori* :move-centroid-on-foot :both (list :rleg :lleg))
      (if (> frame (setq tmp (read-shm-frame)))
          (while (> frame (setq tmp (read-shm-frame)))
            ;; sleep ??
            ;;(print (list 's tmp frame))
            (unix::usleep (* 5 100))
            )
        (progn
          (print (list 'j tmp frame))
          (setq frame tmp)
          )
        )
      (write-ref-angle-shm
       (scale (deg2rad 1) (send *chidori* :angle-vector))
       dt)
      (incf frame dt)
      (pprint 'n)
      )
    )
  )
|#

