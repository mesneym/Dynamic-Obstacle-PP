
(cl:in-package :asdf)

(defsystem "ardrone_as-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ArdroneAction" :depends-on ("_package_ArdroneAction"))
    (:file "_package_ArdroneAction" :depends-on ("_package"))
    (:file "ArdroneActionFeedback" :depends-on ("_package_ArdroneActionFeedback"))
    (:file "_package_ArdroneActionFeedback" :depends-on ("_package"))
    (:file "ArdroneActionGoal" :depends-on ("_package_ArdroneActionGoal"))
    (:file "_package_ArdroneActionGoal" :depends-on ("_package"))
    (:file "ArdroneActionResult" :depends-on ("_package_ArdroneActionResult"))
    (:file "_package_ArdroneActionResult" :depends-on ("_package"))
    (:file "ArdroneFeedback" :depends-on ("_package_ArdroneFeedback"))
    (:file "_package_ArdroneFeedback" :depends-on ("_package"))
    (:file "ArdroneGoal" :depends-on ("_package_ArdroneGoal"))
    (:file "_package_ArdroneGoal" :depends-on ("_package"))
    (:file "ArdroneResult" :depends-on ("_package_ArdroneResult"))
    (:file "_package_ArdroneResult" :depends-on ("_package"))
  ))