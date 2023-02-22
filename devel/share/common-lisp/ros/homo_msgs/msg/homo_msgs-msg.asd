
(cl:in-package :asdf)

(defsystem "homo_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HomographyResult" :depends-on ("_package_HomographyResult"))
    (:file "_package_HomographyResult" :depends-on ("_package"))
  ))