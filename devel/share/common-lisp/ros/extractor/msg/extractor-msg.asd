
(cl:in-package :asdf)

(defsystem "extractor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "feature" :depends-on ("_package_feature"))
    (:file "_package_feature" :depends-on ("_package"))
    (:file "featureArray" :depends-on ("_package_featureArray"))
    (:file "_package_featureArray" :depends-on ("_package"))
    (:file "map" :depends-on ("_package_map"))
    (:file "_package_map" :depends-on ("_package"))
  ))