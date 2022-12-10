#!/usr/bin/env python3

import numpy as np
import rospy
import tf

from core.interfaces import ObjectDetector

# Broadcasts T as the transform from from_frame to to_frame
def show_pose(T,frame_from,frame_to):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(T),
        tf.transformations.quaternion_from_matrix(T),
        rospy.Time.now(),
        frame_from,
        frame_to
    )

if __name__ == "__main__":

    np.set_printoptions(precision=4,suppress=True)

    rospy.init_node("detector_test")
    tf_broad  = tf.TransformBroadcaster()

    detector = ObjectDetector()

    r = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        r.sleep()
        detections = detector.get_detections()
        print(len(detections),'tags detected')

        detections.sort(key=lambda y: y[0])
        mult = {}
        for tag in detections:
            (name, pose) = tag
            if name in mult:
                mult[name].append(pose)
            else:
                mult[name] = [pose]
        for (name, Ts) in mult.items():
            for i, T in enumerate(Ts):
                frame = name+"_"+str(i)
                show_pose(T,frame,'camera')
