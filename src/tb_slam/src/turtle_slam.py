#!/usr/bin/env python
import roslib

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from tf.transformations import euler_from_matrix, decompose_matrix, quaternion_from_euler
import message_filters
import threading
from numpy import mat, vstack, diag, zeros, eye, inf
from numpy.linalg import inv
from math import atan2, hypot, pi, cos, sin, fmod, sqrt
from extractor.msg import featureArray

from DataAssociator import DataAssociator

MAX_MATCH_DIST = 0.5

MAX_RANGE = 10


def norm_angle(x):
    return fmod(x + pi, 2 * pi) - pi


class HuskySLAM:
    """
    class that hold the SLAM node for a robot. To work it must:
    - have a working odometry, accessible with ~odom_frame
    - have an accessible feature extraction topic with the /feature topic
    """

    def __init__(self):
        self.i = 0
        self.seen_count = {}
        rospy.init_node('husky_slam')
        rospy.loginfo("Starting husky slam")
        self.ignore_id = rospy.get_param("~ignore_id", False)
        self.target_frame = rospy.get_param("~target_frame", "/map")
        self.body_frame = rospy.get_param("~body_frame", "/base_link")
        self.odom_frame = rospy.get_param("~odom_frame", "/odom")
        self.ar_precision = rospy.get_param("~ar_precision", 0.5)
        self.position_uncertainty = rospy.get_param("~position_uncertainty", 0.01)
        self.angular_uncertainty = rospy.get_param("~angular_uncertainty", 0.01)
        self.initial_x = rospy.get_param("~initial_x", 0.0)
        self.initial_y = rospy.get_param("~initial_y", 0.0)
        self.initial_theta = rospy.get_param("~initial_theta", 0.0)
        self.lastt = (0.0, 0.0, 0.0)
        self.lastr = 0.0
        # instantiate the right filter based on launch parameters
        initial_pose = [self.initial_x, self.initial_y, self.initial_theta]
        initial_uncertainty = [0.01, 0.01, 0.01]

        self.lock = threading.Lock()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.idx = []
        self.dataAssociator = DataAssociator(self)
        self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("~landmarks", MarkerArray, queue_size=1)

        rospy.sleep(1.0)
        now = rospy.Time.now()
        lasttf = rospy.Time(0)
        self.listener.waitForTransform(self.odom_frame, self.body_frame, now,
                                       rospy.Duration(5.0))  # rospy.Duration(5.0)
        (trans, rot) = self.listener.lookupTransform(self.odom_frame, self.body_frame, lasttf)
        self.ar_sub = rospy.Subscriber("/feature", featureArray, self.ar_cb)
        print('AR_SLAM up and running.\n')


    def predict(self, dt, dr):
        """
        predict step of the kalman filter
        :param dt: translation variation 2 values vector for x translation and y translation
        :param dr: rotation variation: 1 values vector for the Z axis rotation
        :return:
        """
        theta = self.X[2, 0]
        self.X[0] = self.X[0] + dt[0]
        self.X[1] = self.X[1] + dt[1]
        self.X[2] = self.X[2] + dr
        Jx = mat([[1, 0, -sin(theta) * dt[0] - cos(theta) * dt[1]],
                  [0, 1, cos(theta) * dt[0] - sin(theta) * dt[1]],
                  [0, 0, 1]])
        Qs = mat(diag([self.position_uncertainty ** 2, self.position_uncertainty ** 2, self.angular_uncertainty ** 2]))
        P = mat(self.P[0:3, 0:3])
        self.P[0:3, 0:3] = Jx * P * Jx.T + Qs
        return self.X, self.P

    @staticmethod
    def getRotation(theta):
        """
        :param theta: the angle of the rotation
        :return: the X and Y rotation matrix around Z axis.
        """
        R = mat(zeros((2, 2)))
        R[0, 0] = cos(theta)
        R[0, 1] = -sin(theta)
        R[1, 0] = sin(theta)
        R[1, 1] = cos(theta)
        return R

    def h(self, Z):
        """
        convert from robot frame to world frame
        :param Z: 2 * 1 matrix of the element
        :return: coordinate of Z in the world frame
        """
        assert Z.shape == (2, 1)
        result = Z - self.X[0:2, 0]
        mRtheta = self.getRotation(-self.X[2, 0])
        return mRtheta * result

    def update_ar(self, Z, id, uncertainty):
        """
        the update step of the EKF
        :param Z: observation vector [[x],[y]] of the landmark in the robot frame
        :param id: location of landmark in self.X
        :param uncertainty: of the observation
        :return:
        """
        # Z is a dictionary of id->vstack([x,y])
        # print "Update: Z="+str(Z.T)+" X="+str(self.X.T)+" Id="+str(id)
        (n, _) = self.X.shape
        R = mat(diag([uncertainty, uncertainty]))
        theta = self.X[2, 0]
        Rmtheta = self.getRotation(-theta)
        H = mat(zeros((0, n)))
        if self.idx.__contains__(id):
            H = mat(zeros((2, n)))
            H[0:2, 0:2] = -Rmtheta
            H[0:2, 2] = mat(vstack(
                [-(self.X[id + 0, 0] - self.X[0, 0]) * sin(theta) + (self.X[id + 1, 0] - self.X[1, 0]) * cos(theta),
                 -(self.X[id + 0, 0] - self.X[0, 0]) * cos(theta) - (self.X[id + 1, 0] - self.X[1, 0]) * sin(theta)]))
            H[0:2, id:id + 2] = Rmtheta
            Zpred = Rmtheta * (self.X[id:id + 2, 0] - self.X[0:2, 0])
            S = H * self.P * H.T + R
            K = self.P * H.T * inv(S)
            zdiff = Z - Zpred
            if abs(zdiff[0]) + abs(zdiff[1]) <= 10:
                self.X = self.X + K * (Z - Zpred)
                self.P = (mat(eye(n)) - K * H) * self.P
            else:
                print('id: ' + repr(id) + ' error is too large.')
        else:
            print("no such id: " + str(id))
            # print("X is :"+repr(self.X[0:6, :]))
        return (self.X, self.P)

    def ar_cb(self, f):
        """
        call all the semi-independents EKF on each feature
        :param f: the feature list
        :return:
        """
        self.i = self.i+1
        print("step:"+str(self.i))
        if fmod(self.i, 10) == 0:
            for k in self.seen_count.keys():
                if self.seen_count[k] < 5:
                    self.seen_count.pop(k)
                    self.idx.remove(k)
        self.lock.acquire()
        Z_w = None
        Z = None
        for feat in f.features:
            if (feat.diameter < 0.7) and (abs(feat.position.x) < MAX_RANGE) and (abs(feat.position.y) < MAX_RANGE):
                z_r = mat([[feat.position.x], [-feat.position.y]])
                R = self.getRotation(theta=self.X[2, 0])
                x = mat(self.X[0:2, 0]) + (R * z_r)
                if Z_w is None:
                    Z_w = x
                    Z = z_r
                else:
                    Z_w = vstack((Z_w, x))
                    Z = vstack((Z, z_r))
        if Z is not None:
            best_H = self.dataAssociator.JCBB_wrapper(Z)
        else:
            best_H = []
        print("best H"+str(best_H))
        # print("Z_w"+str(Z_w))
        # print("current X" + repr(self.X))
        for row in best_H:
            if row[0, 0] != -1:
                Z_landmark = Z[row[0, 1]:row[0, 1]+2]
                id_landmark = row[0, 0]
                self.seen_count[id_landmark] += 1
                self.update_ar(Z_landmark, id_landmark, self.ar_precision)
            else:
                print(row)
                self.add_landmark_to_map(Z_w[row[0, 1]:row[0, 1]+2])
        self.lock.release()


    def add_landmark_to_map(self, Y_l):
        # print('created entry for ' + repr(x))
        # print('old X = ' + str(len(self.X)))
        (n, width) = self.X.shape
        assert width == 1, "shape :"+str(n)+","+str(width)+" is invalid !!!!"
        self.idx.append(n)
        self.seen_count[n] = 0
        self.X = mat(vstack(
            (self.X[:, 0], Y_l[:, 0])))
        Pnew = mat(diag([self.ar_precision] * (n + 2)))
        Pnew[0:n, 0:n] = self.P
        self.P = mat(Pnew)
        # print('new X = ' + str(len(self.X)))
        print("landmark", n, " added")

    def get_landmark_from_id(self, id):
        """
        return the x and y coordinates of landmark knowing it's id
        :param id: the id of the landmark in the map
        :return: x and y of the landmark in the world frame
        """
        return self.X[id:id+2, 0]

    def run(self):
        """
        update the EKF from the odom topic, at 10Hz
        :return:
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            lasttf = now  # rospy.Time(0)
            self.listener.waitForTransform(self.odom_frame, self.body_frame, now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform(self.odom_frame, self.body_frame, lasttf)
            new_odom = mat(self.listener.fromTranslationRotation(trans, rot))
            euler = tf.transformations.euler_from_quaternion(rot)
            deltar = euler[2] - self.lastr
            deltat = map(lambda x, y: x - y, trans, self.lastt)
            self.lastt = trans
            self.lastr = euler[2]
            self.lock.acquire()
            self.predict(deltat, deltar)
            theta = self.X[2, 0]
            pose_mat = mat([[cos(theta), -sin(theta), 0, self.X[0, 0]],
                            [sin(theta), cos(theta), 0, self.X[1, 0]],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1],
                            ])
            correction_mat = pose_mat * inv(new_odom)
            self.lock.release()
            scale, shear, angles, trans, persp = decompose_matrix(correction_mat)
            self.broadcaster.sendTransform(trans,
                                           quaternion_from_euler(*angles), now, self.odom_frame, self.target_frame)
            self.lock.acquire()
            self.publish(now)
            self.lock.release()
            rate.sleep()

    def publish(self, timestamp):
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = timestamp
        pose.pose.position.x = self.X[0, 0]
        pose.pose.position.y = self.X[1, 0]
        pose.pose.position.z = 0.0
        Q = quaternion_from_euler(0, 0, self.X[2, 0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        self.pose_pub.publish(pose)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        try:
            marker.scale.x = 3 * sqrt(self.P[0, 0])
            marker.scale.y = 3 * sqrt(self.P[1, 1])
        except:
            print('Error with self.P\n: ' + repr(self.P))
            marker.scale.x = 0.1
            marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        ma.markers.append(marker)
        for l in self.idx:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = self.target_frame
            marker.ns = "landmark_kf"
            marker.id = l
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = self.X[l, 0]
            marker.pose.position.y = self.X[l + 1, 0]
            marker.pose.position.z = -0.1
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = 1  # 3*sqrt(self.P[l,l])
            marker.scale.y = 1  # 3*sqrt(self.P[l+1,l+1])
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.25
            marker.color.g = 0
            marker.color.b = 0.25
            marker.lifetime.secs = 3.0
            ma.markers.append(marker)
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = self.target_frame
            marker.ns = "landmark_kf"
            marker.id = 1000 + l
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = self.X[l + 0, 0]
            marker.pose.position.y = self.X[l + 1, 0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.text = str(l)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.lifetime.secs = 3.0
            ma.markers.append(marker)
        self.marker_pub.publish(ma)


if __name__ == "__main__":
    demo = HuskySLAM()
    demo.run()
