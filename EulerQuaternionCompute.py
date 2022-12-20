import numpy as np

def radToDeg(rad):
    return np.array(rad)*180.0/np.pi

def degToRad(deg):
    return np.array(deg) * (np.pi/180.0)

def printEulerAngles(title, attitude):
    print("Euler Angles {} [{},{},{}]".format(title, attitude[0], attitude[1], attitude[2]))

def printQuaternion(title, quat):
    print("Quaternion {} [{}, {}, {}, {}]".format(title, quat[0], quat[1], quat[2], quat[3]))

def quaternionFromEulerXYZ(attitude_xyz):
    c1 = np.cos(0.5*attitude_xyz[0])
    s1 = np.sin(0.5*attitude_xyz[0])
    c2 = np.cos(0.5*attitude_xyz[1])
    s2 = np.sin(0.5*attitude_xyz[1])
    c3 = np.cos(0.5*attitude_xyz[2])
    s3 = np.sin(0.5*attitude_xyz[2])
    q0 = c1*c2*c3 + s1*s2*s3
    q1 = s1*c2*c3 - c1*s2*s3
    q2 = c1*s2*c3 + s1*c2*s3
    q3 = c1*c2*s3 - s1*s2*c3
    return np.array([q0,q1,q2,q3])
    
def eulerXYZFromQuaternion(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    r11 = q0*q0 + q1*q1 - q2*q2 - q3*q3
    r12 = 2.0*(q1*q2 + q0*q3)
    r13 = 2.0*(q1*q3 - q0*q2)
    r23 = 2.0*(q2*q3 + q0*q1)
    r33 = q0*q0 - q1*q1 - q2*q2 +q3*q3
    phi = np.arctan2(r23, r33)
    theta = -np.arcsin(r13)
    psi = np.arctan2(r12, r11)
    return np.array([phi,theta,psi])

attitude_test = degToRad([15,-35,75])
printEulerAngles("Test", radToDeg(attitude_test))

q = quaternionFromEulerXYZ(attitude_test)
printQuaternion("Euler",q)

attitude = eulerXYZFromQuaternion(q)
printEulerAngles("Quaternion", radToDeg(attitude))
