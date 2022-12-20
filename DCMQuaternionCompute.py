import numpy as np

def rotationX(theta):
    return np.array ([[1,0,0],
                      [0,np.cos(theta),np.sin(theta)],
                      [0,-np.sin(theta),np.cos(theta)]])

def rotationY(theta):
    return np.array ([[np.cos(theta),0,-np.sin(theta)],
                      [0,1,0],
                      [np.sin(theta),0,np.cos(theta)]])

def rotationZ(theta):
    return np.array ([[np.cos(theta),np.sin(theta),0],
                      [-np.sin(theta),np.cos(theta),0],
                      [0,0,1]])
def radToDeg(rad):
    return np.array(rad) * (180.0/np.pi)

def degToRad(deg):
    return np.array(deg) * (np.pi/180.0)

def radToDeg1(rad):
    return rad * (180.0/np.pi)

def degToRad1(deg):
    return deg * (np.pi/180.0)

def dcmFromEulerXYZ(attitude_xyz):
    Rx = rotationX(attitude_xyz[0])
    Ry = rotationY(attitude_xyz[1])
    Rz = rotationZ(attitude_xyz[2])
    R = np.matmul(Rx, np.matmul(Ry,Rz))
    return R

def eulerAnglesFromRxyz(Rxyz):
    phi = np.arctan2(Rxyz[1][2],Rxyz[2][2])
    theta = -np.arcsin(Rxyz[0][2])
    psi = np.arctan2(Rxyz[0][1], Rxyz[0][0])
    return (phi, theta, psi)

def printEulerAngles(title, attitude):
    print("Euler Angles {} [{}, {}, {}]".format(title, attitude[0], attitude[1], attitude[2]))

def printQuaternion(title, quat):
    print("Quaternion {} [{}, {}, {}, {}]".format(title, quat[0], quat[1], quat[2], quat[3]))

def printDCM(title, dcm):
    print("DCM {} {}".format(title, dcm))
    
def quaternionNorm(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    return np.sqrt(q0*q0 + q1*q1 + q2*q2+ q3*q3)

def quaternionConj(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    return np.array([q0, -q1, -q2, -q3])

def quaternionInv(quat):
    conj = quaternionConj(quat)
    norm = quaternionNorm(quat)
    return conj/norm

def quaternionMult(quat1, quat2):
    q0 = quat1[0]
    q1 = quat1[1]
    q2 = quat1[2]
    q3 = quat1[3]
    Q = np.array([[q0, -q1, -q2, -q3],
                  [q1, q0, q3, -q2],
                  [q2, -q3, q0, q1],
                  [q3, q2, -q1, q0]])
    return np.matmul(Q,quat2)

def quaternionFromDCM(dcm):
    q1_den = (1.0 + dcm[0][0] + dcm[1][1] + dcm[2][2])
    q2_den = (1.0 + dcm[0][0] - dcm[1][1] - dcm[2][2])
    q3_den = (1.0 - dcm[0][0] + dcm[1][1] - dcm[2][2])
    q4_den = (1.0 - dcm[0][0] - dcm[1][1] + dcm[2][2])
    r23mr32 = dcm[1][2] - dcm[2][1]
    r31mr13 = dcm[2][0] - dcm[0][2]
    r12mr21 = dcm[0][1] - dcm[1][0]
    r23pr32 = dcm[1][2] + dcm[2][1]
    r31pr13 = dcm[2][0] + dcm[0][2]
    r12pr21 = dcm[0][1] + dcm[1][0]

    if q1_den > q2_den and q1_den > q3_den and q1_den > q4_den:
       den = np.sqrt(q1_den)
       q0 = 0.5*den
       q1 = 0.5*r23mr32/den
       q2 = 0.5*r31mr13/den
       q3 = 0.5*r12mr21/den
    elif q2_den > q1_den and q2_den > q3_den and q2_den > q4_den:
       den = np.sqrt(q2_den)
       q1 = 0.5*den
       q0 = 0.5*r23mr32/den
       q3 = 0.5*r31pr13/den
       q2 = 0.5*r12pr21/den
    elif q3_den > q1_den and q3_den > q2_den and q3_den > q4_den:
       den = np.sqrt(q3_den)
       q2 = 0.5*den
       q1 = 0.5*r12pr21/den
       q3 = 0.5*r23pr32/den
       q0 = 0.5*r31mr13/den
    elif q4_den > q1_den and q4_den > q2_den and q4_den > q3_den:
       den = np.sqrt(q3_den)
       q3 = 0.5*den
       q1 = 0.5*r31pr13/den
       q2 = 0.5*r23pr32/den
       q0 = 0.5*r12mr21/den
    return np.array([q0, q1, q2, q3])

def dcmFromQuaternion(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    r11 = q0*q0 + q1*q1 - q2*q2 - q3*q3
    r12 = 2*(q1*q2 + q0*q3)
    r13 = 2*(q1*q3 - q0*q2)
    r21 = 2*(q1*q2 - q0*q3)
    r22 = q0*q0 - q1*q1 + q2*q2 - q3*q3
    r23 = 2*(q2*q3 + q0*q1)
    r31 = 2*(q1*q3 + q0*q2)
    r32 = 2*(q2*q3 - q0*q1)
    r33 = q0*q0 - q1*q1 - q2*q2 + q3*q3
    return np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

R = dcmFromEulerXYZ(degToRad([15,30,75]))
print('Euler', R)

q = quaternionFromDCM(R)
printQuaternion("DCM", q)

x = np.array([0.5, 0.2, 0.8])
y = np.matmul(R,x)

qx = np.append(0,x)
qy = quaternionMult(q, quaternionMult(qx, quaternionInv(q)))

print("Rotation DCM: {} -> {}".format(x,y))
print("Rotation Quaternion: {} -> {}".format(qx,qy))

R1 = dcmFromQuaternion(q)
printDCM("Quaternion", R1)
