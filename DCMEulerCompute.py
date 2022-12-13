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

def rotationEulerXYZ(phi, theta, psi):
    Rx = rotationX(phi)
    Ry = rotationY(theta)
    Rz = rotationZ(psi)
    R = np.matmul(Rx, np.matmul(Ry,Rz))
    return R

def eulerAnglesFromRxyz(Rxyz):
    phi = np.arctan2(Rxyz[1][2],Rxyz[2][2])
    theta = -np.arcsin(Rxyz[0][2])
    psi = np.arctan2(Rxyz[0][1], Rxyz[0][0])
    return (phi, theta, psi)

def rotationEulerZXZ(phi, theta, psi):
    Rz2 = rotationZ(phi)
    Rx = rotationX(theta)
    Rz1 = rotationZ(psi)
    R = np.matmul(Rz2, np.matmul(Rx,Rz1))
    return R

def eulerAnglesFromRzxz(Rzxz):
    psi =  np.arctan2(Rzxz[0][2], Rzxz[1][2])
    theta = np.arccos(Rzxz[2][2])
    phi = np.arctan2(Rzxz[2][0], -Rzxz[2][1])
    return (phi, theta, psi)

def eulerAngleRatesXYZ(attitude, omega_body):
    phi = attitude[0]
    theta = attitude[1]
    E = np.array([[1, np.tan(theta)*np.sin(phi),np.tan(theta)*np.cos(phi)],
                  [0, np.cos(phi), -np.sin(phi)],
                  [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])
    return np.matmul(E, omega_body)

def eulerAngleRatesXYZHans(attitude, omega_body):
    phi = attitude[2]
    theta = attitude[1]
    E = np.array([[0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)],
                  [0, np.cos(phi), -np.sin(phi)],
                  [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)]])
    return np.matmul(E, omega_body)

def eulerAngleRatesZXZHans(attitude, omega_body):
    theta = attitude[1]
    psi = attitude[2]
    E = np.array ([[np.sin(psi)/np.sin(theta), np.cos(psi)/np.sin(theta), 0],
                   [np.cos(psi), -np.sin(psi), 0],
                   [-np.sin(psi)/np.tan(theta), -np.cos(psi)/np.tan(theta), 1]])
    return np.matmul(E, omega_body)

def eulerIntegration(X, Xdot, dt):
    return X + Xdot*dt

attitude = degToRad([20,-10,120])
DCM = rotationEulerXYZ(attitude[2], attitude[1], attitude[0])
print("DCM {}".format(DCM))


##dt = 0.01
##time = []
##phi = []
##theta = []
##psi = []
##for t in np.arange(0, 60, dt):
##    omega_body = degToRad([20*np.sin(0.1*t),20*0.01,20*np.cos(0.1*t)])
##    attitude_dot = eulerAngleRatesXYZHans(attitude, omega_body)
##    attitude = eulerIntegration(attitude, attitude_dot, dt)
##    time.append(t)
##    phi.append(radToDeg(attitude[0]))
##    theta.append(radToDeg(attitude[1]))
##    psi.append(radToDeg(attitude[2]))
##    if t == 42:
##        norm = np.sqrt(attitude[0]**2+attitude[1]**2+attitude[2]**2)
##        print("norm : {}".format(norm))
##        print("psi : {} ".format(attitude[0]))
##        print("theta : {} ".format(attitude[1]))
##        print("phi : {} ".format(attitude[2]))
    
##phiBN_deg = 30.0
##thetaBN_deg = 20.0
##psiBN_deg = 10.0
##phiRN_deg = 5.0
##thetaRN_deg = 5.0
##psiRN_deg = -5.0
##
##print(" Euler Angles [{}, {}, {}]".format(phiBN_deg, thetaBN_deg, psiBN_deg))
##print(" Euler Angles [{}, {}, {}]".format(phiRN_deg, thetaRN_deg, psiRN_deg))
##
##phiBN = degToRad(phiBN_deg)
##thetaBN = degToRad(thetaBN_deg)
##psiBN = degToRad (psiBN_deg)
##phiRN = degToRad(phiRN_deg)
##thetaRN = degToRad(thetaRN_deg)
##psiRN = degToRad (psiRN_deg)
##
##RxyzBN = rotationEulerXYZ(phiBN, thetaBN, psiBN)
##RxyzRN = rotationEulerXYZ(phiRN, thetaRN, psiRN)
##RxyzNR = np.transpose(RxyzRN)
##RxyzBR = np.matmul(RxyzBN, RxyzNR)
##print("RxyzBR {}".format(RxyzBR))
##
##attitude = eulerAnglesFromRxyz(RxyzBR)
##phi2_deg = radToDeg(attitude[0])
##theta2_deg = radToDeg(attitude[1])
##psi2_deg = radToDeg(attitude[2])
##print("Recovered Euler Angles [{}, {}, {}]".format(attitude[0], attitude[1], attitude[2]))
##print("Recovered Euler Angles [{}, {}, {}]".format(phi2_deg, theta2_deg, psi2_deg))
##
##
