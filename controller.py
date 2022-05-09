from ctypes import sizeof
from email.headerregistry import ParameterizedMIMEHeader
import numpy as np

from utilities import *

class Controller:
    "Your Controller"
    def __init__(self):
        self.e1_prev1    = 0.0
        self.e1_prev2    = 0.0
        self.e1_prev3    = 0.0
        self.e1_prev4    = 0.0
        self.u1_prev1    = 0.0
        self.u1_prev2    = 0.0
        self.u1_prev3    = 0.0
        self.u1_prev4    = 0.0
        self.k1 = 0.0
        self.z11    = 0.0
        self.z12    = 0.0
        self.z13    = 0.0
        self.p11    = 0.0
        self.p12    = 0.0
        self.p13    = 0.0
        self.p14    = 0.0

        self.e2_prev1    = 0.0
        self.e2_prev2    = 0.0
        self.u2_prev1    = 0.0
        self.u2_prev2    = 0.0
        self.k2 = 0.0
        self.z21    = 0.0
        self.z22    = 0.0
        self.p21    = 0.0
        self.p22    = 0.0

        self.x_hat = np.zeros((4, 1))
        self.observer = Observer()

        self.angle_factor = 11 / 10 

    
    def set_params(self, parameters):
        "params from matlab set_mode_params"
        if params.mode == "CLASSICAL_ANG":
            self.e2_prev1    = 0.0
            self.e2_prev2    = 0.0
            self.u2_prev1    = 0.0
            self.u2_prev2    = 0.0
            self.k2  = parameters[0]
            self.z21  = parameters[1]
            self.z22  = parameters[2]
            self.p21  = parameters[3]
            self.p22  = parameters[4]

        elif params.mode == "CLASSICAL_COMB":
            self.e1_prev1    = 0.0
            self.e1_prev2    = 0.0
            self.e1_prev3    = 0.0
            self.e1_prev4    = 0.0
            self.u1_prev1    = 0.0
            self.u1_prev2    = 0.0
            self.u1_prev3    = 0.0
            self.u1_prev4    = 0.0
            self.k1  = parameters[0]
            self.z11  = parameters[1]
            self.z12  = parameters[2]
            self.z13  = parameters[3]
            self.p11  = parameters[4]
            self.p12  = parameters[5]
            self.p13  = parameters[6]
            self.p14  = parameters[7]

            self.e2_prev1    = 0.0
            self.e2_prev2    = 0.0
            self.u2_prev1    = 0.0
            self.u2_prev2    = 0.0
            self.k2  = parameters[8]
            self.z21  = parameters[9]
            self.z22  = parameters[10]
            self.p21  = parameters[11]
            self.p22  = parameters[12]

        elif params.mode == "OBSERVER_TEST":
            self.e1_prev1    = 0.0
            self.e1_prev2    = 0.0
            self.e1_prev3    = 0.0
            self.e1_prev4    = 0.0
            self.u1_prev1    = 0.0
            self.u1_prev2    = 0.0
            self.u1_prev3    = 0.0
            self.u1_prev4    = 0.0
            self.k1  = parameters[0]
            self.z11  = parameters[1]
            self.z12  = parameters[2]
            self.z13  = parameters[3]
            self.p11  = parameters[4]
            self.p12  = parameters[5]
            self.p13  = parameters[6]
            self.p14  = parameters[7]

            self.e2_prev1    = 0.0
            self.e2_prev2    = 0.0
            self.u2_prev1    = 0.0
            self.u2_prev2    = 0.0
            self.k2  = parameters[8]
            self.z21  = parameters[9]
            self.z22  = parameters[10]
            self.p21  = parameters[11]
            self.p22  = parameters[12]

            L1 = np.reshape(parameters[13:21], (4, 2), order='F')
            A1 = np.reshape(parameters[21:37], (4, 4), order='F')
            B1 = np.reshape(parameters[37:41], (1, 4), order='F')
            C1 = np.reshape(parameters[41:49], (2, 4), order='F')
            L2 = np.reshape(parameters[49:57], (4, 2), order='F')

            self.x_hat[2] = np.pi

            # Set observer params
            self.observer.set_arrays(L1, A1, B1, C1, L2)

        elif params.mode == "STATE_SPACE":
            pass
        elif params.mode == 'EXTENDED':
            pass


    def __call__(self, y):
        """Call controller with measurement y
        This method is called by the system.System class
        Returns:
            u (float): the command signal for the system
            l (list): a list of values that is returned to matlab
        """
        if params.mode == 'OPEN_LOOP':
            u = params.w
            out = list(y)+[u]

        elif params.mode == 'CLASSICAL_ANG':
            # Enkel implementatie hoek
            e = params.w - y[1] #Enkel het 2e argument omdat we momenteel slechts een controller voor de hoek implementeren
            # Classical controller. Een van de polen (degene in 0) zal een deel van de vgl doen wegvallen waardoor self.u_prev2 overbodig wordt
            u = self.k2 * (e - (self.z21+self.z22)*self.e2_prev1 + self.z21*self.z22*self.e2_prev2) + (self.p21+self.p22)*self.u2_prev1 - self.p21*self.p22*self.u2_prev2
            if abs(u) > 10:
                u = np.sign(u)*10
            # Updaten van de fouten
            self.e2_prev2 = self.e2_prev1
            self.e2_prev1 = e
            self.u2_prev2 = self.u2_prev1
            self.u2_prev1 = u
            out = list(y)+[u]

        elif params.mode == 'CLASSICAL_COMB':
            e1 = params.w - y[0]
            e2 = - y[1]

            A1 = (self.p11+self.p12+self.p13+self.p14)
            A2 = (self.p11*self.p12+self.p12*self.p13+self.p13*self.p14+self.p11*self.p13+self.p12*self.p14+self.p11*self.p14)
            A3 = (self.p11*self.p12*self.p13+self.p11*self.p13*self.p14+self.p12*self.p13*self.p14+self.p11*self.p12*self.p14)
            A4 = self.p11*self.p12*self.p13*self.p14

            B1 = 1
            B2 = (self.z11+self.z12+self.z13)
            B3 = (self.z11*self.z12+self.z11*self.z13+self.z12*self.z13)
            B4 = self.z11*self.z12*self.z13

            u1 = -A4*self.u1_prev4+A3*self.u1_prev3-A2*self.u1_prev2+A1*self.u1_prev1+self.k1*(B1*self.e1_prev1-B2*self.e1_prev2+B3*self.e1_prev3-B4*self.e1_prev4)
            u2 = self.k2 * (e2 - (self.z21+self.z22)*self.e2_prev1 + self.z21*self.z22*self.e2_prev2) + (self.p21+self.p22)*self.u2_prev1 - self.p21*self.p22*self.u2_prev2
    
            u = u1 + u2

            if abs(u) > 10:
                u1 = np.sign(u) * 10 * u1 / (u1 + u2) * self.angle_factor
                u2 = np.sign(u) * 10 * u2 / (u1 + u2) * (1-self.angle_factor)
                u = u1 + u2

            self.e1_prev4 = self.e1_prev3
            self.e1_prev3 = self.e1_prev2
            self.e1_prev2 = self.e1_prev1
            self.e1_prev1 = e1
            self.u1_prev4 = self.u1_prev3
            self.u1_prev3 = self.u1_prev2
            self.u1_prev2 = self.u1_prev1
            self.u1_prev1 = u1

            self.e2_prev2 = self.e2_prev1
            self.e2_prev1 = e2
            self.u2_prev2 = self.u2_prev1
            self.u2_prev1 = u2
            out = list(y)+[u]

        elif params.mode == 'OBSERVER_TEST':
            e1 = params.w - y[0]
            e2 = - y[1]

            A1 = (self.p11+self.p12+self.p13+self.p14)
            A2 = (self.p11*self.p12+self.p12*self.p13+self.p13*self.p14+self.p11*self.p13+self.p12*self.p14+self.p11*self.p14)
            A3 = (self.p11*self.p12*self.p13+self.p11*self.p13*self.p14+self.p12*self.p13*self.p14+self.p11*self.p12*self.p14)
            A4 = self.p11*self.p12*self.p13*self.p14

            B1 = 1
            B2 = (self.z11+self.z12+self.z13)
            B3 = (self.z11*self.z12+self.z11*self.z13+self.z12*self.z13)
            B4 = self.z11*self.z12*self.z13

            u1 = -A4*self.u1_prev4+A3*self.u1_prev3-A2*self.u1_prev2+A1*self.u1_prev1+self.k1*(B1*self.e1_prev1-B2*self.e1_prev2+B3*self.e1_prev3-B4*self.e1_prev4)
            u2 = self.k2 * (e2 - (self.z21+self.z22)*self.e2_prev1 + self.z21*self.z22*self.e2_prev2) + (self.p21+self.p22)*self.u2_prev1 - self.p21*self.p22*self.u2_prev2
    
            u = u1 + u2

            if abs(u) > 10:
                u1 = np.sign(u) * 10 * u1 / (u1 + u2) * self.angle_factor
                u2 = np.sign(u) * 10 * u2 / (u1 + u2) * (1-self.angle_factor)
                u = u1 + u2

            self.e1_prev4 = self.e1_prev3
            self.e1_prev3 = self.e1_prev2
            self.e1_prev2 = self.e1_prev1
            self.e1_prev1 = e1
            self.u1_prev4 = self.u1_prev3
            self.u1_prev3 = self.u1_prev2
            self.u1_prev2 = self.u1_prev1
            self.u1_prev1 = u1

            self.e2_prev2 = self.e2_prev1
            self.e2_prev1 = e2
            self.u2_prev2 = self.u2_prev1
            self.u2_prev1 = u2
            self.x_hat = self.observer(u, y, self.x_hat)
            out = list(y)+[u]+list(np.concatenate(self.x_hat))

        elif params.mode == 'STATE_SPACE':
            pass
        elif params.mode == 'EXTENDED':
            pass
        else:
            u = 0.0
        #print(f"y = {y:5.3f}, e = {e:5.3f}, u = {u:5.3f}")
        return u, out

class Observer:
    "Implement your observer"
    def __init__(self):
        self.L1 = np.zeros((4, 2))
        self.A1 = np.zeros((4, 4))
        self.B1 = np.zeros((1, 4))
        self.C1 = np.zeros((2, 4))
        self.L2 = np.zeros((4, 2))

    def set_arrays(self, L1, A1, B1, C1, L2):
        self.L1 = np.array(L1)
        self.A1 = np.array(A1)
        self.B1 = np.array(B1)
        self.C1 = np.array(C1)
        self.L2 = np.array(L2)

    def __call__(self, u, y, x_hat):
        "Call observer with this method; Inputs: command u and measurement y"
        if abs(y[1]) > np.pi/3:
            x_hat = np.matmul(self.A1 - np.matmul(self.L1,self.C1),x_hat) + np.transpose(self.B1*u) + [[x] for x in np.matmul(self.L1,y)]
        else:
            x_hat = np.matmul(self.A1 - np.matmul(self.L2,self.C1),x_hat) + np.transpose(self.B1*u) + [[x] for x in np.matmul(self.L2,y)]
        return x_hat


#### ------ don't change anything below ----------- ####

if __name__ == '__main__': 
    params = Params()
    if len(sys.argv) > 1:
        params.ip = int(sys.argv[1])
    if len(sys.argv) > 2:
        params.Ts = float(sys.argv[2])
    
    print(f'Sampling period = {params.Ts}\nListening on port {params.ip}')
    controller = Controller()

    sys_comms_thread = threading.Thread(target=system_comms, args=(controller, params))
    sys_comms_thread.start()

    matlab_comms_thread = threading.Thread(target=matlab_comms, args=(controller, params))
    matlab_comms_thread.start()
