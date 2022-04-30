from utilities import *

class Controller:
    "Your Controller"
    def __init__(self):
        self.e_prev1    = 0.0
        self.e_prev2    = 0.0
        self.u_prev1    = 0.0
        self.u_prev2    = 0.0
        self.k  = 0.0
        self.z1  = 0.0
        self.z2  = 0.0
        self.p1  = 0.0
        self.p2  = 0.0

    
    def set_params(self, parameters):
        "params from matlab set_mode_params"
        if params.mode == "CLASSICAL":
            self.e_prev1    = 0.0
            self.e_prev2    = 0.0
            self.u_prev1    = 0.0
            self.u_prev2    = 0.0
            self.k  = parameters[0]
            self.z1  = parameters[1]
            self.z2  = parameters[2]
            self.p1  = parameters[3]
            self.p2  = parameters[4]

    def __call__(self, y):
        """Call controller with measurement y
        This method is called by the system.System class
        Returns:
            u (float): the command signal for the system
            l (list): a list of values that is returned to matlab
        """
        if params.mode == 'OPEN_LOOP':
            u = params.w
        elif params.mode == 'CLASSICAL':
            e = params.w - y[1] #Enkel het 2e argument omdat we momenteel slechts een controller voor de hoek implementeren
            # Classical controller. Een van de polen (degene in 0) zal een deel van de vgl doen wegvallen waardoor self.u_prev2 overbodig wordt
            u = self.k * (e - (self.z1+self.z2)*self.e_prev1 + self.z1*self.z2*self.e_prev2) + (self.p1+self.p2)*self.u_prev1 - self.p1*self.p2*self.u_prev2
            # Updaten van de fouten
            self.e_prev2 = self.e_prev1
            self.e_prev1 = e
            self.u_prev2 = self.u_prev1
            self.u_prev1 = u
        elif params.mode == 'CLASSICAL_POS':
            pass  # Remove this line
            # u_prev1, u_prev2, e_prev1, e_prev2 allemaal verdubbelen (bv u_prev1 -> u_prev_angle_1 en u_prev_pos_1)
            # Zelfde code als bij classical 2x hier kopiëren (1x exact en 1x aanpassen voor de hoek)

            # Code hier & dan de comment hieronder uncommenten

            # u = u_angle + u_pos
        elif params.mode == 'STATE_SPACE':
            pass
        elif params.mode == 'EXTENDED':
            pass
        else:
            u = 0.0
        #print(f"y = {y:5.3f}, e = {e:5.3f}, u = {u:5.3f}")
        return u, list(y)+[u]

class Observer:
    "Implement your observer"
    def __init__(self):
        pass
    def __call__(self, u, y):
        "Call observer with this method; Inputs: command u and measurement y"
        pass

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
