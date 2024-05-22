# interface between casadi and python control 
class OptRes():
    def __init__(self,time,inputs,states,Xd,Ud):
        self.time = time
        self.inputs = inputs
        self.states = states
        self.Xd = Xd
        self.Ud = Ud