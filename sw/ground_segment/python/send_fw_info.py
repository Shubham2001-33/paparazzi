import sys
from os import path, getenv
from time import sleep
import time


PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC =  getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

class UAV:
    def __init__(self, ac_id):
        self.initialized = False
        self.id = ac_id
        self.Ps_x = 0
        self.Ps_y = 0
        self.Ps_z = 0
        self.headingcmd = 0
        self.timeout = 0

class FFMessage:
    def __init__(self):
        self.ids = 19      #uav id
        self.uav = UAV(19)
        self._interface = IvyMessagesInterface("Formation Flight Positioning")

        def ff_cb(ac_id, msg):
            #print(msg.name)
            if ac_id == self.ids and msg.name == "FORMATION_SP":
                uav = self.uav
                uav.Ps_x = msg['Ps_x']
                uav.Ps_y = msg['Ps_y']
                uav.Ps_z = msg['Ps_z']
                uav.headingcmd = msg['headingcmd']
                uav.timeout = 0
                uav.initialized = True       
        self._interface.subscribe(ff_cb, PprzMessage("telemetry", "FORMATION_SP"))

    def send_ff_info(self):

        msgw = PprzMessage("datalink", "FORMATION_SP_to_follower")
        msgw["Ps_x"] =  float(self.uav.Ps_x) 
        msgw["Ps_y"] =  float(self.uav.Ps_y) 
        msgw["Ps_z"] =  float(self.uav.Ps_z) 
        msgw["headingcmd"] = float(self.uav.headingcmd)
        msgw["ac_id"] = 22
        self._interface.send(msgw)

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    def run(self):
        try:
            while True:
                self.send_ff_info()
                sleep(1/50)
                
        except KeyboardInterrupt:
            self.stop()
    
    


if __name__ == '__main__':
    import argparse

    Message_ff = FFMessage()
    Message_ff.run()