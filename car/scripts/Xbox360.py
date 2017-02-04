"""
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
Author: Ryan Dellana
Modify Date: 01/04/2017
About:
Takes raw ROS joy messages and generates events like 'X_pressed', 'X_released', etc.
Also converts X/Y format of sticks to theta and magnitude.
Usage Example:
# import class
import XBox360
# Do this once in your main method.
controller = XBox360()
# Do this every time you receive a new joy message (let's call it 'msg')
controller.update(msg)
# To get new button events use:
evts = controller.buttonEvents()
# evts should now contain a list of strings describing button events.
# To respond to a specific event, you might do something like:
if 'Y_pressed' in evts:
    # do something in response to event ...
# To access the stick angles and magnitudes, use:
controller.left_stick['angle']
controller.left_stick['magnitude']
"""

import math, copy

class XBox360(object):

    def __init__(self):
        # XBOX 360
        self.idx_2_btn = {0:'A',1:'B',2:'X',3:'Y',4:'LB',5:'RB',6:'back',7:'start',8:'bigX',9:'LStick',10:'RStick',11:'ArrowL',12:'ArrowR',13:'ArrowU',14:'ArrowD'}
        self.btn_2_idx = {'A':0,'B':1,'X':2,'Y':3,'LB':4,'RB':5,'back':6,'start':7,'bigX':8,'LStick':9,'RStick':10,'ArrowL':11,'ArrowR':12,'ArrowU':13,'ArrowD':14}
        # Logitech F310        
        #self.idx_2_btn = {0:'A',1:'B',2:'X',3:'Y',4:'LB',5:'RB',6:'back',7:'start',8:'bigX',9:'LStick',10:'RStick'}
        #self.btn_2_idx = {'A':0,'B':1,'X':2,'Y':3,'LB':4,'RB':5,'back':6,'start':7,'bigX':8,'LStick':9,'RStick':10}
        self.btn_state_prev = {}
        for k in self.btn_2_idx.keys():
            self.btn_state_prev[k] = False
        self.btn_state_curr = copy.deepcopy(self.btn_state_prev)
        self.btn_events = []
        self.left_stick = {'angle':0.0,'magnitude':0.0,'X':0.0,'Y':0.0}
        self.right_stick = {'angle':0.0,'magnitude':0.0,'X':0.0,'Y':0.0}
        self.left_trigger, self.right_trigger = 0.0, 0.0

    def update(self, data):
        axes = {'LTrigger':data.axes[2],'RTrigger':data.axes[5],'LStickX':data.axes[0],
                'LStickY':data.axes[1],'RStickX':data.axes[3],'RStickY':data.axes[4]}
        self.left_stick['X'], self.left_stick['Y'] = axes['LStickX'], axes['LStickY']
        self.right_stick['X'], self.right_stick['Y'] = axes['RStickX'], axes['RStickY']
        lstick_theta, lstick_mag = self.cartesian_2_polar(axes['LStickX'],axes['LStickY'])
        rstick_theta, rstick_mag = self.cartesian_2_polar(axes['RStickX'],axes['RStickY'])
        self.left_stick['angle'], self.right_stick['angle'] = lstick_theta, rstick_theta
        self.left_stick['magnitude'], self.right_stick['magnitude'] = lstick_mag, rstick_mag
        self.left_trigger, self.right_trigger = axes['LTrigger'], axes['RTrigger']
        self.btn_events = self._get_btn_events(data)
        
    def buttonEvents(self):
        tmp = self.btn_events
        self.btn_events = []
        return tmp

    def leftStick(self):
        return self.left_stick

    def rightStick(self):
        return self.right_stick

    def leftTrigger(self):
        return self.left_trigger

    def rightTrigger(self):
        return self.right_trigger

    def cartesian_2_polar(self, x, y):
        theta = math.atan2(y, x) * 180 / math.pi
        mag = math.sqrt(x*x + y*y)
        return theta, mag

    def _get_btn_events(self, data):
        btn_evts = []
        for i in range(0, 11):
            self.btn_state_prev[self.idx_2_btn[i]] = self.btn_state_curr[self.idx_2_btn[i]]
            self.btn_state_curr[self.idx_2_btn[i]] = data.buttons[i]
        for i in range(0, 11):
            if self.btn_state_curr[self.idx_2_btn[i]] != self.btn_state_prev[self.idx_2_btn[i]]:
                if self.btn_state_curr[self.idx_2_btn[i]] == True:
                    btn_evts.append(self.idx_2_btn[i] + "_pressed")
                else:
                    btn_evts.append(self.idx_2_btn[i] + "_released")
        return btn_evts
