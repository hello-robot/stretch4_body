import stretch4_body.subsystem.omnibase as base
import stretch4_body.subsystem.power_periph as pimu
import time
import matplotlib.pyplot as plt

p = pimu.PowerPeriph()
p.startup()

b = base.OmniBase()
if not b.startup():
    exit()

b.set_guarded_contact_sensitivity('off')
b.push_command()
p.trigger_motor_sync()

class BaseTunning():
    def init(self):
        self.x_des = 0
        self.y_des = 0
        self.v_des = 0
        self.a_des = 0
        self.start = 0
        self.time = []
        self.y_cmd0 = []
        self.y_act0 = []
        self.y_cmd1 = []
        self.y_act1 = []
        self.y_cmd2 = []
        self.y_act2 = []

    def get_gains(self):
        b.pull_status()
        print('Current gains: ')
        for w in b.wheels:
            print('Wheel: ', w.name)
            print('kp = ', w.gains['pKp_d'])
            print('ki = ', w.gains['pKi_d'])
            print('kd = ', w.gains['pKd_d'])

    def set_gains(self):
        kp = float(input('Enter new kp value: '))
        ki = float(input('Enter new ki value: '))
        kd = float(input('Enter new kd value: '))

        for w in b.wheels:
            w.gains['pKp_d'] = kp
            w.gains['pKi_d'] = ki
            w.gains['pKd_d'] = kd
            w.set_gains(w.gains)
            w.push_command()
        # b.push_command()
        # p.trigger_motor_sync()

    def set_traj(self):
        self.x_des = float(input('Enter x desired: '))
        self.y_des = float(input('Enter y desired: '))
        self.v_des = float(input('Enter v desired: '))
        self.a_des = float(input('Enter a desired: '))

    def reset(self):
        self.start = time.time()
        self.time = []
        self.y_cmd0 = []
        self.y_act0 = []
        self.y_cmd1 = []
        self.y_act1 = []
        self.y_cmd2 = []
        self.y_act2 = []
    
    def plot(self):
        plt.figure(1)
        plt.plot(self.time, self.y_cmd0, label = 'CMD')
        plt.plot(self.time, self.y_act0, label = 'Act')
        plt.title('Wheel 0')

        plt.figure(2)
        plt.plot(self.time, self.y_cmd1, label = 'CMD')
        plt.plot(self.time, self.y_act1, label = 'Act')
        plt.title('Wheel 1')

        plt.figure(3)
        plt.plot(self.time, self.y_cmd2, label = 'CMD')
        plt.plot(self.time, self.y_act2, label = 'Act')
        plt.title('Wheel 2')

        plt.show()
        return

    def update(self):
        self.reset()
        b.translate_by(self.x_des, self.y_des, self.v_des, self.a_des)
        b.push_command()
        p.trigger_motor_sync()
        time.sleep(0.1)
        b.pull_status()
        x0 = b.wheels[0].status['pos'] + b.wheels[0]._command['x_des']
        x1 = b.wheels[1].status['pos'] + b.wheels[1]._command['x_des']
        x2 = b.wheels[2].status['pos'] + b.wheels[2]._command['x_des']
        while not (b.wheels[0].status['near_pos_setpoint'] and b.wheels[1].status['near_pos_setpoint'] and b.wheels[2].status['near_pos_setpoint']):
            b.pull_status()
            self.time.append(time.time() - self.start)
            self.y_cmd0.append(x0)
            self.y_act0.append(b.wheels[0].status['pos'])
            self.y_cmd1.append(x1)
            self.y_act1.append(b.wheels[1].status['pos'])
            self.y_cmd2.append(x2)
            self.y_act2.append(b.wheels[2].status['pos'])
        self.plot()

    def menu(self):
        print("-------------------------------")
        print('1: print current gains')
        print('2: set new gains')
        print('3: set new set points')
        print('4: run and plot')
        print('q: quit()')
        print("-------------------------------")


if __name__ == "__main__":
    bt = BaseTunning()
    while True:
        bt.menu()
        i = input('Input: ')
        
        if i == '1':
            bt.get_gains()
        
        if i == '2':
            bt.set_gains()
        
        if i == '3':
            bt.set_traj()
        
        if i == '4':
            bt.update()
        
        if i == 'q' or i == 'Q':
            b.stop()
            p.stop()
            quit()

        
    
