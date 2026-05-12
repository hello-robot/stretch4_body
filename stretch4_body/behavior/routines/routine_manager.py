from stretch4_body.core.device import Device
import stretch4_body.core.hello_utils as hu
import time
import importlib

# #########################################################3


class RoutineManager(Device):
    """
    Manages a set of plug-ins that execute routines on the robot
    """
    def __init__(self,robot):
        Device.__init__(self,"routine_manager")
        self.robot=robot
        self.status = {'last_routine_id': 0,
                      'active_routine': 'routine_nop',
                      'active_routine_id': 0,
                      'last_routine_successful': False}
        self.next_routine = 'routine_nop'
        self.next_routine_id = 0
        self.next_routine_args = ()
        self.next_routine_kwargs = {}
        self.routines = {}
        for k in self.params['controllers']:
            s = getattr(importlib.import_module(self.robot_params[k]['py_module_name']),
                        self.robot_params[k]['py_class_name'])(self.robot)
            if s.params.get('enabled',1):
                self.routines[k] = s

    def startup(self):
        success = True
        for k in self.routines:
            if hasattr(self.routines[k], 'startup'):
                success = success & self.routines[k].startup()
        return success

    def stop(self):
        success = True
        for k in self.routines:
            if hasattr(self.routines[k], 'stop'):
                success = success & self.routines[k].stop()
        return success

    def cancel(self, id:str|None=None):
        if self.status['active_routine'] != 'routine_nop':
            if id is None or str(id) == str(self.status['active_routine_id']):
                self.logger.info(f'Cancelling routine {self.status["active_routine"]}')
                self.routines[self.status['active_routine']].cancel()
        elif self.next_routine != 'routine_nop':
            if id is None or str(id) == str(self.next_routine_id):
                self.logger.info(f'Cancelling upcoming routine {self.next_routine}')
                self.next_routine = 'routine_nop'
                self.next_routine_id = 0

    def _routine_run_next(self):
        # Run the routine's loop. This will be called every control cycle return when the routine finishes
        # Or when server exits the control loop
        self.status['active_routine']=self.next_routine
        self.status['active_routine_id']=self.next_routine_id
        self.next_routine='routine_nop'
        self.next_routine_id=0
        success = self.routines[self.status['active_routine']].run(self.status['active_routine_id'],*self.next_routine_args,**self.next_routine_kwargs)
        if self.status['active_routine_id']!=0: #Record routine as finished for non nop routines
            self.status['last_routine_id'] = self.status['active_routine_id']
            self.status['last_routine_successful'] = success

    def _routine_set_next(self, routine_name,cmd_id,*args,**kwargs):
        routine_id=cmd_id
        #Called on new command from client
        if self.status['active_routine'] != 'routine_nop':  # Reject commands while a routine is executing
            self.next_routine = 'routine_nop'
            self.next_routine_id = 0
            self.logger.warning(f'Routine {self.status["active_routine"]} is running. Rejecting command {routine_name}|{routine_id}')
            return
        if routine_name in self.routines.keys():
            if not self.routines[routine_name].is_valid:
                self.logger.warning(f'Routine {routine_name} is not valid. Rejecting command {routine_name}|{routine_id}')
                return
            self.logger.info(f'Got routine to run: {routine_name} with id {routine_id}')
            self.next_routine=routine_name
            self.next_routine_id = routine_id
            self.next_routine_args=args
            self.next_routine_kwargs = kwargs
        else:
            self.logger.error(f'Invalid routine name: {routine_name}')
