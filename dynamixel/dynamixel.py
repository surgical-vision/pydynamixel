#!/usr/bin/env python2.6 

"""
This is a Python version of the ForestMoon Dynamixel library originally
written in C# by Scott Ferguson.

The Python version was created by Patrick Goebel (mailto:patrick@pirobot.org)
for the Pi Robot Project which lives at http://www.pirobot.org.

The original license for the C# version is as follows:

This software was written and developed by Scott Ferguson.
The current version can be found at http://www.forestmoon.com/Software/.
This free software is distributed under the GNU General Public License.
See http://www.gnu.org/licenses/gpl.html for details.
This license restricts your usage of the software in derivative works.

* * * * * 

Dynamixel interface

"""

import defs
import time
import dynamixel_network
import re
import operator

from defs import DEVICE

AX12 = DEVICE['AX12']
AXS1 = DEVICE['AXS1']
AXS1PropNames = {}


class Dynamixel (object):
    """ Dynamixel class """
    def __init__(self, ident, dyn_net, servo_type):
        """ Constructor
        ident - the id for this dynamixel
        dyn_net - the parent dynamixel network
        """
        self._id = ident
        self._dyn_net = dyn_net
        self.cache = {}
        self.changed = False
        self._synchronized = True
        if type(servo_type) is DEVICE:
            self._register = servo_type
        else:
            self._register = DEVICE[servo_type]

    def __getitem__(self, register):
        """ Get a cache value
        
        register - register to retrieve

        returns value or -1 if not in cache
        """
        data = -1
        if register in self.cache:
            data = self.cache[register]
        return data

    def __setitem__(self, register, value):
        """ Set a cache value
        
        register - register to retrieve
        """

        self.cache[register] = value

    def __str__(self):
        return "Dyn %d" % (self._id)

    def _no_cache(self, register):
        """ deteremine if a register value should be cached

        register - register
        
        returns True if should not be cached
        """
        return register in [self._register.CurrentLoad, 
                            self._register.CurrentPosition, 
                            self._register.CurrentSpeed, 
                            self._register.CurrentTemperature, 
                            self._register.CurrentVoltage, 
                            self._register.Moving,
                            self._register.TorqueEnable]

    def _get_register_value(self, register):
        """ Get a register value from the cache, if present,
        or by reading the value from the Dynamixel

        reg - register to read
        
        return  the register value"""
        register_length = self.register_length(register)
        
        if register in [self._register.GoalPosition, self._register.MovingSpeed]:
            return self[register]
        if self._no_cache(register):
            return self._dyn_net.read_register(self._id,
                                               register,
                                               register_length)
        else:
            value = self[register]
            if value == -1:
                return self._dyn_net.read_register(self._id,
                                                   register,
                                                   register_length)
            else:
                return value

    def set_register_value(self, register, value):
        """Set a register value and record in the cache, if applicable.
        
        register - register
        value - byte or word value
        """
        if register in [self._register.GoalPosition, self._register.MovingSpeed]:
            if self._synchronized:
                if register == self._register.MovingSpeed and value == 0:
                    value = 1
                    print "Moving speed %d " % (value)
                self[register] = value
                self.changed = True
            elif register in [self._register.ModelNumber, 
                              self._register.FirmwareVersion, 
                              self._register.CurrentPosition, 
                              self._register.CurrentSpeed, 
                              self._register.CurrentLoad, 
                              self._register.CurrentVoltage,
                              self._register.CurrentTemperature, 
                              self._register.Moving]:
                raise ValueError("Cannot set register")
            
        register_length = self.register_length(register)
        if self._no_cache(register):
            self._dyn_net.write_register(self._id,
                                         register,
                                         register_length,
                                         value,
                                         False)
            return
        if self[register] == value:
            return
        
        self._dyn_net.write_register(self._id,
                                     register,
                                     register_length,
                                     value, False)
        self[register] = value

    def register_length(self, register):
        """
        Returns if the logical register is 1 or 2 bytes in length
        """
        
        # Get this value from the module enum
        desc = self._register.description(value = register)
        
        return desc['registerLen']

    def read_all(self):
        """ Read all register values into the cache """
        
        registers = []
        # Create tuples of (RegisterAddress, RegisterLength)
        for i in sorted(self._register.values()):
            description = self._register.description(value = i)
            regLen = description['registerLen']
            registers.append((i, regLen))
        
        # Get all relevant values
        values = self._dyn_net.read_registers(self._id, registers)
        
        # Put those into the cache
        for i in range(len(values)):
            addr = registers[i][0]
            val = values[i]
            self[addr] = val

    def reset(self, ident):
        
        """Resets a dynamixel
        
        ident - id to reset

        Note:
        This function should be used carefully, if at all.
        It resets the unit ID to 1, so careful planning is required to to
        avoid collisions on a network with more than one Dynamixel.
        """
        self._dyn_net.write_instruction(ident, defs.INSTRUCTION.Reset, None)
        self._dyn_net.await_packet(ident, 0)
                
    def reset_registers(self):
        """Reset register values to factory default"""       
        self._dyn_net.dynamixel_id_change(self, 1)
        self.reset(self._id)
        self._id = 1
        time.sleep(0.3)
        self.read_all()

    def stop(self):
        """ 
        Stop the Dynamixel from moving.

        There is no direct way to command a Dynamixel to stop.
        And there is no way to set the speed to 0, since the value 0 is
        specially interpreted to mean 'as fast as possibly'.
        The best we can do is command it to move to its current position
        and set the speed to 1 to slow it down as much as possible.
        If there is any significant lag between receiving the CurrentPosition
        and setting it and the speed, there will be some residual movement
        as the Dynamixel moves to its observed but stale CurrentPosition.
         If the Dynamixel is in Sychronized mode, a call to 
        'DynamixelNextwork.Synchronize' will be required to complete the operation.        
        """
        self.goal_position = self.current_position
        self.moving_speed = 1

    ###########################################################################
    # Properties
    
    def _get_synchronized(self):
        """getter"""
        return self._synchronized
    
    def _set_synchronized(self, value):
        """ setter """   
        self._synchronized = value                  
        
    synchronized = property(_get_synchronized, _set_synchronized)
       
    def _get_goal_position(self):
        """getter"""
        return self._get_register_value(self._register.GoalPosition)
    
    def _set_goal_position(self, value):
        """ setter """                
        self.set_register_value(self._register.GoalPosition, value)     
        
    goal_position = property(_get_goal_position, _set_goal_position)

    def _get_moving_speed(self):
        """getter"""        
        return self._get_register_value(self._register.MovingSpeed)
    
    def _set_moving_speed(self, value):
        """ setter """                
        self.set_register_value(self._register.MovingSpeed, value)
    
    moving_speed = property(_get_moving_speed, _set_moving_speed)

    def _get_alarm_led(self):
        """getter"""        
        return self._get_register_value(self._register.AlarmLED)
    
    def _set_alarm_led(self, value):
        """ setter """                
        self.set_register_value(self._register.AlarmLED, value)

    alarm_led = property(_get_alarm_led, _set_alarm_led)

    def _get_alarm_shutdown(self):
        """getter"""        
        return self._get_register_value(self._register.AlarmShutdown)
    
    def _set_alarm_shutdown(self, value):
        """ setter """                
        self.set_register_value(self._register.AlarmShutdown, value)

    alarm_shutdown = property(_get_alarm_shutdown, _set_alarm_shutdown)

    def _get_baud_rate(self):
        """getter"""        
        return self._get_register_value(self._register.BaudRate)
    
    def _set_baud_rate(self, value):
        """ setter """                
        self.set_register_value(self._register.BaudRate, value)

    baud_rate = property(_get_baud_rate, _set_baud_rate)

    def _get_cw_angle_limit(self):
        """getter"""        
        return self._get_register_value(self._register.CWAngleLimit)
    
    def _set_cw_angle_limit(self, value):
        """ setter """                
        self.set_register_value(self._register.CWAngleLimit, value)

    cw_angle_limit = property(_get_cw_angle_limit, _set_cw_angle_limit)

    def _get_ccw_angle_limit(self):
        """getter"""        
        return self._get_register_value(self._register.CCWAngleLimit)
    
    def _set_ccw_angle_limit(self, value):
        """ setter """                
        self.set_register_value(self._register.CCWAngleLimit, value)

    ccw_angle_limit = property(_get_ccw_angle_limit, _set_ccw_angle_limit)

    def _get_d_gain(self):
        """getter"""
        return self._get_register_value(self._register.Dgain)

    def _set_d_gain(self, value):
        """ setter """
        self.set_register_value(self._register.Dgain, value)

    d_gain = property(_get_d_gain, _set_d_gain)

    def _get_i_gain(self):
        """getter"""
        return self._get_register_value(self._register.Igain)

    def _set_i_gain(self, value):
        """ setter """
        self.set_register_value(self._register.Igain, value)

    i_gain = property(_get_d_gain, _set_d_gain)

    def _get_p_gain(self):
        """getter"""
        return self._get_register_value(self._register.Pgain)

    def _set_p_gain(self, value):
        """ setter """
        self.set_register_value(self._register.Pgain, value)

    p_gain = property(_get_d_gain, _set_d_gain)

    def _get_ccw_compliance_margin(self):
        """getter"""
        return self._get_register_value(self._register.CCWComplianceMargin)

    def _set_ccw_compliance_margin(self, value):
        """ setter """
        self.set_register_value(self._register.CCWComplianceMargin, value)

    ccw_compliance_margin = property(_get_ccw_compliance_margin,
                                      _set_ccw_compliance_margin)

    def _get_cw_compliance_margin(self):
        """getter"""
        return self._get_register_value(self._register.CWComplianceMargin)

    def _set_cw_compliance_margin(self, value):
        """ setter """
        self.set_register_value(self._register.CWComplianceMargin, value)

    cw_compliance_margin = property(_get_cw_compliance_margin,
                                     _set_cw_compliance_margin)

    def _get_ccw_compliance_slope(self):
        """getter"""
        return self._get_register_value(self._register.CCWComplianceSlope)

    def _set_ccw_compliance_slope(self, value):
        """ setter """
        self.set_register_value(self._register.CCWComplianceSlope, value)

    ccw_compliance_slope = property(_get_ccw_compliance_slope,
                                     _set_ccw_compliance_slope)

    def _get_cw_compliance_slope(self):
        """getter"""
        return self._get_register_value(self._register.CWComplianceSlope)

    def _set_cw_compliance_slope(self, value):
        """ setter """
        self.set_register_value(self._register.CWComplianceSlope, value)

    cw_compliance_slope = property(_get_cw_compliance_slope,
                                    _set_cw_compliance_slope)

    def _get_current_load(self):
        """getter"""        
        val = self._get_register_value(self._register.CurrentLoad)
        if (val & 0x400) != 0:
            return -(val & 0x3FF)
        return val

    current_load = property(_get_current_load)

    def _get_current_position(self):
        """getter"""        
        return self._get_register_value(self._register.CurrentPosition)

    current_position = property(_get_current_position)

    def _get_current_speed(self):
        """getter"""
        val =  self._get_register_value(self._register.CurrentSpeed)
        if (val & 0x400) != 0:
            return -(val & 0x3FF)
        return val

    current_speed = property(_get_current_speed)

    def _get_current_temperature(self):
        """getter"""        
        return self._get_register_value(self._register.CurrentTemperature)

    current_temperature = property(_get_current_temperature)

    def _get_current_voltage(self):
        """getter"""        
        volts = self._get_register_value(self._register.CurrentVoltage) 
        return volts / 10.0

    current_voltage = property(_get_current_voltage)

    def _get_torque_enable(self):
        """getter"""        
        return (self._get_register_value(self._register.TorqueEnable) != 0)
    
    def _set_torque_enable(self, value):
        """ setter """        
        self.set_register_value(self._register.TorqueEnable, 1 if value else 0)

    torque_enable = property(_get_torque_enable, _set_torque_enable)

    def _get_firmware_version(self):
        """getter"""        
        return self._get_register_value(self._register.FirmwareVersion)

    firmware_version = property(_get_firmware_version)

    def _get_id(self):
        """getter"""        
        return self._id
    
    def _set_id(self, value):
        """change id of the dynamixel"""
        broadcast_id = dynamixel_network.DynamixelInterface.BROADCAST_ID
        if value < 0 or value >= broadcast_id:
            raise ValueError("Id must be in range 0 to 253")
        if value == self._id:
            return
        self._dyn_net.dynamixel_id_change(self, value)
        registerLength = self.register_length(self._register.Id)
        self._dyn_net.write_register(self._id,
                                     self._register.Id,
                                     registerLength,
                                     value,
                                     False)
        self._id = value

    id = property(_get_id, _set_id)

    def _get_led(self):
        """getter"""        
        return (self._get_register_value(self._register.LED) != 0)
    
    def _set_led(self, value):
        """setter"""        
        self.set_register_value(self._register.LED, 1 if value else 0)

    led = property(_get_led, _set_led)

    def _get_lock(self):
        """getter"""        
        return (self._get_register_value(self._register.Lock) != 0)

    lock = property(_get_lock)

    def _get_temperature_limit(self):
        """getter"""        
        return self._get_register_value(self._register.TemperatureLimit)
    
    def _set_temperature_limit(self, value):
        """setter"""        
        self.set_register_value(self._register.TemperatureLimit, value)

    temperature_limit = property(_get_temperature_limit, 
                                  _set_temperature_limit)

    def _get_max_torque(self):
        """getter"""        
        return self._get_register_value(self._register.MaxTorque)
    
    def _set_max_torque(self, value):
        """setter"""        
        self.set_register_value(self._register.MaxTorque, value)

    max_torque = property(_get_max_torque, _set_max_torque)

    def _get_high_voltage_limit(self):
        """getter"""        
        return self._get_register_value(self._register.HighVoltageLimit)/10.0
    
    def _set_high_voltage_limit(self, value):
        """setter"""        
        adj_value = int(round(value* 10.0))
        self.set_register_value(self._register.HighVoltageLimit, adj_value) 

    high_voltage_limit = property(_get_high_voltage_limit, 
                                   _set_high_voltage_limit)

    def _get_low_voltage_limit(self):
        """getter"""        
        return self._get_register_value(self._register.LowVoltageLimit)/10.0
    
    def _set_low_voltage_limit(self, value):
        """setter"""        
        adj_value = int(round(value * 10.0))
        self.set_register_value(self._register.LowVoltageLimit, adj_value)

    low_voltage_limit = property(_get_low_voltage_limit, 
                                  _set_low_voltage_limit)

    def _get_model_number(self):
        """getter"""        
        return self._get_register_value(self._register.ModelNumber)

    model_number = property(_get_model_number)

    def _get_moving(self):
        """getter"""                
        is_moving = (self._get_register_value(self._register.Moving) != 0)
        return self.changed or is_moving

    moving = property(_get_moving)

    def _get_punch(self):
        """getter"""        
        return self._get_register_value(self._register.Punch)
    
    def _set_punch(self, value):
        """setter"""        
        self.set_register_value(self._register.Punch, value)

    punch = property(_get_punch, _set_punch)

    def _get_registered_instruction(self):
        """getter"""        
        result = self._get_register_value(self._register.RegisteredInstruction) 
        return result != 0
    
    def _set_registered_instruction(self, value):
        """setter"""        
        self.set_register_value(self._register.RegisteredInstruction, 
                                1 if value else 0)
    
    registered_instruction = property(_get_registered_instruction, 
                                      _set_registered_instruction)

    def _get_return_delay(self):
        """getter"""        
        return self._get_register_value(self._register.ReturnDelay) * 2
    
    def _set_return_delay(self, value):
        """setter"""        
        self.set_register_value(self._register.ReturnDelay, value / 2)
    
    return_delay = property(_get_return_delay, _set_return_delay)

    def _get_status_return_level(self):
        """getter"""        
        return self._get_register_value(self._register.StatusReturnLevel)
    
    def _set_status_return_level(self, value):
        """setter"""        
        self.set_register_value(self._register.StatusReturnLevel, value)
    
    status_return_level = property(_get_status_return_level, 
                                    _set_status_return_level)

    def _get_torque_limit(self):
        """getter"""        
        return self._get_register_value(self._register.TorqueLimit)

    def _set_torque_limit(self, value):
        """setter"""
        self.set_register_value(self._register.TorqueLimit, value)

    torque_limit = property(_get_torque_limit, _set_torque_limit)
    

    