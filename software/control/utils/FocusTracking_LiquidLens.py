# -*- coding: utf-8 -*-

#Tuning the parameters:
    #Integrator max(resp .min): max nb of steps the motor can make in DeltaT

from collections import deque
import scipy.optimize as opt
import scipy.interpolate as interpolate
import scipy.signal as signal
import numpy as np


class FocusTracker():
    
    def __init__(self,parent=None):
        
        self.YdequeLen=50
        self.YfocusMeasure = deque(maxlen = self.YdequeLen)  #The length of the buffer must be ~fps_sampling/liquid_lens_freq
        self.YfocusPosition = deque(maxlen = self.YdequeLen) #in mm
        self.YfocusPhase = deque(maxlen = self.YdequeLen)
        
        # kept for class compatibility
        self.YmaxFM = 0
        self.gain=1
        self.maxGain=1

        self.error = 0
        
        self.ampl=0.025
        self.freq=2

        self.window_size = 25
        
        # flags
        self.TrackingCycleStarted = 0
        self.TrackingCycleFinished = 0

        # cycle counter
        self.cycleCounter = 0

        # arrays used in the tracking cycle
        self.FocusMeasure = np.empty((0,0))
        self.Position = np.empty((0,0))
        
        # kept for class compatibility
        self.FM_slope = 0 # measured decay rate of FM with distance (Delta FM/ Delta mm)
        
        freq=[0,10]
        phase_lag=[0,0]
        self.phase_lag_funct=interpolate.interp1d(freq,phase_lag)
        self.phase_lag=self.phase_lag_funct(self.freq)
    
    def update_data(self,phase):
        self.YfocusPhase.append(phase)
        self.YfocusPosition.append(self.ampl*np.sin(phase))

    def get_error(self, focusMeasure):

        self.YfocusMeasure.append(focusMeasure)
        focusMeasure_list=np.array(self.YfocusMeasure)
        focusPosition_list=np.array(self.YfocusPosition)
        focusPhase_list=np.array(self.YfocusPhase)
        isYorder = 0

        if len(focusMeasure_list) < self.window_size:
            
            # fill the buffer first
            isYorder = 0

        else:
            
            # arduino firmware ensures that phase is between 0 and 2*pi
            # set check if a new sweep cycle has started
            if focusPhase_list[-1] < focusPhase_list[-2]:
                
                self.cycleCounter = self.cycleCounter % 2
                print('tracking cycle: ' + str(self.cycleCounter)) # @@@
                
                if self.cycleCounter == 0:
                    
                    self.TrackingCycleStarted = 1
                    self.FocusMeasure = np.empty((0,0))
                    self.Position = np.empty((0,0))
                
                elif self.cycleCounter == 1:

                    self.TrackingCycleStarted = 0
                    self.TrackingCycleFinished = 1
                
                # do nothing for the remaining cycles
                        
                self.cycleCounter = self.cycleCounter + 1
            
            # in the tracking cycle
            if self.TrackingCycleStarted:
            
                # record focus measure and position
                self.FocusMeasure = np.append(self.FocusMeasure,focusMeasure_list[-1])
                self.Position = np.append(self.Position,focusPosition_list[-1])
            
            # after the tracking cycle is finished
            if self.TrackingCycleFinished:
                
                self.TrackingCycleFinished = 0
                # now it's time to calculate focus error
                idx = np.argmax(self.FocusMeasure)
                print('Max FM: {}'.format(max(self.FocusMeasure)))
                self.error = self.Position[idx]
                isYorder = 1

        print('Y-error: {}'.format(self.error))
        return -self.error, isYorder

    #@@@
    def resize_buffers(self,buffer_size):
        self.YdequeLen=buffer_size
        self.YfocusMeasure=deque(self.YfocusMeasure,maxlen = self.YdequeLen)
        self.YfocusPosition=deque(self.YfocusPosition,maxlen = self.YdequeLen)
        self.YfocusPhase=deque(self.YfocusPhase,maxlen = self.YdequeLen)
        self.YmaxFM=deque(self.YmaxFM,maxlen = self.YdequeLen)
        
    def set_ampl(self,ampl):
        self.ampl=ampl
            
    def set_freq(self,freq):
        # print("Frequency: {}".format(freq))
        self.freq=freq
        self.phase_lag=self.phase_lag_funct(self.freq)
        
    def set_maxGain(self,gain):
        self.maxGain=gain

    def initialise_ytracking(self):
        self.YfocusMeasure = deque(maxlen = self.YdequeLen)
        self.YfocusPosition = deque(maxlen = self.YdequeLen) #in mm
        self.YfocusPhase = deque(maxlen = self.YdequeLen)
        self.TrackingCycleStarted = 0 # @@@ rename
        self.TrackingCycleFinished = 0 # @@@ rename
        self.cycleCounter = 0





