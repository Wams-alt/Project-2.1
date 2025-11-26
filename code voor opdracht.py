# -*- coding: utf-8 -*-
"""
Created on Mon Jul  3 14:33:49 2023

@author: Rik
"""

import wis_2_2_utilities as util
import wis_2_2_systems as systems
#import random
import numpy as np

#set timestep
timestep = 2e-3 




class PID_controller():
  def __init__(self, target=0):
    self.integral1=0
    self.integral2=0
    self.K_P1= 0
    self.K_I1= 0 
    self.K_D1= 0.985
    self.K_P2= 346.1
    self.K_I2= 0 
    self.K_D2= 99.2 
    
  def feedBack(self, observe):
    self.integral1+=observe[0]
    self.integral2+=observe[2]
    u=self.K_P1*observe[0]+\
      self.K_I1*self.integral1+\
      self.K_D1*observe[1]+\
      self.K_P2*observe[2]+\
      self.K_I2*self.integral2+\
      self.K_D2*observe[3]
    return u
  
class pp_controller():
  def __init__(self, target=0):
    self.matrix_gain=np.array([[0, 0, 0, 0]])
    
  def feedBack(self, observe):
    u= -self.matrix_gain @ observe
    return u  

class null_controller():
  def __init__(self, target=0):
    pass
    
  def feedBack(self, observe):
    u= 0
    return u 
  
class controller():
    def __init__(self, target=0):
        self.target = target
        self.Integral = 0.
        
        self.K_P = 3
        self.K_I = 25
        self.K_D = -130
        
    def feedBack(self, observe):
#update integral term
        self.Integral += observe[0]*timestep
    #calculate feedback
        u=self.K_P*observe[0]+\
            self.K_I*self.Integral+\
            self.K_D*observe[1]
        return u

def main():
    model=systems.flywheel_inverted_pendulum(second_pendulum = False, pendulum1_length= 0.6, pendulum2_length= 0.6, high_kick= 150)  
    control = PID_controller()
    simulation = util.simulation(model=model,timestep=timestep)
    simulation.setCost()
    simulation.max_duration = 600 #seconde
    simulation.GIF_toggle = False #set to false to avoid frame and GIF creation

    # EIGEN VARIABELEN VOOR LOGGING
    time_list = []
    u_list = []

    try:
        while simulation.vis.Run(): 
            if simulation.time<simulation.max_duration:
                simulation.step()
                u = control.feedBack(simulation.observe())
                simulation.control(u)
                simulation.log()
                simulation.refreshTime()
                
                # ZELF DATA BIJHOUDEN
                time_list.append(simulation.time)
                u_list.append(u)
            else:
                print('Max duration reached...')
                break
    except Exception as e:
        print(f'Simulation interrupted: {e}')
    finally:
        # DEZE CODE WORDT ALTIJD UITGEVOERD
        print('Ending visualisation...')
        simulation.vis.GetDevice().closeDevice()
        
        # === CALCULEER TOTALE KWADRATISCHE KOSTEN ===
        if len(time_list) > 1 and len(u_list) > 1:
            # Bereken kwadratische kosten (u² * dt geïntegreerd over tijd)
            quadratic_cost = 0.0
            for i in range(1, len(time_list)):
                dt = time_list[i] - time_list[i-1]
                u_squared = u_list[i] ** 2
                quadratic_cost += u_squared * dt
            
            print(f"==========================================")
            print(f"TOTALE KWADRATISCHE KOSTEN VAN INPUT: {quadratic_cost:.6f}")
            print(f"Simulatieduur: {time_list[-1]:.2f} seconden")
            print(f"Aantal datapunten: {len(time_list)}")
            print(f"==========================================")
            
            # Optioneel: Kosten over eerste 3 seconden
            if time_list[-1] >= 3.0:
                cost_first_3s = 0.0
                for i in range(1, len(time_list)):
                    if time_list[i] <= 3.0:
                        dt = time_list[i] - time_list[i-1]
                        u_squared = u_list[i] ** 2
                        cost_first_3s += u_squared * dt
                
                print(f"Kwadratische kosten eerste 3 seconden: {cost_first_3s:.6f}")
                print(f"==========================================")
        else:
            print("Geen data beschikbaar voor kostenberekening")
            print(f"time_list: {len(time_list)}, u_list: {len(u_list)}")
        
        simulation.writeData()
        
   
        
  
if __name__ == "__main__":
  main()

