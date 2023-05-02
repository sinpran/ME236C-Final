#!/usr/bin python3

# Your import statements here
from mpclab_common.tracks.radius_arclength_track import RadiusArclengthTrack
from mpclab_common.pytypes import VehicleState, VehiclePrediction
from mpclab_common.track import get_track
from mpclab_controllers.abstract_controller import AbstractController
import numpy as np
from numpy import load
from scipy import interpolate
# The ProjectController class will be instantiated when creating the ROS node.
class ProjectController(AbstractController):
    def __init__(self, dt: float, print_method=print):
        # The control interval is set at 10 Hz
        self.dt = dt

        # If printing to terminal, use self.print_method('some string').
        # The ROS print method will be passed in when instantiating the class
        if print_method is None:
            self.print_method = lambda s: None
        else:
            self.print_method = print_method

        # The state and input prediction object
        self.state_input_prediction = VehiclePrediction()

        # The state and input reference object
        self.state_input_reference = VehiclePrediction()

        # Load the track used in the MPC Lab. The functions for transforming between
        # Global (x, y, psi) and Frenet (s, e_y, e_psi) frames is contained in the returned
        # object. i.e. global_to_local and local_to_global
        self.track = get_track('L_track_barc')
        self.L = self.track.track_length
        self.W = self.track.track_width

        # Example for obtaining the x-y points of the track boundaries
        track_xy = self.track.get_track_xy()
        bound_in_xy = track_xy['bound_in']
        bound_out_xy = track_xy['bound_out'] 

        # Convert x-y points to frenet frame
        bound_in_sey = []
        for _x, _y in zip(bound_in_xy['x'], bound_in_xy['y']):
            _s, _ey, _, = self.track.global_to_local((_x, _y, 0))
            bound_in_sey.append([_s, _ey])

        bound_out_sey = []
        for _x, _y in zip(bound_out_xy['x'], bound_out_xy['y']):
            _s, _ey, _, = self.track.global_to_local((_x, _y, 0))
            bound_out_sey.append([_s, _ey])

    # This method will be called upon starting the control loop
    def initialize(self, vehicle_state: VehicleState):

        self.step_count=0
        self.t0 = vehicle_state.t
        self.print_method(f"starting importing")
        self.a = load('raceline.npz')
        self.print_method(f"done importing") 

        self.e_psi = self.a['e_psi']

        self.e_y = self.a['e_y']

        self.psi = self.a['psi']

        self.psidot = self.a['psidot']

        self.s =self.a['s']

        self.t = self.a['t']

        self.u_a = self.a['u_a']

        self.u_s = self.a['u_s']

        self.v_long = self.a['v_long']

        self.v_tran = self.a['v_tran']

        self.x = self.a['x']

        self.y = self.a['y']

        self.b = [self.e_psi , self.e_y,self.psi ,self.psidot,self.s ,self.t,self.u_a,self.u_s,self.v_long,self.v_tran,self.x,self.y ]

        self.b_prime = []

        old_time_points = np.linspace(0, 14, 1001, endpoint=False)

        global new_time_points
        new_time_points = np.linspace(0, 14, 1401, endpoint=False)   

        for i in range(len(self.b)):

            if new_time_points[-1] > 14 - 0.014:

                new_time_points = new_time_points[:-1]

            interpolator = interpolate.interp1d(old_time_points, self.b[i], kind='linear')

            interpolated_data = interpolator(new_time_points)

            self.b_prime.append(interpolated_data)

        global e_psi_ref
        e_psi_ref = self.b_prime[0]

        global e_y_ref    
        e_y_ref =self.b_prime[1]

        self.integralError = 0

        self.psi_ref = self.b_prime[2]

        self.psidot_ref = self.b_prime[3]

        self.s_ref = self.b_prime[4]

        self.t_ref =self.b_prime[5]

        self.u_a_ref = self.b_prime[6]

        self.u_s_ref = self.b_prime[7]

        self.v_long_ref = self.b_prime[8]

        self.v_tran_ref = self.b_prime[9]

        self.x_ref = self.b_prime[10]

        self.y_ref = self.b_prime[11]
        
        self.k=[]    

        for i in range(len(self.s)):
            self.k.append(self.track.get_curvature(self.s[i]))    

    # This method will be called once every time step, make sure to modify the vehicle_state
    # object in place with your computed control actions for acceleration (m/s^2) and steering (rad)
    def step(self, vehicle_state: VehicleState):
        
        # Modify the vehicle state object in place to pass control inputs to the ROS node
        t = vehicle_state.t - self.t0
        #self.print_method(f"step")
        #self.print_method(f"time:{self.data['t']}")
        #lst = data.files
        # for item in lst:
        #     print("\n" + item + "\n")
        #     print("\n" + data[item] + "\n")
        
        #idx = np.abs(data['t'] - t).argmin()
        #val= self.data['u_a']
        #self.print_method(f'Val: {val}')

        # Example transformation from global to Frenet frame coordinates
        s, e_y, e_psi = self.track.global_to_local((vehicle_state.x.x, vehicle_state.x.y, vehicle_state.e.psi))
        # accel = 0.3*np.sin(t/1*(2*np.pi)) + 0.3
        # steer = 0.2*np.sin(t/1*(2*np.pi))
           
        lf = 0.13
        lr = 0.13
        L = 0.37
        m = 2.2187
        k_step=self.k[self.step_count]
        C_alpha = 0.9
        k_p=-.1
        k_i = -.1
        limit = 10
        #x_LA=.1
        #beta_ss=lr*k_step-((lf*m*vehicle_state.v.v_long**2)/(2*C_alpha*L))*k_step

        #steering = k_p*((e_y_ref[self.step_count]-e_y)+x_LA*((e_psi_ref[self.step_count]-e_psi)+beta_ss))+L*k_step

        error = (e_y_ref[self.step_count]-e_y) + (e_psi_ref[self.step_count]-e_psi)
        self.integralError += error
        if self.integralError > limit:
            self.integralError = limit
        elif self.integralError < -limit:
            self.integralError = -limit

        # P Controller
        accel = -1*(vehicle_state.v.v_long - 1.0)
        steer = 2*((e_y_ref[self.step_count]-e_y) + (e_psi_ref[self.step_count]-e_psi))
        # steer = 2*(error) + k_i * self.integralError

        #steer=steering

        vehicle_state.u.u_a = accel
        vehicle_state.u.u_steer = steer
        
        # Example of printing
        self.print_method(f's: {s} | e_y: {e_y} | e_psi: {e_psi}')
        self.print_method(f'Accel: {accel} | Steering: {steer}')
        self.print_method(f'Accel: {accel} | Steering: {steer}')
        self.print_method(f'STEP COUNT: {self.step_count}')
        self.step_count=self.step_count+1
        # if(self.step_count>1401):
        #     self.step_count=0

        return

    # This method will be called once every time step. If you would like to visualize
    # the predictions made by your controller, make sure to populate the state_input_prediction
    # object
    def get_prediction(self):
        return self.state_input_prediction

    # This method will be called once every time step. If you would like to visualize
    # some user defined reference, make sure to populate the state_input_reference
    # object
    def get_reference(self):
        return self.state_input_reference
