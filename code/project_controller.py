# #!/usr/bin python3

# # Your import statements here

from mpclab_common.pytypes import VehicleState, VehiclePrediction
from mpclab_common.track import get_track
from mpclab_controllers.abstract_controller import AbstractController
import numpy as np
from numpy import load
from scipy.interpolate import interp1d

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
        self.t0 = vehicle_state.t
        self.stepvalue= 0
        #interpolation data 
        self.a = np.load('raceline_new.npz')
        self.print_method(self.a.files)
        self.e_psi1 = self.a['e_psi']
        self.e_y1 = self.a['e_y']
        self.psi = self.a['psi']
        self.psidot = self.a['psidot']
        self.s1 =self.a['s']
        self.t = self.a['t']
        self.u_a = self.a['u_a']
        self.u_s = self.a['u_s']
        self.v_long = self.a['v_long']
        self.v_tran = self.a['v_tran']
        self.x = self.a['x']
        self.y = self.a['y']
        self.b = [self.e_psi1 , self.e_y1,self.psi ,self.psidot,self.s1 ,self.t,self.u_a,self.u_s,self.v_long,self.v_tran,self.x,self.y ]
        self.b_prime = []
        old_time_points = np.linspace(0, 14, 1001, endpoint=False)
        new_time_points = np.linspace(0, 14, 1401, endpoint=False)
        for i in range(len(self.b)):
            if new_time_points[-1] > 14 - 0.014:
                new_time_points = new_time_points[:-1]
            interpolator = interp1d(old_time_points, self.b[i], kind='linear')
            interpolated_data = interpolator(new_time_points)
            self.b_prime.append(interpolated_data)
        self.e_psi1 = self.b_prime[0]
        self.e_y1 =self.b_prime[1]
        self.psi = self.b_prime[2]
        self.psidot = self.b_prime[3]
        self.s1 = self.b_prime[4]
        self.t =self.b_prime[5]
        self.u_a = self.b_prime[6]
        self.u_s = self.b_prime[7]
        self.v_long = self.b_prime[8]
        self.v_tran = self.b_prime[9]
        self.x = self.b_prime[10]
        self.y = self.b_prime  [11]

    # This method will be called once every time step, make sure to modify the vehicle_state
    # object in place with your computed control actions for acceleration (m/s^2) and steering (rad)
    def step(self, vehicle_state: VehicleState):
        
        # Modify the vehicle state object in place to pass control inputs to the ROS node
        t = vehicle_state.t - self.t0

        # data = load('project_files/raceline.npz')
        # lst = data.files
        # for item in lst:
        #     print("\n" + item + "\n")
        #     print("\n" + data[item] + "\n")
        
        # idx = np.abs(data['t'] - t).argmin()
        # val= data['u_a'][idx]

        # Example transformation from global to Frenet frame coordinates
        s, e_y, e_psi = self.track.global_to_local((vehicle_state.x.x, vehicle_state.x.y, vehicle_state.e.psi))
        # accel = 0.3*np.sin(t/1*(2*np.pi)) + 0.3
        # steer = 0.2*np.sin(t/1*(2*np.pi))

        # interp_func_ey = interp1d(t, data['e_y'])
        # interp_func_epsi = interp1d(t, data['e_psi'])

        # e_y_ref = interp_func_ey(s)
        # e_psi_ref = interp_func_epsi(s)

        # print(interp_func_ey(0.7))
        # print(interp_func_epsi(0.7))

        # P Controller
        accel = -10*(vehicle_state.v.v_long - 1.0)
        steer = -1*(e_y + e_psi)

        vehicle_state.u.u_a = accel
        vehicle_state.u.u_steer = steer
        
        # Example of printing
        self.print_method(f's: {s} | e_y: {e_y} | e_psi: {e_psi}')
        self.print_method(f'Accel: {accel} | Steering: {steer}')
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


# def initialize(self, vehicle_state: VehicleState):
#         self.t0 = vehicle_state.t
#         self.stepvalue= 0
#         #interpolation data 
#         self.a = np.load('raceline.npz')
#         self.e_psi = self.a['e_psi']
#         self.e_y = self.a['e_y']
#         self.psi = self.a['psi']
#         self.psidot = self.a['psidot']
#         self.s =self.a['s']
#         self.t = self.a['t']
#         self.u_a = self.a['u_a']
#         self.u_s = self.a['u_s']
#         self.v_long = self.a['v_long']
#         self.v_tran = self.a['v_tran']
#         self.x = self.a['x']
#         self.y = self.a['y']
#         self.b = [self.e_psi , self.e_y,psi ,self.psidot,self.s ,self.t,self.u_a,self.u_s,self.v_long,self.v_tran,self.x,self.y ]
#         self.b_prime = []
#         old_time_points = np.linspace(0, 14, 1001, endpoint=False)
#         new_time_points = np.linspace(0, 14, 1401, endpoint=False)
#         for i in range(len(self.b)):
#             if new_time_points[-1] > 14 - 0.014:
#                 new_time_points = new_time_points[:-1]
#             interpolator = interp1d(old_time_points, self.b[i], kind='linear')
#             interpolated_data = interpolator(new_time_points)
#             self.b_prime.append(interpolated_data)
#         self.e_psi = self.b_prime[0]
#         self.e_y =self.b_prime[1]
#         self.psi = self.b_prime[2]
#         self.psidot = self.b_prime[3]
#         self.s = self.b_prime[4]
#         self.t =self.b_prime[5]
#         self.u_a = self.b_prime[6]
#         self.u_s = self.b_prime[7]
#         self.v_long = self.b_prime[8]
#         self.v_tran = self.b_prime[9]
#         self.x = self.b_prime[10]
#         self.y = self.b_prime  [11]
