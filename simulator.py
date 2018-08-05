"""
This simulator can be used to generate laser reflections in dynamic environments.
Ransalu Senanayake
06-Aug-2018

Step 1: Specify the output folder and file name
Step 2: Specify the environemnt (use an existing or create your own)
Step 3: Specify the lidar parameters such as distance, angle, etc,
Step 4: Draw the robot's path by clicking on various locations on the gui (close to exit) or hard code the pose/s
Output: .csv or carmen file and images

Note: Output file type .csv: column1=time, column2=longitude, column3=latitude, column4=occupied/free
"""

import sys
import numpy as np
import matplotlib.pylab as pl
#from moviepy.editor import VideoClip #moviepy v. 0.2.2.11


class Obstacle():
    """
    Dynamic or static rectangular obstacle. It is assumed that dynamic objects are under constant acceleration.
    E.g. moving vehicle, parked vehicle, wall
    """
    def __init__(self, centroid, dx, dy, angle=0, vel=[1, 0], acc=[0, 0]):
        """
        :param centroid: centroid of the obstacle
        :param dx: length of the vehicle >=0
        :param dy: width of the vegicle >= 0
        :param angle: anti-clockwise rotation from the x-axis
        :param vel: [x-velocity, y-velocity], put [0,0] for static objects
        :param acc: [x-acceleration, y-acceleration], put [0,0] for static objects/constant velocity
        """
        self.centroid = centroid
        self.dx = dx
        self.dy = dy
        self.angle = angle
        self.vel = vel #moving up/right is positive
        self.acc = acc
        self.time = 0 #time is incremented for every self.update() call

    def __get_points(self, centroid):
        """
        :return A line: ((x1,y1,x1',y1'))
                or four line segments: ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (x4,y4,x4',y4'))
        """
        dx_cos = self.dx*np.cos(self.angle)
        dx_sin = self.dx*np.sin(self.angle)
        dy_sin = self.dy*np.sin(self.angle)
        dy_cos = self.dy*np.cos(self.angle)

        BR_x = centroid[0] + 0.5*(dx_cos + dy_sin) #BR=Bottom-right
        BR_y = centroid[1] + 0.5*(dx_sin - dy_cos)
        BL_x = centroid[0] - 0.5*(dx_cos - dy_sin)
        BL_y = centroid[1] - 0.5*(dx_sin + dy_cos)
        TL_x = centroid[0] - 0.5*(dx_cos + dy_sin)
        TL_y = centroid[1] - 0.5*(dx_sin - dy_cos)
        TR_x = centroid[0] + 0.5*(dx_cos - dy_sin)
        TR_y = centroid[1] + 0.5*(dx_sin + dy_cos)

        seg_bottom = (BL_x, BL_y, BR_x, BR_y)
        seg_left = (BL_x, BL_y, TL_x, TL_y)

        if self.dy == 0: #if no height
            return (seg_bottom,)
        elif self.dx == 0: # if no width
            return (seg_left,)
        else: #if rectangle
            seg_top = (TL_x, TL_y, TR_x, TR_y)
            seg_right = (BR_x, BR_y, TR_x, TR_y)
            return (seg_bottom, seg_top, seg_left, seg_right)

    def __get_points_old(self, centroid):
        """
        :return A line: ((x1,y1,x1',y1'))
                or four line segments: ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (x4,y4,x4',y4'))
        """
        seg_bottom = (centroid[0] - self.dx/2, centroid[1] - self.dy/2, centroid[0] + self.dx/2, centroid[1] - self.dy/2)
        seg_left = (centroid[0] - self.dx/2, centroid[1] - self.dy/2, centroid[0] - self.dx/2, centroid[1] + self.dy/2)

        if self.dy == 0: #if no height
            return (seg_bottom,)
        elif self.dx == 0: # if no width
            return (seg_left,)
        else: #if rectangle
            seg_top = (centroid[0] - self.dx/2, centroid[1] + self.dy/2, centroid[0] + self.dx/2, centroid[1] + self.dy/2)
            seg_right = (centroid[0] + self.dx/2, centroid[1] - self.dy/2, centroid[0] + self.dx/2, centroid[1] + self.dy/2)
            return (seg_bottom, seg_top, seg_left, seg_right)

    def update(self, pos=None, recycle_pos=True):
        """
        :param pos: manually give a position. If None, update based on time.
        :return: updated centroid
        """
        if pos is None:
            disp_x = self.centroid[0] + self.vel[0]*self.time + 0.5*self.acc[0]*(self.time**2) #s_x = ut + 0.5at^2
            disp_y = self.centroid[1] + self.vel[1]*self.time + 0.5*self.acc[1]*(self.time**2) #s_y = ut + 0.5at^2
        else:
            if recycle_pos is True:
                if self.time >= pos.shape[0]:
                    t = self.time%pos.shape[0]
                else:
                    t = self.time
            else: #stay at where it is when t > t_max
                if self.time > pos.shape[0]:
                    t = pos.shape[0]
                else:
                    t = self.time
            disp_x = pos[t, 0]
            disp_y = pos[t, 1]
        self.time += 1 #time is incremented for every self.update() call
        return self.__get_points(centroid=[disp_x, disp_y])

def connect_segments(segments, resolution = 0.01):
    """
    :param segments: start and end points of all segments as ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (...))
           step_size : resolution for plotting
    :return: stack of all connected line segments as (X, Y)
    """

    for i, seg_i in enumerate(segments):
        if seg_i[1] == seg_i[3]: #horizontal segment
            x = np.arange(min(seg_i[0],seg_i[2]), max(seg_i[0],seg_i[2]), resolution)
            y = seg_i[1]*np.ones(len(x))
        elif seg_i[0] == seg_i[2]: #vertical segment
            y = np.arange(min(seg_i[1],seg_i[3]), max(seg_i[1],seg_i[3]), resolution)
            x = seg_i[0]*np.ones(len(y))
        else: # gradient exists
            m = (seg_i[3] - seg_i[1])/(seg_i[2] - seg_i[0])
            c = seg_i[1] - m*seg_i[0]
            x = np.arange(min(seg_i[0],seg_i[2]), max(seg_i[0],seg_i[2]), resolution)
            y = m*x + c

        obs = np.vstack((x, y)).T
        if i == 0:
            connected_segments = obs
        else:
            connected_segments = np.vstack((connected_segments, obs))

    return connected_segments

def get_intersection(a1, a2, b1, b2) :
    """
    :param a1: (x1,y1) line segment 1 - starting position
    :param a2: (x1',y1') line segment 1 - ending position
    :param b1: (x2,y2) line segment 2 - starting position
    :param b2: (x2',y2') line segment 2 - ending position
    :return: point of intersection, if intersect; None, if do not intersect
    #adopted from https://github.com/LinguList/TreBor/blob/master/polygon.py
    """
    def perp(a) :
        b = np.empty_like(a)
        b[0] = -a[1]
        b[1] = a[0]
        return b

    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )

    intersct = np.array((num/denom.astype(float))*db + b1) #TODO: check divide by zero!

    delta = 1e-3
    condx_a = min(a1[0], a2[0])-delta <= intersct[0] and max(a1[0], a2[0])+delta >= intersct[0] #within line segment a1_x-a2_x
    condx_b = min(b1[0], b2[0])-delta <= intersct[0] and max(b1[0], b2[0])+delta >= intersct[0] #within line segment b1_x-b2_x
    condy_a = min(a1[1], a2[1])-delta <= intersct[1] and max(a1[1], a2[1])+delta >= intersct[1] #within line segment a1_y-b1_y
    condy_b = min(b1[1], b2[1])-delta <= intersct[1] and max(b1[1], b2[1])+delta >= intersct[1] #within line segment a2_y-b2_y
    if not (condx_a and condy_a and condx_b and condy_b):
        intersct = None #line segments do not intercept i.e. interception is away from from the line segments

    return intersct

def get_laser_ref(segments, realm_in_radians=np.pi, n_reflections=180, max_dist=100, xytheta_robot=np.array([0.0, 0.0])):
    """
    :param segments: start and end points of all segments as ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (...))
           realm_in_radians: sight of the robot - typically pi or 4/3*pi
           n_reflections: resolution=realm_in_radians/n_reflections
           max_dist: max distance the robot can see. If no obstacle, laser end point = max_dist
           xy_robot: robot's position in the global coordinate system
    :return: 1xn_reflections array indicating the laser end point
    """
    xy_robot = xytheta_robot[:2] #robot position
    theta_robot = xytheta_robot[2] #robot angle in rad

    angles = np.linspace(theta_robot, theta_robot+realm_in_radians, n_reflections)
    dist_theta = max_dist*np.ones(n_reflections) # set all laser reflections to 100

    for seg_i in segments:
        xy_i_start, xy_i_end = np.array(seg_i[:2]), np.array(seg_i[2:]) #starting and ending points of each segment
        for j, theta in enumerate(angles):
            xy_ij_max = xy_robot + np.array([max_dist*np.cos(theta), max_dist*np.sin(theta)]) # max possible distance
            intersection = get_intersection(xy_i_start, xy_i_end, xy_robot, xy_ij_max)

            if intersection is not None: #if the line segments intersect
                r = np.sqrt(np.sum((intersection-xy_robot)**2)) #radius

                if r < dist_theta[j]:
                    dist_theta[j] = r

    return dist_theta

def get_laser_ref_covered(segments, realm_in_radians=np.pi, n_reflections=180, max_dist=100, xytheta_robot=np.array([0.0, 0.0])):
    """
    :param segments: start and end points of all segments as ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (...))
           realm_in_radians: sight of the robot - typically pi or 4/3*pi
           n_reflections: resolution=realm_in_radians/n_reflections
           max_dist: max distance the robot can see. If no obstacle, laser end point = max_dist
           xy_robot: robot's position in the global coordinate system
    :return: 1xn_reflections array indicating the laser end point
    """

    covered_segments = segments[:2]
    print(segments)
    sys.exit()

    xy_robot = xytheta_robot[:2] #robot position
    theta_robot = xytheta_robot[2] #robot angle in rad

    angles = np.linspace(theta_robot, theta_robot+realm_in_radians, n_reflections)
    dist_theta = max_dist*np.ones(n_reflections) # set all laser reflections to 100

    for seg_i in segments:
        xy_i_start, xy_i_end = np.array(seg_i[:2]), np.array(seg_i[2:]) #starting and ending points of each segment
        for j, theta in enumerate(angles):
            xy_ij_max = xy_robot + np.array([max_dist*np.cos(theta), max_dist*np.sin(theta)]) # max possible distance
            intersection = get_intersection(xy_i_start, xy_i_end, xy_robot, xy_ij_max)

            if intersection is not None: #if the line segments intersect
                r = np.sqrt(np.sum((intersection-xy_robot)**2)) #radius

                if r < dist_theta[j]:
                    dist_theta[j] = r

    return dist_theta

def update_text_file(text_file, data, file_format='carmen'):
    """
    :param text_file: file created - open(output_file_name, 'w')
    :param data: dist_theta for carmen; (T,X,Y) numpy array for txy comma-seperated format
    :param file_format: 'carmen' or 'txy'
    :return:
    """
    if file_format == 'carmen':
        #http://www2.informatik.uni-freiburg.de/~stachnis/datasets.html
        data = ''.join(['%f ' % num for num in data])
        data = 'FLASER 180 ' + data[:-1] + '\n'#Get rid of the last comma
    elif file_format == 'txy':
        data = ''.join(['%f, %f, %f \n' % (t,x,y) for (t,x,y) in data])
    elif file_format == 'txyout':
        data = ''.join(['%f, %f, %f, %f\n' % (t, x, y, out) for (t, x, y, out) in data])
    else:
        pass
    text_file.write(data)

def load_obstacles(environment):
    if environment == 'icra_main_test':
        obs1 = Obstacle(centroid=[70, 23], dx=10, dy=40, angle=np.pi/6, vel=[0, 0], acc=[0, 0]) #a wall right
        obs2 = Obstacle(centroid=[25+20, 17], dx=3, dy=5, angle=-np.pi/2, vel=[0, 0], acc=[0, 0]) #parked right
        obs3 = Obstacle(centroid=[-65, 20], dx=15, dy=500, angle=0, vel=[0, 0], acc=[0, 0]) #a wall left
        obs4 = Obstacle(centroid=[-40, 10], dx=2, dy=5, angle=-np.pi/30, vel=[0, 0], acc=[0, 0]) #parked left
        obs5 = Obstacle(centroid=[-40, 25], dx=2, dy=5, angle=-np.pi/30, vel=[0, 0], acc=[0, 0]) #parked left up
        up_shift = 40 + 40
        right_sift = 0
        veh1 = Obstacle(centroid=[25-right_sift, 50+30], dx=3, dy=4, angle=0, vel=[0, -0.7], acc=[0, 0]) #move down
        veh2 = Obstacle(centroid=[-25+right_sift, -10+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.5], acc=[0, 0.1]) #move up accel
        veh3 = Obstacle(centroid=[-25+right_sift, -15+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.8], acc=[0, 0]) #move up no accel
        veh4 = Obstacle(centroid=[-25+right_sift, -22+up_shift], dx=3, dy=6, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh5 = Obstacle(centroid=[-25+right_sift, -30+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh6 = Obstacle(centroid=[-25+right_sift, -40+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh7 = Obstacle(centroid=[-25+right_sift, -45+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh8 = Obstacle(centroid=[-25+right_sift, -54+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh9 = Obstacle(centroid=[-25+right_sift, -70+up_shift], dx=3.5, dy=7, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh10 = Obstacle(centroid=[-25+right_sift, -82+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh11 = Obstacle(centroid=[-25+right_sift, -93+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh12 = Obstacle(centroid=[-25+right_sift, -108+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move down behind
        veh13 = Obstacle(centroid=[25-right_sift, 100+30], dx=4, dy=7, angle=0, vel=[0, -1.0], acc=[0, 0]) #move down
        veh14 = Obstacle(centroid=[-25+right_sift, -120+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh15 = Obstacle(centroid=[-25+right_sift, -130+up_shift], dx=3.5, dy=7, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh16 = Obstacle(centroid=[-25+right_sift, -145+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh17 = Obstacle(centroid=[-25+right_sift, -162+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel behind
        veh18 = Obstacle(centroid=[-25+right_sift, -180+up_shift], dx=3, dy=4, angle=0, vel=[0, 0.7], acc=[0, 0]) #move up no accel
        veh19 = Obstacle(centroid=[25-right_sift, 130+30], dx=3, dy=4, angle=0, vel=[0, -1.1], acc=[0, 0]) #move down

        all_obstacles = (obs1, obs2, obs3, obs4, obs5, veh1, veh2, veh3, veh4, veh5, veh6, veh8, veh9,\
                         veh10, veh11, veh12, veh13, veh14, veh15, veh16, veh17, veh18, veh19)
        #all_obstacles = (obs1, obs2, obs3, obs4, obs5)
        area = (-100, 100, -5, 100)
    elif environment == 'toy':
        obs1 = Obstacle(centroid=[0, 40], dx=40, dy=0, angle=np.pi/10, vel=[0, 0], acc=[0, 0]) #a wall
        obs2 = Obstacle(centroid=[-10, 10], dx=4, dy=2, angle=0, vel=[0.5, 0], acc=[0, 0]) #vehicle 1 - move right
        obs3 = Obstacle(centroid=[30, 20], dx=2, dy=5, angle=0, vel=[0, -0.25], acc=[0, 0]) #vehicle 2 - move down
        obs4 = Obstacle(centroid=[0, 35], dx=4, dy=2, angle=-np.pi/6, vel=[0, 0], acc=[0, 0]) #vehicle 3 - parked
        all_obstacles = (obs1, obs2, obs3, obs4)
    elif environment == 'dynamic1':
        obs1 = Obstacle(centroid=[-100, 200], dx=250, dy=20, angle=np.pi/6, vel=[0, 0], acc=[0, 0])  # a wall
        obs2 = Obstacle(centroid=[180, 190], dx=200, dy=40, angle=0, vel=[0, 0], acc=[0, 0])  # a wall
        obs3 = Obstacle(centroid=[200, 100], dx=60, dy=40, angle=np.pi/2, vel=[0, 0], acc=[0, 0])  # a wall
        obs4 = Obstacle(centroid=[-50, 60], dx=60, dy=40, angle=np.pi/3, vel=[0, 0], acc=[0, 0])  # a wall
        obs5 = Obstacle(centroid=[100, 50], dx=40, dy=20, angle=np.pi/6, vel=[0, 0], acc=[0, 0])  # a wall
        obs5 = Obstacle(centroid=[-210, 250], dx=40, dy=20, angle=-np.pi/6, vel=[0, 0], acc=[0, 0])  # a wall
        all_obstacles = (obs1, obs2, obs3, obs4, obs5)
        area = (-300, 300, 0, 300)
    elif environment == 'dynamic2':  # dynamic robot in a dynamic environment
        obs1 = Obstacle(centroid=[70, 23], dx=10, dy=40, angle=np.pi / 6, vel=[0, 0], acc=[0, 0])  # a wall right
        obs2 = Obstacle(centroid=[25 + 20, 17], dx=3, dy=5, angle=-np.pi / 2, vel=[0, 0], acc=[0, 0])  # parked right
        obs3 = Obstacle(centroid=[-65, 20], dx=15, dy=500, angle=0, vel=[0, 0], acc=[0, 0])  # a wall left
        obs4 = Obstacle(centroid=[-40, 10], dx=2, dy=5, angle=-np.pi / 30, vel=[0, 0], acc=[0, 0])  # parked left
        obs5 = Obstacle(centroid=[-40, 25], dx=2, dy=5, angle=-np.pi / 30, vel=[0, 0], acc=[0, 0])  # parked left up
        obs6 = Obstacle(centroid=[-25, 0], dx=15, dy=15, angle=0, vel=[0, 0], acc=[0, 0])  # vehicle 1 - move right
        all_obstacles = (obs1, obs2, obs3, obs4, obs5, obs6)
        area = (-100, 100, -5, 100)
    elif environment == 'dynamic3': #dynamic robot in a dynamic environment
        obs1 = Obstacle(centroid=[-100, 200], dx=250, dy=20, angle=np.pi/6, vel=[0, 0], acc=[0, 0])  # a wall
        obs2 = Obstacle(centroid=[180, 190], dx=200, dy=40, angle=0, vel=[0, 0], acc=[0, 0])  # a wall
        obs3 = Obstacle(centroid=[200, 100], dx=60, dy=40, angle=np.pi/2, vel=[0, 0], acc=[0, 0])  # a wall
        obs4 = Obstacle(centroid=[-50, 60], dx=60, dy=40, angle=np.pi/3, vel=[0, 0], acc=[0, 0])  # a wall
        obs5 = Obstacle(centroid=[-210, 250], dx=40, dy=20, angle=-np.pi/6, vel=[0, 0], acc=[0, 0])  # a wall
        obs6 = Obstacle(centroid=[-10, 10], dx=10, dy=10, angle=0, vel=[0, 0], acc=[0, 0])  # vehicle 1 - move right
        all_obstacles = (obs1, obs2, obs3, obs4, obs5, obs6)
        area = (-300, 300, 0, 300)
    elif environment == 'dynamic4':  # dynamic robot in a dynamic environment
        obs1 = Obstacle(centroid=[70, 23], dx=10, dy=40, angle=np.pi / 6, vel=[0, 0], acc=[0, 0])  # a wall right
        obs2 = Obstacle(centroid=[25 + 20, 17], dx=3, dy=5, angle=-np.pi / 2, vel=[0, 0], acc=[0, 0])  # parked right
        obs3 = Obstacle(centroid=[0, 60], dx=35, dy=10, angle=0, vel=[0, 0], acc=[0, 0])  # a wall top
        obs4 = Obstacle(centroid=[-40, 10], dx=2, dy=5, angle=-np.pi / 30, vel=[0, 0], acc=[0, 0])  # parked left
        obs5 = Obstacle(centroid=[-40, 25], dx=2, dy=5, angle=-np.pi / 30, vel=[0, 0], acc=[0, 0])  # parked left up
        obs6 = Obstacle(centroid=[-25, 0], dx=15, dy=15, angle=0, vel=[0, 0], acc=[0, 0])  # vehicle 1 - move right
        all_obstacles = (obs1, obs2, obs3, obs4, obs5, obs6)
        area = (-100, 100, -5, 100)
    elif environment == 'uiuc1':
        veh1 = Obstacle(centroid=[-23, 15], dx=3, dy=3, angle=0, vel=[0.5, 0], acc=[0, 0]) #move down
        all_obstacles = (veh1,)
        #all_obstacles = (obs1, obs2, obs3, obs4, obs5)
        area = (-25, 25, -5, 25)
    elif environment == 'uiuc2':
        veh1 = Obstacle(centroid=[-23, 15], dx=3, dy=3, angle=0, vel=[1.5, -0.5], acc=[0, 0]) #move down
        all_obstacles = (veh1,)
        #all_obstacles = (obs1, obs2, obs3, obs4, obs5)
        area = (-25, 25, -5, 25)
    else:
        print(environment + ' not specified!')

    return all_obstacles, area

def get_way_points_gui(environment, vehicle_poses=None):
    class mouse_events:
        def __init__(self, fig, line):
            self.path_start = False #If true, capture data
            self.fig = fig
            self.line = line
            self.xs = list(line.get_xdata())
            self.ys = list(line.get_ydata())
            self.orientation = []

        def connect(self):
            self.a = self.fig.canvas.mpl_connect('button_press_event', self.__on_press)
            self.b = self.fig.canvas.mpl_connect('motion_notify_event', self.__on_motion)

        def __on_press(self, event):
            print('You pressed', event.button, event.xdata, event.ydata)
            self.path_start = not self.path_start

        def __on_motion(self, event):
            if self.path_start is True:
                if len(self.orientation) == 0:
                    self.orientation.append(0)
                else:
                    self.orientation.append( np.pi/2 + np.arctan2( (self.ys[-1] - event.ydata), (self.xs[-1] - event.xdata) ) )
                self.xs.append(event.xdata)
                self.ys.append(event.ydata)
                self.line.set_data(self.xs, self.ys)
                self.line.figure.canvas.draw()

    # set up the environment
    all_obstacles, area = load_obstacles(environment=environment)

    # update obstacles
    all_obstacle_segments = []
    for obs_i in all_obstacles:
        all_obstacle_segments += obs_i.update()

    connected_components = connect_segments(all_obstacle_segments)

    # plot
    pl.close('all')
    fig = pl.figure(figsize=(10, 5))  # (9,5)
    ax = fig.add_subplot(111)
    ax.scatter(connected_components[:, 0], connected_components[:, 1], marker='.', c='y', edgecolor='', alpha=0.2)  # obstacles
    if vehicle_poses is not None:
        pl.plot(vehicle_poses[:, 0], vehicle_poses[:, 1], 'o--', c='m')
    pl.xlim(area[:2]); pl.ylim(area[2:])

    line, = ax.plot([], [])
    mouse = mouse_events(fig, line)
    mouse.connect()

    pl.show()

    return np.hstack( (np.array(mouse.xs)[:, None], np.array(mouse.ys)[:, None], np.array(mouse.orientation)[:,None]) )

def get_filled_txy(dist_theta, robot_pos, realm_in_radians, n_reflections, max_laser_distance, unoccupied_points_per_meter=0.1, margin=0.001):

    angles = np.linspace(robot_pos[2], robot_pos[2] + realm_in_radians, n_reflections)
    laser_data_xy = np.vstack([dist_theta * np.cos(angles), dist_theta * np.sin(angles)]).T + robot_pos[:2]

    for i, ang in enumerate(angles):
        dist = dist_theta[i]
        laser_endpoint = laser_data_xy[i,:]

        # parametric filling
        para = np.sort(np.random.random(np.int16(dist * unoccupied_points_per_meter)) * (1 - 2 * margin) + margin)[:, np.newaxis]  # TODO: Uniform[0.05, 0.95]
        points_scan_i = robot_pos[:2] + para*(laser_endpoint - robot_pos[:2])  # y = <x0, y0, z0> + para <x, y, z>; para \in [0, 1]

        if i == 0:  # first data point
            if dist >= max_laser_distance:  # there's no laser reflection
                points = points_scan_i
                labels = np.zeros((points_scan_i.shape[0], 1))
            else:  # append the arrays with laser end-point
                points = np.vstack((points_scan_i, laser_endpoint))
                labels = np.vstack((np.zeros((points_scan_i.shape[0], 1)), np.array([1])[:, np.newaxis]))
        else:
            if dist >= max_laser_distance:  # there's no laser reflection
                points = np.vstack((points, points_scan_i))
                labels = np.vstack((labels, np.zeros((points_scan_i.shape[0], 1))))
            else:  # append the arrays with laser end-point
                points = np.vstack((points, np.vstack((points_scan_i, laser_endpoint))))
                labels = np.vstack((labels, np.vstack((np.zeros((points_scan_i.shape[0], 1)), np.array([1])[:, np.newaxis]))))

    #pl.scatter(points[:,0], points[:,1], c=labels, s=10)
    #pl.axis('equal')
    #pl.show()
    #sys.exit()
    return np.hstack((points, labels))

def main():
    #Step 1: output folder and file name pre
    ofn = 'outputs/proj1/proj1'
    output_file_ext = '.csv' #other supported formats are '.txt' or  '.gfs.log' #TODO: rember to set to 'carmen' at the end of the function, if gfs.log

    #Step 2: set up the environment
    env = 'dynamic4'
    all_obstacles, area = load_obstacles(environment=env)

    #Step 3: set up the robot
    n_reflections = 180
    realm_in_radians = np.pi
    max_laser_distance = 30

    #Step 4: run the robot - click on various locations on the gui and then close the gui to exit
    robot_poses = get_way_points_gui(environment=env) # Or, hard code: robot_poses = np.array([[0.0, 0.0, -45.0], [-90, 220, 0], [30.0, 30.0, 0], [40.0, 40.0, 0]]); robot_poses[:,2] *= np.pi/180

    output_file_name = ofn + output_file_ext
    text_file = open(output_file_name, 'w')

    for t in range(robot_poses.shape[0]):
        print('time = {}...'.format(t))

        #update obstacles
        all_obstacle_segments = []
        for obs_i in all_obstacles:
            all_obstacle_segments += obs_i.update()

        #get robot pose
        robot_pos = robot_poses[t,:]

        #update laser reflections
        dist_theta = get_laser_ref(all_obstacle_segments, realm_in_radians, n_reflections, max_laser_distance, robot_pos)

        #populate with zeros in-between the robot and laser hit. the density parameter is 'unoccupied_points_per_meter'
        laser_data_xyout_filled = get_filled_txy(dist_theta, robot_pos, realm_in_radians, n_reflections, max_laser_distance, unoccupied_points_per_meter=0.1)

        #(x,y) of laser reflections
        angles = np.linspace(robot_pos[2], robot_pos[2]+realm_in_radians, n_reflections)
        laser_data_xy = np.vstack([dist_theta*np.cos(angles), dist_theta*np.sin(angles)]).T + robot_pos[:2]

        #get the environment for plotting purposes
        connected_components = connect_segments(all_obstacle_segments)

        #plot
        pl.close('all')
        fig = pl.figure(figsize=(10,5)) #(9,5)
        ax = fig.add_subplot(111)
        ax.scatter(connected_components[:,0], connected_components[:,1], marker='.', c='y', edgecolor='', alpha=0.2) #obstacles
        for i in range(n_reflections): #laser beams
            ax.plot(np.asarray([robot_pos[0], laser_data_xy[i, 0]]), np.asarray([robot_pos[1], laser_data_xy[i, 1]]), c='b', zorder=1, alpha=0.2)
            if dist_theta[i] < max_laser_distance:
                ax.scatter(laser_data_xy[i,0], laser_data_xy[i,1], marker='o', c='r', zorder=2, edgecolor='') #laser end points
        ax.scatter([0], [0], marker='*', c='k', s=20, alpha=1.0, zorder=3, edgecolor='k')  # global origin
        ax.scatter(robot_pos[0], robot_pos[1], marker=(3, 0, robot_pos[2]/np.pi*180), c='k', s=300, alpha=1.0, zorder=3, edgecolor='k')#robot's position
        ax.plot(robot_poses[:,0], robot_poses[:,1], 'k--')
        ax.set_xlim([area[0], area[1]])
        ax.set_ylim([area[2], area[3]])
        #pl.tight_layout()
        pl.savefig(ofn+'_frame_{}.png'.format(t)) #Or, pl.show()

        #update_text_file(text_file, data=dist_theta, file_format='carmen') #TODO: automate switching between various output file types
        update_text_file(text_file, data=np.hstack((t*np.ones(laser_data_xyout_filled.shape[0])[:, np.newaxis], laser_data_xyout_filled)), file_format='txyout')

    text_file.close()
    print(output_file_name + ' created!')

if __name__ == "__main__":
    main()