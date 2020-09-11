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
import yaml
from obstacle import Obstacle
#from moviepy.editor import VideoClip #moviepy v. 0.2.2.11


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

def get_laser_ref(segments, fov=np.pi, n_reflections=180, max_dist=100, xytheta_robot=np.array([0.0, 0.0])):
    """
    :param segments: start and end points of all segments as ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (...))
           fov: sight of the robot - typically pi or 4/3*pi
           n_reflections: resolution=fov/n_reflections
           max_dist: max distance the robot can see. If no obstacle, laser end point = max_dist
           xy_robot: robot's position in the global coordinate system
    :return: 1xn_reflections array indicating the laser end point
    """
    xy_robot = xytheta_robot[:2] #robot position
    theta_robot = xytheta_robot[2] #robot angle in rad

    angles = np.linspace(theta_robot, theta_robot+fov, n_reflections)
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

def get_laser_ref_covered(segments, fov=np.pi, n_reflections=180, max_dist=100, xytheta_robot=np.array([0.0, 0.0])):
    """
    :param segments: start and end points of all segments as ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (...))
           fov: sight of the robot - typically pi or 4/3*pi
           n_reflections: resolution=fov/n_reflections
           max_dist: max distance the robot can see. If no obstacle, laser end point = max_dist
           xy_robot: robot's position in the global coordinate system
    :return: 1xn_reflections array indicating the laser end point
    """

    covered_segments = segments[:2]
    print(segments)
    sys.exit()

    xy_robot = xytheta_robot[:2] #robot position
    theta_robot = xytheta_robot[2] #robot angle in rad

    angles = np.linspace(theta_robot, theta_robot+fov, n_reflections)
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

def get_way_points_gui(environment, vehicle_poses=None):
    """
    :param environment: yaml config file
    :param vehicle_poses: vehicle poses
    :return:
    """
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
                    self.orientation.append(np.pi/2 + np.arctan2((self.ys[-1] - event.ydata), (self.xs[-1] - event.xdata)))
                self.xs.append(event.xdata)
                self.ys.append(event.ydata)
                self.line.set_data(self.xs, self.ys)
                self.line.figure.canvas.draw()

    # set up the environment
    all_obstacles, area = load_obstacles_config(environment=environment)

    # update obstacles
    all_obstacle_segments = []
    for obs_i in all_obstacles:
        all_obstacle_segments += obs_i.update()

    connected_components = connect_segments(all_obstacle_segments)

    # plot
    pl.close('all')
    fig = pl.figure()#figsize=(10, 5))  # (9,5)
    ax = fig.add_subplot(111)
    pl.title('Generate waypoints: 1) Click to start. 2) Move the mouse. \n3) Click to stop. 4) lose the gui to exit')
    ax.scatter(connected_components[:, 0], connected_components[:, 1], marker='.', c='y', edgecolor='', alpha=0.2)  # obstacles
    if vehicle_poses is not None:
        pl.plot(vehicle_poses[:, 0], vehicle_poses[:, 1], 'o--', c='m')
    pl.xlim(area[:2]); pl.ylim(area[2:])

    line, = ax.plot([], [])
    mouse = mouse_events(fig, line)
    mouse.connect()

    pl.show()

    return np.hstack((np.array(mouse.xs)[:, None], np.array(mouse.ys)[:, None], np.array(mouse.orientation)[:,None]))[1:]

def get_filled_txy(dist_theta, robot_pos, fov, n_reflections, max_laser_distance, unoccupied_points_per_meter=0.1, margin=0.1):
    """
    :param dist_theta: lidar hit distance
    :param robot_pos: robot pose
    :param fov: robot field of view
    :param n_reflections: number of lidar hits
    :param max_laser_distance: maximum lidar distance
    :param unoccupied_points_per_meter: in-fill density
    :param margin: in-fill density of free points
    :return: (points, labels) - 0 label for free points and 1 label for hits
    """

    angles = np.linspace(robot_pos[2], robot_pos[2] + fov, n_reflections)
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

def load_obstacles_config(environment):
    """
    :param environment: name of the yaml config file
    :return: all obstacles, area of the environment
    """
    with open('config/'+environment+'.yaml') as file:
        yaml_data = yaml.load(file, Loader=yaml.FullLoader)

        # load environment area parameters
        area = yaml_data['area']
        area = (area['x_min'], area['x_max'], area['y_min'], area['y_max'])

        # load static and dynamic obstacles
        obs = yaml_data['obstacles']
        all_obstacles = []
        for i in range(len(obs)):
            obs_i = Obstacle(centroid=[obs[i]['centroid_x'], obs[i]['centroid_y']], dx=obs[i]['dx'], dy=obs[i]['dy'],
                            angle=obs[i]['orientation']*np.pi/180, vel=[obs[i]['velocity_x'], obs[i]['velocity_y']],
                            acc=[obs[i]['acc_x'], obs[i]['acc_y']])
            all_obstacles.append(obs_i)
    return all_obstacles, area

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

def main(env='toy1', out_fn='toy1_setting1', out_file_type = 'txyocc', save_all_data_as_npz = True, n_reflections = 360,
         fov = 180, max_laser_distance = 12, unoccupied_points_per_meter = 0.5):
    """
    :param env: name of the yaml file inside the config folder
    :param out_fn: name of the output folder - create this folder inside the output folder
    :param out_file_type: 'txyocc' or 'carmen'
    :param save_all_data_as_npz: True or False
    :param n_reflections: number of lidar beams in the 2D plane
    :param fov: lidar field of view in degrees
    :param max_laser_distance: maximum lidar distance in meters
    :papram unoccupied_points_per_meter: density of zeros between the robot and laser hit
    """

    # Step 1: output folder and file name pre
    ofn = 'outputs/' + out_fn + '/'
    if out_file_type == 'carmen':
        output_file_ext = out_fn + '.gfs.log'
    else:
        output_file_ext = out_fn + '.csv'

    # Step 2: set up the environment
    all_obstacles, area = load_obstacles_config(environment=env)

    # Step 3: run the robot - click on various locations on the gui and then close the gui to exit
    fov = fov*np.pi/180
    robot_poses = get_way_points_gui(environment=env) # Or, hard code: robot_poses = np.array([[0.0, 0.0, -45.0], [-90, 220, 0], [30.0, 30.0, 0], [40.0, 40.0, 0]]); robot_poses[:,2] *= np.pi/180
    np.savez(ofn + out_fn + '_robot_poses.npz', robot_poses=robot_poses)

    output_file_name = ofn + output_file_ext
    text_file = open(output_file_name, 'w')

    for t in range(len(robot_poses)):
        print('time = {}...'.format(t))

        #update obstacles
        all_obstacle_segments = []
        for obs_i in all_obstacles:
            all_obstacle_segments += obs_i.update()

        # get robot pose
        robot_pos = robot_poses[t,:]

        # update laser reflections
        dist_theta = get_laser_ref(all_obstacle_segments, fov, n_reflections, max_laser_distance, robot_pos)

        # populate with zeros in-between the robot and laser hit. the density parameter is 'unoccupied_points_per_meter'
        laser_data_xyout_filled = get_filled_txy(dist_theta, robot_pos, fov, n_reflections, max_laser_distance,
                                                 unoccupied_points_per_meter)

        # (x,y) of laser reflections
        angles = np.linspace(robot_pos[2], robot_pos[2] + fov, n_reflections)
        laser_data_xy = np.vstack([dist_theta*np.cos(angles), dist_theta*np.sin(angles)]).T + robot_pos[:2]

        # get the environment for plotting purposes
        connected_components = connect_segments(all_obstacle_segments)

        # save all data for plotting purposes
        if save_all_data_as_npz is True:
            np.savez(ofn + out_fn +'_frame_{}'.format(t), t=t, area=area, connected_components=connected_components,
                     n_reflections=n_reflections, max_laser_distance=max_laser_distance, fov=fov,
                     all_robot_poses=robot_poses, laser_data_xy_at_t=laser_data_xy, dist_theta_at_t=dist_theta)

        #plot
        pl.close('all')
        fig = pl.figure() #figsize=(9,5)
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
        pl.tight_layout()
        pl.savefig(ofn + out_fn + '_frame_{}.png'.format(t)) #Or,
        # pl.show()

        if out_file_type == 'carmen':
            update_text_file(text_file, data=dist_theta, file_format='carmen')
        else:
            update_text_file(text_file, data=np.hstack((t*np.ones(laser_data_xyout_filled.shape[0])[:, np.newaxis], laser_data_xyout_filled)), file_format='txyout')

    text_file.close()
    print('Images printed in ' + ofn)
    print(ofn + out_fn + '_robot_poses.npz' + ' created!')
    print(output_file_name + ' created!')
    if save_all_data_as_npz is True:
        print('All data files saved in' + ofn)

if __name__ == "__main__":
    # file configuration
    env = 'toy1' # name of the yaml file inside the config folder
    out_fn = 'toy1_setting1'# name of the output folder - create this folder inside the output folder
    out_file_type = 'txyocc' # or 'carmen'
    save_all_data_as_npz = True # save all data for each time step for offline use - this will require memory

    # robot configuration
    n_reflections = 360 # number of lidar beams in the 2D plane
    fov = 180 # lidar field of view in degrees
    max_laser_distance = 12 # maximum lidar distance in meters

    # beam filling configuration
    unoccupied_points_per_meter = 0.5

    main(env = env, out_fn = out_fn, out_file_type = out_file_type, save_all_data_as_npz = save_all_data_as_npz,
         n_reflections = n_reflections, fov= fov, max_laser_distance = max_laser_distance,
         unoccupied_points_per_meter= unoccupied_points_per_meter)