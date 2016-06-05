"""
This simulator can be used to generate laser reflections in a dynamic environment.
Ransalu Senanayake
06-June-2016
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

    def update(self):
        """
        :return: updated centroid
        """
        disp_x = self.vel[0]*self.time + 0.5*self.acc[0]*(self.time**2) #s_x = ut + 0.5at^2
        disp_y = self.vel[1]*self.time + 0.5*self.acc[1]*(self.time**2) #s_y = ut + 0.5at^2
        self.time += 1 #time is incremented for every self.update() call
        return self.__get_points(centroid=[self.centroid[0] + disp_x, self.centroid[1] + disp_y])


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


def get_laser_ref(segments, realm_in_radians=np.pi, n_reflections=180, max_dist=100, xy_robot=np.array([0.0, 0.0])):
    """
    :param segments: start and end points of all segments as ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (...))
           realm_in_radians: sight of the robot - typically pi or 4/3*pi
           n_reflections: resolution=realm_in_radians/n_reflections
           max_dist: max distance the robot can see. If no obstacle, laser end point = max_dist
           xy_robot: robot's position in the global coordinate system
    :return: 1xn_reflections array indicating the laser end point
    """

    angles = np.linspace(0, realm_in_radians, n_reflections)
    dist_theta = max_dist*np.ones(n_reflections) # set all laser reflections to 100

    for seg_i in segments:
        xy_i_start, xy_i_end = np.array(seg_i[:2]), np.array(seg_i[2:]) #starting and ending points of each segment
        for j, theta in enumerate(angles):
            x_pos = max_dist*np.cos(theta)
            y_pos = max_dist*np.sin(theta)
            xy_ij_max = np.array([x_pos, y_pos]) # max possible distance

            intersection = get_intersection(xy_i_start, xy_i_end, xy_robot, xy_ij_max)
            #TODO: when the robot is moving
            if intersection is not None: #if the line segments intersect
                r = np.sqrt( intersection[0]**2 + intersection[1]**2 )

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
    else:
        pass
    text_file.write(data)


def main():
    #set up the robot
    n_reflections = 180
    realm_in_radians = np.pi
    max_laser_distance = 50
    robot_pos = np.array([0.0, 0.0])

    #set up the environment
    obs1 = Obstacle(centroid=[0, 40], dx=40, dy=0, angle=np.pi/10, vel=[0, 0], acc=[0, 0]) #a wall
    obs2 = Obstacle(centroid=[-10, 10], dx=4, dy=2, angle=0, vel=[2, 0], acc=[0, 0]) #vehicle 1 - move right
    obs3 = Obstacle(centroid=[30, 20], dx=2, dy=5, angle=0, vel=[0, -1], acc=[0, 0]) #vehicle 2 - move down
    obs4 = Obstacle(centroid=[0, 35], dx=4, dy=2, angle=-np.pi/6, vel=[0, 0], acc=[0, 0]) #vehicle 3 - parked
    all_obstacles = (obs1, obs2, obs3, obs4)

    output_file_name = 'simulator_out.txt'
    text_file = open(output_file_name, 'w')
    for t in range(25):
        print('time = {}...'.format(t))

        #update obstacles
        all_obstacle_segments = []
        for obs_i in all_obstacles:
            all_obstacle_segments += obs_i.update()

        #update laser reflections
        dist_theta = get_laser_ref(all_obstacle_segments, realm_in_radians, n_reflections, max_laser_distance, robot_pos)

        #(x,y) of laser reflections
        angles = np.linspace(0, realm_in_radians, n_reflections)
        laser_data_xy = np.vstack([dist_theta*np.cos(angles), dist_theta*np.sin(angles)]).T

        #get the environment for plotting purposes
        connected_components = connect_segments(all_obstacle_segments)

        #plot
        pl.clf()
        pl.scatter(connected_components[:,0], connected_components[:,1], marker='.', c='y', edgecolor='', alpha=0.2) #obstacles
        pl.scatter(laser_data_xy[:,0], laser_data_xy[:,1], marker='o', c='b', edgecolor='') #laser end points
        for i in range(n_reflections): #laser beams
            pl.plot(np.asarray([robot_pos[0], laser_data_xy[i, 0]]), np.asarray([robot_pos[1], laser_data_xy[i, 1]]), c='r', alpha=0.2)
        pl.scatter(robot_pos[0], robot_pos[1], marker='o', c='k', s=100, edgecolor='')#robot's position
        pl.grid()
        pl.axis('equal') #;pl.xlim([-40, 40]); pl.ylim([-10, 50])
        #pl.show()
        pl.savefig('frame_{}.png'.format(t))

        update_text_file(text_file, data=dist_theta, file_format='carmen')
        #update_text_file(text_file, data=np.hstack((t*np.ones(laser_data_xy.shape[0])[:, np.newaxis], laser_data_xy)), file_format='txy')

    text_file.close()
    print(output_file_name + ' created!')


def make_a_video():
    #replace with main()
    #Make a video with moviepy

    #set up the robot
    n_reflections = 180
    realm_in_radians = np.pi
    max_laser_distance = 50
    robot_pos = np.array([0.0, 0.0])

    #set up the environment
    obs1 = Obstacle(centroid=[0, 40], dx=40, dy=0, angle=np.pi/10, vel=[0, 0], acc=[0, 0]) #a wall
    obs2 = Obstacle(centroid=[-10, 10], dx=4, dy=2, angle=0, vel=[0.5, 0], acc=[0, 0]) #vehicle 1 - move right
    obs3 = Obstacle(centroid=[30, 20], dx=2, dy=5, angle=0, vel=[0, -0.25], acc=[0, 0]) #vehicle 2 - move down
    obs4 = Obstacle(centroid=[0, 35], dx=4, dy=2, angle=-np.pi/6, vel=[0, 0], acc=[0, 0]) #vehicle 3 - parked
    all_obstacles = (obs1, obs2, obs3, obs4)

    def mplfig_to_npimage_newer_ver(fig):
        """ Converts a matplotlib figure to a RGB frame after updating the canvas"""
        #  only the Agg backend now supports the tostring_rgb function
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        canvas = FigureCanvasAgg(fig)
        canvas.draw() # update/draw the elements

        # get the width and the height to resize the matrix
        l,b,w,h = canvas.figure.bbox.bounds
        w, h = int(w), int(h)

        #  exports the canvas to a string buffer and then to a numpy nd.array
        buf = canvas.tostring_rgb()
        image= np.fromstring(buf,dtype=np.uint8)
        return image.reshape(h,w,3)

    fig, ax = pl.subplots(1)
    def make_frame(t):
        print(t)
        #update obstacles
        all_obstacle_segments = []
        for obs_i in all_obstacles:
            all_obstacle_segments += obs_i.update()

        #update laser reflections
        dist_theta = get_laser_ref(all_obstacle_segments, realm_in_radians, n_reflections, max_laser_distance, robot_pos)

        #(x,y) of laser reflections
        angles = np.linspace(0, realm_in_radians, n_reflections)
        laser_data_xy = np.vstack([dist_theta*np.cos(angles), dist_theta*np.sin(angles)]).T

        #get the environment for plotting purposes
        connected_components = connect_segments(all_obstacle_segments)

        #plot
        ax.clear()
        pl.scatter(connected_components[:,0], connected_components[:,1], marker='.', c='y', edgecolor='', alpha=0.2) #obstacles
        pl.scatter(laser_data_xy[:,0], laser_data_xy[:,1], marker='o', c='b', edgecolor='') #laser end points
        for i in range(n_reflections): #laser beams
            pl.plot(np.asarray([robot_pos[0], laser_data_xy[i, 0]]), np.asarray([robot_pos[1], laser_data_xy[i, 1]]), c='r', alpha=0.2)
        pl.scatter(robot_pos[0], robot_pos[1], marker='o', c='k', s=100, edgecolor='')#robot's position
        pl.grid()
        pl.axis('equal') #;pl.xlim([-40, 40]); pl.ylim([-10, 50])
        ax.set_title('time={} s'.format(np.round(t,2)), fontsize=16)
        return mplfig_to_npimage_newer_ver(fig)

    animation = VideoClip(make_frame, duration = 10)
    animation.write_gif('output_robot_moving.gif', fps=5)


if __name__ == "__main__":
    main()