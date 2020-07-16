#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Vector3, Quaternion
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
import math
import copy
import numpy as np
import pickle
import tf
import cProfile
import re


goal = PoseStamped()
tf_msg = TFMessage()
curr_robot_transform = Transform()
distance_map = []
input_map_scale = 0.01
input_map_width = 4000
input_map_height = 4000
# scale for dist map
grid_scale = 0.02
grid_width = input_map_width*input_map_scale/grid_scale
grid_height = input_map_height*input_map_scale/grid_scale
drawing_map = True
# publisher
pub = None
dist_map_pub = None
# repulsive gradient
cut_off_dist = 0.6
grad_scale = 1.5
# attractive gradient
max_grad_dist = 1.0
att_grad_scale = 1.0


def goal_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "goal: %s", data.pose.position)
    global goal
    goal = data
    pass

def map_callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "map data: %s", data.data)
    bush_fire(data)
    global distance_map
    global drawing_map
    # distance_map = read_map_from_file_pickle()
    drawing_map = False
    pass

def get_quaternion(v1, v2):
    q = Quaternion()
    theta = np.arccos(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
    if np.cross(v1, v2)[2] > 0:
        rot_vec = np.array([0, 0, 1])
    else:
        rot_vec = np.array([0, 0, -1])
    q_vec = rot_vec*np.sin(theta/2)
    q.x, q.y, q.z = q_vec[0], q_vec[1], q_vec[2]
    q.w = np.cos(theta/2)
    return q


def update_tf(data):
    global tf_msg 
    tf_msg = data
    if not drawing_map:
        grad_vec2 = get_gradient( tf_msg, goal.pose.position)
        grad_vec3 = np.array([grad_vec2[0], grad_vec2[1], 0])
        q = get_quaternion(np.array([1,0,0]), grad_vec3)
        # print(q)

        grad_pose = PoseStamped()
        grad_pose.header.frame_id = "odom"
        grad_pose.pose.position.x = tf_msg[0]
        grad_pose.pose.position.y = tf_msg[1]  
        grad_pose.pose.position.z = tf_msg[2]
        grad_pose.pose.orientation = q
        pub.publish(grad_pose)
        # publish_distance_map(distance_map)



def find_list_of_dist(dist, dist_map):
    dis_list = []
    for row in range(len(dist_map)):
        for col in range(len(dist_map[row])):
            if dist_map[row][col] == dist:
                dis_list.append((row,col))
    return dis_list

def write_map_to_file(occupancy_map):
    f = open('./map.txt','w')
    width = np.sqrt(len(occupancy_map))
    print("start writing")
    count = 0
    for row in occupancy_map:
        for elem in row:
            f.write(str(elem)+" ")
            count += 1
        f.write('\n')

    f.close()
    print("Done")

def write_map_to_file_pickle(map):
    print("pickel write")
    with open("map_pickle.txt", "wb") as fp:   #Pickling
        pickle.dump(map, fp)
    print("Done")

def read_map_from_file_pickle():
    print('read pickel')
    with open("map_pickle.txt", "rb") as fp:   # Unpickling
        map = pickle.load(fp)
    print('done')
    return map

def bush_fire(data):
    print("Start bush fire")
    global drawing_map
    drawing_map = True
    step_size = int(grid_scale/input_map_scale)
    input_map = np.array(data.data)
    input_map = np.reshape(input_map, (input_map_width, input_map_height))

    global distance_map
    for row in range(0, len(input_map), step_size):
        row_list = []
        for col in range(0, len(input_map[0]), step_size):
            elem = input_map[row][col]
            if elem == 100:
                elem = 1
            if elem == -1:
                elem = 0
            row_list.append(int(elem))
        distance_map.append(row_list)

    # propagation
    current_list = find_list_of_dist(1, distance_map)
    current_dist = 1
    print("calculating")
    while len(current_list) > 0:
        next_list = []
        for elem in current_list:
            curr_row = elem[0]
            curr_col = elem[1]
            try:
                # left
                if curr_row - 1 >= 0:
                    if distance_map[curr_row-1][curr_col] == 0:
                        distance_map[curr_row-1][curr_col] = current_dist + 1
                        next_list.append((curr_row-1,curr_col))
                if curr_row+1 < grid_height:
                    if distance_map[curr_row+1][curr_col] == 0:
                        distance_map[curr_row+1][curr_col] = current_dist + 1
                        next_list.append((curr_row+1,curr_col))
                if curr_col-1 >= 0:
                    if distance_map[curr_row][curr_col-1] == 0:
                        distance_map[curr_row][curr_col-1] = current_dist + 1
                        next_list.append((curr_row,curr_col-1))
                if curr_col+1 < grid_width:
                    if distance_map[curr_row][curr_col+1] == 0:
                        distance_map[curr_row][curr_col+1] = current_dist + 1
                        next_list.append((curr_row,curr_col+1))
            except:
                print("exception")
        current_list = next_list   
        current_dist +=1

    print("done")
    drawing_map = False
    # write_map_to_file_pickle(distance_map)
    # write_map_to_file(distance_map)
    # draw_graph(distance_map)


# the gradient here is the opposite direction of traditional gradient
def get_attraction_grad(curr_loc, goal_loc):
    global max_grad_dist
    global att_grad_scale
    dist_vec = goal_loc - curr_loc
    dist_to_goal = np.linalg.norm(dist_vec)
    if dist_to_goal < max_grad_dist:
        grad = dist_vec*att_grad_scale
    else:
        grad = dist_vec * (max_grad_dist*att_grad_scale/dist_to_goal)
    return grad
    
# given location in real world, tranlate into grid index
def world_to_grid(location):
    grid_loc = np.array([0, 0])
    grid_loc[0] = int(location[0]/grid_scale + (grid_width/2))
    grid_loc[1] = int(location[1]/grid_scale + (grid_height/2))
    return grid_loc

def get_distance_to_obstacle(curr_loc):
    curr_grid_loc = world_to_grid(curr_loc)
    # x and y are flip in the map in comparison to robot x y
    x_grid = int(curr_grid_loc[1])
    y_grid = int(curr_grid_loc[0])
    distance = distance_map[x_grid][y_grid]*grid_scale
    return distance

def get_repulsion_grad_direction(curr_loc):
    curr_loc = world_to_grid(curr_loc)
    x_grid = int(curr_loc[0])
    y_grid = int(curr_loc[1])
    offsets = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    vec_list = []
    curr_dist = distance_map[x_grid][y_grid]
    for offset in offsets:
        x_offset = int(x_grid + int(offset[0]))
        y_offset = int(y_grid + int(offset[1]))
        if x_offset >= 0 and x_offset < grid_width and y_offset >= 0 and y_offset < grid_height:
            vec = np.array([x_offset, y_offset], dtype=float)
            vec = vec / np.linalg.norm(vec)
            vec *= distance_map[x_offset][y_offset] - curr_dist
            vec_list.append(vec)
    grad_direction = np.array([0.0,0.0], dtype=float)
    for vec in vec_list:
        grad_direction += vec
    grad_direction /= np.linalg.norm(grad_direction)
    return grad_direction
            
    

def get_repulsion_grad(curr_loc):
    global cut_off_dist
    global grad_scale
    # offset to map center

    dist_to_obstacle = get_distance_to_obstacle(curr_loc)
    grad_mag = grad_scale*(1/cut_off_dist - 1/dist_to_obstacle)*(1/dist_to_obstacle**2)
    # find the gradient direction
    grad = get_repulsion_grad_direction(curr_loc)
    grad *= grad_mag
    # print("x: " + str(curr_loc[0])+" y: "+str(curr_loc[1]))
    # print("dist:" + str(dist_to_obstacle))
    # print("grad mag: " + str(grad_mag))
    return grad

def get_gradient(curr_loc_vec3, goal_vec3):
    curr_loc = np.array([curr_loc_vec3[0], curr_loc_vec3[1]])
    goal = np.array([goal_vec3.x, goal_vec3.y])
    attract_vec = get_attraction_grad(curr_loc, goal)
    repulsion_vec = get_repulsion_grad(curr_loc)
    final_grad = attract_vec + repulsion_vec
    return final_grad

def flatten_list(list):
    flat_list = []
    for sublist in list:
        for item in sublist:
            flat_list.append(item)
    ret_val = np.array(flat_list, dtype='int8')
    return ret_val

def flip_x(map):
    map.reverse()

def flip_y(map):
    for row in range(len(map)):
        map[row].reverse()
        

# publish dist map for debugging
def publish_distance_map(map):
    map_copy = copy.deepcopy(map)
    # flip_x(map_copy)
    # flip_y(map_copy)
    for row in range(len(map_copy)):
        for col in range(len(map_copy[row])):
            if map_copy[row][col] == 1:
                map_copy[row][col] = 100
            if map_copy[row][col] < 0:
                map_copy[row][col] = 0
    occ_map = OccupancyGrid()
    occ_map.header.frame_id = 'odom'
    occ_map.info.resolution = grid_scale
    occ_map.info.width = int(input_map_width/int(grid_scale/input_map_scale))
    occ_map.info.height = int(input_map_height/int(grid_scale/input_map_scale))
    occ_map.info.origin.position.x = -20
    occ_map.info.origin.position.y = -20
    occ_map.data = flatten_list(map_copy)
    dist_map_pub.publish(occ_map)


def draw_graph(map):
    temp_map = copy.deepcopy(map)
    for row in range(len(temp_map)):
        for col in range(len(temp_map[row])):
            if temp_map[row][col]  > 25:
                temp_map[row][col] = 25
    plt.figure()
    plt.imshow(temp_map, cmap='jet')
    plt.show()

def transform_listener():
    tf_lisener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = tf_lisener.lookupTransform('odom', 'chassis', rospy.Time(0))
            update_tf(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            

def listener():
    rospy.Subscriber("move_base_simple/goal", PoseStamped, goal_callback)
    rospy.Subscriber("map", OccupancyGrid, map_callback)
    # rospy.Subscriber("/tf", TFMessage, tf_callback)
    transform_listener()
    rospy.spin()

def talker():
    global pub
    pub = rospy.Publisher('gradient', PoseStamped , queue_size=10)
    global dist_map_pub
    dist_map_pub = rospy.Publisher('dist_map', OccupancyGrid, queue_size=10)


if __name__=='__main__':
    print("starting motion planner")
    rospy.init_node('motion_planner', anonymous=False)
    talker()
    listener()
