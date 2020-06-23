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

goal = PoseStamped()
tf = TFMessage()
curr_robot_transform = Transform()
distance_map = []
input_map_scale = 0.01
input_map_width = 4000
input_map_height = 4000
grid_scale = 0.1
grid_width = input_map_width*input_map_scale/grid_scale
grid_height = input_map_height*input_map_scale/grid_scale
drawing_map = True
# publisher
pub = None


def goal_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "goal: %s", data.pose.position)
    global goal
    goal = data
    pass

def map_callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "map data: %s", data.data)
    # brush_fire(data)


    global distance_map
    global drawing_map
    distance_map = read_map_from_file_pickle()
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

def tf_callback(data):
    global tf 
    tf = data
    for transformStamped in tf.transforms:
        child_id = transformStamped.child_frame_id
        current_id = transformStamped.header.frame_id
        transform = transformStamped.transform
        translation = transform.translation
        rotation = transform.rotation
        if current_id == "odom" and child_id == "chassis":
            # print("Current:" + current_id + " child:" + child_id)
            # print("X:" + str(translation.x) + " Y:" + str(translation.y) + " Z:" + str(translation.z))
            # print(rotation)
            global curr_robot_transform
            curr_robot_transform = transform
            # current_loc = Vector3()
            # current_loc.x = translation.x
            # current_loc.y = translation.y
            # current_loc.z = 0
            # goal_loc = Vector3()
            # goal_loc.x = goal.pose.position.x
            # goal_loc.y = goal.pose.position.y
            # goal_loc.z = 0
            if not drawing_map:
                grad_vec2 = get_gradient( curr_robot_transform.translation, goal.pose.position)
                grad_vec3 = np.array([grad_vec2[0], grad_vec2[1], 0])
                q = get_quaternion(np.array([1,0,0]), grad_vec3)
                # print(q)

                grad_pose = PoseStamped()
                grad_pose.header.frame_id = "odom"
                grad_pose.pose.position.x = translation.x
                grad_pose.pose.position.y = translation.y  
                grad_pose.pose.position.z = translation.z
                grad_pose.pose.orientation = q
                pub.publish(grad_pose)
                # print("grad")
                # print(grad_vec3)
            # grad = get_attraction_grad(current_loc, goal_loc)
            # print(vec3_magnitude(grad))
    pass



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
    with open("map_pickle.txt", "rb") as fp:   # Unpickling
        map = pickle.load(fp)
    return map

def brush_fire(data):
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
    write_map_to_file_pickle(distance_map)
    write_map_to_file(distance_map)
    draw_graph(distance_map)


# the gradient here is the opposite direction of traditional gradient
def get_attraction_grad(curr_loc, goal_loc):
    max_grad_dist = 1.0
    grad_scale = 1.0
    dist_vec = goal_loc - curr_loc
    dist_to_goal = np.linalg.norm(dist_vec)
    if dist_to_goal < max_grad_dist:
        grad = dist_vec*grad_scale
    else:
        grad = dist_vec * (max_grad_dist*grad_scale/dist_to_goal)
    return grad
    
# given location in real world, tranlate into grid index
def world_to_grid(location):
    grid_loc = np.array([0, 0])
    grid_loc[0] = int(location[0]/grid_scale + (grid_width/2))
    grid_loc[1] = int(location[1]/grid_scale + (grid_height/2))
    return grid_loc

def get_distance_to_obstacle(curr_loc):
    curr_grid_loc = world_to_grid(curr_loc)
    x_grid = int(curr_grid_loc[0])
    y_grid = int(curr_grid_loc[1])
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
    cut_off_dist = 0.5
    grad_scale = 1
    # offset to map center

    dist_to_obstacle = get_distance_to_obstacle(curr_loc)
    grad_mag = grad_scale*(1/cut_off_dist - 1/dist_to_obstacle)*(1/dist_to_obstacle**2)
    # find the gradient direction
    grad = get_repulsion_grad_direction(curr_loc)
    grad *= grad_mag
    return grad

def get_gradient(curr_loc_vec3, goal_vec3):
    curr_loc = np.array([curr_loc_vec3.x, curr_loc_vec3.y])
    goal = np.array([goal_vec3.x, goal_vec3.y])
    attract_vec = get_attraction_grad(curr_loc, goal)
    repulsion_vec = get_repulsion_grad(curr_loc)
    final_grad = attract_vec + repulsion_vec
    return repulsion_vec


def draw_graph(map):
    plt.figure()
    plt.imshow(map, cmap='jet')
    plt.show()


            

def listener():
    rospy.Subscriber("move_base_simple/goal", PoseStamped, goal_callback)
    rospy.Subscriber("map", OccupancyGrid, map_callback)
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.spin()

def talker():
    global pub
    pub = rospy.Publisher('gradient', PoseStamped , queue_size=10)
    


if __name__=='__main__':
    rospy.init_node('motion_planner', anonymous=False)
    talker()
    listener()
