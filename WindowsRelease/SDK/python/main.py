#!/bin/bash
import sys
import numpy as np
import logging
import math

class Config:
    def __init__(self) -> None:
        
        # Kinematic constraints
        self.max_a = 50  # Max longitudinal acceleration
        self.max_d_w = 50  # Max change in Yaw rate
        self.max_v = 6  # Max linear velocity
        self.min_v = 0  # Min linear velocity
        self.max_w = np.pi  # Max angular velocity
        self.min_w = -np.pi # Min angular velocity
        
        # Discretization
        self.dt = 0.02
        self.v_res = 0.2  # Linear velocity resolution
        self.w_res = np.pi/9  # Angular velocity resolution

        # Robot
        self.chassis_radius = 0.53

        # Dynamic window
        self.dw_time = 0.04
        self.heading_gain = 2.0
        self.velocity_gain = 1.0
        self.distance_gain = 2.0
        # self.distance_gain = 1.5

        self.obstacles = np.array([
            # [1.6, 2.3],
            # [3.0, 3.0],
            # [2.0, 7.0],
            # [3.0, 5.5],
            # [6.0, 4.2],
            # [6.0, 8.3],
            # [7.0, 1.5],
            # [8.0, 6.0],
        ])

class DWA:
    # String literals 
    def __init__(self)-> None:
        self.config_params = Config()
        pass

    # I should add the type of the incoming arguments 
    def update_motion(self, x: np.array, u: np.array)->list:
        # Yaw rate update
        x[2] += u[1] * self.config_params.dt

        # X, Y positions update
        x[0] += u[0] * np.cos(x[2]) * self.config_params.dt
        x[1] += u[0] * np.sin(x[2]) * self.config_params.dt
        
        # Linear and Angular velocities update
        x[3] = u[0]
        x[4] = u[1]

        # Return the updated state
        return x

    # I should add the type of the incoming arguments 
    def calculate_dw(self, x: np.array)->list:
        dw = [
            max(x[3] - self.config_params.max_a * self.config_params.dt, self.config_params.min_v), # Min lin. vel 
            min(x[3] + self.config_params.max_a * self.config_params.dt, self.config_params.max_v), # Max lin. vel
            max(x[4] - self.config_params.max_d_w * self.config_params.dt, self.config_params.min_w), # Min ang. vel
            min(x[4] + self.config_params.max_d_w * self.config_params.dt, self.config_params.max_w) # Max ang. vel
        ]

        # Dynamic window -> [V_min, V_max, W_min, W_max]  
        return dw

    def calculate_traj(self, x: np.array, v: float, w: float)->np.array:
        # Temporal vector state
        x_tmp = np.array(x)
        # Array for saving the trajectory
        predicted_traj = np.array(x_tmp)
        # Control input
        u = np.array([v, w])
        time = 0.0

        # Prediction window (Lower than 3.0 seconds)
        while(time <= self.config_params.dw_time):
            # Update the motion and save the value for the trajectory
            x_tmp = self.update_motion(x_tmp, u)
            # Append the predicted state to the trajectory
            predicted_traj = np.vstack((predicted_traj, x_tmp))
            time += self.config_params.dt

        # Predicted trajectory array containing the trajectory as -> [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        # for each time step in the next three seconds.
        return predicted_traj
        
    def target_heading(self, trajectory: np.array, goal: np.array)->float:
        pos_x = trajectory[-1, 0]  # Extract the last X position in the trajectory 
        pos_y = trajectory[-1, 1]  # Extract the last Y position in the trajectory

        diff_x = goal[0] - pos_x
        diff_y = goal[1] - pos_y

        heading = math.atan2(diff_y, diff_x)

        # Calculate the difference or error between the trajectory angle and the the current yaw
        error = heading - trajectory[-1, 2]
        return abs(math.atan2(np.sin(error), np.cos(error)))

    def obstacle_distance(self, trajectory: np.array)->float:

        obst_x = self.config_params.obstacles[:, 0]
        obst_y = self.config_params.obstacles[:, 1]

        # Calculate the difference between the trajectory and obstacles
        diff_x = trajectory[:, 0] - obst_x[:, None] # Broadcasting
        diff_y = trajectory[:, 1] - obst_y[:, None] # Broadcasting
        
        # Calculate the euclidean distance between trajectory and obstacles
        dist = np.array(np.hypot(diff_x, diff_y))

        # Some point in the whole array finds an obstacle
        if np.any(dist <= self.config_params.chassis_radius * 0.5):
            # Hit ane obstacle, hence discard this trajectory
            return float("inf")

        # We can either return the cost or the distance
        min_value = np.min(dist)

        # The closer we are the greater this number as the goal is to maximize
        # the cost function.
        return 1 / min_value

    def calculate_ctrl_traj(self, x: np.array, goal: np.array):

        # Calculate the admissible velocities (DW) - [V_min, V_max, W_min, W_max] 
        dw_v = self.calculate_dw(x)

        # Array of velocities (Initial, Final, Resolution)
        v_vect = np.arange(dw_v[0], dw_v[1], self.config_params.v_res)
        w_vect = np.arange(dw_v[2], dw_v[3], self.config_params.w_res)

        minimum_cost = float("inf")

        optimal_u = [0.0, 0.0]
        optimal_traj = x

        # Iterate through the linear velocties
        for v in v_vect:
            # Iterate through the angular velocties
            for w in w_vect:
                # Trajectory generation for the following three seconds
                trajectory = self.calculate_traj(x, v, w)

                # Obstacle distance cost
                clearance_cost = self.config_params.distance_gain * self.obstacle_distance(trajectory) 
                # Heading angle cost
                heading_cost = self.config_params.heading_gain * self.target_heading(trajectory, goal)
    
                # Velocity cost. Difference between max velocity and final trajectory velocity
                velocity_cost = self.config_params.velocity_gain * abs(self.config_params.max_v - trajectory[-1, 3])

                total_cost = clearance_cost + heading_cost + velocity_cost

                if (minimum_cost >= total_cost):
                    # Set the new optimal cost
                    minimum_cost = total_cost
                    # Set the optimal control input
                    optimal_u = [v, w]
                    # Set the optimal trajectory
                    optimal_traj = trajectory
                    
                    # Both veloties are too small, we run into troublew if it happens

        return optimal_u, optimal_traj
    
def read_util_ok():
    while input() != "OK":
        pass

def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()

def getPriority(product_type):
    return  round(product_type / 3) # 1,2,3 / 3 -> 1; 4,5,6/3->2; 7/3->3

# 找到能卖product_type最近的工作台
def find_nearest_bench_to_sell(product_type, buy_bench_axis):
    dist = float('inf')
    result = None
    for sell_bench_type in where_to_sell[product_type]:
        for sell_bench_seq in infor_working_benchs_dict[sell_bench_type]:
            sell_bench = infor_working_benchs_list[sell_bench_seq]
            if((not sell_benchs_in_quuee[sell_bench_seq][product_type]) and sell_bench["material_status"]&(1<<product_type) == 0):
                c_dist = np.linalg.norm(np.array([sell_bench["coordination_x"], sell_bench["coordination_y"]]) - buy_bench_axis)
                if(c_dist < dist):
                    dist = c_dist
                    result = {
                        "sell_bench_seq" : sell_bench_seq, 
                        "coordination_x" : sell_bench["coordination_x"], 
                        "coordination_y" : sell_bench["coordination_y"]
                        }
    return result

# 找到能卖product_type用来生产高级物品的最近的工作台 优先把已经有原材料的工作台填满
def find_produceP_bench_to_sell(product_type, buy_bench_axis):
    dist = float('inf')
    result = None

    for sell_bench_type in where_to_sell[product_type]:
        sell_bench_logical_dist = 0 # 如果去9卖东西，那么将其置为50，这样就能让机器人尽量去4，5，6，7卖东西生产高级物品了
        for sell_bench_seq in infor_working_benchs_dict[sell_bench_type]:
            sell_bench = infor_working_benchs_list[sell_bench_seq]
            # if(frame_id == 1): logging.error("frame_id: " + str(frame_id) + " sell seq " + str(sell_bench_seq) + " 卖产品类型 " +str(product_type) + " 判断条件：是否材料已经在卖队列中，有无原材料: " + str(sell_benchs_in_quuee[sell_bench_seq][product_type]) + str(sell_bench["material_status"]&(1<<product_type)))
            if((not sell_benchs_in_quuee[sell_bench_seq][product_type]) and (sell_bench["material_status"]&(1<<product_type) == 0 )):
                if(sell_bench_type == 9):
                    sell_bench_logical_dist = 50
                # 保证先填满已经有了原材料的工作台
                count_have_materials = sell_benchs_in_quuee[sell_bench_seq].count(True)
                c_dist = np.linalg.norm(np.array([sell_bench["coordination_x"], sell_bench["coordination_y"]]) - buy_bench_axis) + sell_bench_logical_dist - count_have_materials*50
                # logging.error("frame_id: " + str(frame_id) + " c_dist to sell seq " + str(sell_bench_seq) + " is: " + str(c_dist))
                if(c_dist < dist):
                    dist = c_dist
                    result = {
                        "sell_bench_seq" : sell_bench_seq, 
                        "coordination_x" : sell_bench["coordination_x"], 
                        "coordination_y" : sell_bench["coordination_y"]
                    }
    return result

def update_queue(workbenchs):
    for b_i in range(workbenchs):
        line = sys.stdin.readline().split(" ") #这k行数据是工作台数据
        bench_type = int(line[0])
        coordination_x = float(line[1])
        coordination_y = float(line[2])
        infor_working_benchs_list[b_i] = {
            "bench_seq" : b_i,
            "bench_type" : bench_type, 
            "left_production_frames" : int(line[3]), 
            "coordination_x" : coordination_x, 
            "coordination_y" : coordination_y, 
            "material_status" : int(line[4]), 
            "product_status" : int(line[5])
        }
        if(frame_id == 1): 
            infor_working_benchs_dict[bench_type].append(b_i)
            
    # # 任务队列生成，尽量将最近的买卖关系连在一起 (试试尽量将最能生产出高级物品的买卖关系连在一起) 优先把已经有原材料的工作台填满
    for bench_type in range(1, 8): # 不买7 效果似乎好很多
        for to_buy_bench_seq in infor_working_benchs_dict[bench_type]:
            to_buy_bench = infor_working_benchs_list[to_buy_bench_seq]
            if((not buy_benchs_in_quuee[to_buy_bench_seq]) and (to_buy_bench["product_status"] == 1 or to_buy_bench["left_production_frames"] > 0)):
            # if((not buy_benchs_in_quuee[to_buy_bench_seq]) and to_buy_bench["product_status"] == 1):
                buy_bench_axis = np.array([to_buy_bench["coordination_x"], to_buy_bench["coordination_y"]])
                # res_bench = find_nearest_bench_to_sell(bench_type, buy_bench_axis)
                
                res_bench = find_produceP_bench_to_sell(bench_type, buy_bench_axis)
                # if(frame_id == 1) :logging.error("frame_id: " + str(frame_id) + " try to generate task: " + str(to_buy_bench_seq) + " to " + str(res_bench))
                if(res_bench):
                            
                    buy_benchs_in_quuee[to_buy_bench_seq] = True
                    sell_benchs_in_quuee[res_bench["sell_bench_seq"]][int(bench_type)] = True

                    task_queue.append({
                        "priority" : getPriority(bench_type),
                        "predict_frame_id_product_done" : to_buy_bench["left_production_frames"] + frame_id,
                        "buy_bench_seq": to_buy_bench_seq, 
                        "sell_bench_seq" : res_bench["sell_bench_seq"],
                        "product_type": bench_type, 
                        "buy_coordination_x": to_buy_bench["coordination_x"], 
                        "buy_coordination_y": to_buy_bench["coordination_y"],
                        "sell_coordination_x": res_bench["coordination_x"], 
                        "sell_coordination_y": res_bench["coordination_y"],
                    })
    # task_queue.sort(key=lambda a : a["priority"])
    # logging.error("frame id: " + str(frame_id) + " task_queue length: " + str(len(task_queue)))
    for r_id in range(4):
        line = sys.stdin.readline().split(" ") #这4行数据是机器人数据
        info_robots[r_id] = {
                            "at_bench_id" : int(line[0]), 
                            "carry_product_type" : int(line[1]), 
                            "time_value" : float(line[2]), 
                            "collision_value" : float(line[3]), 
                            "angular_velocity" : float(line[4]), 
                            "line_speed_x" : float(line[5]),
                            "line_speed_y" : float(line[6]), 
                            "toward" : float(line[7]), 
                            "coordination_x" : float(line[8]), 
                            "coordination_y" : float(line[9])
                            }

        # 机器人分配任务，开始只考虑了距离，这次考虑距离加上逻辑距离即优先级
        if(len(task_queue) and free_robots[r_id]):
            robot_axit = np.array([info_robots[r_id]["coordination_x"], info_robots[r_id]["coordination_y"]])

            
            can_get_profit = 0
        
            dist = float("inf")
            t = None
            for task in task_queue:
                buy_dist = np.linalg.norm(robot_axit-np.array([task["buy_coordination_x"], task["buy_coordination_y"]]))
                to_buy_need_frames = buy_dist / 0.10 + 30
                if(to_buy_need_frames + frame_id < task["predict_frame_id_product_done"]):
                    continue
                sell_dist = np.linalg.norm(np.array([task["buy_coordination_x"], task["buy_coordination_y"]]) \
                                                     - np.array([task["sell_coordination_x"], task["sell_coordination_y"]]))
                run_to_sell_done_need_frames = sell_dist / 0.10 + 30
                needed_frames_to_finish = to_buy_need_frames + run_to_sell_done_need_frames# 0.1 # 取5米/秒 级0.1米/帧；取4m/s 4/50 = 0.08
                if(needed_frames_to_finish > 9001-frame_id):
                    continue

                if(dist > buy_dist):
                    dist = buy_dist
                    t = task
                # product_type = task["product_type"]

                # product_profit = product_profit_dict[product_type]
                # profit_per_frame = product_profit / needed_frames_to_finish * value_sigma[task["product_type"]]
                # # logging.error(product_profit) # profit 大致在3000 到 7800之间
                # if(can_get_profit < profit_per_frame):
                #     can_get_profit = profit_per_frame
                #     t = task
            if(t):
                free_robots[r_id] = False
                task_queue.remove(t)
                working_robots.append({
                    "robot_id" : r_id, 
                    "buy_bench_seq" : t["buy_bench_seq"], 
                    "sell_bench_seq" : t["sell_bench_seq"], 
                    "product_type" : t["product_type"],
                    "buy_coordination_x": t["buy_coordination_x"], 
                    "buy_coordination_y": t["buy_coordination_y"],
                    "sell_coordination_x": t["sell_coordination_x"], 
                    "sell_coordination_y": t["sell_coordination_y"],
                    "task_type" : 1 # 1->get 0->sell
                }) # 机器人r_i取任务
                
    input() # 最后一行是ok

def run_task():
    # 工作中机器人队列 robot_id, bench_id, to, 产品类型, 取/卖
    for task in working_robots:
        # logging.error("running task: " + str(task))
        # logging.error("robot info: " + str(info_robots[task["robot_id"]]))
        r_x, r_y = info_robots[task["robot_id"]]["coordination_x"], info_robots[task["robot_id"]]["coordination_y"]
        target_x = 0.0
        target_y = 0.0

        if(task["task_type"] == 1): # 取任务
            # 判断到达取位置
            if(info_robots[task["robot_id"]]["at_bench_id"] == task["buy_bench_seq"] and infor_working_benchs_list[task["buy_bench_seq"]]["product_status"] == 1):
                print("buy", task["robot_id"])
                # logging.error("frame id: " + str(frame_id) +  " robot " +  str(task["robot_id"]) + " buy " + str(task["product_type"]))

                buy_benchs_in_quuee[task["buy_bench_seq"]] = False
                task["task_type"] = 0
                continue
            else:
                target_x, target_y = task["buy_coordination_x"], task["buy_coordination_y"]

        else:
             # 判断到达卖位置 新想法，到达之后若当前工作台能买东西 就立即买
            if(info_robots[task["robot_id"]]["at_bench_id"] == task["sell_bench_seq"]):
                
                print("sell", task["robot_id"])
                sell_benchs_in_quuee[task["sell_bench_seq"]][task["product_type"]] = False
                
                # logging.error("frame id: " + str(frame_id) + " robot " +  str(task["robot_id"]) + " sell " + str(task["product_type"]))
                working_robots.remove(task)
                free_robots[task["robot_id"]] = True


                # 找到当前卖东西的工作台
                current_sell_bench = infor_working_benchs_list[task["sell_bench_seq"]]
                # 如果在当前工作台卖出产品后 能买产品并且没有人要过来买
                if(current_sell_bench["product_status"] == 1 and (not buy_benchs_in_quuee[current_sell_bench["bench_seq"]])):
                    product_type = current_sell_bench["bench_type"]
                    buy_bench_axis = np.array([current_sell_bench["coordination_x"], current_sell_bench["coordination_y"]])
                    res_bench = find_produceP_bench_to_sell(product_type, np.array(buy_bench_axis))

                    # 尽量使卖出能在9000帧完成
                    if(res_bench and (np.linalg.norm(buy_bench_axis-np.array([res_bench["coordination_x"], res_bench["coordination_y"]])) / 0.10 + frame_id + 30 < 9001)):
                        
                        print("buy", task["robot_id"])
                        free_robots[task["robot_id"]] = False
                        sell_benchs_in_quuee[res_bench["sell_bench_seq"]][product_type] = True
                        working_robots.append({
                            "robot_id" : task["robot_id"], 
                            "buy_bench_seq" : current_sell_bench["bench_seq"], 
                            "sell_bench_seq" : res_bench["sell_bench_seq"],
                            "product_type": product_type, 
                            "buy_coordination_x": current_sell_bench["coordination_x"], 
                            "buy_coordination_y": current_sell_bench["coordination_y"],
                            "sell_coordination_x": res_bench["coordination_x"], 
                            "sell_coordination_y": res_bench["coordination_y"],
                            "task_type" : 0 # 1->get 0->sell
                    }) # 机器人r_i取任务
                        
                
                continue
            else:
                target_x, target_y = task["sell_coordination_x"], task["sell_coordination_y"]

        speed = np.linalg.norm(np.array([info_robots[task["robot_id"]]["line_speed_x"], info_robots[task["robot_id"]]["line_speed_y"]]))
        toward = info_robots[task["robot_id"]]["toward"]

        x = np.array([r_x, r_y, toward, speed, info_robots[task["robot_id"]]["angular_velocity"]])
        obstacles = np.array([[info_robots[i]["coordination_x"], info_robots[i]["coordination_y"]] for i in range(4) if i != task["robot_id"]])
        dwa.config_params.obstacles = obstacles
        # 计算动作
        u_control, _ = dwa.calculate_ctrl_traj(x, [target_x, target_y])

        print("rotate", task["robot_id"], u_control[1])
        print("forward", task["robot_id"], u_control[0])





# 全局变量
task_queue = [] # benchid, 产品类型, 坐标
free_robots = [True, True, True, True] # 空闲机器人队列 id
# free_robots = [[True, False, False, False]] # 先测试一个机器人
working_robots = [] # 工作中机器人队列 robot_id, bench_id, to, 产品类型, 取/卖

info_robots = {0 : {}, 1 : {}, 2 : {}, 3 : {}}

dwa = DWA()

frame_id = 0

# 两个人不能同时去一个工作台买东西
buy_benchs_in_quuee = [False]*50# 防止重复添加任务进入任务队列
# 两个人不能同时去同一个工作台卖同一种东西
# sell_benchs_in_quuee = [[False]*8]*50 # 不能这样初始化，a[0][0]=True会把一整列都设置为True
sell_benchs_in_quuee = [[False]*8 for _ in range(50)]
infor_working_benchs_dict = {i : [] for i in range(1, 10)} # 按照同一类型归类存储工作台
infor_working_benchs_list = [{}] * 50 # 根据seq来存工作台

workbenchs = 0
where_to_sell = {
    1: [4, 5, 9],
    2: [4, 6, 9],
    3: [5, 6, 9],
    4: [7, 9],
    5: [7, 9],
    6: [7, 9],
    7: [8, 9]
}

# need_1to3_chart = {1:1, 2:1, 3:1}
# used_dict = {"14" : 2, "15":3,"24":1,"26":3, "35":1, "36":2}

# 试试总是以追求单位时间最大价值的导向
value_sigma = {
    1 : 1,
    2 : 1,
    3 : 1,
    4 : 1,
    5 : 1,
    6 : 1,
    7 : 1,
}
product_profit_dict = {
    1: 3000,
    2: 3200,
    3: 3400,
    4: 7100,
    5: 7800,
    6: 8300,
    7: 29000
}

import time
if __name__ == '__main__':
    read_util_ok()
    finish()
    while True:
        line = sys.stdin.readline()#第一行数据 帧序号和金钱数
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])

        line = sys.stdin.readline()#第二行数据 工作台数量
        workbenchs = int(line)
        update_queue(workbenchs) # 更新任务队列s


        # 选手执行输入
        print(frame_id)
        run_task() #执行任务队列
        
        finish()
