from matplotlib import pyplot as plt
import numpy as np

d_id = 9
data_foler = 'data/'

class Pose2:
    def __init__(self, x, y, theta):
        self.x_ = x
        self.y_ = y
        self.theta_ = theta
    
    def compose(self, rel_pose):
        new_x = self.x_ + np.cos(self.theta_)*rel_pose.x_ - np.sin(self.theta_) * rel_pose.y_
        new_y = self.y_ + np.sin(self.theta_)*rel_pose.x_ + np.cos(self.theta_) * rel_pose.y_
        new_theta = self.theta_ + rel_pose.theta_
        return Pose2(new_x, new_y, new_theta)

    def inverse(self):
        new_x = np.cos(self.theta_)*(-self.x_) + np.sin(self.theta_) * (-self.y_)
        new_y = -np.sin(self.theta_)*(-self.x_) + np.cos(self.theta_) * (-self.y_)
        new_theta = -self.theta_
        return Pose2(new_x, new_y, new_theta)

    def mult(self, point2):
        new_x = self.x_ + np.cos(self.theta_)*point2[0] - np.sin(self.theta_) * point2[1]
        new_y = self.y_ + np.sin(self.theta_)*point2[0] + np.cos(self.theta_) * point2[1]
        return [new_x, new_y]

def get_gt_filename(dataset_id, robot_id):
    return data_foler + "MRCLAM_Dataset{}/Robot{}_Groundtruth.dat".format(dataset_id, robot_id)

def get_landmark_gt_filename(dataset_id):
    return data_foler + "MRCLAM_Dataset{}/Landmark_Groundtruth.dat".format(dataset_id) 

def get_barcode_filename(dataset_id):
    return data_foler + "MRCLAM_Dataset{}/Barcodes.dat".format(dataset_id)

def get_odometry_filename(dataset_id, robot_id):
    return data_foler + "MRCLAM_Dataset{}/Robot{}_Odometry.dat".format(dataset_id, robot_id)

def get_measurement_filename(dataset_id, robot_id):
    return data_foler + "MRCLAM_Dataset{}/Robot{}_Measurement.dat".format(dataset_id, robot_id)

def load_barcodes(filename):
    barcode_subject_map = {}
    with open(filename, "r") as file:
        lines = list(file.readlines())
        for line in lines[4:]:
            parts = line.split(" ")
            parts = [int(part) for part in parts if (part!='' and part!= '\t' and part!='\n')]
            barcode_subject_map[parts[1]] = parts[0]
    return barcode_subject_map

def load_data(filename):
    results = []
    with open(filename, "r") as file:
        lines = list(file.readlines())
        for line in lines[4:]:
            parts = line.split(" ")
            # print(parts)
            parts = [float(part) for part in parts if (part!='' and part!= '\t' and part!='\n')]
            results.append(parts)
    return np.array(results)

def load_landmarks(filename):
    landmarks = {}
    with open(filename, "r") as file:
        lines = list(file.readlines())
        for line in lines[4:]:
            parts = line.split(" ")
            parts = [part for part in parts if (part!='' and part!= '\t' and part!='\n')]
            landmark_id = int(parts[0])
            landmark_x = float(parts[1])
            landmark_y = float(parts[2])
            landmarks[landmark_id] = [landmark_x, landmark_y]
    return landmarks

def output_gt(filename, data):
    lines = []
    for i in range(len(data)):
        pose = data[i]
        lines.append(str(i) + " " + str(pose.x_)+ " " + str(pose.y_) + " " + str(pose.theta_) + "\n")

    with open(filename, "w") as file:
        file.writelines(lines)

def output_landmark_gt(filename, data):
    lines = []
    for landmark_id, landmark_point in data.items():
        lines.append(str(landmark_id) + " " + str(landmark_point[0]) + " " + str(landmark_point[1]) + "\n")
    with open(filename, "w") as file:
        file.writelines(lines)

def output_measurements(filename, data):
    lines = []
    for measurement in data:
        lines.append(str(measurement[0])+ " " + str(measurement[1]) + " " + str(measurement[2]) + " " + str(measurement[3]) + "\n")

    with open(filename, "w") as file:
        file.writelines(lines)

def output_odometry(filename, data):
    lines = []
    for i in range(len(data)):
        pose = data[i]
        lines.append(str(i) + " " + str(i+1) + " " + str(pose.x_)+ " " + str(pose.y_) + " " + str(pose.theta_) + "\n")

    with open(filename, "w") as file:
        file.writelines(lines)


def linear_interpolate(t, t1, t2, v1, v2):  # TODO: angle need special treatment
    return v1 + (t-t1)/(t2-t1)*(v2-v1)

def get_sample(data, t):
    index = np.searchsorted(data[:, 0], t)
    t1 = data[index-1, 0]
    t2 = data[index, 0]
    data1 = data[index-1, :]
    data2 = data[index, :]
    xy = linear_interpolate(t, t1, t2, data1[1:3], data2[1:3])
    theta1 = data1[3]
    theta2 = data2[3]
    if (abs(theta1-theta2) > 3): # sign flipped
        if (theta1 < 0):
            theta1 += 2*np.pi
        if (theta2 < 0):
            theta2 += 2*np.pi
    theta = linear_interpolate(t, t1, t2, theta1, theta2)
    return Pose2(xy[0], xy[1], theta)


def get_sampled_gt(data, sampled_times):
    sampled_poses = []
    for t in sampled_times:
        pose = get_sample(data, t)
        sampled_poses.append(pose)
    return sampled_poses


def get_rel_pose(v, w, dt):
    if w == 0:
        return Pose2(v*dt, 0, 0)
    r = abs(v/w)
    theta = w*dt
    dx = r * abs(np.sin(theta))
    dy = r * (1-np.cos(theta))
    if (theta < 0):
        dy = -dy
    return Pose2(dx, dy, theta)

def compose_poses(pose_list):
    pose = Pose2(0, 0, 0)
    for rel_pose in pose_list:
        pose = pose.compose(rel_pose)
    return pose

def integrate_odom(data, start_t, end_t):
    start_index = np.searchsorted(data[:, 0], start_t)
    start_t1 = data[start_index-1, 0]
    start_t2 = data[start_index, 0]

    end_index = np.searchsorted(data[:, 0], end_t)
    end_t1 = data[end_index-1, 0]
    end_t2 = data[end_index, 0]

    # total_time = 0
    rel_pose_list = []
    vw_start = linear_interpolate(start_t, start_t1, start_t2, data[start_index-1, 1:3], data[start_index, 1:3])
    vw_end = linear_interpolate(end_t, end_t1, end_t2, data[end_index-1, 1:3], data[end_index, 1:3])
    rel_pose_list.append(get_rel_pose((vw_start[0] + data[start_index, 1])/ 2, (vw_start[1] + data[start_index, 2])/2, start_t2 - start_t))
    # total_time += start_t2 - start_t
    for i in range(start_index, end_index-1):
        v = (data[i, 1] + data[i+1, 1]) / 2
        w = (data[i, 2] + data[i+1, 2]) / 2
        dt = data[i+1, 0] - data[i, 0]
        rel_pose = get_rel_pose(v, w, dt)
        rel_pose_list.append(rel_pose)
        # total_time += dt
    rel_pose_list.append(get_rel_pose((vw_end[0] + data[end_index-1, 1])/ 2, (vw_end[1] + data[end_index-1, 2])/2, end_t - end_t1))
    # total_time += end_t - end_t1
    # print(total_time)
    return compose_poses(rel_pose_list)

def get_sampled_odom(data, sampled_times):
    rel_poses = []
    for i in range(len(sampled_times)-1):
        start_t = sampled_times[i]
        end_t = sampled_times[i+1]
        rel_poses.append(integrate_odom(data, start_t, end_t))
    return rel_poses

def get_sampled_measurements(data, sampled_times, barcode_subject_map):
    sampled_measurements = []

    start_t = sampled_times[0]
    end_t = sampled_times[-1]
    start_index = np.searchsorted(data[:, 0], start_t)
    end_index = np.searchsorted(data[:, 0], end_t)

    for i in range(start_index, end_index):
        t = data[i, 0]
        sample_idx2 = np.searchsorted(sampled_times, t)
        # if sample_idx2 == 0:
        #     sample_idx = sample_idx2
        # else:
        sample_idx1 = sample_idx2-1
        if (abs(sampled_times[sample_idx2] - t) > abs(t - sampled_times[sample_idx1])):
            sample_idx = sample_idx1
        else:
            sample_idx = sample_idx2
        subject_barcode = int(data[i,1])
        if subject_barcode not in barcode_subject_map.keys():
            continue
        subject_id = barcode_subject_map[subject_barcode]
        sampled_measurements.append([sample_idx, subject_id, data[i, 2], data[i, 3]])

    return sampled_measurements

def main():

    ############## read data ##############
    print("read data...")
    barcode_subject_map = load_barcodes(get_barcode_filename(d_id))
    landmarks_gt = load_landmarks(get_landmark_gt_filename(d_id))

    gt_list = []
    measurements_list = []
    odometry_list = []
    start_time_list = []
    end_time_list = []

    for r in range(1, 6):
        gt = load_data(get_gt_filename(d_id, r))
        measurements = load_data(get_measurement_filename(d_id, r))
        odometry = load_data(get_odometry_filename(d_id, r))
        start_time = max([gt[0, 0], measurements[0, 0], odometry[0, 0]])
        end_time = min(gt[-1, 0], measurements[-1, 0], odometry[-1, 0])

        gt_list.append(gt)
        measurements_list.append(measurements)
        odometry_list.append(odometry)
        start_time_list.append(start_time)
        end_time_list.append(end_time)
    
    ############## sample times ##############
    start_time = max(start_time_list)
    end_time = min(end_time_list)
    step = 0.2
    sampled_times = np.arange(start_time, end_time, step)
    print("from", start_time, "to", end_time)

    ############## extract gt data ##############
    print("get sampled gt ...")
    sampled_gt_list = []
    for gt in gt_list:
        sampled_gt = get_sampled_gt(gt, sampled_times)
        sampled_gt_list.append(sampled_gt)

    ############## extract odom data ##############
    print("get sampled odom ...")
    sampled_odom_list = []
    for odometry in odometry_list:
        sampled_odom = get_sampled_odom(odometry, sampled_times)
        sampled_odom_list.append(sampled_odom)

    ############## extract measurement data ##############
    print("get sampled measurements ...")
    sampled_measurements_list = []
    for measurements in measurements_list:
        sampled_measurements = get_sampled_measurements(measurements, sampled_times, barcode_subject_map)
        sampled_measurements_list.append(sampled_measurements)


    ############## filter bad measurements and calibrate noise ##############
    print("calibrate noise and filtering ...")

    sampled_measurements_list_filtered_l = []
    sampled_measurements_list_filtered_r = []
    odom_x_error_list = []
    odom_y_error_list = []
    odom_theta_error_list = []
    landmark_meas_range_error_list = []
    landmark_meas_bearing_error_list = []
    robot_meas_range_error_list = []
    robot_meas_bearing_error_list = []
    for r in range(5):
        sampled_gt = sampled_gt_list[r]
        sampled_odometry = sampled_odom_list[r]
        for i in range(len(sampled_odometry)):
            odom_pose = sampled_odometry[i]
            pose1 = sampled_gt[i]
            pose2 = sampled_gt[i+1]
            rel_pose = pose1.inverse().compose(pose2)
            odom_x_error_list.append(odom_pose.x_ - rel_pose.x_)
            odom_y_error_list.append(odom_pose.y_ - rel_pose.y_)
            theta_error = odom_pose.theta_ - rel_pose.theta_
            if (theta_error > 3):
                theta_error -= 2*np.pi
            if (theta_error < -3):
                theta_error += 2*np.pi
            odom_theta_error_list.append(theta_error)

        sampled_measurements = sampled_measurements_list[r]
        sampled_measurements_filtered_l = []
        sampled_measurements_filtered_r = []
        for i in range(len(sampled_measurements)):
            measurement = sampled_measurements[i]
            t_idx = measurement[0]
            obj_idx = measurement[1]
            range_meas = measurement[2]
            bearing_meas = measurement[3]

            pose = sampled_gt[t_idx]
            if obj_idx > 5:
                landmark = landmarks_gt[obj_idx]
                rel_xy = pose.inverse().mult(landmark)
                range_gt = (rel_xy[0] ** 2 + rel_xy[1] ** 2) ** 0.5
                bearing_gt = np.arctan2(rel_xy[1], rel_xy[0])
                range_error = range_meas - range_gt
                bearing_error = bearing_meas - bearing_gt
                if (abs(range_error) < 0.5 and abs(bearing_error)<0.1):
                    landmark_meas_range_error_list.append(range_error)
                    landmark_meas_bearing_error_list.append(bearing_error)
                    sampled_measurements_filtered_l.append(measurement)
            else:
                robot_idx = obj_idx - 1
                pose_robot = sampled_gt_list[robot_idx][t_idx]
                rel_pose = pose.inverse().compose(pose_robot)
                range_gt = (rel_pose.x_ ** 2 + rel_pose.y_ ** 2) ** 0.5
                bearing_gt = np.arctan2(rel_pose.y_, rel_pose.x_)
                range_error = range_meas - range_gt
                bearing_error = bearing_meas - bearing_gt
                if (abs(range_error) < 0.5 and abs(bearing_error)<0.1):
                    robot_meas_range_error_list.append(range_error)
                    robot_meas_bearing_error_list.append(bearing_error)
                    sampled_measurements_filtered_r.append(measurement)


        sampled_measurements_list_filtered_l.append(sampled_measurements_filtered_l)
        sampled_measurements_list_filtered_r.append(sampled_measurements_filtered_r)
    # plt.figure()
    # plt.hist(odom_x_error_list, color = 'blue', edgecolor = 'black',
    #         bins = int(180/5))
    # plt.title("odom x error")

    # plt.figure()
    # plt.hist(odom_y_error_list, color = 'blue', edgecolor = 'black',
    #         bins = int(180/5))
    # plt.title("odom y error")

    # plt.figure()
    # plt.hist(odom_theta_error_list, color = 'blue', edgecolor = 'black',
    #         bins = int(180/5))
    # plt.title("theta error")
    # print(max(odom_theta_error_list))
    # print(min(odom_theta_error_list))

    # plt.figure()
    # plt.hist(landmark_meas_range_error_list, color = 'blue', edgecolor = 'black',
    #         bins = int(180/5))
    # plt.title("landmark range error")

    # plt.figure()
    # plt.hist(landmark_meas_bearing_error_list, color = 'blue', edgecolor = 'black',
    #         bins = int(180/5))
    # plt.title("landmark bearing error")

    # plt.figure()
    # plt.hist(robot_meas_range_error_list, color = 'blue', edgecolor = 'black',
    #         bins = int(180/5))
    # plt.title("robot range error")

    # plt.figure()
    # plt.hist(robot_meas_bearing_error_list, color = 'blue', edgecolor = 'black',
    #         bins = int(180/5))
    # plt.title("robot bearing error")
    
    # plt.show()

    print("std for odom x: ", np.std(np.array(odom_x_error_list)))
    print("std for odom y: ", np.std(np.array(odom_y_error_list)))
    print("std for odom theta: ", np.std(np.array(odom_theta_error_list)))

    print("std for bearing landmark: ", np.std(np.array(landmark_meas_bearing_error_list)))
    print("std for range landmark: ", np.std(np.array(landmark_meas_range_error_list)))

    print("std for bearing robot: ", np.std(np.array(robot_meas_bearing_error_list)))
    print("std for range robot: ", np.std(np.array(robot_meas_range_error_list)))

    ############## output to file ##############
    print("output to file ...")
    gt_landmark_out_filename = data_foler + 'Dataset{}/gt_landmark.txt'.format(d_id)
    output_landmark_gt(gt_landmark_out_filename, landmarks_gt)
    for r in range(5):
        sampled_gt = sampled_gt_list[r]
        gt_out_filename = data_foler + 'Dataset{}/gt_{}.txt'.format(d_id, r+1)
        output_gt(gt_out_filename, sampled_gt)

        sampled_measurements = sampled_measurements_list_filtered_l[r]
        lmeasurements_out_filename = data_foler + 'Dataset{}/landmark_measurements_{}.txt'.format(d_id, r+1)
        output_measurements(lmeasurements_out_filename, sampled_measurements)

        sampled_measurements = sampled_measurements_list_filtered_r[r]
        rmeasurements_out_filename = data_foler + 'Dataset{}/robot_measurements_{}.txt'.format(d_id, r+1)
        output_measurements(rmeasurements_out_filename, sampled_measurements)

        sampled_odometry = sampled_odom_list[r]
        odometry_out_filename = data_foler + 'Dataset{}/odometry_{}.txt'.format(d_id, r+1)
        output_odometry(odometry_out_filename, sampled_odometry)






def testPose2():
    pose = Pose2(4, 5, np.pi/2)
    inv_pose = pose.inverse()
    print(inv_pose.x_, inv_pose.y_, inv_pose.theta_)
    new_pose = inv_pose.compose(pose)
    print(new_pose.x_, new_pose.y_, new_pose.theta_)

    # pose1 = Pose2(1, 1, np.pi/2)
    # pose2 = Pose2(2, 2, np.pi/2)
    # pose = pose1.compose(pose2)
    # print(pose.x_, pose.y_, pose.theta_)

if __name__=="__main__":
    main()
    # testPose2() 

