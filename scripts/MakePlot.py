from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import tqdm

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

def makePlot(filename, poses_old, poses_new, landmarks_old, landmarks_new):


    colors_new = {0: 'k', 1: 'r', 2 : 'chocolate', 3: 'g', 4: 'b', 5: 'indigo'}
    colors_old = {0: 'k', 1: 'lightcoral', 2 : 'wheat', 3: 'lightgreen', 4: 'lightblue', 5: 'plum'}

    fig = plt.figure(figsize=(10, 10), dpi=160)
    size = 5
    for robot_id, poses in poses_old.items():
        if len(poses) > 0:
            plt.scatter(poses[:,0], poses[:,1], c=colors_old[robot_id+1], s = size, alpha = 1, label="poses_old{}".format(robot_id))
        # plt.plot(server_poses[:,0], server_poses[:,1], '#1f77b4', alpha = 0.5)
    # if len(serverBest_poses) > 0:
    #     plt.plot(serverBest_poses[:,0], serverBest_poses[:,1], 'y', alpha = 0.5)
    for robot_id, poses in poses_new.items():
        if len(poses) > 0:
            plt.scatter(poses[:,0], poses[:,1], c=colors_new[robot_id+1], s=10, alpha = 1, label="poses_new{}".format(robot_id))
    if len(landmarks_old) > 0:
        plt.scatter(landmarks_old[:,0], landmarks_old[:,1], c='k', s=20, alpha = 1, marker='+', label="landmarks_old")
    if len(landmarks_new) > 0:
        plt.scatter(landmarks_new[:,0], landmarks_new[:,1], c='k', s=20, alpha = 1, marker = '+', label="landmarks_new")

    # if len(initial_poses) > 0:
    #     plt.plot(initial_poses[:,0], initial_poses[:,1], 'g', alpha = 0.5)
    # if len(frontal_poses) > 0:
    #     plt.plot(frontal_poses[:,0], frontal_poses[:,1], 'y.', alpha = 0.5)
    plt.xlim(-60, 60)
    plt.ylim(-60, 60)
    # plt.xlim(-3, 7)
    # plt.ylim(-4, 6)
    plt.axis('off')
    # plt.legend(bbox_to_anchor=(1.1, 1.1), prop={'size': 12})
    # if len(error) > 0:
    #     plt.text(-50, 25, "total error : " + str(error[0]))
    #     plt.text(-50, 22, "marginal error: " + str(error[1]))
    plt.savefig(filename)
    plt.close(fig)


def loadDetail (filename):
    results = []
    with open(filename, "r") as file:
        lines = list(file.readlines())
        for line in lines:
            parts = line.split(" ")
            parts = [float(part) for part in parts]
            results.append(parts)
    return np.array(results)

def loadData (filename) :
    poses_old = {}
    poses_new = {}
    landmarks_old = []
    landmarks_new = []
    with open(filename, "r") as file:
        lines = list(file.readlines())
        for line in lines:
            parts = line.split(" ")

            if parts[0] == "POSE":
                robot_id = int(parts[1])
                robot_step = int(parts[2])
                x = float(parts[3])
                y = float(parts[4])
                theta = float(parts[5])
                is_new = int(parts[6])
                if is_new == 1:
                    if robot_id not in poses_new:
                        poses_new[robot_id] = []
                    poses_new[robot_id].append([x, y, theta])
                else:
                    if robot_id not in poses_old:
                        poses_old[robot_id] = []
                    poses_old[robot_id].append([x, y, theta])
            if parts[0] == "LANDMARK":
                x = float(parts[1])
                y = float(parts[2])
                is_new = int(parts[3])
                if is_new:
                    landmarks_new.append([x, y])
                else:
                    landmarks_old.append([x, y])

    for key in poses_old.keys():
        poses_old[key] = np.array(poses_old[key])
    for key in poses_new.keys():
        poses_new[key] = np.array(poses_new[key])
    return poses_old, poses_new, np.array(landmarks_old), np.array(landmarks_new)



def drawSequence(exp_name):

    folder_path = "results_" + exp_name + "/"
    figure_folder_path = "figures_" + exp_name + "/"
    if not os.path.exists(figure_folder_path):
        os.makedirs(figure_folder_path)
    server_buffer = []
    for i in tqdm.tqdm(range(0, 10000, 10)):
        result_file = folder_path + "mrisam2_" + str(i) + ".txt"
        figure_file = figure_folder_path + "fig_" + str(i) + ".png"

        if (os.path.exists(result_file)):
            poses_old, poses_new, landmarks_old, landmarks_new = loadData(result_file)
            makePlot(figure_file, poses_old, poses_new, landmarks_old, landmarks_new)

def plotComparison_ct(num_robots, steps_per_iter):
    exp_name = "CT_" + str(num_robots) + "_" + str(steps_per_iter)
    plotComparison(exp_name)

def plotComparison_UTIAS(dataset_id):
    exp_name = "UTIAS_" + str(dataset_id)
    plotComparison(exp_name)


def avg_result(result, skip):
    num_data = len(result)
    new_result = []
    for i in range(0, len(result), skip):
        new_result.append(np.average(result[i:i+skip]))
    return np.array(new_result)

def plotAll_ct(num_robots, steps_per_iter):
    exp_name = "CT_" + str(num_robots) + "_" + str(steps_per_iter)
    result_folder_name = "results_" + exp_name + "/"
    isam2_result = loadDetail(result_folder_name + "isam2_summary.txt")
    mrisam2_result = loadDetail(result_folder_name + "mrisam2_summary.txt")
    
    title_font_size = 14
    label_font_size = 12
    label_size = 12

    matplotlib.rc('xtick', labelsize=14) 
    matplotlib.rc('ytick', labelsize=14) 
    matplotlib.rcParams.update({'font.size': 14})

    fig, axs = plt.subplots(2, 2, sharex=True, figsize=(10, 6.7))

    skip = 10



    axs[0, 0].plot(avg_result(isam2_result[:, 1], skip), avg_result(isam2_result[:, 9], skip), label="iSAM2")
    axs[0, 0].plot(avg_result(mrisam2_result[:, 1], skip), avg_result(mrisam2_result[:, 9], skip), label="MR-iSAM2")
    axs[0, 0].set_title("ate error", fontsize=title_font_size)
    axs[0, 0].set(ylabel='error(m)')

    axs[0, 1].plot(avg_result(isam2_result[:, 1], skip), avg_result(isam2_result[:, 4], skip), label="iSAM2")
    axs[0, 1].plot(avg_result(mrisam2_result[:, 1], skip), avg_result(mrisam2_result[:, 4], skip), label="MR-iSAM2")
    axs[0, 1].set_title("average re-eliminated variables", fontsize=title_font_size)
    axs[0, 1].set(ylabel='number of variables')

    axs[1, 0].plot(avg_result(isam2_result[:, 1], skip), avg_result(isam2_result[:, 6], skip), label="iSAM2")
    axs[1, 0].plot(avg_result(mrisam2_result[:, 1], skip), avg_result(mrisam2_result[:, 6], skip), label="MR-iSAM2")
    axs[1, 0].set_title("average recalculated cliques", fontsize=title_font_size)
    axs[1, 0].set(xlabel = 'time step', ylabel='number of cliques')

    axs[1, 1].plot(isam2_result[:, 1], isam2_result[:, 7], label="iSAM2")
    axs[1, 1].plot(mrisam2_result[:, 1], mrisam2_result[:, 7], label="MR-iSAM2")
    axs[1, 1].set_title("max clique size", fontsize=title_font_size)
    axs[1, 1].set(xlabel = 'time step', ylabel='number of variables')

    handles, labels = axs[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower right', bbox_to_anchor=(0.98, 0.1), fontsize=14)

    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_all.pdf")
    plt.show()



def plotAll_UTIAS(dataset_id):
    exp_name = "UTIAS_" + str(dataset_id)
    result_folder_name = "results_" + exp_name + "/"
    isam2_result = loadDetail(result_folder_name + "isam2_summary.txt")
    mrisam2_result = loadDetail(result_folder_name + "mrisam2_summary.txt")
    
    dt = 0.2
    title_font_size = 14
    label_font_size = 12
    label_size = 12

    skip = 10

    matplotlib.rc('xtick', labelsize=14) 
    matplotlib.rc('ytick', labelsize=14) 
    matplotlib.rcParams.update({'font.size': 14})

    fig, axs = plt.subplots(2, 3, sharex=True, figsize=(20, 10))
    axs[0, 0].plot(isam2_result[:, 1] * dt, isam2_result[:, 3]/1e6, label="iSAM2")
    axs[0, 0].plot(mrisam2_result[:, 1] * dt, mrisam2_result[:, 3]/1e6, label="MR-iSAM2")
    axs[0, 0].set_yscale('log')
    axs[0, 0].set_title("accumulative update time", fontsize=title_font_size)
    axs[0, 0].set(ylabel='time(s)')

    axs[0, 1].plot(avg_result(isam2_result[:, 1] * dt, skip), avg_result(isam2_result[:, 9], skip), label="iSAM2")
    axs[0, 1].plot(avg_result(mrisam2_result[:, 1] * dt, skip), avg_result(mrisam2_result[:, 9], skip), label="MR-iSAM2")
    axs[0, 1].set_title("ate error", fontsize=title_font_size)
    axs[0, 1].set(ylabel='error(m)')

    axs[0, 2].plot(avg_result(isam2_result[:, 1] * dt, skip), avg_result(isam2_result[:, 4], skip), label="iSAM2")
    axs[0, 2].plot(avg_result(mrisam2_result[:, 1] * dt, skip), avg_result(mrisam2_result[:, 4], skip), label="MR-iSAM2")
    axs[0, 2].set_title("average re-eliminated variables", fontsize=title_font_size)
    axs[0, 2].set(ylabel='number of variables')

    axs[1, 0].plot(avg_result(isam2_result[:, 1] * dt, skip), avg_result(isam2_result[:, 6], skip), label="iSAM2")
    axs[1, 0].plot(avg_result(mrisam2_result[:, 1] * dt, skip), avg_result(mrisam2_result[:, 6], skip), label="MR-iSAM2")
    axs[1, 0].set_title("average recalculated cliques", fontsize=title_font_size)
    axs[1, 0].set(ylabel='number of cliques')

    axs[1, 1].plot(isam2_result[:, 1] * dt, isam2_result[:, 8], label="iSAM2")
    axs[1, 1].plot(mrisam2_result[:, 1] * dt, mrisam2_result[:, 8], label="MR-iSAM2")
    axs[1, 1].set_title("average clique size", fontsize=title_font_size)
    axs[1, 1].set(xlabel = 'time (s)', ylabel='number of variables')

    axs[1, 2].plot(avg_result(isam2_result[:, 1] * dt, skip), avg_result(isam2_result[:, 7], skip), label="iSAM2")
    axs[1, 2].plot(avg_result(mrisam2_result[:, 1] * dt, skip), avg_result(mrisam2_result[:, 7], skip), label="MR-iSAM2")
    axs[1, 2].set_title("max clique size", fontsize=title_font_size)
    axs[1, 2].set(xlabel = 'time (s)', ylabel='number of variables')

    handles, labels = axs[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower right', bbox_to_anchor=(0.98, 0.07), fontsize=14)

    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_all.pdf")
    plt.show()



def plotComparison(exp_name):
    result_folder_name = "results_" + exp_name + "/"
    isam2_result = loadDetail(result_folder_name + "isam2_summary.txt")
    mrisam2_result = loadDetail(result_folder_name + "mrisam2_summary.txt")

    # fig = plt.figure()
    plt.plot(isam2_result[:, 1], isam2_result[:, 2]/1e3, label="iSAM2")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 2]/1e3, label="MR-iSAM2")
    plt.title("step time")
    plt.legend()
    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_step_time.png")

    plt.figure()
    plt.plot(isam2_result[:, 1], isam2_result[:, 3]/1e6, label="iSAM2")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 3]/1e6, label="MR-iSAM2")
    plt.title("accumulative update time")
    plt.legend()
    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_accum_time.png")

    plt.figure()
    plt.plot(isam2_result[:, 1], isam2_result[:, 4], label="iSAM2")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 4], label="MR-iSAM2")
    plt.legend()
    plt.title("avg re-eliminated variables")
    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_variables.png")

    plt.figure()
    plt.plot(isam2_result[:, 1], isam2_result[:, 5], label="iSAM2")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 5], label="MR-iSAM2")
    plt.legend()
    plt.title("avg relinearized factors")
    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_factors.png")
    
    plt.figure()
    plt.plot(isam2_result[:, 1], isam2_result[:, 6], label="iSAM2")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 6], label="MR-iSAM2")
    plt.legend()
    plt.title("avg recalculated cliques")
    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_recal_cliques.png")

    plt.figure()
    plt.plot(isam2_result[:, 1], isam2_result[:, 7], label="iSAM2")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 7], label="MR-iSAM2")
    plt.legend()
    plt.title("max clique size")
    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_max_clique_size.png")

    plt.figure()
    plt.plot(isam2_result[:, 1], isam2_result[:, 8], label="iSAM2")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 8], label="MR-iSAM2")
    plt.legend()
    plt.title("average clique size")
    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_avg_clique_size.png")

    plt.figure()
    plt.plot(isam2_result[:, 1], isam2_result[:, 9], label="iSAM2")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 9], label="MR-iSAM2")
    plt.legend()
    plt.title("ate error")
    plt.tight_layout()
    plt.savefig("figures/" +exp_name + "_error.png")

    plt.figure()
    # plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 5], label="cliques")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 13]/1000, label="gather keys")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 14]/1000, label="extract top")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 15]/1000, label="relinearization")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 16]/1000, label="symbolic create top")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 17]/1000, label="calculate top edges")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 18]/1000, label="propagate marginals")
    plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 19]/1000, label="propagate deltas")
    # plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 13], label="number of marginals")
    # plt.plot(mrisam2_result[:, 1], mrisam2_result[:, 14], label="number of deltas")
    plt.legend()
    plt.title("time")
    plt.tight_layout()

    plt.show()
    plt.close('all')

def plotComparison12550():

    num_robots_list = [50, 5, 2, 1]
    isam2_result_list = []
    mrisam2_result_list = []

    for num_robots in num_robots_list:
        exp_name = "CT_" + str(num_robots) + "_1"
        result_folder_name = "results_" + exp_name + "/"
        isam2_result = loadDetail(result_folder_name + "isam2_summary.txt")
        mrisam2_result = loadDetail(result_folder_name + "mrisam2_summary.txt")
        isam2_result_list.append(isam2_result)
        mrisam2_result_list.append(mrisam2_result)
    

    dt = 0.2
    title_font_size = 14
    label_font_size = 12
    label_size = 12

    matplotlib.rc('xtick', labelsize=14) 
    matplotlib.rc('ytick', labelsize=14) 
    matplotlib.rcParams.update({'font.size': 14})

    fig, axs = plt.subplots(1, 2, sharex=True, figsize=(10, 5))

    np.set_printoptions(precision=3)

    colors = ['r', 'y', 'g', 'b']

    for i in range(len(num_robots_list)):
        num_robots = num_robots_list[i]
        isam2_result = isam2_result_list[i]
        mrisam2_result = mrisam2_result_list[i]

        color = colors[i]

        info = "({} robots)".format(num_robots)
        if (num_robots==1):
            info = "(1 robot)"

        skip = 10

        axs[0].plot(isam2_result[:, 1] * num_robots, isam2_result[:, 3]/1e6, color+'-', label="isam2 " + info)
        axs[0].plot(mrisam2_result[:, 1] * num_robots, mrisam2_result[:, 3]/1e6, color+'--', label="mrisam2 " + info)

        axs[1].plot(avg_result(isam2_result[:, 1] * num_robots, skip), avg_result(isam2_result[:, 8], skip), color + '-', label="isam2 " + info)
        axs[1].plot(avg_result(mrisam2_result[:, 1] * num_robots, skip), avg_result(mrisam2_result[:, 8], skip), color + '--', label="mrisam2 " + info)

        print(num_robots, "&", '%.1f' % (isam2_result[-1, 3]/1e6), "&", '%.1f' % (mrisam2_result[-1, 3]/1e6), "&", '%.3f' % np.average(isam2_result[:, 9]), "&", '%.3f' % np.average(mrisam2_result[:, 9]), "&", '%.1f' % np.average(isam2_result[:, 4]), "&", '%.1f' % np.average(mrisam2_result[:, 4]), "\\\\")

    axs[0].set_yscale('log')
    axs[0].set_ylim(0.001, 500)
    axs[0].set_title("accumulative update time", fontsize=title_font_size)
    axs[0].set(xlabel = 'total steps for all robots', ylabel='time(s)')

    axs[1].set_title("average clique size", fontsize=title_font_size)
    axs[1].set(xlabel = 'total steps for all robots', ylabel='number of variables')

    handles, labels = axs[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', bbox_to_anchor=(0.5, 0.58), fontsize=12)

    plt.tight_layout()
    plt.savefig("figures/comp_ct_all.pdf")
    plt.show()


def showAvgUTIAS():

    sequences = np.arange(1,10)
    isam2_result_list = []
    mrisam2_result_list = []

    for sequence in sequences:
        exp_name = "UTIAS_" + str(sequence)
        result_folder_name = "results_" + exp_name + "/"
        isam2_result = loadDetail(result_folder_name + "isam2_summary.txt")
        mrisam2_result = loadDetail(result_folder_name + "mrisam2_summary.txt")
        isam2_result_list.append(isam2_result)
        mrisam2_result_list.append(mrisam2_result)

    for i in range(len(sequences)):
        sequence = sequences[i]
        isam2_result = isam2_result_list[i]
        mrisam2_result = mrisam2_result_list[i]

        print(sequence, "&", '%.1f' % (isam2_result[-1, 3]/1e6), "&", '%.1f' % (mrisam2_result[-1, 3]/1e6), "&", '%.3f' % np.average(isam2_result[:, 9]), "&", '%.3f' % np.average(mrisam2_result[:, 9]), "&", '%.1f' % np.average(isam2_result[:, 4]), "&", '%.1f' % np.average(mrisam2_result[:, 4]), "\\\\")



def main():
    # drawSequence("resultsUTIAS_1/")
    # plotComparison(1, 1)
    # plotComparison(2, 1)
    # plotComparison_ct(5, 1)
    # plotComparison(50, 1)
    # plotComparison(5, 10)

    # plotComparison_UTIAS(1)
    # plotAll_ct(5,1)
    # plotComparison12550()
    plotAll_UTIAS(1)
    # showAvgUTIAS()

    # drawSequence("CT_5_10")
    # drawSequence("UTIAS_1")

if __name__ == "__main__":
    main()