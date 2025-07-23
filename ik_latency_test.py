import time
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

def main():
    robot = rtb.models.UR10e()
    robot_dh = rtb.models.DH.UR10e()
    
    start_pose = [-1, -0.350, 0.400, 0, np.pi, 0]
    start_se3 = sm.SE3(start_pose[:3]) * sm.SE3.RPY(start_pose[3:])
    start_q = robot_dh.ikine_LM(start_se3).q
    
    poses = []
    joint_positions = []

    iterations = 1000
    np.random.seed(42)

    for i in range(iterations + 1):
        x_offset = np.random.uniform(-0.05, 0.05)
        y_offset = np.random.uniform(-0.05, 0.05)
        z_offset = np.random.uniform(-0.05, 0.05)

        pose = sm.SE3([start_pose[0] + x_offset, start_pose[1] + y_offset, start_pose[2] + z_offset]) * sm.SE3.RPY(start_pose[3:])
        target_q = robot_dh.ikine_LM(pose, q0=start_q).q
        
        poses.append(pose)
        joint_positions.append(target_q)
    
    latencies = []

    for i in range(1, iterations + 1):
        current_q = joint_positions[i-1]
        target_pose = poses[i]
        
        start_time = time.perf_counter()
        robot_dh.ikine_LM(target_pose, q0=current_q)
        end_time = time.perf_counter()
        
        latencies.append(end_time - start_time)
    
    avg_latency = np.mean(latencies) * 1000
    min_latency = np.min(latencies) * 1000
    max_latency = np.max(latencies) * 1000
    std_latency = np.std(latencies) * 1000
    
    print(f"Average latency: {avg_latency:.4f} ms")
    print(f"Min latency: {min_latency:.4f} ms")
    print(f"Max latency: {max_latency:.4f} ms")
    print(f"Std deviation: {std_latency:.4f} ms")

if __name__ == "__main__":
    main()