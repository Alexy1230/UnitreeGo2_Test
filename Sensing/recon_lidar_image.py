"reconstruct L1 lidar (unitree lidar) images"

import struct
import numpy as np
import open3d as o3d

def recover_lidar_cloud(bin_file, point_step=32, offsets=(0, 4, 8, 16, 20, 24)):
    """
    from binary .bin to depth map
    """
    points = []
    rings = []
    off_x, off_y, off_z, off_intensity, off_ring, off_time = offsets

    with open(bin_file, "rb") as f:
        raw_data = f.read()

    for i in range(0, len(raw_data), point_step):
        try:
            x = struct.unpack_from('f', raw_data, i + off_x)[0]
            y = struct.unpack_from('f', raw_data, i + off_y)[0]
            z = struct.unpack_from('f', raw_data, i + off_z)[0]
            
            intensity = struct.unpack_from('f', raw_data, i + off_intensity)[0]
            ring = struct.unpack_from('I', raw_data, i + off_ring)[0]

            if not (x == 0 and y == 0 and z == 0):
                points.append([x, y, z])
                rings.append(ring)
        except struct.error:
            break

    return np.array(points), np.array(rings)

def plot_cloud(points, rings):
    """ plot pointcloud """
    if len(points) == 0:
        print("No valid points?")
        return

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    if len(rings) > 0:
        max_ring = rings.max()
    else:
        max_ring = 1
    norm_rings = rings / (max_ring + 1e-6)
    colors = np.zeros_like(points)
    colors[:, 0] = norm_rings
    colors[:, 1] = 0.5
    colors[:, 2] = 1.0 - norm_rings

    pcd.colors = o3d.utility.Vector3dVector(colors)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    print(f"Total {len(points)} points are detected.")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="L1 Unitree Recovery")

    vis.add_geometry(pcd)
    vis.add_geometry(axes)

    render_option = vis.get_render_option()
    render_option.point_size = 2.0
    render_option.background_color = np.array([0, 0, 0])

    vis.run()
    vis.destroy_window()



if __name__ == "__main__":
    FILE_NAME = "Sensing/lidar_1772232410331.bin" 
    points, rings = recover_lidar_cloud(FILE_NAME, point_step=32, offsets=(0, 4, 8, 16, 20, 24))
    plot_cloud(points, rings)