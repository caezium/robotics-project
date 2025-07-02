import numpy as np
import pybullet as p

# ----- Camera Class -----
class TopDownCamera:
    def __init__(self, img_width, img_height, camera_position, floor_plane_size, target_position=None):
        self._img_width = img_width
        self._img_height = img_height
        self._camera_position = camera_position
        self._floor_plane_size = floor_plane_size
        self._roll, self._pitch, self._yaw = 0, -90, 90

        if target_position is not None:
            self._view_matrix = p.computeViewMatrix(
                cameraEyePosition=camera_position,
                cameraTargetPosition=target_position,
                cameraUpVector=[0, 0, 1]
            )
        else:
            target = camera_position.copy()
            target[2] = 0
            self._view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=target,
                distance=camera_position[2],
                yaw=self._yaw,
                pitch=self._pitch,
                roll=self._roll,
                upAxisIndex=2
            )

        aspect = img_width / img_height
        near, far = 0.01, 10
        fov = 2 * np.degrees(np.arctan((floor_plane_size / 2) / camera_position[2]))

        self._projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    def get_image(self):
        img_arr = p.getCameraImage(
            width=self._img_width,
            height=self._img_height,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._projection_matrix
        )
        
        # Handle case where getCameraImage returns None
        if img_arr is None:
            # Return a black image as fallback
            return np.zeros((self._img_height, self._img_width, 3), dtype=np.uint8)
        
        rgba = np.reshape(np.array(img_arr[2], dtype=np.uint8), (self._img_height, self._img_width, 4))
        return rgba[:, :, :3]

    def get_pixel_world_coords(self, pixel_x, pixel_y):
        u = pixel_x / self._img_width
        v = 1.0 - (pixel_y / self._img_height)
        world_y = (u * self._floor_plane_size) - self._floor_plane_size / 2
        world_x = -(v * self._floor_plane_size - self._floor_plane_size / 2)
        return [world_x, world_y, 0.0] 