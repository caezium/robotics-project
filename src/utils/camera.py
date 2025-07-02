import numpy as np
import pybullet as p
from enum import Enum

class CameraPreset(Enum):
    """Predefined camera configurations for different viewing scenarios"""
    TOP_DOWN = "top_down"
    SIDE_VIEW = "side_view" 
    ANGLED_VIEW = "angled_view"
    CLOSE_UP = "close_up"

# ----- Camera Class -----
class TopDownCamera:
    def __init__(self, img_width, img_height, camera_position=None, floor_plane_size=1.0, preset=CameraPreset.TOP_DOWN):
        self._img_width = img_width
        self._img_height = img_height
        self._floor_plane_size = floor_plane_size
        
        # Apply camera preset or use custom position
        if camera_position is None:
            camera_position, roll, pitch, yaw = self._get_preset_config(preset)
        else:
            # Default top-down configuration for custom positions
            roll, pitch, yaw = 0, -90, 90
        
        self._camera_position = camera_position
        self._roll, self._pitch, self._yaw = roll, pitch, yaw

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
    
    def _get_preset_config(self, preset):
        """Get camera configuration for the specified preset"""
        if preset == CameraPreset.TOP_DOWN:
            return [0, 0, 3], 0, -90, 90  # position, roll, pitch, yaw
        elif preset == CameraPreset.SIDE_VIEW:
            return [0, -2, 1], 0, 0, 90   # Side view from y=-2
        elif preset == CameraPreset.ANGLED_VIEW:
            return [1.5, -1.5, 2], 0, -45, 45  # Angled perspective view
        elif preset == CameraPreset.CLOSE_UP:
            return [0, 0, 1.5], 0, -90, 90  # Closer top-down view
        else:
            return [0, 0, 3], 0, -90, 90  # Default to top-down

    def get_image(self):
        img_arr = p.getCameraImage(
            width=self._img_width,
            height=self._img_height,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._projection_matrix
        )
        rgba = np.reshape(np.array(img_arr[2], dtype=np.uint8), (self._img_height, self._img_width, 4))
        return rgba[:, :, :3]

    def get_pixel_world_coords(self, pixel_x, pixel_y):
        u = pixel_x / self._img_width
        v = 1.0 - (pixel_y / self._img_height)
        world_y = (u * self._floor_plane_size) - self._floor_plane_size / 2
        world_x = -(v * self._floor_plane_size - self._floor_plane_size / 2)
        return [world_x, world_y, 0.0] 