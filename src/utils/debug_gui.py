import pybullet as p

class DebugInterface:
    """
    Pybullet's debug stuff
    """
    def __init__(self, kuka_id, num_joints, config):
        self.kuka_id = kuka_id
        self.num_joints = num_joints
        self.config = config
        self.debug_params = {}
        self.debug_text_id = None
        self._create_debug_parameters()

    def _create_debug_parameters(self):
        # Joint Sliders
        for i in range(self.num_joints):
            info = p.getJointInfo(self.kuka_id, i)
            joint_name = info[1].decode('utf-8')
            current_pos = p.getJointState(self.kuka_id, i)[0]
            self.debug_params[f"joint_{i}"] = p.addUserDebugParameter(joint_name, info[8], info[9], current_pos)
        # SimConfig Sliders
        self.debug_params["belt_velocity"] = p.addUserDebugParameter("Belt Velocity", 0, 10, self.config.belt_velocity)
        self.debug_params["pickup_x_coord"] = p.addUserDebugParameter("Pickup X Coord", -1, 1, self.config.pickup_x_coord)
        self.debug_params["confidence_threshold"] = p.addUserDebugParameter("Confidence Threshold", 0, 1, self.config.confidence_threshold)
        self.debug_params["arm_lead_time"] = p.addUserDebugParameter("Arm Lead Time", 0, 2, self.config.arm_lead_time)
        # Debug overlay checkbox
        self.debug_params["show_overlay"] = p.addUserDebugParameter("Show On-Screen Debug", 0, 1, 1)

    def update(self, controller, fsm_state=None, sim_time=None, target_info=None, boxes=None):
        try:
            self.config.belt_velocity = p.readUserDebugParameter(self.debug_params["belt_velocity"])
            self.config.pickup_x_coord = p.readUserDebugParameter(self.debug_params["pickup_x_coord"])
            self.config.confidence_threshold = p.readUserDebugParameter(self.debug_params["confidence_threshold"])
            self.config.arm_lead_time = p.readUserDebugParameter(self.debug_params["arm_lead_time"])
        except (KeyError, p.error):
            return
        if not controller.picked and controller.target_info is None:
            for i in range(self.num_joints):
                try:
                    joint_pos = p.readUserDebugParameter(self.debug_params[f"joint_{i}"])
                    p.setJointMotorControl2(bodyIndex=self.kuka_id,
                                            jointIndex=i,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=joint_pos)
                except (KeyError, p.error):
                    continue
        # Overlay logic
        self.update_overlay(fsm_state, sim_time, target_info, boxes)

    def update_overlay(self, fsm_state, sim_time, target_info, boxes):
        try:
            show_overlay = p.readUserDebugParameter(self.debug_params["show_overlay"]) > 0.5
        except (KeyError, p.error):
            show_overlay = False
        debug_info = None
        if show_overlay:
            debug_info = f"FSM: {fsm_state}"
            if fsm_state == "PREPARE_PICK" and target_info is not None and sim_time is not None:
                elapsed_time = sim_time - target_info["detection_time"]
                predicted_x = target_info["initial_pos"][0] + self.config.belt_velocity * elapsed_time
                time_to_pickup = (self.config.pickup_x_coord - predicted_x) / self.config.belt_velocity if self.config.belt_velocity > 0 else float('inf')
                debug_info += f"\nDet X: {target_info['initial_pos'][0]:.2f}  Pickup X: {self.config.pickup_x_coord:.2f}  t_pick: {time_to_pickup:.2f}s"
            elif fsm_state == "WAIT_FOR_OBJECT" and target_info is None and boxes is not None and len(boxes) > 0:
                # Use the first detected box's X (or object's X if available)
                try:
                    from pybullet import getBasePositionAndOrientation
                    current_object_pos, _ = getBasePositionAndOrientation(self.kuka_id)
                    debug_info += f"\nDet X: {current_object_pos[0]:.2f}  Waiting for detection line ({self.config.detection_line_x:.2f})" # this means smth else now dw
                except Exception:
                    pass
        # Remove/add overlay as needed
        if self.debug_text_id is not None:
            p.removeUserDebugItem(self.debug_text_id)
            self.debug_text_id = None
        if show_overlay and debug_info is not None:
            self.debug_text_id = p.addUserDebugText(debug_info, [0, 0, 1.5], textColorRGB=[1,0,0], textSize=2) 