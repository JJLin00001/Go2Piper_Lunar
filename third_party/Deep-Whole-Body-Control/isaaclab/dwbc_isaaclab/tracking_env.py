"""Batched floating-base locomotion + Cartesian TCP tracking on the lunar crop."""
from pathlib import Path
import math
import numpy as np
import torch
from pxr import UsdGeom
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import ContactSensor
from .config import JOINT_NAMES,FINGER_NAMES,NUM_PROPRIO,NUM_PRIV
from .math_utils import HeightField,yaw_quat,rotate,inverse_rotate,smooth_trajectory

class Go2PiperLunarTrackingEnv(DirectRLEnv):
    def __init__(self,cfg,render_mode=None,**kwargs):
        super().__init__(cfg,render_mode,**kwargs)
        self.settings=cfg.settings
        self.joint_ids,names=self.robot.find_joints(JOINT_NAMES,preserve_order=True)
        if names!=JOINT_NAMES: raise RuntimeError("Unexpected policy joint mapping")
        self.finger_ids,_=self.robot.find_joints(FINGER_NAMES,preserve_order=True)
        self.ee_id=self.robot.find_bodies("link6")[0][0]
        self.feet_ids,_=self.contacts.find_bodies(["FL_foot","FR_foot","RL_foot","RR_foot"],preserve_order=True)
        self.feet_body_ids,_=self.robot.find_bodies(["FL_foot","FR_foot","RL_foot","RR_foot"],preserve_order=True)
        self.base_contact_id=self.contacts.find_bodies("base")[0][0]
        self.bad_contact_ids,_=self.contacts.find_bodies(["base",".*_thigh","link[1-6]"])
        n=self.num_envs; device=self.device
        self.actions=torch.zeros(n,18,device=device)
        self.previous_actions=torch.zeros_like(self.actions)
        self.command=torch.zeros(n,3,device=device)
        self.command_time=torch.zeros(n,device=device)
        self.traj_time=torch.zeros(n,device=device)
        self.duration=torch.ones(n,device=device)
        self.hold=torch.ones(n,device=device)
        self.start=torch.zeros(n,3,device=device); self.goal=torch.zeros_like(self.start)
        self.target=torch.zeros_like(self.start); self.target_velocity=torch.zeros_like(self.start)
        self.history=torch.zeros(n,self.settings["history_len"],NUM_PROPRIO,device=device)
        self.history_fill=torch.ones(n,dtype=torch.bool,device=device)
        self.last_observation_step=-1
        self.curriculum_scale=self.settings["curriculum"]["initial_scale"]
        self.action_scale=torch.tensor(self.settings["action_scale"],device=device)
        self.tcp_offset=torch.tensor(self.settings["tcp_offset_link6_m"],device=device).repeat(n,1)
        self.targets=self.robot.data.default_joint_pos.clone()
        self.default_q=self.robot.data.default_joint_pos[:,self.joint_ids]
        self.limits=self.robot.data.soft_joint_pos_limits[:,self.joint_ids]
        a=np.load(Path(cfg.asset_dir)/"heightfield.npz")
        self.height=HeightField(a["axis"],a["heights"],device)
        bank=np.load(Path(cfg.asset_dir)/"goals.npz")
        self.goal_bank=torch.tensor(bank["positions"],device=device,dtype=torch.float32)
        self.home=torch.tensor(bank["home"],device=device,dtype=torch.float32)
        self.leg_reward=torch.zeros(n,device=device); self.arm_reward=torch.zeros_like(self.leg_reward)
        self.episode_sums={}
        self._ore_reset_state=self.ore.data.root_state_w.clone()
        self._ore_reset_state[:,:3]-=self.scene.env_origins
        self._ore_reset_state[:,7:]=0
        self.termination_reasons={}
        self._cached_observation=None
        self._markers=None
        if self.sim.has_gui() or render_mode=="rgb_array":
            from isaaclab.markers import VisualizationMarkers,VisualizationMarkersCfg
            self._markers=VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/TrackingMarkers",
                markers={"target":sim_utils.SphereCfg(radius=.022,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.05,1,.15))),
                         "tcp":sim_utils.SphereCfg(radius=.015,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1,.15,.05)))}))


    def _setup_scene(self):
        # One scene contains robot, local terrain, ore and material definitions.
        stage=self.sim.stage
        prim=UsdGeom.Xform.Define(stage,"/World/envs/env_0/Scene").GetPrim()
        prim.GetReferences().AddReference(str(Path(self.cfg.asset_dir)/"scene.usda"))
        self.robot=Articulation(self.cfg.robot)
        self.ore=RigidObject(self.cfg.ore)
        self.contacts=ContactSensor(self.cfg.contacts)
        self.scene.articulations["robot"]=self.robot
        self.scene.rigid_objects["ore"]=self.ore
        self.scene.sensors["contacts"]=self.contacts
        self.scene.clone_environments(copy_from_source=False)
        if self.device=="cpu": self.scene.filter_collisions(global_prim_paths=[])
        # Local light only; no network-resolved HDR or material dependencies.
        from pxr import Gf
        lighting=self.cfg.asset_manifest["lunar_config"]["lighting"]
        elevation=math.radians(lighting["sun_elevation_deg"])
        azimuth=math.radians(lighting["sun_azimuth_deg"])
        direction=Gf.Vec3d(math.cos(elevation)*math.cos(azimuth),
            math.cos(elevation)*math.sin(azimuth),-math.sin(elevation))
        rotation=Gf.Rotation(Gf.Vec3d(0,0,-1),direction).GetQuat()
        sun=sim_utils.DistantLightCfg(intensity=lighting["sun_intensity"],color=(1,.98,.95),angle=.53)
        sun.func("/World/Sun",sun,orientation=(rotation.GetReal(),*rotation.GetImaginary()))
        fill=sim_utils.DistantLightCfg(intensity=lighting["fill_intensity"],color=(.85,.9,1),angle=30)
        fill.func("/World/Fill",fill)

    def set_iteration(self,iteration):
        c=self.settings["curriculum"]
        self.curriculum_scale=c["initial_scale"]+(1-c["initial_scale"])*min(iteration/c["ramp_iterations"],1)

    def _uniform(self,ids,bounds):
        return torch.rand(len(ids),device=self.device)*(bounds[1]-bounds[0])+bounds[0]

    def _sample_velocity(self,ids):
        c=self.settings["command"]
        for i,k in enumerate(("velocity_x_m_s","velocity_y_m_s","yaw_rate_rad_s")):
            self.command[ids,i]=self._uniform(ids,c[k])*self.curriculum_scale
        standing=torch.rand(len(ids),device=self.device)<c["standing_fraction"]
        self.command[ids[standing]]=0
        self.command_time[ids]=0

    def _sample_goal(self,ids):
        c=self.settings["command"]
        self.start[ids]=self.target[ids]
        chosen=torch.randint(len(self.goal_bank),(len(ids),),device=self.device)
        self.goal[ids]=self.home+(self.goal_bank[chosen]-self.home)*self.curriculum_scale
        self.duration[ids]=self._uniform(ids,c["trajectory_time_s"])
        self.hold[ids]=self._uniform(ids,c["hold_time_s"])
        self.traj_time[ids]=0

    def _pre_physics_step(self,actions):
        if actions.shape!=(self.num_envs,18) or not torch.isfinite(actions).all():
            raise ValueError("Expected finite [num_envs,18] policy actions")
        self.previous_actions.copy_(self.actions)
        self.actions.copy_(actions.clamp(-1,1))
        q=self.default_q+self.actions*self.action_scale
        self.targets[:,self.joint_ids]=torch.max(torch.min(q,self.limits[:,:,1]),self.limits[:,:,0])
        self.traj_time+=self.step_dt
        self.command_time+=self.step_dt
        self.target[:],self.target_velocity[:]=smooth_trajectory(self.start,self.goal,self.traj_time,self.duration)

    def _apply_action(self):
        # The sole command writer: one position target for all joints; fixed finger openings.
        self.robot.set_joint_position_target(self.targets)

    def _kinematics(self):
        d=self.robot.data
        self.local_base=d.root_pos_w-self.scene.env_origins
        self.ground_height=self.height(self.local_base[:,:2])
        self.base_height=self.local_base[:,2]-self.ground_height
        self.yaw=yaw_quat(d.root_quat_w)
        self.anchor=d.root_pos_w.clone()
        self.anchor[:,2]=self.scene.env_origins[:,2]+self.ground_height+self.settings["nominal_base_height_m"]
        self.tcp_world=d.body_pos_w[:,self.ee_id]+rotate(d.body_quat_w[:,self.ee_id],self.tcp_offset)
        self.tcp_yaw=inverse_rotate(self.yaw,self.tcp_world-self.anchor)
        self.target_world=self.anchor+rotate(self.yaw,self.target)
        self.ee_error=self.tcp_yaw-self.target
        self.velocity_yaw=inverse_rotate(self.yaw,d.root_lin_vel_w)
        forces=self.contacts.data.net_forces_w[:,self.feet_ids]
        self.feet_contact=forces.norm(dim=-1)>.3
        # Velocity of the virtual TCP, including angular motion about the body COM.
        offset=self.tcp_world-d.body_com_pos_w[:,self.ee_id]
        tcp_velocity=d.body_com_vel_w[:,self.ee_id,:3]+torch.cross(
            d.body_com_vel_w[:,self.ee_id,3:],offset,dim=-1)
        # Relative velocity in moving yaw frame; approximate anchor vertical velocity
        # from local surface slope and horizontal velocity using the same height mesh.
        eps=.02
        p=self.local_base[:,:2]
        dx=torch.tensor([eps,0.],device=self.device); dy=torch.tensor([0.,eps],device=self.device)
        slope_x=(self.height(p+dx)-self.height(p-dx))/(2*eps)
        slope_y=(self.height(p+dy)-self.height(p-dy))/(2*eps)
        anchor_v=d.root_lin_vel_w.clone()
        anchor_v[:,2]=slope_x*anchor_v[:,0]+slope_y*anchor_v[:,1]
        yaw_omega=torch.zeros_like(anchor_v); yaw_omega[:,2]=d.root_ang_vel_w[:,2]
        self.tcp_velocity_yaw=inverse_rotate(self.yaw,tcp_velocity-anchor_v-
            torch.cross(yaw_omega,self.tcp_world-self.anchor,dim=-1))

    def _observation(self,commit=False):
        self._kinematics()
        d=self.robot.data
        proprio=torch.cat((d.root_lin_vel_b,d.root_ang_vel_b,d.projected_gravity_b,self.command,
            d.joint_pos[:,self.joint_ids]-self.default_q,d.joint_vel[:,self.joint_ids]*.05,
            self.actions,self.feet_contact.float(),self.tcp_yaw,self.target,
            self.target-self.tcp_yaw,self.target_velocity,self.base_height[:,None]),dim=-1)
        if proprio.shape[1]!=NUM_PROPRIO: raise RuntimeError("Observation layout changed")
        c=self.cfg.asset_manifest["lunar_config"]
        # Privileged environment parameters; no perception claim or hidden terrain randomization.
        priv=torch.tensor([c["robot"]["foot_static_friction"],c["robot"]["foot_dynamic_friction"],
            c["physics"]["gravity_m_s2"],math.radians(c["terrain"]["slope_deg"]),
            c["ore"]["mass_kg"],self.cfg.asset_manifest["robot"]["mass_kg"]],
            device=self.device).repeat(self.num_envs,1)
        priv=priv/torch.tensor([1,1,1.62,.35,10,20],device=self.device)
        history=torch.cat((self.history[:,1:],proprio[:,None]),dim=1)
        history[self.history_fill]=proprio[self.history_fill,None].expand(-1,self.settings["history_len"],-1)
        obs=torch.cat((proprio,priv,history.flatten(1)),dim=-1).clamp(-100,100)
        if commit:
            self.history.copy_(history)
            self.history_fill[:]=False
        return obs

    def _get_observations(self):
        if self.last_observation_step!=self.common_step_counter or self.history_fill.any():
            ids=(self.command_time>=self.settings["command"]["resample_time_s"]).nonzero().flatten()
            if len(ids): self._sample_velocity(ids)
            ids=(self.traj_time>=self.duration+self.hold).nonzero().flatten()
            if len(ids): self._sample_goal(ids)
            self._cached_observation=self._observation(commit=True)
            self.last_observation_step=self.common_step_counter
            if self._markers is not None:
                self._markers.visualize(translations=torch.cat((self.target_world[:1],self.tcp_world[:1])),
                    marker_indices=torch.tensor([0,1],device=self.device))
        return {"policy":self._cached_observation}

    def _get_dones(self):
        self._kinematics()
        d=self.robot.data
        boundary=self.cfg.asset_manifest["crop_bounds_m"][1]-self.settings["boundary_margin_m"]
        self.termination_reasons={
            "fallen":d.projected_gravity_b[:,2]>-.5,
            "low_base":self.base_height<.16,
            "high_base":self.base_height>.85,
            "base_contact":self.contacts.data.net_forces_w_history[:,:,self.base_contact_id].norm(dim=-1).amax(1)>2,
            "outside_patch":self.local_base[:,:2].abs().amax(1)>boundary,
            "nonfinite":~torch.isfinite(d.root_state_w).all(dim=1)|~torch.isfinite(d.joint_pos).all(dim=1)}
        died=torch.stack(list(self.termination_reasons.values())).any(0)
        return died,self.episode_length_buf>=self.max_episode_length

    def _get_rewards(self):
        d=self.robot.data
        w=self.settings["rewards"]
        error_v=(self.command[:,:2]-self.velocity_yaw[:,:2]).square().sum(1)
        error_yaw=(self.command[:,2]-d.root_ang_vel_w[:,2]).square()
        ee=self.ee_error.square().sum(1)
        tau=d.applied_torque[:,self.joint_ids]; qd=d.joint_vel[:,self.joint_ids]
        rate=(self.actions-self.previous_actions).square()
        q=d.joint_pos[:,self.joint_ids]
        joint_excess=(self.limits[:,:,0]-q).clamp_min(0)+(q-self.limits[:,:,1]).clamp_min(0)
        slip=(d.body_lin_vel_w[:,self.feet_body_ids,:2].square().sum(-1)*self.feet_contact).sum(1)
        undesired=(self.contacts.data.net_forces_w[:,self.bad_contact_ids].norm(dim=-1)>2).float().sum(1)
        terms={
            "linear_tracking":torch.exp(-error_v/.12),
            "yaw_tracking":torch.exp(-error_yaw/.2),
            "ee_tracking":torch.exp(-ee/.015),
            "ee_velocity":torch.exp(-(self.tcp_velocity_yaw-self.target_velocity).square().sum(1)/.1),
            "alive":torch.ones(self.num_envs,device=self.device),
            "orientation":d.projected_gravity_b[:,:2].square().sum(1),
            "base_height":(self.base_height-self.settings["nominal_base_height_m"]).square(),
            "vertical_velocity":d.root_lin_vel_b[:,2].square(),
            "angular_xy":d.root_ang_vel_b[:,:2].square().sum(1),
            "leg_energy":(tau[:,:12]*qd[:,:12]).abs().sum(1),
            "arm_energy":(tau[:,12:]*qd[:,12:]).abs().sum(1),
            "leg_action_rate":rate[:,:12].sum(1),"arm_action_rate":rate[:,12:].sum(1),
            "joint_limits":joint_excess.sum(1),"foot_slip":slip,"undesired_contact":undesired,
            "termination":self.reset_terminated.float()}
        arm_keys={"ee_tracking","ee_velocity","arm_energy","arm_action_rate"}
        weighted={k:v*w[k]*self.step_dt for k,v in terms.items()}
        self.leg_reward=sum(v for k,v in weighted.items() if k not in arm_keys)
        self.arm_reward=sum(weighted[k] for k in arm_keys)
        for k,v in weighted.items():
            if k not in self.episode_sums: self.episode_sums[k]=torch.zeros_like(v)
            self.episode_sums[k]+=v
        # Terminal observations MUST be captured before DirectRLEnv auto-reset.
        self.extras["terminal_observation"]=self._observation(commit=False)
        self.extras["arm_reward"]=self.arm_reward.clone()
        self.extras["leg_reward"]=self.leg_reward.clone()
        self.extras["metrics"]={
            "velocity_rmse_m_s":error_v.mean().sqrt(),
            "yaw_rmse_rad_s":error_yaw.mean().sqrt(),"tcp_rmse_m":ee.mean().sqrt(),
            "base_height_m":self.base_height.mean(),
            "mean_foot_contacts":self.feet_contact.float().sum(1).mean(),
            "curriculum_scale":self.curriculum_scale}
        return self.leg_reward+self.arm_reward

    def _reset_idx(self,env_ids):
        if env_ids is None: env_ids=self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        self.robot.reset(env_ids); self.ore.reset(env_ids)
        state=self.robot.data.default_root_state[env_ids].clone()
        state[:,:3]+=self.scene.env_origins[env_ids]
        self.robot.write_root_pose_to_sim(state[:,:7],env_ids)
        self.robot.write_root_velocity_to_sim(state[:,7:],env_ids)
        self.robot.write_joint_state_to_sim(self.robot.data.default_joint_pos[env_ids],
            self.robot.data.default_joint_vel[env_ids],env_ids=env_ids)
        ore=self._ore_reset_state[env_ids].clone(); ore[:,:3]+=self.scene.env_origins[env_ids]
        self.ore.write_root_state_to_sim(ore,env_ids)
        self.actions[env_ids]=0; self.previous_actions[env_ids]=0
        self.targets[env_ids]=self.robot.data.default_joint_pos[env_ids]
        self.robot.set_joint_position_target(self.targets[env_ids],env_ids=env_ids)
        self.history[env_ids]=0; self.history_fill[env_ids]=True
        self.target[env_ids]=self.home
        self.target_velocity[env_ids]=0
        self._sample_velocity(env_ids); self._sample_goal(env_ids)
        self.extras["log"]={f"reward/{k}":v[env_ids].mean()/self.max_episode_length_s
                            for k,v in self.episode_sums.items()}
        for v in self.episode_sums.values(): v[env_ids]=0
        for k,v in self.termination_reasons.items():
            self.extras["log"]["termination/"+k]=v[env_ids].float().mean()
