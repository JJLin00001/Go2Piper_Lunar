"""CPU tests for control semantics and the upstream PPO compatibility changes."""
import copy
from pathlib import Path
import sys
import unittest
import torch

BACKEND=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BACKEND))
sys.path.insert(0,str(BACKEND.parent/"rsl_rl"))
from dwbc_isaaclab.config import *
from dwbc_isaaclab.math_utils import *
from rsl_rl.modules import ActorCritic
from rsl_rl.algorithms import PPO

class CoreTests(unittest.TestCase):
    def test_contract_and_registration(self):
        import gymnasium as gym
        import dwbc_isaaclab
        self.assertIn("Go2_Piper_Lunar_Tracking",gym.registry)
        self.assertEqual(sum(OBSERVATION_FIELDS.values()),NUM_PROPRIO)
        self.assertEqual(len(set(JOINT_NAMES)),18)
        self.assertEqual(set(JOINT_NAMES+FINGER_NAMES),{x["name"] for x in metadata()["joints"]})
        validate_settings(load_settings())

    def test_invalid_settings(self):
        for key,value in (("history_len",12),("num_envs",0),("patch_size_m",100),
                          ("tcp_offset_link6_m",[0,0]),("episode_length_s",float("nan"))):
            c=load_settings(); c[key]=value
            with self.assertRaises(ValueError): validate_settings(c)

    def test_quaternion_conventions(self):
        q=torch.tensor([[2**-.5,0,0,2**-.5]])
        v=torch.tensor([[1.,0,0]])
        self.assertTrue(torch.allclose(rotate(q,v),torch.tensor([[0.,1,0]]),atol=1e-6))
        self.assertTrue(torch.allclose(inverse_rotate(q,rotate(q,v)),v,atol=1e-6))
        self.assertTrue(torch.allclose(yaw_quat(q),q,atol=1e-6))

    def test_smooth_endpoints_and_velocity(self):
        start=torch.tensor([[.3,0.,.1]]); goal=torch.tensor([[.6,.2,.3]])
        for t,expected in ((0.,start),(2.,goal),(3.,goal)):
            p,v=smooth_trajectory(start,goal,torch.tensor([t]),torch.tensor([2.]))
            self.assertTrue(torch.allclose(p,expected))
            self.assertTrue(torch.allclose(v,torch.zeros_like(v)))
        t=torch.tensor([.8]); dt=.001
        p,v=smooth_trajectory(start,goal,t,torch.tensor([2.]))
        a,_=smooth_trajectory(start,goal,t-dt,torch.tensor([2.]))
        b,_=smooth_trajectory(start,goal,t+dt,torch.tensor([2.]))
        self.assertTrue(torch.allclose(v,(b-a)/(2*dt),atol=3e-5))

    def test_triangle_height_not_bilinear(self):
        field=HeightField([0.,1.],[[0.,0.],[0.,1.]],"cpu")
        points=torch.tensor([[.75,.25],[.25,.75],[1.,1.]])
        self.assertTrue(torch.allclose(field(points),torch.tensor([.25,.25,1.])))

    def test_ppo_and_history_updates_and_timeout_bootstrap(self):
        torch.set_num_threads(2)
        torch.manual_seed(1)
        c=load_settings(); c["num_envs"]=2
        model=ActorCritic(NUM_PROPRIO,NUM_PROPRIO,18,**policy_kwargs(c))
        alg=PPO(model,num_learning_epochs=1,num_mini_batches=1,device="cpu",
            torque_supervision=False,adaptive_arm_gains=False,min_policy_std=c["policy"]["min_std"],
            priv_reg_coef_schedual=[0,.1,1,2])
        obs_dim=NUM_PROPRIO*11+NUM_PRIV
        alg.init_storage(2,4,[obs_dim],[None],[18])
        initial=[p.detach().clone() for p in model.parameters()]
        for step in range(4):
            obs=torch.randn(2,obs_dim)
            with torch.no_grad():
                alg.act(obs,obs)
                # Timeout values intentionally different from current state value.
                terminal=torch.tensor([[4.,5.],[6.,7.]])
                alg.process_env_step(torch.ones(2),torch.ones(2)*2,torch.tensor([True,False]),
                    {"time_outs":torch.tensor([True,False]),"terminal_values":terminal})
            self.assertTrue(torch.allclose(alg.storage.rewards[step,0],torch.tensor([1.,2.])+alg.gamma*terminal[0]))
            self.assertTrue(torch.allclose(alg.storage.rewards[step,1],torch.tensor([1.,2.])))
        with torch.no_grad(): alg.compute_returns(obs)
        result=alg.update()
        self.assertTrue(all(torch.isfinite(torch.tensor(result))))
        self.assertGreater(sum((p-x).abs().sum().item() for p,x in zip(model.parameters(),initial)),0)
        hist_before=[p.detach().clone() for p in model.actor.history_encoder.parameters()]
        for _ in range(4):
            obs=torch.randn(2,obs_dim)
            with torch.no_grad():
                alg.act(obs,obs,True)
                alg.process_env_step(torch.ones(2),torch.ones(2),torch.zeros(2,dtype=torch.bool),{})
        with torch.no_grad(): alg.compute_returns(obs)
        self.assertTrue(torch.isfinite(torch.tensor(alg.update_dagger())))
        self.assertGreater(sum((p-x).abs().sum().item() for p,x in zip(model.actor.history_encoder.parameters(),hist_before)),0)
        self.assertFalse(any(k.startswith("isaacgym") for k in sys.modules))

if __name__=="__main__":
    unittest.main()
