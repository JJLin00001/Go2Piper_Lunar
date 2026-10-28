"""Offline runner for the upstream DWBC dual-head PPO and history encoder."""
import copy
import hashlib
import json
import math
import time
from pathlib import Path
import torch
from torch.utils.tensorboard import SummaryWriter
from rsl_rl.algorithms import PPO
from rsl_rl.modules import ActorCritic
from .config import NUM_PROPRIO,policy_kwargs,JOINT_NAMES

def contract(settings,manifest):
    # Batch size, durations and logging can change at replay; control and observation semantics cannot.
    return {"task":settings["task"],"joints":JOINT_NAMES,"history_len":settings["history_len"],
        "action_scale":settings["action_scale"],"tcp_offset":settings["tcp_offset_link6_m"],
        "nominal_base_height_m":settings["nominal_base_height_m"],
        "decimation":settings["decimation"],"lunar_config":manifest["lunar_config"],
        "source_build_sha256":manifest["source_build_sha256"]}

class Runner:
    def __init__(self,env,settings,log_dir):
        self.env=env; self.settings=copy.deepcopy(settings)
        self.log_dir=Path(log_dir); self.log_dir.mkdir(parents=True,exist_ok=True)
        self.writer=SummaryWriter(str(self.log_dir/"tensorboard"))
        self.policy=ActorCritic(NUM_PROPRIO,NUM_PROPRIO,18,**policy_kwargs(settings)).to(env.device)
        p=copy.deepcopy(settings["ppo"])
        self.steps=p.pop("num_steps_per_env")
        self.dagger_frequency=p["dagger_update_freq"]
        self.alg=PPO(self.policy,device=env.device,torque_supervision=False,adaptive_arm_gains=False,
            min_policy_std=settings["policy"]["min_std"],**p)
        self.alg.init_storage(env.num_envs,self.steps,[env.cfg.observation_space],[None],[18])
        self.iteration=0; self.total_steps=0
        self.run_contract=contract(settings,env.cfg.asset_manifest)
        self.records=[]
        self.obs=env.reset()[0]["policy"]

    def save(self,path):
        torch.save({"format_version":1,"model_state_dict":self.policy.state_dict(),
            "optimizer_state_dict":self.alg.optimizer.state_dict(),
            "history_optimizer_state_dict":self.alg.hist_encoder_optimizer.state_dict(),
            "iteration":self.iteration,"algorithm_counter":self.alg.counter,
            "learning_rate":self.alg.learning_rate,"total_steps":self.total_steps,
            "settings":self.settings,"contract":self.run_contract,
            "torch_rng_state":torch.get_rng_state(),
            "cuda_rng_state":torch.cuda.get_rng_state_all() if torch.cuda.is_available() else []},path)

    def load(self,path,optimizer=True):
        saved=torch.load(path,map_location=self.env.device,weights_only=True)
        if saved["contract"]!=self.run_contract:
            raise ValueError("Checkpoint control/physics contract differs from this environment. Use its saved settings.")
        self.policy.load_state_dict(saved["model_state_dict"])
        self.iteration=saved["iteration"]; self.total_steps=saved["total_steps"]
        self.alg.counter=saved["algorithm_counter"]
        self.alg.learning_rate=saved["learning_rate"]
        if optimizer:
            self.alg.optimizer.load_state_dict(saved["optimizer_state_dict"])
            self.alg.hist_encoder_optimizer.load_state_dict(saved["history_optimizer_state_dict"])
            torch.set_rng_state(saved["torch_rng_state"].cpu())
            if torch.cuda.is_available() and saved["cuda_rng_state"]:
                torch.cuda.set_rng_state_all([x.cpu() for x in saved["cuda_rng_state"]])
        self.env.set_iteration(self.iteration)
        # Resume at a new episode, never reuse stale simulation buffers from another process.
        self.obs=self.env.reset()[0]["policy"]

    def learn(self,iterations,save_interval=100):
        initial={k:v.detach().clone() for k,v in self.policy.state_dict().items()}
        self.policy.train()
        for _ in range(iterations):
            start=time.perf_counter()
            self.env.set_iteration(self.iteration)
            history_rollout=(self.iteration+1)%self.dagger_frequency==0
            totals=torch.zeros(2,device=self.env.device)
            metrics={}; resets=0
            with torch.no_grad():
                for _ in range(self.steps):
                    actions=self.alg.act(self.obs,self.obs,history_rollout)
                    obs,_,terminated,truncated,extras=self.env.step(actions)
                    dones=terminated|truncated
                    # Bootstrap V(s_terminal), not V(s_before_action) or the auto-reset observation.
                    info={"time_outs":truncated & ~terminated,
                          "terminal_values":self.policy.evaluate(extras["terminal_observation"])}
                    self.alg.process_env_step(extras["leg_reward"],extras["arm_reward"],dones,info)
                    self.obs=obs["policy"]
                    totals+=torch.stack((extras["leg_reward"].mean(),extras["arm_reward"].mean()))
                    resets+=int(dones.sum())
                    for k,v in extras["metrics"].items(): metrics[k]=metrics.get(k,0.)+float(v)
                self.alg.compute_returns(self.obs)
            if history_rollout:
                losses={"history_loss":self.alg.update_dagger()}
            else:
                result=self.alg.update()
                losses=dict(zip(("value_loss","surrogate_loss","arm_torque_loss","mixing_ratio",
                                  "torque_weight","privileged_loss","privileged_weight"),result))
            self.iteration+=1; self.total_steps+=self.env.num_envs*self.steps
            record={"iteration":self.iteration,"total_steps":self.total_steps,
                "update":"history" if history_rollout else "ppo","resets":resets,
                "leg_reward":float(totals[0]/self.steps),"arm_reward":float(totals[1]/self.steps),
                "steps_per_second":self.env.num_envs*self.steps/(time.perf_counter()-start),
                **{k:v/self.steps for k,v in metrics.items()},**losses}
            if not all(math.isfinite(v) for v in record.values() if isinstance(v,(int,float))):
                raise FloatingPointError("Nonfinite training metric")
            if not all(torch.isfinite(p).all() for p in self.policy.parameters()):
                raise FloatingPointError("Nonfinite policy parameter")
            self.records.append(record)
            with (self.log_dir/"metrics.jsonl").open("a") as f: f.write(json.dumps(record)+"\n")
            for k,v in record.items():
                if isinstance(v,(int,float)): self.writer.add_scalar(k,v,self.iteration)
            print("[TRAIN] "+json.dumps(record),flush=True)
            if self.iteration%save_interval==0: self.save(self.log_dir/f"model_{self.iteration}.pt")
        self.save(self.log_dir/f"model_{self.iteration}.pt")
        changed=sum(float((v-initial[k]).abs().sum()) for k,v in self.policy.state_dict().items())
        result={"iterations_completed":iterations,"iteration":self.iteration,
            "parameter_l1_change":changed,"finite":True,"task":self.settings["task"]}
        (self.log_dir/"training_result.json").write_text(json.dumps(result,indent=2)+"\n")
        self.writer.flush()
        if iterations and changed<=0: raise RuntimeError("Training completed without updating the policy")
        return result

    def close(self):
        self.writer.close()
