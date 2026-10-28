"""Pure torch helpers; quaternion convention is w,x,y,z."""
import torch

def yaw_quat(q):
    w,x,y,z = q.unbind(-1)
    yaw = torch.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    out = torch.zeros_like(q)
    out[:,0] = torch.cos(yaw/2)
    out[:,3] = torch.sin(yaw/2)
    return out

def rotate(q, v):
    t = 2 * torch.cross(q[...,1:], v, dim=-1)
    return v + q[...,:1]*t + torch.cross(q[...,1:], t, dim=-1)

def inverse_rotate(q, v):
    conj = q.clone()
    conj[...,1:] *= -1
    return rotate(conj, v)

def smooth_trajectory(start, goal, elapsed, duration):
    u = (elapsed/duration).clamp(0,1).unsqueeze(-1)
    blend = 10*u**3-15*u**4+6*u**5
    derivative = (30*u**2-60*u**3+30*u**4)/duration.unsqueeze(-1)
    return start+(goal-start)*blend, (goal-start)*derivative

class HeightField:
    """Same diagonal triangulation as env/terrain.py."""
    def __init__(self, axis, heights, device):
        self.axis = torch.as_tensor(axis, dtype=torch.float32, device=device)
        self.z = torch.as_tensor(heights, dtype=torch.float32, device=device)
    def __call__(self, xy):
        ix = torch.searchsorted(self.axis, xy[...,0].contiguous(), right=True)-1
        iy = torch.searchsorted(self.axis, xy[...,1].contiguous(), right=True)-1
        ix = ix.clamp(0,len(self.axis)-2); iy = iy.clamp(0,len(self.axis)-2)
        u = ((xy[...,0]-self.axis[ix])/(self.axis[ix+1]-self.axis[ix])).clamp(0,1)
        v = ((xy[...,1]-self.axis[iy])/(self.axis[iy+1]-self.axis[iy])).clamp(0,1)
        a,b,c,d = self.z[iy,ix],self.z[iy,ix+1],self.z[iy+1,ix],self.z[iy+1,ix+1]
        return torch.where(u>=v, a+u*(b-a)+v*(d-b), a+v*(c-a)+u*(d-c))
