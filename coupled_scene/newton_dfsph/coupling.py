"""Reusable explicit liquid/rigid exchange, independent of scene motion policy."""
import numpy as np
from .runtime import boundary_state, com_wrench


class FluidRigidCoupling:
    """Map DFSPH dynamic boundaries in scene order to Newton body indices.

    respond_to_fluid is False for motor-prescribed bodies, True for free bodies.
    Existing Newton external forces are preserved; the caller clears them at
    the beginning of a step and advances its chosen rigid solver afterwards.
    """
    def __init__(self, reference, model, body_indices, respond_to_fluid):
        self.reference=reference
        self.indices=np.asarray(body_indices,dtype=int)
        self.respond=np.asarray(respond_to_fluid,dtype=bool)
        if len(self.indices)!=reference.bodies or self.respond.shape!=self.indices.shape:
            raise ValueError('Every reference boundary requires one explicit Newton body binding')
        if len(set(self.indices.tolist()))!=len(self.indices) or np.any(self.indices<0) or np.any(self.indices>=model.body_count):
            raise ValueError('Invalid or repeated Newton body binding')
        self.com=model.body_com.numpy()
        self.maximum_origin_error_m=0.

    def advance_fluid(self, state, dt):
        q,qd=state.body_q.numpy(),state.body_qd.numpy()
        if not np.isfinite(q).all() or not np.isfinite(qd).all():
            raise RuntimeError('Nonfinite Newton body state')
        packed=np.asarray([boundary_state(q[i],qd[i],self.com[i]) for i in self.indices],dtype=np.float32).reshape(-1,13)
        wrenches=self.reference.step(packed,dt)
        force=state.body_f.numpy()
        for boundary,body in enumerate(self.indices):
            actual=self.reference.pose(boundary)
            error=float(np.linalg.norm(actual[:3]-q[body,:3]))
            self.maximum_origin_error_m=max(self.maximum_origin_error_m,error)
            if error>2e-6 or abs(float(actual[3:]@q[body,3:]))<1-1e-5:
                raise RuntimeError('Fluid boundary and Newton body frame disagree')
            if self.respond[boundary]:
                force[body]+=com_wrench(wrenches[boundary],q[body],self.com[body])
        if self.respond.any():state.body_f.assign(force)
        return wrenches
