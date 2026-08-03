from dataclasses import dataclass

from symengine import atan2, asin, cos, sin, tan, sqrt
import cnkalman.codegen as cg
from filterpy.kalman import ExtendedKalmanFilter as EKF
import numpy as np

@dataclass
class BearingAccelPose:
    Pos: np.array = (0., 0.)
    Theta: float = 0
@dataclass
class BearingAccelModel:
    Position: BearingAccelPose
    Velocity: BearingAccelPose
    Accel: np.array = (0., 0.)

@dataclass
class BearingAccelLandmark:
    Position: np.array = (0., 0.)

@cg.generate_code()
def BearingAccelModel_predict(t: float, model: BearingAccelModel):
    pos = model.Position.Pos
    vpos = model.Velocity.Pos
    acc = model.Accel
    new_pos = (
        pos[0] + vpos[0] * t + acc[0] * t * t / 2,
        pos[1] + vpos[1] * t + acc[1] * t * t / 2,
    )
    new_theta = model.Position.Theta + model.Velocity.Theta * t

    new_vpos = (
        vpos[0] + acc[0] * t,
        vpos[1] + acc[1] * t,
    )
    return BearingAccelModel(
        Position=BearingAccelPose(new_pos, new_theta),
        Velocity=BearingAccelPose(new_vpos, model.Velocity.Theta),
        Accel=model.Accel
    )


@cg.generate_code()
def BearingAccelModel_imu_predict(model: BearingAccelModel):
    return [
        model.Velocity.Theta,
        model.Accel[0] * cos(model.Position.Theta),
        model.Accel[1] * sin(model.Position.Theta),
        cos(model.Position.Theta),
        sin(model.Position.Theta),
    ]

def BearingAccelModel_toa_predict(A : BearingAccelLandmark, model: BearingAccelModel):
    dx = A.Position[0] - model.Position.Pos[0]
    dy = A.Position[1] - model.Position.Pos[1]
    speed_of_light = 299792458
    return sqrt(dx*dx + dy*dy + 1e-5) / speed_of_light * 1e9

@cg.generate_code()
def BearingAccelModel_tdoa_predict(A : BearingAccelLandmark, B : BearingAccelLandmark, model: BearingAccelModel):
    return BearingAccelModel_toa_predict(A, model) - BearingAccelModel_toa_predict(B, model)
