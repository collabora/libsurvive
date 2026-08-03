import pysurvive
import sys

actx = pysurvive.SimpleContext(sys.argv)

for obj in actx.Objects():
    print(str(obj.Name(), 'utf-8'))

while actx.Running():
    updated = actx.NextUpdated()
    if updated:
        poseObj = updated.Pose()
        poseData = poseObj[0]
        poseTimestamp = poseObj[1]
        print("%s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))
