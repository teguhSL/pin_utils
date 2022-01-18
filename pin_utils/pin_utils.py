import pinocchio as pin

def computeJacobian(rmodel,rdata,ee_frame_id,q):
    pin.forwardKinematics(rmodel,rdata,q)
    pin.updateFramePlacements(rmodel,rdata)
    pin.computeJointJacobians(rmodel, rdata, q)
    J = pin.getFrameJacobian(rmodel, rdata,ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    return J[:,:7]

def computePose(rmodel, rdata, ee_frame_id, q):
    pin.forwardKinematics(rmodel, rdata, q)
    pin.updateFramePlacements(rmodel, rdata)
    pos, ori = rdata.oMf[ee_frame_id].translation, rdata.oMf[ee_frame_id].rotation
    return pos,ori
    
