import pytest
from stretch4_body.robot.robot_client import RobotClient

def test_self_overwriting(capsys):
    """
    Test that queueing consecutive commands for the SAME joint properly overrides the older command in the 
    queue before it gets sent to the server. Expected behavior: emitting a warning message to the console 
    and keeping the internal dictionary size for that particular joint at 1.
    """
    r = RobotClient()
    joints = r.end_of_arm.joints
    joint = joints[0]
    
    # perform 1st command
    r.end_of_arm.move_by(joint, 0.1)
    
    # clear capsys
    capsys.readouterr()
    
    # perform 2nd command on the SAME joint
    r.end_of_arm.move_by(joint, 0.1)
    captured = capsys.readouterr()
    
    assert "Warn: overwriting previous command" in captured.out
    assert f"for {joint}.end_of_arm" in captured.out
    
    # check the command queue
    assert len(r.cmd_dict) == 1
    assert f"{joint}.end_of_arm" in r.cmd_dict

def test_cross_joint_overwriting(capsys):
    """
    Test that queuing commands across TWO DIFFERENT joints on the same tool (like wrist yaw and pitch)
    do NOT overwrite each other, as they are capable of operating simultaneously. 
    Expected behavior: no warning is emitted and the queue stores both commands (size of 2).
    """
    r = RobotClient()
    joints = r.end_of_arm.joints
    joint1 = joints[0]
    joint2 = joints[1]
    
    # perform command on 1st joint
    r.end_of_arm.move_by(joint1, 0.1)
    
    # clear capsys
    capsys.readouterr()
    
    # perform command on 2nd joint
    r.end_of_arm.move_by(joint2, 0.1)
    captured = capsys.readouterr()
    
    assert "Warn: overwriting previous command" not in captured.out
    
    # check the command queue
    assert len(r.cmd_dict) == 2
    assert f"{joint1}.end_of_arm" in r.cmd_dict
    assert f"{joint2}.end_of_arm" in r.cmd_dict
