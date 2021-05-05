function [x, y, z] = getRotationInGround(model, joint_name)
% Returns a vector giving a joint centre rotation in the ground frame.

    % Import OpenSim libraries.
    import org.opensim.modeling.*
    
    % Initialise system & advance to position stage.
    state = model.initSystem();
    model.realizePosition(state);
    
    % Access the joint.
    joint_set = model.getJointSet();
    joint = joint_set.get(joint_name);
    
    % Get the parent frame. 
    parent = joint.getParentFrame();
    
    % Access direction by expressing the zero vector in the ground frame.
    ones = Vec3(1, 1, 1);
    new_ones = parent.expressVectorInGround(state, ones);
    
    % Compute rotation matrix.
    v1 = [1, 1, 1];
    v2 = [new_ones.get(0), new_ones.get(1), new_ones.get(2)];
    R = computeRotationMatrix(v1', v2');
    
    % Convert to Euler angles and convert output.
    euler = rotm2eul(R);
    x = euler(3);
    y = euler(2);
    z = euler(1);

end

