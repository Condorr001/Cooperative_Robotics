  function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

% Vehicle position task
uvms.A.gv = eye(3);

% Vehicle altitude task
uvms.A.a = DecreasingBellShapedFunction(1, 2, 0, 1, uvms.altitude);