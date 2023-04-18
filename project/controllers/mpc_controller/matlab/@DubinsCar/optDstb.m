function dOpt = optDstb(obj, ~, ~, deriv, dMode)
% dOpt = optCtrl(obj, t, y, deriv, dMode)
%     Dynamics of the DubinsCar
%         \dot{x}_1 = v * cos(x_3) + d_1
%         \dot{x}_2 = v * sin(x_3) + d_2
%         \dot{x}_3 = u

%% Input processing
if nargin < 5
  dMode = 'max';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

dOpt = cell(obj.nd, 1);

%% TODO 
% Compute the optimal disturbance 
% min { alpha) deriv(1) * 0.8 cos alphaa + deriv(2) * 0.8 sin alpha
params = get_params();

alpha = atan2(-deriv{2}, -deriv{1});
dOpt{1} =   obj.dMax * cos(alpha); % Compute the optimal disturbance in x
dOpt{2} =   obj.dMax * sin(alpha);  % Compute the optimal disturbance in y


end