function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end
params = get_params();
%% TODO 
% Compute the optimal controller 
% uOpt = zeros(size(deriv{3}));
% for i = 1:size(deriv{3},1)
%     for k = 1: size(deriv{3}, 2)
%         for j = 1:size(deriv{3},3)
%             if (deriv{3}(i,k,j) >= 0)
%                 uOpt(i,k,j) = params.wMax;
%             else
%                 uOpt(i,k,j) = -params.wMax;
%             end
%         end
%     end
uOpt= (deriv{obj.dims==3} >=0) * (obj.wRange (2) ) + (deriv{obj.dims==3} < 0 ) * (obj.wRange (1));
end
