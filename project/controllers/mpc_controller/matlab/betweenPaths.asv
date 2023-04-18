function y = betweenPaths(posMatrix,sampleIndex, H)
    for i = 1:H
        if (posMatrix(1,i, sampleIndex) == 0)
            if ~((posMatrix(2,i, sampleIndex) > -5 && posMatrix(2,i,sampleIndex) < -1) || ...
                    (posMatrix(2,i, sampleIndex) < 5 && posMatrix(2,i,sampleIndex) > 1))
                   %posMatrix(1,i, sampleIndex)
                %posMatrix(2,i, sampleIndex)
                y = false;
                return;
            end
        end
        if (posMatrix(2,i, sampleIndex) == 0)
            if ~((posMatrix(1,i, sampleIndex) < 5 && posMatrix(1,i,sampleIndex) > 1) || ...
                    (posMatrix(1,i, sampleIndex) > -5 && posMatrix(1,i,sampleIndex) < -1))
                   %posMatrix(1,i, sampleIndex)
                %posMatrix(2,i, sampleIndex)
                y = false;
                return;
            end
        end
%         if (posMatrix(1,i, sampleIndex) > .80 && posMatrix(1,i, sampleIndex) < 1.2)
%             posMatrix(1,i, sampleIndex)
%         end

        
        if (abs(posMatrix(2,i, sampleIndex)) < 1 &&  abs(posMatrix(1,i, sampleIndex)) < 1)
            y = false;
            return
        end

        %top left corner (left side)
        if (posMatrix(1,i, sampleIndex) <= -1 && posMatrix(1,i, sampleIndex) <= 1)
            if ~(posMatrix(1,i, sampleIndex) > -5 && posMatrix(1,i,sampleIndex) < -1)
                   %posMatrix(1,i, sampleIndex)
                %posMatrix(2,i, sampleIndex)
                y = false;
                return
            end
        end
%         % top left corner (top side)
        if (posMatrix(1,i, sampleIndex) < -1 && posMatrix(2,i, sampleIndex) > 1)
            if ~(posMatrix(2,i, sampleIndex) > 1 && posMatrix(2,i,sampleIndex) < 5)
                   %posMatrix(1,i, sampleIndex)
                %posMatrix(2,i, sampleIndex)
                y = false;
                return
            end
        end
% 
%         % bottom left corner (left side)
        if(posMatrix(1,i,sampleIndex)  < - 1 && posMatrix(2,i, sampleIndex) > -1)
            if ~(posMatrix(1,i, sampleIndex) > -5 && posMatrix(1,i,sampleIndex) < -1)
                   %posMatrix(1,i, sampleIndex)
                %posMatrix(2,i, sampleIndex)
                y = false;
                return;
            end
        end

        % bottom left corner (bottom side)
        if (posMatrix(1,i, sampleIndex) > -1 && posMatrix(2,i, sampleIndex) < -1.1)
            if ~(posMatrix(2,i, sampleIndex) > -5 && posMatrix(2,i,sampleIndex) < -1)
                   posMatrix(1,i, sampleIndex)
                posMatrix(2,i, sampleIndex)
                y = false;
                return;
            end
        end 
        
%         % bottom right corner (right side)
        if (posMatrix(1, i, sampleIndex) >= 1 && posMatrix(2, i, sampleIndex) <= 1)
            if ~(posMatrix(1,i, sampleIndex) > 1 && posMatrix(1,i,sampleIndex) < 5)
                   %posMatrix(1,i, sampleIndex)
                %posMatrix(2,i, sampleIndex)
                y = false;
                return;
            end
        end 

        %bottom right cornet (bottom side)
        if (posMatrix(1, i, sampleIndex) > -1 && posMatrix(1, i, sampleIndex) < 1 && posMatrix(2,i,sampleIndex) < -1)
            if ~(posMatrix(2,i, sampleIndex) > -5 && posMatrix(2,i,sampleIndex) < -1)
                %posMatrix(1,i, sampleIndex)
                %posMatrix(2,i, sampleIndex)
                y = false;
                return;
            end
        end

        % top right corner (right side)
        if ((1 <= posMatrix(1,i, sampleIndex) && posMatrix(2,i, sampleIndex) < 1))
            if ~(posMatrix(1,i, sampleIndex) >= 1 && posMatrix(1,i,sampleIndex) < 5)
%                    posMatrix(1,i, sampleIndex)
%                 posMatrix(2,i, sampleIndex)
                y = false;
                return;
            end
        end
         
         % top right cornet (top Side)
        if ((posMatrix(2,i, sampleIndex) > 1 ) && posMatrix(1, i, sampleIndex) > 1)
           if ~(posMatrix(2,i, sampleIndex) < 5 && posMatrix(2,i,sampleIndex) >= 1)
                  %posMatrix(1,i, sampleIndex)
                %posMatrix(2,i, sampleIndex)
               y = false;
               return
           end
        end
    end
    y = true;
end