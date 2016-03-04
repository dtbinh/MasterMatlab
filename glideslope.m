function [Path,correctHeight] = glideslope(dubinPath,h0,h1,glideangle)
Path = [dubinPath;zeros(1,length(dubinPath))];
Path(3,1) = h0;
correctHeight = false;
for i=1:length(dubinPath)-1
    D = sqrt((Path(1,i+1)-Path(1,i))^2+(Path(2,i+1)-Path(2,i))^2);
    if abs(atan2(h1-Path(3,i),D))<abs(glideangle)
        correctHeight = true;
        glideangle = atan2(h1-Path(3,i),D);
        Path(3,i+1) = Path(3,i)+D*tan(glideangle);
    
    elseif ~correctHeight
        Path(3,i+1) = Path(3,i)+D*tan(glideangle);
    else
        Path(3,i+1) = Path(3,i);
    end
     
end