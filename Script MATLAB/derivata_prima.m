function pd = derivata_prima(p,tc)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
pd = zeros(length(p),3);
for k=1:length(p)-1
    pd(k,:)=(p(k+1,:)-p(k,:))/tc;
end

end

