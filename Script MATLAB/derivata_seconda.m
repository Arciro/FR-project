function pdd = derivata_seconda(p,tc)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

pdd = zeros(length(p),3);
for k=2:length(p)-1
   pdd(k,:)= (p(k+1,:)-2*p(k,:)+p(k-1,:))/(tc*tc);
end

end

