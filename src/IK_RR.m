% inverse kinematics of 2r arm
function [th1,th2]= IK_RR(x,y,l1,l2)
l=sqrt(x^2+y^2);
th1=0;
th2=0;
c2=(x^2+y^2-l1^2-l2^2)/(2*l1*l2);
s2=-sqrt(1-c2^2);
if (l<=(l1+l2))
    th1=wrapTo2Pi(atan2(y,x)-atan2(l2*s2,l1+l2*c2));
    th2=wrapTo2Pi(atan2(s2,c2));
end

end