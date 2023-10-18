function [P]=Antropomorfo_Cin_Dir(L,Q)

c1=cos(Q(1));
s1=sin(Q(1));
c2=cos(Q(2));
s2=sin(Q(2));
c23=cos(Q(2)+Q(3));
s23=sin(Q(2)+Q(3));
D=L(2)*c2+L(3)*c23;
P=[D*c1; D*s1; L(2)*s2+L(3)*s23];

end