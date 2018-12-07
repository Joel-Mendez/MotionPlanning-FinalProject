function T = genTmat(m,r)
A = cos(m(1));
B = sin(m(1));
C = cos(m(2));
D = sin(m(2));
E = cos(m(3));
F = sin(m(3));
AD = A*D;
BD = B*D;
T(1,1)=C*E;
T(2,1)=-C*F;
T(3,1)=D;
T(1,2)=BD*E+A*F;
T(2,2)=-BD*F+A*E;
T(3,1)=-B*C;
T(1,3)=-AD*E+B*F;
T(2,3)=AD*F+B*E;
T(3,3)=A*C;
T(4,1)=r(1);
T(4,2)=r(2);
T(4,3)=r(3);
end