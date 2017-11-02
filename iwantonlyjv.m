function[jv] = iwantonlyjv(q,a)
L1 = 3*25.4;          %base height (in mm)
L2 = 5.75*25.4;       %shoulder to elbow length (in mm)
L3 = 7.375*25.4;      %elbow to wrist length (in mm)
L4 = 1.75*25.4;       %Wrist1 to Wrist2 (in mm)
L5 = 1.25*25.4;       %wrist2 to base of gripper (in mm)
L6 = 1.125*25.4;      %gripper length (in mm)
PI = pi();            %PI constant
%syms q1 q2 q3 q4 q5 q6 L1 L2 L3 L4 L5 L6 q1dot q2dot q3dot q4dot q5dot q6dot xedot yedot zedot wxedot wyedot wzedot
jv2=zeros();
jv=zeros();
jv1=zeros();
%qdot = [q1dot;q2dot ;q3dot; q4dot; q5dot];
%zhi = [xedot;yedot;zedot;wxedot;wyedot;wzedot];
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
%Frame 1 w.r.t Frame 0
A1 = [cos(q1)               0              -sin(q1)  0;
      sin(q1)               0             cos(q1)  0;
              0            -1            0 L1;
              0                     0                  0     1];
          
%Frame 2 w.r.t Frame 1          
A2 = [sin(q2) cos(q2)  0   L2*sin(q2);
      -cos(q2)  sin(q2)  0   -L2*cos(q2);
              0                        0  1                     0;
              0                        0  0                     1];
         

%Frame 3 w.r.t Frame 2
A3 = [-sin(q3) -cos(q3)  0   -L3*sin(q3);
      cos(q3)  -sin(q3)  0   L3*cos(q3);
              0                        0  1                     0;
              0                        0  0                     1];
         

%Frame 4 w.r.t Frame 3
A4 = [sin(q4) 0   cos(q4)   0;
      -cos(q4)  0  sin(q4)   0;
              0                          -1                    0   0;
              0                                   0                             0   1];
%Frame 5 w.r.t Frame 4 
A5 = [cos(q5) -sin(q5)  0        0;
      sin(q5)  cos(q5)  0        0;
              0          0  1  L4 + L5;
              0          0  0        1];
          
%Gripper Frame w.r.t. Frame 5
A6 = [1 0 0  0;
      0 1 0  0;
      0 0 1  L6;
      0 0 0  1];
  
  T01 = A1;
  T02 = A1*A2;
  T03= A1*A2*A3;
  T04 = A1*A2*A3*A4;
  T05 = A1*A2*A3*A4*A5;
  T06 = A1*A2*A3*A4*A5*A6;
  
  jvc1 = T06(1:3,4) - [0;0;0];
  jvc2 = cross(T01(1:3,3),(T06(1:3,4) - T01(1:3,4)));
  jvc3 = cross(T02(1:3,3),(T06(1:3,4) - T02(1:3,4)));
  jvc4 = cross(T03(1:3,3),(T06(1:3,4) - T03(1:3,4)));
  jvc5 = cross(T04(1:3,3),(T06(1:3,4) - T04(1:3,4)));
  %jvc6 = cross(T05(1:3,3),(T06(1:3,4) - T05(1:3,4)));
  
  mainjv = horzcat(jvc1,jvc2,jvc3,jvc4,jvc5);
  
%   if a==1
%   jv2= zeros(3,4);
%   jv = horzcat(jv1,jv2);
%   end
      
  if a==5
      jvc1 = T05(1:3,4) - [0;0;0];
  jvc2 = cross(T01(1:3,3),(T05(1:3,4) - T01(1:3,4)));
  jvc3 = cross(T02(1:3,3),(T05(1:3,4) - T02(1:3,4)));
 jvc4 = cross(T03(1:3,3),(T05(1:3,4) - T03(1:3,4)));
 jvc5 = cross(T04(1:3,3),(T05(1:3,4) - T04(1:3,4)));
  
  jv1 = horzcat(jvc1,jvc2,jvc3,jvc4,jvc5);
  jv = jv1;
   
  end
  if a==2
  jvc1 = T02(1:3,4) - [0;0;0];
  jvc2 = cross(T01(1:3,3),(T02(1:3,4) - T01(1:3,4)));
  %jvc3 = cross(T02(1:3,3),(T02(1:3,4) - T02(1:3,4)));
   %jvc6 = cross(T05(1:3,3),(T06(1:3,4) - T05(1:3,4)));
  jv2= zeros(3,3);
  jv1 = horzcat(jvc1,jvc2);
  jv = horzcat(jv1,jv2);
  end
  if a==1
  jv2= zeros(3,4);
  jv1 = mainjv(:,1:a);
  jv = horzcat(jv1,jv2);
  end
  if a==3
  jvc1 = T03(1:3,4) - [0;0;0];
  jvc2 = cross(T01(1:3,3),(T03(1:3,4) - T01(1:3,4)));
  jvc3 = cross(T02(1:3,3),(T03(1:3,4) - T02(1:3,4)));
   %jvc6 = cross(T05(1:3,3),(T06(1:3,4) - T05(1:3,4)));
  jv2= zeros(3,2);
  jv1 = horzcat(jvc1,jvc2,jvc3);
  jv = horzcat(jv1,jv2);
  end
  if a==4
 jvc1 = T04(1:3,4) - [0;0;0];
  jvc2 = cross(T01(1:3,3),(T04(1:3,4) - T01(1:3,4)));
  jvc3 = cross(T02(1:3,3),(T04(1:3,4) - T02(1:3,4)));
 jvc4 = cross(T03(1:3,3),(T04(1:3,4) - T03(1:3,4)));
  jv2= zeros(3,1);
  jv1 = horzcat(jvc1,jvc2,jvc3,jvc4);
  jv = horzcat(jv1,jv2);
  end
  if a==6
      jv= mainjv;
  end
  
%   jwc1 = [0;0;1];
%   jwc2 = T01(1:3,3);
%   jwc3 = T02(1:3,3);
%   jwc4 = T03(1:3,3);
%   jwc5 = T04(1:3,3);
%   %jwc6 = [0;0;0];
  
%   jw = horzcat(jwc1,jwc2,jwc3,jwc4,jwc5);
  
  %j = vertcat(jv,jw);
end