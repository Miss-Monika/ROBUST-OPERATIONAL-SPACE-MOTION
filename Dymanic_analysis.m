  clear all; clc;
% link length

waist_width=0.3; % waist width in meters
crus=0.5; % length of crus
thigh=0.5; % length of thigh
spine=0.7; % length of spine
shoulder=0.6; % width of shoulder
bicep_arm=0.4; % length of bicep arm
forearm=0.4; % length of forearm

% mass of the limbs (in kg)

g = 9.8;   % gravitational acceleration

weight= 50;
m1 = weight/10;  % mass of crus
m2 = weight/10;  % mass of thigh
m3= weight/2.5;   % mass of central body
m4= weight/20;   % mass of arm
m5= weight/20;  % mass of forearm


% time variables
ts=2;    % simulation time
dt=0.005;    % time step
t=0:dt:ts;   % timeline

% Desired motion
th1_des=pi/6 * (1-cos(2*pi*t/10))+pi/90*ones(size(t)); % ankle joint
th2_des=pi/3 * (1-cos(2*pi*t/10))+pi/90*ones(size(t)); % knee joint
th3_des=pi/3 * (1-cos(2*pi*t/10))+pi/90*ones(size(t)); % waist joint
th4_des=pi/6 * (1-cos(2*pi*t/10))+pi/90*ones(size(t)); %shoulder joint
th5_des=pi/3*ones(size(t)); % elbow joint

% Initial values
th = [ th1_des(1) ; th2_des(1) ; th3_des(1) ; th4_des(1) ; th5_des(1) ];
thdot = [ 0;0;0;0;0];
thddot = [ 0;0;0;0;0];

Kp=0.1;
Kd=5;


for i = 1:length(t)
   
    th1= th(1,i);
    th2= th(2,i);
    th3= th(3,i);
    th4= th(4,i);
    th5= th(5,i);
    
    th1dot= thdot(1,i);
    th2dot= thdot(2,i);
    th3dot= thdot(3,i);
    th4dot= thdot(4,i);
    th5dot= thdot(5,i);
    
    th1ddot= thddot(1,i);
    th2ddot= thddot(2,i);
    th3ddot=thddot(3,i);
    th4ddot= thddot(4,i);
    th5ddot= thddot(5,i);
    
    th_des= [ th1_des(i) ; th2_des(i) ; th3_des(i) ; th4_des(i) ; th5_des(i) ] ;
    thdot_des=[0;0;0;0;0];
    
    % Dynamic Behavior
    
 C1= m4*(spine*sin(th1 - th2 + th3) - (bicep_arm*sin(th1 - th2 + th3 - th4))/2 + crus*sin(th1) + thigh*sin(th1 - th2))*(crus*th1dot^2*cos(th1) - (bicep_arm*cos(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2)/2 + spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2) - m5*(crus*th1dot^2*sin(th1) - bicep_arm*sin(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2 + spine*sin(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*sin(th1 - th2)*(th1dot - th2dot)^2)*(spine*cos(th1 - th2 + th3) - bicep_arm*cos(th1 - th2 + th3 - th4) + crus*cos(th1) + thigh*cos(th1 - th2)) - m3*(crus*th1dot^2*sin(th1) + thigh*sin(th1 - th2)*(th1dot - th2dot)^2)*(crus*cos(th1) + thigh*cos(th1 - th2)) + m2*(crus*th1dot^2*cos(th1) + (thigh*cos(th1 - th2)*(th1dot - th2dot)^2)/2)*(crus*sin(th1) + (thigh*sin(th1 - th2))/2) - m4*(spine*cos(th1 - th2 + th3) + crus*cos(th1) + thigh*cos(th1 - th2))*(crus*th1dot^2*sin(th1) + spine*sin(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*sin(th1 - th2)*(th1dot - th2dot)^2) + m3*((spine*sin(th1 - th2 + th3))/2 + crus*sin(th1) + thigh*sin(th1 - th2))*(crus*th1dot^2*cos(th1) + (spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2)/2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2) + m5*(spine*sin(th1 - th2 + th3) - bicep_arm*sin(th1 - th2 + th3 - th4) + crus*sin(th1) + thigh*sin(th1 - th2) - (forearm*sin(th1 - th2 + th3 - th4 - th5))/2)*(crus*th1dot^2*cos(th1) - bicep_arm*cos(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2 - (forearm*cos(th1 - th2 + th3 - th4 - th5)*(th2dot - th1dot - th3dot + th4dot + th5dot)^2)/2 + spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2) + (crus^2*m1*th1dot^2*cos(th1)*sin(th1))/4 - crus^2*m2*th1dot^2*cos(th1)*sin(th1);
C2= m4*(spine*cos(th1 - th2 + th3) + thigh*cos(th1 - th2))*(crus*th1dot^2*sin(th1) + spine*sin(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*sin(th1 - th2)*(th1dot - th2dot)^2) + m5*(bicep_arm*sin(th1 - th2 + th3 - th4) - spine*sin(th1 - th2 + th3) - thigh*sin(th1 - th2) + (forearm*sin(th1 - th2 + th3 - th4 - th5))/2)*(crus*th1dot^2*cos(th1) - bicep_arm*cos(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2 - (forearm*cos(th1 - th2 + th3 - th4 - th5)*(th2dot - th1dot - th3dot + th4dot + th5dot)^2)/2 + spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2) - m4*(spine*sin(th1 - th2 + th3) - (bicep_arm*sin(th1 - th2 + th3 - th4))/2 + thigh*sin(th1 - th2))*(crus*th1dot^2*cos(th1) - (bicep_arm*cos(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2)/2 + spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2) + m5*(spine*cos(th1 - th2 + th3) - bicep_arm*cos(th1 - th2 + th3 - th4) + thigh*cos(th1 - th2))*(crus*th1dot^2*sin(th1) - bicep_arm*sin(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2 + spine*sin(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*sin(th1 - th2)*(th1dot - th2dot)^2) - m3*((spine*sin(th1 - th2 + th3))/2 + thigh*sin(th1 - th2))*(crus*th1dot^2*cos(th1) + (spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2)/2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2) - (m2*thigh*sin(th1 - th2)*(crus*th1dot^2*cos(th1) + (thigh*cos(th1 - th2)*(th1dot - th2dot)^2)/2))/2 + m3*thigh*cos(th1 - th2)*(crus*th1dot^2*sin(th1) + thigh*sin(th1 - th2)*(th1dot - th2dot)^2);
C3=m5*(bicep_arm*cos(th1 - th2 + th3 - th4) - spine*cos(th1 - th2 + th3))*(crus*th1dot^2*sin(th1) - bicep_arm*sin(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2 + spine*sin(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*sin(th1 - th2)*(th1dot - th2dot)^2) - m4*((bicep_arm*sin(th1 - th2 + th3 - th4))/2 - spine*sin(th1 - th2 + th3))*(crus*th1dot^2*cos(th1) - (bicep_arm*cos(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2)/2 + spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2) - m5*(bicep_arm*sin(th1 - th2 + th3 - th4) - spine*sin(th1 - th2 + th3) + (forearm*sin(th1 - th2 + th3 - th4 - th5))/2)*(crus*th1dot^2*cos(th1) - bicep_arm*cos(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2 - (forearm*cos(th1 - th2 + th3 - th4 - th5)*(th2dot - th1dot - th3dot + th4dot + th5dot)^2)/2 + spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2) + (m3*spine*sin(th1 - th2 + th3)*(crus*th1dot^2*cos(th1) + (spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2)/2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2))/2 - m4*spine*cos(th1 - th2 + th3)*(crus*th1dot^2*sin(th1) + spine*sin(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*sin(th1 - th2)*(th1dot - th2dot)^2);
C4=m5*(bicep_arm*sin(th1 - th2 + th3 - th4) + (forearm*sin(th1 - th2 + th3 - th4 - th5))/2)*(crus*th1dot^2*cos(th1) - bicep_arm*cos(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2 - (forearm*cos(th1 - th2 + th3 - th4 - th5)*(th2dot - th1dot - th3dot + th4dot + th5dot)^2)/2 + spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2) + (bicep_arm*m4*sin(th1 - th2 + th3 - th4)*(crus*th1dot^2*cos(th1) - (bicep_arm*cos(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2)/2 + spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2))/2 - bicep_arm*m5*cos(th1 - th2 + th3 - th4)*(crus*th1dot^2*sin(th1) - bicep_arm*sin(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2 + spine*sin(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*sin(th1 - th2)*(th1dot - th2dot)^2);
C5=(forearm*m5*sin(th1 - th2 + th3 - th4 - th5)*(crus*th1dot^2*cos(th1) - bicep_arm*cos(th1 - th2 + th3 - th4)*(th1dot - th2dot + th3dot - th4dot)^2 - (forearm*cos(th1 - th2 + th3 - th4 - th5)*(th2dot - th1dot - th3dot + th4dot + th5dot)^2)/2 + spine*cos(th1 - th2 + th3)*(th1dot - th2dot + th3dot)^2 + thigh*cos(th1 - th2)*(th1dot - th2dot)^2))/2;

C= [C1;C2;C3;C4;C5];

m11=(bicep_arm^2*m4)/8 + bicep_arm^2*m5 + (crus^2*m1)/8 + crus^2*m2 + crus^2*m3 + crus^2*m4 + crus^2*m5 + (forearm^2*m5)/8 + (m3*spine^2)/8 + m4*spine^2 + m5*spine^2 + (m2*thigh^2)/8 + m3*thigh^2 + m4*thigh^2 + m5*thigh^2 - (m2*thigh^2*cos(2*th1 - 2*th2))/8 - (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 - (crus^2*m1*cos(2*th1))/8 - (m3*spine^2*cos(2*th1 - 2*th2 + 2*th3))/8 - (crus*m2*thigh*cos(2*th1 - th2))/2 - (bicep_arm*crus*m4*cos(th2 - th3 + th4))/2 - 2*bicep_arm*crus*m5*cos(th2 - th3 + th4) + (crus*forearm*m5*cos(2*th1 - th2 + th3 - th4 - th5))/2 + (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/2 + (bicep_arm*crus*m4*cos(2*th1 - th2 + th3 - th4))/2 - (forearm*m5*spine*cos(th4 + th5))/2 + (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/2 + (bicep_arm*forearm*m5*cos(th5))/2 - (bicep_arm*m4*spine*cos(th4))/2 - 2*bicep_arm*m5*spine*cos(th4) - (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 + (crus*m2*thigh*cos(th2))/2 + 2*crus*m3*thigh*cos(th2) + 2*crus*m4*thigh*cos(th2) + 2*crus*m5*thigh*cos(th2) - (crus*m3*spine*cos(2*th1 - th2 + th3))/2 - (forearm*m5*thigh*cos(th3 - th4 - th5))/2 - (crus*forearm*m5*cos(th2 - th3 + th4 + th5))/2 + (m3*spine*thigh*cos(th3))/2 + 2*m4*spine*thigh*cos(th3) + 2*m5*spine*thigh*cos(th3) + (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/2 - (m3*spine*thigh*cos(2*th1 - 2*th2 + th3))/2 - (bicep_arm*m4*thigh*cos(th3 - th4))/2 - 2*bicep_arm*m5*thigh*cos(th3 - th4) + (crus*m3*spine*cos(th2 - th3))/2 + 2*crus*m4*spine*cos(th2 - th3) + 2*crus*m5*spine*cos(th2 - th3) + (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/2;
m12=(m2*thigh^2*cos(2*th1 - 2*th2))/8 - bicep_arm^2*m5 - (forearm^2*m5)/8 - (m3*spine^2)/8 - m4*spine^2 - m5*spine^2 - (m2*thigh^2)/8 - m3*thigh^2 - m4*thigh^2 - m5*thigh^2 - (bicep_arm^2*m4)/8 + (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 + (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (m3*spine^2*cos(2*th1 - 2*th2 + 2*th3))/8 + (crus*m2*thigh*cos(2*th1 - th2))/4 + (bicep_arm*crus*m4*cos(th2 - th3 + th4))/4 + bicep_arm*crus*m5*cos(th2 - th3 + th4) - (crus*forearm*m5*cos(2*th1 - th2 + th3 - th4 - th5))/4 - (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/2 - (bicep_arm*crus*m4*cos(2*th1 - th2 + th3 - th4))/4 + (forearm*m5*spine*cos(th4 + th5))/2 - (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/2 - (bicep_arm*forearm*m5*cos(th5))/2 + (bicep_arm*m4*spine*cos(th4))/2 + 2*bicep_arm*m5*spine*cos(th4) + (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 - (crus*m2*thigh*cos(th2))/4 - crus*m3*thigh*cos(th2) - crus*m4*thigh*cos(th2) - crus*m5*thigh*cos(th2) + (crus*m3*spine*cos(2*th1 - th2 + th3))/4 + (forearm*m5*thigh*cos(th3 - th4 - th5))/2 + (crus*forearm*m5*cos(th2 - th3 + th4 + th5))/4 - (m3*spine*thigh*cos(th3))/2 - 2*m4*spine*thigh*cos(th3) - 2*m5*spine*thigh*cos(th3) - (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/2 + (m3*spine*thigh*cos(2*th1 - 2*th2 + th3))/2 + (bicep_arm*m4*thigh*cos(th3 - th4))/2 + 2*bicep_arm*m5*thigh*cos(th3 - th4) - (crus*m3*spine*cos(th2 - th3))/4 - crus*m4*spine*cos(th2 - th3) - crus*m5*spine*cos(th2 - th3) - (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/2;
m13=(bicep_arm^2*m4)/8 + bicep_arm^2*m5 + (forearm^2*m5)/8 + (m3*spine^2)/8 + m4*spine^2 + m5*spine^2 - (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 - (m3*spine^2*cos(2*th1 - 2*th2 + 2*th3))/8 - (bicep_arm*crus*m4*cos(th2 - th3 + th4))/4 - bicep_arm*crus*m5*cos(th2 - th3 + th4) + (crus*forearm*m5*cos(2*th1 - th2 + th3 - th4 - th5))/4 + (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/4 + (bicep_arm*crus*m4*cos(2*th1 - th2 + th3 - th4))/4 - (forearm*m5*spine*cos(th4 + th5))/2 + (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/4 + (bicep_arm*forearm*m5*cos(th5))/2 - (bicep_arm*m4*spine*cos(th4))/2 - 2*bicep_arm*m5*spine*cos(th4) - (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 - (crus*m3*spine*cos(2*th1 - th2 + th3))/4 - (forearm*m5*thigh*cos(th3 - th4 - th5))/4 - (crus*forearm*m5*cos(th2 - th3 + th4 + th5))/4 + (m3*spine*thigh*cos(th3))/4 + m4*spine*thigh*cos(th3) + m5*spine*thigh*cos(th3) + (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/2 - (m3*spine*thigh*cos(2*th1 - 2*th2 + th3))/4 - (bicep_arm*m4*thigh*cos(th3 - th4))/4 - bicep_arm*m5*thigh*cos(th3 - th4) + (crus*m3*spine*cos(th2 - th3))/4 + crus*m4*spine*cos(th2 - th3) + crus*m5*spine*cos(th2 - th3) + (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/2;
m14=(forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - bicep_arm^2*m5 - (forearm^2*m5)/8 - (bicep_arm^2*m4)/8 + (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (bicep_arm*crus*m4*cos(th2 - th3 + th4))/4 + bicep_arm*crus*m5*cos(th2 - th3 + th4) - (crus*forearm*m5*cos(2*th1 - th2 + th3 - th4 - th5))/4 - (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/4 - (bicep_arm*crus*m4*cos(2*th1 - th2 + th3 - th4))/4 + (forearm*m5*spine*cos(th4 + th5))/4 - (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/4 - (bicep_arm*forearm*m5*cos(th5))/2 + (bicep_arm*m4*spine*cos(th4))/4 + bicep_arm*m5*spine*cos(th4) + (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 + (forearm*m5*thigh*cos(th3 - th4 - th5))/4 + (crus*forearm*m5*cos(th2 - th3 + th4 + th5))/4 - (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/4 + (bicep_arm*m4*thigh*cos(th3 - th4))/4 + bicep_arm*m5*thigh*cos(th3 - th4) - (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/4;
m15=-(forearm*m5*(forearm + 2*crus*cos(2*th1 - th2 + th3 - th4 - th5) + 2*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5) - 2*spine*cos(th4 + th5) + 2*bicep_arm*cos(th5) - 2*bicep_arm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5) - forearm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5) - 2*thigh*cos(th3 - th4 - th5) - 2*crus*cos(th2 - th3 + th4 + th5) + 2*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5)))/8;


m21=(m2*thigh^2*cos(2*th1 - 2*th2))/8 - bicep_arm^2*m5 - (forearm^2*m5)/8 - (m3*spine^2)/8 - m4*spine^2 - m5*spine^2 - (m2*thigh^2)/8 - m3*thigh^2 - m4*thigh^2 - m5*thigh^2 - (bicep_arm^2*m4)/8 + (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 + (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (m3*spine^2*cos(2*th1 - 2*th2 + 2*th3))/8 + (crus*m2*thigh*cos(2*th1 - th2))/4 + (bicep_arm*crus*m4*cos(th2 - th3 + th4))/4 + bicep_arm*crus*m5*cos(th2 - th3 + th4) - (crus*forearm*m5*cos(2*th1 - th2 + th3 - th4 - th5))/4 - (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/2 - (bicep_arm*crus*m4*cos(2*th1 - th2 + th3 - th4))/4 + (forearm*m5*spine*cos(th4 + th5))/2 - (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/2 - (bicep_arm*forearm*m5*cos(th5))/2 + (bicep_arm*m4*spine*cos(th4))/2 + 2*bicep_arm*m5*spine*cos(th4) + (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 - (crus*m2*thigh*cos(th2))/4 - crus*m3*thigh*cos(th2) - crus*m4*thigh*cos(th2) - crus*m5*thigh*cos(th2) + (crus*m3*spine*cos(2*th1 - th2 + th3))/4 + (forearm*m5*thigh*cos(th3 - th4 - th5))/2 + (crus*forearm*m5*cos(th2 - th3 + th4 + th5))/4 - (m3*spine*thigh*cos(th3))/2 - 2*m4*spine*thigh*cos(th3) - 2*m5*spine*thigh*cos(th3) - (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/2 + (m3*spine*thigh*cos(2*th1 - 2*th2 + th3))/2 + (bicep_arm*m4*thigh*cos(th3 - th4))/2 + 2*bicep_arm*m5*thigh*cos(th3 - th4) - (crus*m3*spine*cos(th2 - th3))/4 - crus*m4*spine*cos(th2 - th3) - crus*m5*spine*cos(th2 - th3) - (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/2;
m22=(bicep_arm^2*m4)/8 + bicep_arm^2*m5 + (forearm^2*m5)/8 + (m3*spine^2)/8 + m4*spine^2 + m5*spine^2 + (m2*thigh^2)/8 + m3*thigh^2 + m4*thigh^2 + m5*thigh^2 - (m2*thigh^2*cos(2*th1 - 2*th2))/8 - (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 - (m3*spine^2*cos(2*th1 - 2*th2 + 2*th3))/8 + (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/2 - (forearm*m5*spine*cos(th4 + th5))/2 + (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/2 + (bicep_arm*forearm*m5*cos(th5))/2 - (bicep_arm*m4*spine*cos(th4))/2 - 2*bicep_arm*m5*spine*cos(th4) - (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 - (forearm*m5*thigh*cos(th3 - th4 - th5))/2 + (m3*spine*thigh*cos(th3))/2 + 2*m4*spine*thigh*cos(th3) + 2*m5*spine*thigh*cos(th3) + (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/2 - (m3*spine*thigh*cos(2*th1 - 2*th2 + th3))/2 - (bicep_arm*m4*thigh*cos(th3 - th4))/2 - 2*bicep_arm*m5*thigh*cos(th3 - th4) + (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/2;
m23=(forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - bicep_arm^2*m5 - (forearm^2*m5)/8 - (m3*spine^2)/8 - m4*spine^2 - m5*spine^2 - (bicep_arm^2*m4)/8 + (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (m3*spine^2*cos(2*th1 - 2*th2 + 2*th3))/8 - (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/4 + (forearm*m5*spine*cos(th4 + th5))/2 - (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/4 - (bicep_arm*forearm*m5*cos(th5))/2 + (bicep_arm*m4*spine*cos(th4))/2 + 2*bicep_arm*m5*spine*cos(th4) + (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 + (forearm*m5*thigh*cos(th3 - th4 - th5))/4 - (m3*spine*thigh*cos(th3))/4 - m4*spine*thigh*cos(th3) - m5*spine*thigh*cos(th3) - (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/2 + (m3*spine*thigh*cos(2*th1 - 2*th2 + th3))/4 + (bicep_arm*m4*thigh*cos(th3 - th4))/4 + bicep_arm*m5*thigh*cos(th3 - th4) - (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/2;
m24=(bicep_arm^2*m4)/8 + bicep_arm^2*m5 + (forearm^2*m5)/8 - (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/4 - (forearm*m5*spine*cos(th4 + th5))/4 + (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/4 + (bicep_arm*forearm*m5*cos(th5))/2 - (bicep_arm*m4*spine*cos(th4))/4 - bicep_arm*m5*spine*cos(th4) - (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 - (forearm*m5*thigh*cos(th3 - th4 - th5))/4 + (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/4 - (bicep_arm*m4*thigh*cos(th3 - th4))/4 - bicep_arm*m5*thigh*cos(th3 - th4) + (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/4;
m25=(forearm*m5*(forearm + 2*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5) - 2*spine*cos(th4 + th5) + 2*bicep_arm*cos(th5) - 2*bicep_arm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5) - forearm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5) - 2*thigh*cos(th3 - th4 - th5) + 2*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5)))/8;


m31=(bicep_arm^2*m4)/8 + bicep_arm^2*m5 + (forearm^2*m5)/8 + (m3*spine^2)/8 + m4*spine^2 + m5*spine^2 - (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 - (m3*spine^2*cos(2*th1 - 2*th2 + 2*th3))/8 - (bicep_arm*crus*m4*cos(th2 - th3 + th4))/4 - bicep_arm*crus*m5*cos(th2 - th3 + th4) + (crus*forearm*m5*cos(2*th1 - th2 + th3 - th4 - th5))/4 + (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/4 + (bicep_arm*crus*m4*cos(2*th1 - th2 + th3 - th4))/4 - (forearm*m5*spine*cos(th4 + th5))/2 + (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/4 + (bicep_arm*forearm*m5*cos(th5))/2 - (bicep_arm*m4*spine*cos(th4))/2 - 2*bicep_arm*m5*spine*cos(th4) - (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 - (crus*m3*spine*cos(2*th1 - th2 + th3))/4 - (forearm*m5*thigh*cos(th3 - th4 - th5))/4 - (crus*forearm*m5*cos(th2 - th3 + th4 + th5))/4 + (m3*spine*thigh*cos(th3))/4 + m4*spine*thigh*cos(th3) + m5*spine*thigh*cos(th3) + (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/2 - (m3*spine*thigh*cos(2*th1 - 2*th2 + th3))/4 - (bicep_arm*m4*thigh*cos(th3 - th4))/4 - bicep_arm*m5*thigh*cos(th3 - th4) + (crus*m3*spine*cos(th2 - th3))/4 + crus*m4*spine*cos(th2 - th3) + crus*m5*spine*cos(th2 - th3) + (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/2;
m32=(forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - bicep_arm^2*m5 - (forearm^2*m5)/8 - (m3*spine^2)/8 - m4*spine^2 - m5*spine^2 - (bicep_arm^2*m4)/8 + (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (m3*spine^2*cos(2*th1 - 2*th2 + 2*th3))/8 - (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/4 + (forearm*m5*spine*cos(th4 + th5))/2 - (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/4 - (bicep_arm*forearm*m5*cos(th5))/2 + (bicep_arm*m4*spine*cos(th4))/2 + 2*bicep_arm*m5*spine*cos(th4) + (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 + (forearm*m5*thigh*cos(th3 - th4 - th5))/4 - (m3*spine*thigh*cos(th3))/4 - m4*spine*thigh*cos(th3) - m5*spine*thigh*cos(th3) - (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/2 + (m3*spine*thigh*cos(2*th1 - 2*th2 + th3))/4 + (bicep_arm*m4*thigh*cos(th3 - th4))/4 + bicep_arm*m5*thigh*cos(th3 - th4) - (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/2;
m33=(bicep_arm^2*m4)/8 + bicep_arm^2*m5 + (forearm^2*m5)/8 + (m3*spine^2)/8 + m4*spine^2 + m5*spine^2 - (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 - (m3*spine^2*cos(2*th1 - 2*th2 + 2*th3))/8 - (forearm*m5*spine*cos(th4 + th5))/2 + (bicep_arm*forearm*m5*cos(th5))/2 - (bicep_arm*m4*spine*cos(th4))/2 - 2*bicep_arm*m5*spine*cos(th4) - (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 + (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/2 + (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/2;
m34=(forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - bicep_arm^2*m5 - (forearm^2*m5)/8 - (bicep_arm^2*m4)/8 + (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (forearm*m5*spine*cos(th4 + th5))/4 - (bicep_arm*forearm*m5*cos(th5))/2 + (bicep_arm*m4*spine*cos(th4))/4 + bicep_arm*m5*spine*cos(th4) + (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 - (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/4 - (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/4;
m35=-(forearm*m5*(forearm - 2*spine*cos(th4 + th5) + 2*bicep_arm*cos(th5) - 2*bicep_arm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5) - forearm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5) + 2*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5)))/8;


m41=(forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - bicep_arm^2*m5 - (forearm^2*m5)/8 - (bicep_arm^2*m4)/8 + (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (bicep_arm*crus*m4*cos(th2 - th3 + th4))/4 + bicep_arm*crus*m5*cos(th2 - th3 + th4) - (crus*forearm*m5*cos(2*th1 - th2 + th3 - th4 - th5))/4 - (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/4 - (bicep_arm*crus*m4*cos(2*th1 - th2 + th3 - th4))/4 + (forearm*m5*spine*cos(th4 + th5))/4 - (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/4 - (bicep_arm*forearm*m5*cos(th5))/2 + (bicep_arm*m4*spine*cos(th4))/4 + bicep_arm*m5*spine*cos(th4) + (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 + (forearm*m5*thigh*cos(th3 - th4 - th5))/4 + (crus*forearm*m5*cos(th2 - th3 + th4 + th5))/4 - (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/4 + (bicep_arm*m4*thigh*cos(th3 - th4))/4 + bicep_arm*m5*thigh*cos(th3 - th4) - (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/4;
m42=(bicep_arm^2*m4)/8 + bicep_arm^2*m5 + (forearm^2*m5)/8 - (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (forearm*m5*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5))/4 - (forearm*m5*spine*cos(th4 + th5))/4 + (bicep_arm*m4*thigh*cos(2*th1 - 2*th2 + th3 - th4))/4 + (bicep_arm*forearm*m5*cos(th5))/2 - (bicep_arm*m4*spine*cos(th4))/4 - bicep_arm*m5*spine*cos(th4) - (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 - (forearm*m5*thigh*cos(th3 - th4 - th5))/4 + (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/4 - (bicep_arm*m4*thigh*cos(th3 - th4))/4 - bicep_arm*m5*thigh*cos(th3 - th4) + (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/4;
m43=(forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - bicep_arm^2*m5 - (forearm^2*m5)/8 - (bicep_arm^2*m4)/8 + (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (forearm*m5*spine*cos(th4 + th5))/4 - (bicep_arm*forearm*m5*cos(th5))/2 + (bicep_arm*m4*spine*cos(th4))/4 + bicep_arm*m5*spine*cos(th4) + (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2 - (forearm*m5*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5))/4 - (bicep_arm*m4*spine*cos(2*th1 - 2*th2 + 2*th3 - th4))/4;
m44= (bicep_arm^2*m4)/8 + bicep_arm^2*m5 + (forearm^2*m5)/8 - (forearm^2*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5))/8 - (bicep_arm^2*m4*cos(2*th1 - 2*th2 + 2*th3 - 2*th4))/8 + (bicep_arm*forearm*m5*cos(th5))/2 - (bicep_arm*forearm*m5*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5))/2;
m45= (forearm*m5*(forearm + 2*bicep_arm*cos(th5) - 2*bicep_arm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5) - forearm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5)))/8;
 
 
m51= -(forearm*m5*(forearm + 2*crus*cos(2*th1 - th2 + th3 - th4 - th5) + 2*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5) - 2*spine*cos(th4 + th5) + 2*bicep_arm*cos(th5) - 2*bicep_arm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5) - forearm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5) - 2*thigh*cos(th3 - th4 - th5) - 2*crus*cos(th2 - th3 + th4 + th5) + 2*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5)))/8;
m52= (forearm*m5*(forearm + 2*thigh*cos(2*th1 - 2*th2 + th3 - th4 - th5) - 2*spine*cos(th4 + th5) + 2*bicep_arm*cos(th5) - 2*bicep_arm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5) - forearm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5) - 2*thigh*cos(th3 - th4 - th5) + 2*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5)))/8;
m53=-(forearm*m5*(forearm - 2*spine*cos(th4 + th5) + 2*bicep_arm*cos(th5) - 2*bicep_arm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5) - forearm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5) + 2*spine*cos(2*th1 - 2*th2 + 2*th3 - th4 - th5)))/8;
m54= (forearm*m5*(forearm + 2*bicep_arm*cos(th5) - 2*bicep_arm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - th5) - forearm*cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5)))/8;
m55= -(forearm^2*m5*(cos(2*th1 - 2*th2 + 2*th3 - 2*th4 - 2*th5) - 1))/8; 

M=[ m11 m12 m13 m14 m15 ; m21 m22 m23 m24 m25 ; m31 m32 m33 m34 m35 ; m41 m42 m43 m44 m45  ;  m51 m52 m53 m54 m55  ] ;

% Gravity vector
G1=- g*m2*(crus*sin(th1) + (thigh*sin(th1 - th2))/2) - g*m3*((spine*sin(th1 - th2 + th3))/2 + crus*sin(th1) + thigh*sin(th1 - th2)) - g*m5*(spine*sin(th1 - th2 + th3) - bicep_arm*sin(th1 - th2 + th3 - th4) + crus*sin(th1) + thigh*sin(th1 - th2) - (forearm*sin(th1 - th2 + th3 - th4 - th5))/2) - g*m4*(spine*sin(th1 - th2 + th3) - (bicep_arm*sin(th1 - th2 + th3 - th4))/2 + crus*sin(th1) + thigh*sin(th1 - th2)) - (crus*g*m1*sin(th1))/2;
G2= g*m4*(spine*sin(th1 - th2 + th3) - (bicep_arm*sin(th1 - th2 + th3 - th4))/2 + thigh*sin(th1 - th2)) - g*m5*(bicep_arm*sin(th1 - th2 + th3 - th4) - spine*sin(th1 - th2 + th3) - thigh*sin(th1 - th2) + (forearm*sin(th1 - th2 + th3 - th4 - th5))/2) + g*m3*((spine*sin(th1 - th2 + th3))/2 + thigh*sin(th1 - th2)) + (g*m2*thigh*sin(th1 - th2))/2;
G3= g*m5*(bicep_arm*sin(th1 - th2 + th3 - th4) - spine*sin(th1 - th2 + th3) + (forearm*sin(th1 - th2 + th3 - th4 - th5))/2) + g*m4*((bicep_arm*sin(th1 - th2 + th3 - th4))/2 - spine*sin(th1 - th2 + th3)) - (g*m3*spine*sin(th1 - th2 + th3))/2;
G4=- g*m5*(bicep_arm*sin(th1 - th2 + th3 - th4) + (forearm*sin(th1 - th2 + th3 - th4 - th5))/2) - (bicep_arm*g*m4*sin(th1 - th2 + th3 - th4))/2;
G5=-(forearm*g*m5*sin(th1 - th2 + th3 - th4 - th5))/2;

G = [ G1;G2;G3;G4;G5];

%  Control variables error
error(:,i)= th_des-th(:,i);    % error

error_dot(:,i)=thdot_des-thddot(:,i);

%control law
tau(:,i)=M*(Kp.*error (:,i)+ Kd.*error_dot(:,i))+C+G;   % Controller equation   Dynamic Proportional Derivative with Gravity compensation
% tau(:,i)=[0;0;0;0;0];
% if tau(1,i)>200 
%     tau(1,i)=200;
% elseif tau(1,i)<-200
%         tau(1,i)=-200;
% end
%     if tau(2,i)>200 
%     tau(2,i)=200;
% elseif tau(2,i)<-200
%         tau(2,i)=-200;
%     end
%     if tau(3,i)>60 
%     tau(3,i)=60;
% elseif tau(3,i)<-60
%         tau(3,i)=-60;
%     end
%     if tau(4,i)>5 
%     tau(4,i)=5;
% elseif tau(4,i)<-5
%         tau(4,i)=-5;
%     end
%     if tau(5,i)>5 
%     tau(5,i)=5;
% elseif tau(5,i)<-5
%         tau(5,i)=-5;
%     end
    
    
      % System Behavior 
    
    thddot(:,i+1) = inv(M)*( tau(:,i)-G) - 0.8* thdot(:,i);  % introduced friction
     
    thdot(:,i+1) = thdot(:,i) + dt * thddot(:,i+1);  % Changes in this line index for identifying variables in the loop
    
    th(:,i+1)= th(:,i) + dt * thdot(:,i+1);   % similar changes related to indexing
    
    
    if th(1,i+1)>pi/2
    th(1,i+1)=pi/2;
elseif th(1,i+1)<pi/45
        th(1,i+1)=pi/45;
    end
    
     if th(2,i+1)>pi
    th(2,i+1)=pi;
elseif th(2,i+1)<pi/45
        th(2,i+1)=pi/45;
     end
    
      if th(2,i+1)>pi
    th(2,i+1)=pi;
elseif th(2,i+1)<pi/45
        th(2,i+1)=pi/45;
    end
    
    if th(3,i+1)>pi/2
    th(3,i+1)=pi/2;
elseif th(3,i+1)<pi/45
        th(3,i+1)=pi/45;
    end
    
     if th(4,i+1)>pi/2
    th(4,i+1)=pi/2;
elseif th(4,i+1)<pi/45
        th(4,i+1)=pi/45;
     end
    
       if th(5,i+1)>pi/2
    th(5,i+1)=pi/2;
elseif th(5,i+1)<pi/45
        th(5,i+1)=pi/45;
    end
   
   
    %plotting commands


 x_ankle= waist_width/2;
y_ankle= 0;
z_ankle= 0.05;

x_knee= x_ankle;
y_knee= y_ankle+crus*sin(th1);
z_knee= z_ankle+ crus*cos(th1);

x_waist= x_knee - waist_width/2;
y_waist= y_knee + thigh*sin(th1-th2);
z_waist= z_knee + thigh*cos(th1-th2);

x_shoulder= x_waist + shoulder/2;
y_shoulder= y_waist + spine*sin(th1-th2+th3);
z_shoulder= z_waist + spine*cos(th1-th2+th3);

x_elbow= x_shoulder;
y_elbow= y_shoulder + bicep_arm*sin(th1-th2+th3-th4-pi);
z_elbow= z_shoulder +bicep_arm*cos(th1-th2+th3-th4-pi);

x_wrist= x_elbow;
y_wrist= y_elbow + forearm*sin(th1-th2+th3-th4-th5-pi);
z_wrist= z_elbow + forearm*cos(th1-th2+th3-th4-th5-pi);
    
X=[ x_ankle x_knee x_waist x_shoulder x_elbow x_wrist];
Y=[ y_ankle y_knee y_waist y_shoulder y_elbow y_wrist];
Z=[ z_ankle z_knee z_waist z_shoulder z_elbow z_wrist];

a=plot(Y,Z, 'k*-');
set(a,'LineWidth',2);
axis([-2 2 -2 2]);
grid on;

pause(dt)
    
end

plot(t,error)
figure
plot(t,tau)
    
