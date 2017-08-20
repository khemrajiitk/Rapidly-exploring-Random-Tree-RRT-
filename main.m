function main
%obsticale 1
N = 40;
thetavec = linspace(0,pi,N);
phivec = linspace(0,2*pi,2*N);
[th, ph] = meshgrid(thetavec,phivec);
radius1 = 13; % should be your R(theta,phi) surface in general
cen1.x = 25;
cen1.y = 75;
cen1.z = 65;
global xo1;
xo1 = cen1.x +  radius1.*sin(th).*cos(ph);
global yo1;
yo1 = cen1.y + radius1.*sin(th).*sin(ph);
global zo1;
zo1 = cen1.z + radius1.*cos(th);
%storing obstical1 as obst1 object
obst1 = {};
obst1{1}=cen1;
obst1{2} =radius1;

%obsticale 2
N = 40;
thetavec = linspace(0,pi,N);
phivec = linspace(0,2*pi,2*N);
[th, ph] = meshgrid(thetavec,phivec);
radius2 = 19; % should be your R(theta,phi) surface in general
cen2.x = 81;
cen2.y = 30;
cen2.z = 55;
global xo2;
xo2 = cen2.x +  radius2.*sin(th).*cos(ph);
global yo2;
yo2 = cen2.y + radius2.*sin(th).*sin(ph);
global zo2;
zo2 = cen2.z + radius2.*cos(th);
%storing obstical2 as obst2 object
obst2 = {};
obst2{1}=cen2;
obst2{2} =radius2;


%obsticale 3
N = 40;
thetavec = linspace(0,pi,N);
phivec = linspace(0,2*pi,2*N);
[th, ph] = meshgrid(thetavec,phivec);
radius3 = 15; % should be your R(theta,phi) surface in general
cen3.x = 70;
cen3.y = 70;
cen3.z = 80;
global xo3;
xo3 = cen3.x +  radius3.*sin(th).*cos(ph);
global yo3;
yo3 = cen3.y + radius3.*sin(th).*sin(ph);
global zo3;
zo3 = cen3.z + radius3.*cos(th);
%storing obstical3 as obst3 object
obst3 = {};
obst3{1}=cen3;
obst3{2} =radius3;

%obsticale 4
N = 40;
thetavec = linspace(0,pi,N);
phivec = linspace(0,2*pi,2*N);
[th, ph] = meshgrid(thetavec,phivec);
radius4 = 17; % should be your R(theta,phi) surface in general
cen4.x = 32;
cen4.y = 27;
cen4.z = 30;
global xo4;
xo4 = cen4.x +  radius4.*sin(th).*cos(ph);
global yo4;
yo4 = cen4.y + radius4.*sin(th).*sin(ph);
global zo4;
zo4 = cen4.z + radius4.*cos(th);
%storing obstical4 as obst4 object
obst4 = {};
obst4{1} = cen4; 
obst4{2} =radius4;

%define a star point
%plotting start point
N = 40;
thetavec = linspace(0,pi,N);
phivec = linspace(0,2*pi,2*N);
[th, ph] = meshgrid(thetavec,phivec);
radiuss = 2; % should be your R(theta,phi) surface in general
start.x = 5;
start.y = 5;
start.z = 10;
global xos;
xos = start.x +  radiuss.*sin(th).*cos(ph);
global yos;
yos = start.y + radiuss.*sin(th).*sin(ph);
global zos;
zos = start.z + radiuss.*cos(th);

%define a goal point
%plotting goal point
N = 40;
thetavec = linspace(0,pi,N);
phivec = linspace(0,2*pi,2*N);
[th, ph] = meshgrid(thetavec,phivec);
radiusg = 2; % should be your R(theta,phi) surface in general
goal.x = 80;
goal.y = 90;
goal.z = 95;
global xog;
xog = goal.x +  radiusg.*sin(th).*cos(ph);
global yog;
yog = goal.y + radiusg.*sin(th).*sin(ph);
global zog;
zog = goal.z + radiusg.*cos(th);


figure;
surf(xo1,yo1,zo1);
hold on;
surf(xo2,yo2,zo2);
surf(xo3,yo3,zo3);
surf(xo4,yo4,zo4);
surf(xos,yos,zos);
surf(xog,yog,zog);

%storing all obstacle objects in one single object
obstacles = {obst1,obst2,obst3,obst4};

%setting parameters for potentials calculation
k=6;           %for attractive potential           
m=10000000000;          %for repulsive potential
step = 1.0;         % step size to simulate the speed of robot
current = start;     % current position of robot

%check distance between goal and robot
distance = (current.x-goal.x)^2+(current.y-goal.y)^2+(current.z-goal.z)^2;
while(distance > step)
	
	% calculating attractive and repulsive potentials & forces
	[xatt,yatt,zatt] = attractive_f(current,goal,k);  
	[xrep,yrep,zrep] = repulsive_f(current,obstacles,m);
	
	% resultant gradients in x and y direction
	dux = xatt-xrep;
	duy = yatt-yrep;
    duz = zatt-zrep;
	
	theta = atan2(duy,dux);
    phy = atan2(duz,(sqrt(dux^2+duy^2)));
	
	movx = step * cos(theta)*cos(phy);
	movy = step * sin(theta)*cos(phy);
    movz = step * sin(phy);
	
	%updating the current position
	current.x = current.x + movx;
	current.y = current.y + movy;
    current.z = current.z + movz;
	
	%plotting the updates position of robot
    N = 40;
    thetavec = linspace(0,pi,N);
    phivec = linspace(0,2*pi,2*N);
    [th, ph] = meshgrid(thetavec,phivec);
    r5 = 1; % should be your R(theta,phi) surface in general
    xor = current.x +  r5.*sin(th).*cos(ph);
    yor = current.y + r5.*sin(th).*sin(ph);
    zor = current.z + r5.*cos(th);
	surf(xor,yor,zor);
	% pause of 0.001s to visualize the progress
	pause(0.1);
	
	%check distance between goal and robot
	distance = (current.x-goal.x)^2+(current.y-goal.y)^2+(current.z - goal.z)^2;
  
end
end
%function for finding attactive force
function  [Yatx,Yaty,Yatz] = attractive_f(c,g,k)
	r = dist(c,g);
	theta = dir1(c,g);
    phy = dir2(c,g);
	Yatx=k*r*cos(theta)*cos(phy);
	Yaty=k*r*sin(theta)*cos(phy);
    Yatz=k*r*sin(phy);
end

%function for finding repulsive force
function [xrep,yrep,zrep]=repulsive_f(c,obstacles,m)
  n = size(obstacles,2);
  thres = 10;
  for i=1:n
	obs = obstacles{i};
	center = obs{1};
	radi = obs{2};
	theta = dir1(c,center);
    phy = dir2(c,center);
	dis=dist(c,center);
	Po = radi + thres;
	if dis > Po
      	x(i)=0;
      	y(i)=0;
        z(i)=0;
	else
   	rep=m*(1/dis-1/Po)^2*1/(dis^2);
   	x(i)=rep*cos(theta)*cos(phy);
   	y(i)=rep*sin(theta)*cos(phy);
    z(i)=rep*sin(phy);
	end
  end
   xrep=sum(x);
   yrep=sum(y);
   zrep=sum(z);
end
%Function for calculate directions
function distance = dist(c,g)
distance = sqrt((c.x-g.x)^2+(c.y-g.y)^2+(c.z-g.z)^2);
end
%Function for calculate robot direction
function theta = dir1(c,g)
theta = atan2((g.y-c.y),(g.x-c.x));
end
function phy = dir2(c,g)
phy = atan2((g.z-c.z),(sqrt((g.y-c.y)^2+(g.x-c.x)^2)));
end