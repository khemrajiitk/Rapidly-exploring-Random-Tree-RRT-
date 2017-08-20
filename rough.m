function main
%obstical1
radius1 = 9;
cen1.x = 25;
cen1.y= 75;
t=0:0.01:2*pi;
global xo1;
xo1 = cen1.x+cos(t)*radius1;
global yo1;
yo1 = cen1.y+sin(t)*radius1;
%storing obstical1 as obst1 object
obst1 = {};
obst1{1}=cen1;
obst1{2} =radius1;
 
%obstical2
radius2 = 8;
cen2.x = 81;
cen2.y= 30;
t=0:0.01:2*pi;
global xo2;
xo2 = cen2.x+cos(t)*radius2;
global yo2;
yo2 = cen2.y+sin(t)*radius2;
%storing obstical2 as obst2 object
obst2 = {};
obst2{1}=cen2;
obst2{2} =radius2;
 
%obstical3
radius3 = 15;
cen3.x = 70;
cen3.y= 70;
t=0:0.01:2*pi;
global xo3;
xo3 = cen3.x+cos(t)*radius3;
global yo3;
yo3 = cen3.y+sin(t)*radius3;
%storing obstical3 as obst3 object
obst3 = {};
obst3{1}=cen3;
obst3{2} =radius3;
 
%obstical4
radius4 = 17;
cen4.x = 32;
cen4.y= 27;
t=0:0.01:2*pi;
global xo4;
xo4 = cen4.x+cos(t)*radius4;
global yo4;
yo4 = cen4.y+sin(t)*radius4;
%storing obstical4 as obst4 object
obst4 = {};
obst4{1} = cen4; 
obst4{2} =radius4;
 
%Plotting the workspace figure
%f1 = figure;
%rw = 10;
%xw = [0 100 100 0 0];
%xy = [0 0 100 100 0];
%plot(xw,xy),xlabel('Workspace'),axis([-5 105 -5 105 -5 105]);
hold on;
 
%define a goal point
goal.x = 95;
goal.y = 80;
%plotting goal point
r4 = 1.5;
cenx = goal.x;
ceny=goal.y;
t=0:0.01:2*pi;
xog = cenx+cos(t)*r4;
yog = ceny+sin(t)*r4;
patch(xog,yog,'m');
 
%define a start point
start.x = 1;
start.y = 1;
%plotting start point
r5 = 1.0;
cenx = start.x;
ceny=start.y;
t=0:0.01:2*pi;
xos = cenx+cos(t)*r5;
yos = ceny+sin(t)*r5;
patch(xos,yos,'c');
 
%drawing obstacles
patch(xo1,yo1,'r');
patch(xo2,yo2,'g');
patch(xo3,yo3,'b');
patch(xo4,yo4,'r');
 
%storing all obstacle objects in one single object
obstacles = {obst1,obst2,obst3,obst4};
 
%setting parameters for potentials calculation
k=10;           %for attractive potential           
m=10000000000;          %for repulsive potential
step = 1.5;         % step size to simulate the speed of robot
current = start;     % current position of robot
 
%check distance between goal and robot
distance = (current.x-goal.x)^2+(current.y-goal.y)^2;
 





while(distance > step)
    
    % calculating attractive and repulsive potentials & forces
    [xatt,yatt] = attractive_f(current,goal,k);  
    [xrep,yrep] = repulsive_f(current,obstacles,m);
    
    % resultant gradients in x and y direction
    dux = xatt-xrep;
    duy = yatt-yrep;
    direction = atan2(duy,dux);
    
    movx = step * cos(direction);
    movy = step * sin(direction);
    
    %updating the current position
    current.x = current.x + movx;
    current.y = current.y + movy;
    
    %plotting the updates position of robot
    xor = current.x+cos(t)*r5;
    yor = current.y+sin(t)*r5;
    patch(xor,yor,'c');
    % pause of 0.001s to visualize the progress
    pause(0.001);
    %check distance between goal and robot
    distance = (current.x-goal.x)^2+(current.y-goal.y)^2;
end
end
%End of main function

%function for finding attactive force
function  [Yatx,Yaty] = attractive_f(c,g,k)
    r = dist(c,g);
    angle = dir(c,g);
    Yatx=k*r*cos(angle);
    Yaty=k*r*sin(angle);
end
 
%function for finding repulsive force
function [xrep,yrep]=repulsive_f(c,obstacles,m)
  n = size(obstacles,2);
  thres = 10;
  for i=1:n
    obs = obstacles{i};
    center = obs{1};
    radi = obs{2};
    angle = dir(c,center);
    dis=dist(c,center);
    Po = radi + thres;
    if dis > Po
        x(i)=0;
        y(i)=0;
    else
    rep=m*(1/dis-1/Po)^2*1/(dis^2);
    x(i)=rep*cos(angle);
    y(i)=rep*sin(angle);
    end
  end
   xrep=sum(x);
   yrep=sum(y);
end
%Function for calculate directions
function distance = dist(c,g)
distance = sqrt((c.x-g.x)^2+(c.y-g.y)^2);
end

%Function for calculate robot direction
function direction = dir(c,g)
direction = atan2((g.y-c.y),(g.x-c.x));
end
