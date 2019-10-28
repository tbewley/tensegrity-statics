% Static force analysis of a balloon rigging (Design CliffA)
% By Thomas Bewley, UC San Diego (+ faculty fellow at JPL)

clear; figure(1); clf; config=1;
% view([-111.6,8]); axis equal 
switch config
    case 1
        Rb=80;          % Radius of balloon
        Rp=80;          % Radius of payload
        Hb=2200;        % Height of balloon
        Hp=1900;        % Height of payload 
        psi=pi/16;          % Extra rotation of payload & balloon
        off=[2300; -400];   % Center of payload / balloon system
        P(1:3,3)=[2785; -768.8; 1998];   % anchor points on ground
        P(1:3,1)=[1594; -623.8; 1340];
        P(1:3,2)=[2299;  121.6; 2018];
    otherwise
        Rb=2;       % Radius of balloon 
        Rp=2;       % Radius of payload
        Hb=34;      % Height of balloon
        Hp=20;      % Height of payload
        Hcliff=30;  % Height of cliff
        psi=0;      % Extra rotation of payload & balloon
        off=[0; 0]; % Center of payload / balloon system
        Rg=20;      % Radius of ground attachment points
        phi=4*pi/3; P(1:3,1)=[Rg*sin(phi); Rg*cos(phi); 0];  % anchor points on ground
        phi=0;      P(1:3,2)=[Rg*sin(phi); Rg*cos(phi); Hcliff];
        phi=2*pi/3; P(1:3,3)=[Rg*sin(phi); Rg*cos(phi); Hcliff];
end

% Test loads on the balloon:
lift=200;     % (lift force of balloon, in Newtons)
weight=100;   % (weight of payload, in Newtons)
Bdisturb=18.5 % (horizontal force on balloon, in Newtons)
Pdisturb=4.5    % (horizontal force on payload, in Newtons)

% Note: This system needs no addtional constraints and has no soft nodes.

% END OF ADJUSTABLE INPUTS

% Free [Q=Q_(dim x q)] and fixed [Q=Q_(dim x q)] node locations
for i=1:3
    phi1=pi+2*pi*(i-1)/3+psi; phi=2*pi*(i-1)/3+psi;
    Q(1:3,     i)=[Rp*sin(phi)+off(1);  Rp*cos(phi)+off(2);  Hp]; % corners of payload
    Q(1:3,  3+i)=[Rb*sin(phi1)+off(1); Rb*cos(phi1)+off(2); Hb]; % anchor points on balloon
end
[dim,q]=size(Q); p=size(P,2); n=q+p; 

b=6; s=13;  m=b+s;                  % Connectivity matrix
for i=1:3, j=mod(i,3)+1; k=mod(i+1,3)+1; 
  C(   i,  i)=1; C(   i,  j)=-1;    % bars modelling payload
  C( 3+i,3+i)=1; C( 3+i,3+j)=-1;    % bars modelling balloon                                         
  C( 6+i,3+i)=1; C( 6+i,  j)=-1;    % strings from balloon to payload (set 1)
  C( 9+i,3+i)=1; C( 9+i,  k)=-1;    % strings from balloon to payload (set 2)
  C(12+i,3+i)=1; C(12+i,6+i)=-1;    % strings from balloon to ground (set 1)
  C(15+i,3+i)=1; C(15+i,6+k)=-1;    % strings from balloon to ground (set 2)
end
C(19,3)=1; C(19,7)=-1;              % string from payload to ground

[x,y,z] = ellipsoid(0,0,Hb,Rb,Rb,Rb*2/3,30); surf(x+off(1),y+off(2),z); hold on

% Define applied external force U=U_(dim x p)
U(1,[1:3])=0; U(2,[1:3])=Pdisturb/3; U(3,[1:3])=-weight/3;
U(1,[4:6])=0; U(2,[4:6])=Bdisturb/3; U(3,[4:6])=lift/3;

% Solve for the forces at equilibrium, and plot
[c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U);
tensegrity_plot(Q,P,C,b,s,U,V,false,0.5,4); grid on

% end script CliffA
