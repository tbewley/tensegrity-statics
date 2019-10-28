% Static force analysis of a balloon rigged with 6 ground tethers  (Design A)
% By Thomas Bewley, JPL (on loan from UCSD)

clear; figure(1); clf;
Rb=2;     % Radius of balloon
Rp=1;     % Radius of payload
Rg=20;    % Radius of ground attachment points
Hb=34;    % Height of balloon
Hp=30;    % Height of payload (specified)

% Parameters for the 3-dimensional balloon configuration
S1=3;         % Symmetry (3-fold or 4-fold)
alpha=2*pi/3; % Opening angle for ground points

% Test loads on the balloon:
lift=200;     % (lift force of balloon, in Newtons)
weight=100;   % (weight of payload, in Newtons)
Bdisturb=18.5 % (horizontal force on balloon, in Newtons)
Pdisturb=4.5  % (horizontal force on payload, in Newtons)

% Note: This system needs no addtional constraints and has no soft nodes.

% END OF ADJUSTABLE INPUTS

dim=3;       % dimension of system (2D or 3D)
b=2*S1;      % number of bars
s=4*S1;      % number of strings
m=b+s;       % number of members
q=2*S1;      % number of free nodes Q
p=2*S1;      % number of fixed nodes P
n=q+p;       % number of nodes

% Now construct the 3-dimensional balloon configuration from above parameters
% Locations of free nodes are Q=Q_(dim x p) and fixed nodes are P=P_(dim x q)
for i=1:S1
    phi=pi+2*pi*(i-1)/S1; phi1=2*pi*(i-1)/S1;
    Q(1:3,i)   =[Rp*sin(phi);          Rp*cos(phi);         Hp]; % corners of payload
    Q(1:3,S1+i)=[Rb*sin(phi1);         Rb*cos(phi1);        Hb]; % anchor points on balloon
    P(1:3,i)   =[Rg*sin(phi1+alpha/2); Rg*cos(phi1+alpha/2); 0]; % outer ground points
    P(1:3,S1+i)=[Rg*sin(phi1-alpha/2); Rg*cos(phi1-alpha/2); 0];
end
C=zeros(m,n);                               % Connectivity matrix
for i=1:S1, j=mod(i,S1)+1; k=mod(i+1,S1)+1; 
  C(     i,   i)=1; C(     i,     j)=-1;    % bars modelling payload
  C(  S1+i,S1+i)=1; C(  S1+i,  S1+j)=-1;    % bars modelling balloon                                            
  C(2*S1+i,S1+i)=1; C(2*S1+i,     j)=-1;    % strings from balloon to payload (set 1)
  C(3*S1+i,S1+i)=1; C(3*S1+i,     k)=-1;    % strings from balloon to payload (set 2)
  C(4*S1+i,S1+i)=1; C(4*S1+i,2*S1+i)=-1;    % strings from balloon to ground (set 1)
  C(5*S1+i,S1+i)=1; C(5*S1+i,3*S1+i)=-1;    % strings from balloon to ground (set 2)
end

% Define applied external force U=U_(dim x p)
U(1,[1:S1])=0;      U(2,[1:S1])     =Pdisturb/S1; U(3,[1:S1])=-weight/S1;
U(1,[S1+1:2*S1])=0; U(2,[S1+1:2*S1])=Bdisturb/S1; U(3,[S1+1:2*S1])=lift/S1;

% Finally, solve for the forces at equilibrium.
[c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U);
tensegrity_plot(Q,P,C,b,s,U,V,false,1,4); grid on

% end script DesignA