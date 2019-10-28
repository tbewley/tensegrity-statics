% Static force analysis of a balloon rigged with 12 ground tethers  (Design C)
% By Thomas Bewley, JPL (on loan from UCSD)

clear; figure(1); clf;
Rb=2;     % Radius of balloon
Rp=1;     % Radius of payload
Rg=20;    % Radius of outer-most ground attachment points
Hb=36;    % Height of balloon
Hp=30;    % Height of payload  (specified)
Hc=32.5;  % Height of convergence point

% Parameters for the 3-dimensional balloon configuration
S1=3;          % Symmetry of structure (3-fold or 4-fold)
alpha=2*pi/3;  % Opening angle

% Below are a few parameters specific to the balloon problem:
lift=200;     % (lift force of balloon, in Newtons)
weight=100;   % (weight of payload, in Newtons)
Bdisturb=-18.5 % (horizontal force on balloon, in Newtons)
Pdisturb=-4.5  % (horizontal force on payload, in Newtons)

% Note: This system needs 1 additional constraint, in addition to accounting for the
% pullies by taking the tensions in the upper pyramid equal, and has no soft nodes.
% For Bdisturb=0 and Pdisturb up to 33, take gamma7=.001.
% For Pdisturb=0 and Bdisturb up to 24, take gamma1=.001.
% For Bdisturb and Pdisturb both nonzero, use somewhere in between:
% for exampe, for Bdisturb=22.3 and Pdisturb=3, take gamma3==.028.
% If force is from a different side, need to set the tension of one of the
% other strings from payload to ground

extra_constraints=',x19==x20,x20==x21'

% END OF ADJUSTABLE INPUTS

dim=3;    % dimension of system (2D or 3D)
b=2*S1;   % number of bars
s=6*S1;   % number of strings
m=b+s     % number of members
q=2*S1+1; % number of free nodes Q
p=2*S1;   % number of fixed nodes P
n=q+p;    % number of nodes

% Now construct the 3-dimensional balloon configuration from above parameters
% Locations of free nodes are P=P_(dim x p) and fixed nodes are Q=Q_(dim x q)
for i=1:S1                                   % SUBSTRUCTURE
    phi=2*pi*(i-1)/S1;  
    Q(1:3,i)   =[Rp*sin(phi);         Rp*cos(phi);        Hp]; % corners of payload
    Q(1:3,S1+i)=[Rb*sin(phi);         Rb*cos(phi);        Hb]; % anchor points on balloon
    P(1:3,i)   =[Rg*sin(phi+alpha/2); Rg*cos(phi+alpha/2); 0]; % ground points (set 1)
    P(1:3,S1+i)=[Rg*sin(phi-alpha/2); Rg*cos(phi-alpha/2); 0]; % ground points (set 2)
end
Q(1:3,2*S1+1)=[0; 0; Hc]; % Convergence point
C=zeros(m,n);             % Connectivity matrix
for i=1:S1,         j=mod(i,S1)+1;          % BARS:
  C(   i,   i)=1; C(   i,   j)=-1;          % bars modelling payload
  C(S1+i,S1+i)=1; C(S1+i,S1+j)=-1;          % bars modelling balloon
end
for i=1:S1, j=mod(i,S1)+1;                  % STRINGS:
  C(2*S1+i,   i)=1; C(2*S1+i,2*S1+1+i)=-1;  % strings from payload to ground (set 1)
  C(3*S1+i,   i)=1; C(3*S1+i,3*S1+1+i)=-1;  % strings from payload to ground (set 2)
  C(4*S1+i,S1+i)=1; C(4*S1+i,2*S1+1+i)=-1;  % strings from balloon to ground (set 1)
  C(5*S1+i,S1+i)=1; C(5*S1+i,3*S1+1+i)=-1;  % strings from balloon to ground (set 2)
  C(6*S1+i,S1+i)=1; C(6*S1+i,2*S1+1  )=-1;  % from balloon to convergence point
  C(7*S1+i,   i)=1; C(7*S1+i,2*S1+1  )=-1;  % from payload to convergence point
end

% Define applied external force U=U_(dim x p)
U(1,[1:S1])=0;       U(2,[1:S1])     =Pdisturb/S1;  U(3,[1:S1])=-weight/S1;
U(1,[S1+1:2*S1])=0;  U(2,[S1+1:2*S1])=Bdisturb/S1;  U(3,[S1+1:2*S1])=lift/S1;
U([1:3],2*S1+1)=0;

% Finally, solve for the forces at equilibrium.
[c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U,extra_constraints);
tensegrity_plot(Q,P,C,b,s,U,V,false,1,3); grid on

% end script DesignC