% Static force analysis of a balloon rigged with 12 ground tethers (Design B)
% By Thomas Bewley, JPL (on loan from UCSD)
clear; clf; 
% Note: This system needs no addtional constraints and has no soft nodes.

dim=2;    % dimension of system
b=2;      % number of bars
s=4;      % number of strings
m=b+s;    % number of members
q=4;      % number of free nodes Q
p=0;      % number of fixed nodes P
n=q+p;    % number of nodes

% Now construct the 3-dimensional balloon configuration from above parameters
% Locations of free nodes are P=P_(dim x p) and fixed nodes are Q=Q_(dim x q)
Q(:,1)=[0; 0];
Q(:,2)=[2; 0];
Q(:,3)=[1; .5];
Q(:,4)=[1; -.5]; P=[];
C=zeros(m,n);                               % Connectivity matrix
C(1,1)=1; C(1,2)=-1;    % bars 
C(2,3)=1; C(2,4)=-1;                                              
C(3,1)=1; C(3,3)=-1;    % strings 
C(4,3)=1; C(4,2)=-1;    
C(5,2)=1; C(5,4)=-1;    
C(6,4)=1; C(6,1)=-1;    

% Define applied external force U=U_(dim x q)
U(1:dim,1:4)=0; U(1,1)=1; U(1,2)=-1.6; U(1,3)=.3; U(1,4)=.3;

% Finally, solve for the forces at equilibrium.
[c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U);
figure(2); tensegrity_plot(Q,P,C,b,s,U,V); axis equal

% end script 2D
