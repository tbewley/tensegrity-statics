% Static force analysis of 2D TBar (Fig 3.3 of Skelton & de Oliviera 2009)
% By Thomas Bewley, UC San Diego (+ faculty fellow at JPL)
clear; clf; figure(1);

% Free [Q=Q_(dim x q)] and fixed [Q=Q_(dim x q)] node locations
Q(:,1)=[0; 0];
Q(:,2)=[1; 0];
Q(:,3)=[2; 0];
Q(:,4)=[1; .5];
Q(:,5)=[1; -.5]; P=[];
[dim,q]=size(Q), p=size(P,2), n=q+p; 

% Connectivity matrix
C(  1,1)=1; C(  1,2)=-1;       % bars 
C(  2,2)=1; C(  2,3)=-1;    
C(  3,2)=1; C(  3,4)=-1;                                              
C(  4,2)=1; C(  4,5)=-1; b=4;                                           
C(b+1,1)=1; C(b+1,4)=-1;       % strings 
C(b+2,4)=1; C(b+2,3)=-1;    
C(b+3,3)=1; C(b+3,5)=-1;    
C(b+4,5)=1; C(b+4,1)=-1; s=4; m=b+s;

% Applied external force U=U_(dim x q)
U(1:dim,1:4)=0; U(1,1)=1; U(1,3)=-1.6; U(1,4)=.3; U(1,5)=.3;

% Solve for the forces at equilibrium, and plot
[c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U);
tensegrity_plot(Q,P,C,b,s,U,V,true,2.0); grid on;

% end script TBar
