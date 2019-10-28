function tensegrity_plot(Q,P,C,b,s,U,V)
[dim,q]=size(Q); p=size(P,2); N=[Q P]; [M,I]=max(C'); [M,J]=min(C'); hold on
if dim==2
    for k=1:b,     plot([N(1,I(k)) N(1,J(k))],[N(2,I(k)) N(2,J(k))],'r-'), end
    for k=b+1:b+s, plot([N(1,I(k)) N(1,J(k))],[N(2,I(k)) N(2,J(k))],'b-'), end
else
    for k=1:b,     plot3([N(1,I(k)) N(1,J(k))],[N(2,I(k)) N(2,J(k))],[N(3,I(k)) N(3,J(k))],'r-','LineWidth',2), end
    for k=b+1:b+s, plot3([N(1,I(k)) N(1,J(k))],[N(2,I(k)) N(2,J(k))],[N(3,I(k)) N(3,J(k))],'k-','LineWidth',2), end
    view(19,14)
end
grid, axis equal, set(gca,'YTickLabel',[]); set(gca,'XTickLabel',[]); set(gca,'ZTickLabel',[]);

range(1)=max(N(:,1))-min(N(:,1)); range(2)=max(N(:,2))-min(N(:,2));
t=(0:1/3:1)'*2*pi; x=sin(t); y=cos(t);
if dim==2
    y=y-1;
else
    X=[0 x(1) x(2) x(3)]';  range(3)=max(N(:,3))-min(N(:,3));
    Y=[0 y(1) y(2) y(3)]';  h=sqrt(6)/3;
    Z=[0 -h -h -h]';        T=[1 2 3; 1 2 4; 2 3 4; 1 3 4];
end
s=max(range)*.02;
for i=1:p   % Draw a symbol below each ground point.
    if dim==2, fill(s*x+P(1,i),s*y+P(2,i),'r')         % An equilateral triangle
    else trisurf(T,s*X+P(1,i),s*Y+P(2,i),s*Z+P(3,i),0) % A tetrahedron
    end
end
outer_arrows=true;
if nargin>5
    for i=1:q, force_mag(i)=norm(U(:,i)); end, s=s*10/max(force_mag);
    for i=1:q, if dim==3,
            if outer_arrows
               quiver3(Q(1,i)-s*U(1,i),Q(2,i)-s*U(2,i),Q(3,i)-s*U(3,i),s*U(1,i),s*U(2,i),s*U(3,i),0);
            else
               quiver3(Q(1,i),Q(2,i),Q(3,i),s*U(1,i),s*U(2,i),s*U(3,i),0);
            end
        else
            if outer_arrows
               quiver(Q(1,i)-s*U(1,i),Q(2,i)-s*U(2,i),s*U(1,i),s*U(2,i),0);
            else
               quiver(Q(1,i),Q(2,i),s*U(1,i),s*U(2,i),0);
            end
        end, end
end
if nargin>6
    for i=1:p, if dim==3,
            quiver3(P(1,i),P(2,i),P(3,i),s*V(1,i),s*V(2,i),s*V(3,i),0);
        else
            quiver(P(1,i),P(2,i),s*V(1,i),s*V(2,i),0);
        end, end
end
