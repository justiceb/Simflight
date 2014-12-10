
hold on
nabla=linspace(0,2*pi,50);
R=100;
for ii=2:length(xwp)
    xc=R.*cos(nabla);
    yc=R.*sin(nabla);
    for jj=1:50
        wpcirc(ii).x(jj)=xwp(ii)+xc(jj);
        wpcirc(ii).y(jj)=ywp(ii)+yc(jj);
    end
    plot(wpcirc(ii).y(:),wpcirc(ii).x(:),'-r')
    hold on
end
% axis equal