
figure

plot(yaircraft(:,11),yaircraft(:,10),'b-.')
hold on
plot(ywp,xwp,'g:')
xlabel('East (ft)')
ylabel('North (ft)')
text(ywp(1),xwp(1),'Start','FontSize',12)
text(ywp(end),xwp(end),'End','FontSize',12)
axis equal

plot_circ

if sqrt(Vwx^2+Vwy^2)>0
    plot_wind
end

% plot_con