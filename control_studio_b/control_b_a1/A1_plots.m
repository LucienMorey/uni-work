% Here, you have an example on how to create plots for your report.
% Modify it or create new ones according to what you want to present.
 
% Transient response at the instant t=40(ms)
time_ms = 1000*time; %time in miliseconds
figure(1)
subplot(311)
plot(time_ms,vi,'LineWidth',1)
axis([0 80 -110 110])
grid
ylabel('Input voltage $v_i(t)$','fontsize',12,'interpreter','latex')
subplot(312)
plot(time_ms,io_ref*Ro,'--k',time_ms,vo,'LineWidth',1.5)
axis([0 80 -15 15])
grid
legend('$v^{\star}_o(k)$','$v_o(t)$','fontsize',12,'interpreter','latex')
ylabel('Output voltage $v_o(t)$','fontsize',12,'interpreter','latex')
subplot(313)
plot(time_ms,vo-io_ref*Ro,'LineWidth',1.5)
axis([0 80 -15 15])
grid
ylabel('Output Error $e_{vo}(t)$','fontsize',12,'interpreter','latex')
xlabel('Time (ms)','fontsize',12,'interpreter','latex')

figure(2)
subplot(311)
plot(time_ms,io,'--k',time_ms,io_hat,'LineWidth',1)
axis([0 80 -110 110])
grid
ylabel('Output Current $i_o(t)$','fontsize',12,'interpreter','latex')
legend('$i_o(t)$','$\widehat{i_o}(k)$','fontsize',12,'interpreter','latex')
subplot(312)
plot(time_ms,iL,'--k',time_ms,iL_hat,'LineWidth',1.5)
axis([0 80 -110 110])
grid
legend('$i_l(t)$','$\widehat{i_l}(k)$','fontsize',12,'interpreter','latex')
ylabel('Inductor Current $i_l(t)$','fontsize',12,'interpreter','latex')
subplot(313)
plot(time_ms,io-io_hat,'--k',time_ms,iL-iL_hat,'LineWidth',1.5)
axis([0 80 -110 110])
grid
legend('$e_{io}(t)$','$e_{il}(t)$','fontsize',12,'interpreter','latex')
ylabel('Estimation Error $x(k)- \widehat{x}(k)$','fontsize',12,'interpreter','latex')
xlabel('Time (ms)','fontsize',12,'interpreter','latex')