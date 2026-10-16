function plotResults(simdata,n)

    R2D = 180 / pi;     % radians to degrees

    % simdata = ...
    % [t psi chi beta phi p r delta_a delta_a_c delta_r delta_r_c phi_c chi_c]
    t = simdata(:,1); 
    psi = R2D * simdata(:,2); 
    chi = R2D * simdata(:,3); 
    beta = R2D * simdata(:,4); 
    phi = R2D * simdata(:,5); 
    p = R2D * simdata(:,6); 
    r = R2D * simdata(:,7); 
    delta_a = R2D * simdata(:,8);
    delta_a_c = R2D * simdata(:,9);
    delta_r = R2D * simdata(:,10);
    delta_r_c = R2D * simdata(:,11);    
    phi_c= R2D * simdata(:,12);
    chi_c = R2D * simdata(:,13);

    clf; 
    f = figure(gcf);
    f.Position = [100 100 1200 800];

    if n == 0

        subplot(421),plot(t,phi,'-b')
        xlabel('time (s)'),grid
        legend('Roll angle [deg]');
    
        subplot(422),plot(t,p,'-b')
        xlabel('time (s)'),grid
        legend('Roll rate [deg/s]');
    
        subplot(423),plot(t,psi,'-b')
        xlabel('time (s)'),grid
        legend('Yaw angle [deg]');
        
        subplot(424),plot(t,r,'-b')
        xlabel('time (s)'),grid
        legend('Yaw rate [deg/s]');
        
        subplot(425),plot(t,chi,'-b')
        xlabel('time (s)'),grid
        legend('Course angle [deg]');
        
        subplot(426),plot(t,delta_a,'-b',t,delta_a_c,'-r')
        xlabel('time (s)'),grid
        legend('Actual aileron [deg]','Commanded aileron [deg]');

        subplot(427),plot(t,beta,'-b')
        xlabel('time (s)'),grid
        legend('Sideslip angle [deg]');
        
        subplot(428),plot(t,delta_r,'-b',t,delta_r_c,'-r')
        xlabel('time (s)'),grid
        legend('Actual rudder angle [deg]','Commanded rudder angle [deg]');        
        
        sgtitle('Aileron Step Response')

    elseif n == 1

        subplot(421),plot(t,phi,'-b',t,phi_c,'-r')
        xlabel('time (s)'),grid
        legend('Roll angle [deg]','Roll angle command [deg]');
    
        subplot(422),plot(t,p,'-b')
        xlabel('time (s)'),grid
        legend('Roll rate [deg/s]');
    
        subplot(423),plot(t,psi,'-b')
        xlabel('time (s)'),grid
        legend('Yaw angle [deg]');
        
        subplot(424),plot(t,r,'-b')
        xlabel('time (s)'),grid
        legend('Yaw rate [deg/s]');
        
        subplot(425),plot(t,chi,'-b')
        xlabel('time (s)'),grid
        legend('Course angle [deg]');
        
        subplot(426),plot(t,delta_a,'-b',t,delta_a_c,'-r')
        xlabel('time (s)'),grid
        legend('Actual aileron [deg]','Commanded aileron [deg]');

        subplot(427),plot(t,beta,'-b')
        xlabel('time (s)'),grid
        legend('Sideslip angle [deg]');
        
        subplot(428),plot(t,delta_r,'-b',t,delta_r_c,'-r')
        xlabel('time (s)'),grid
        legend('Actual rudder angle [deg]','Commanded rudder angle [deg]');          

        sgtitle('Roll Attitude Hold')

    elseif n == 2

        subplot(421),plot(t,phi,'-b',t,phi_c,'-r')
        xlabel('time (s)'),grid
        legend('Roll angle [deg]','Roll angle command [deg]');
    
        subplot(422),plot(t,p,'-b')
        xlabel('time (s)'),grid
        legend('Roll rate [deg/s]');
    
        subplot(423),plot(t,psi,'-b')
        xlabel('time (s)'),grid
        legend('Yaw angle [deg]');
        
        subplot(424),plot(t,r,'-b')
        xlabel('time (s)'),grid
        legend('Yaw rate [deg/s]');
        
        subplot(425),plot(t,chi,'-b',t,chi_c,'-r')
        xlabel('time (s)'),grid
        legend('Course angle [deg]','Course angle command [deg]');
        
        subplot(426),plot(t,delta_a,'-b',t,delta_a_c,'-r')
        xlabel('time (s)'),grid
        legend('Actual aileron [deg]','Commanded aileron [deg]');

        subplot(427),plot(t,beta,'-b')
        xlabel('time (s)'),grid
        legend('Sideslip angle [deg]');
        
        subplot(428),plot(t,delta_r,'-b',t,delta_r_c,'-r')
        xlabel('time (s)'),grid
        legend('Actual rudder angle [deg]','Commanded rudder angle [deg]');        
        
        sgtitle('Course Hold')

    end
    
    set(findall(gcf,'type','line'),'linewidth',2)
    
end
