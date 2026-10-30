function n_c = open_loop_speed_control(U_ref)

    % Ship parameters
    m = 17.0677e6;          % mass (kg)
    Xudot = -8.9830e5;
    T1 = 20;

    % Propeller parameters
    rho = 1025;
    Dia = 3.3;
    t_thr = 0.05;

    % Propeller thrust coefficient
    persistent KT
    if isempty(KT)
        [KT,~] = wageningen(0, 1.5, 0.65, 4);
    end

    % Linear surge damping coefficient
    Xu = -(m - Xudot)/T1;

    % Required thrust for desired cruise speed
    Td = Xu * U_ref / (t_thr - 1);

    % Required propeller shaft speed
    n_c = sqrt(Td / (rho * Dia^4 * KT));

end