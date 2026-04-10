% Quadrotor simulation in pure MATLAB
clear; clc; close all;

% Simulation settings
dt = 0.005;
t_end = 60;
time = 0:dt:t_end;
N = numel(time);

% Drone physical parameters
m = 1.2;   g = 9.81;
Ixx = 0.0221; Iyy = 0.0221; Izz = 0.0366;
L = 0.23;
kf = 1.91e-6;   km = 2.60e-7;
Cd = 0.12;
w_min = 900;   w_max = 7800;
tau_mot = 0.015;
w_hover = sqrt(m*g/(4*kf));
T_min = 0.2*m*g;   T_max = 2.8*m*g;

% Cascade PID gains
Kp_pos = [1.2;1.2;2.0]; Ki_pos = [0.02;0.02;0.3]; Kd_pos = [0.6;0.6;0.8];
Kp_vel = [3.0;3.0;4.5]; Ki_vel = [0.08;0.08;0.8]; Kd_vel = [0;0;0];
Kp_att = [9.5;9.5;5.5]; Ki_att = [0.06;0.06;0.06]; Kd_att = [3.2;3.2;1.8];
Kp_rate = [0.65;0.65;0.42]; Ki_rate = [0.025;0.025;0.012]; Kd_rate = [0.022;0.022;0.012];
MAX_TILT = deg2rad(22); MAX_VEL_H = 3.5; MAX_VEL_V = 2.0; MAX_YAW_R = deg2rad(60);
IW = 0.6;

% Waypoint mission (x,y,z,yaw_rad,hold_time)
WP = [0,0,0,0,1;
      0,0,10,0,3;
      15,0,10,0,2;
      15,15,10,90,2;
      0,15,10,180,2;
      0,0,10,270,2;
      0,0,10,0,2;
      0,0,0,0,3];
WP(:,4) = deg2rad(WP(:,4));
nWP = size(WP,1);

% Initial state
pos = zeros(3,1); vel = zeros(3,1); ang = zeros(3,1); omg = zeros(3,1);
w_mot = w_hover*ones(4,1);
ip = zeros(3,1); ep0 = zeros(3,1);
iv = zeros(3,1); ev0 = zeros(3,1);
ia = zeros(3,1); ea0 = zeros(3,1);
ir = zeros(3,1); er0 = zeros(3,1);
wp_idx = 1; wp_timer = 0;
tgt_pos = WP(1,1:3)'; tgt_yaw = WP(1,4);
ref_pos = WP(1,1:3)'; ref_yaw = WP(1,4);

% Log arrays
LP = zeros(3,N); LV = zeros(3,N); LA = zeros(3,N);
LR = zeros(4,N); LTP = zeros(3,N);

% Build figure and 3D model
BG = [0.04,0.04,0.06];
fig = figure('Name','Quadrotor Simulation','Color',BG,'Position',[20,20,1700,900]);

% 3D view
ax3 = axes('Parent',fig,'Position',[0.01,0.04,0.55,0.92]);
hold(ax3,'on'); grid(ax3,'on'); axis(ax3,'equal');
set(ax3,'Color',[0.03,0.03,0.05],'XColor',[0.5,0.6,0.7],'YColor',[0.5,0.6,0.7],...
    'ZColor',[0.5,0.6,0.7],'GridColor',[0.10,0.12,0.16],'GridAlpha',1,...
    'FontSize',8,'XLim',[-4,22],'YLim',[-4,22],'ZLim',[0,15],'Projection','perspective');
xlabel(ax3,'X [m]'); ylabel(ax3,'Y [m]'); zlabel(ax3,'Z [m]');
view(ax3,42,22);
camlight(ax3,'headlight'); lighting(ax3,'gouraud');

% Ground grid
[gx,gy] = meshgrid(-2:2:20,-2:2:20);
mesh(ax3,gx,gy,zeros(size(gx)),'EdgeColor',[0.08,0.10,0.13],'FaceAlpha',0,'LineWidth',0.4);

% Waypoints preview
plot3(ax3,WP(:,1),WP(:,2),WP(:,3),'--','Color',[0.35,0.35,0.5],'LineWidth',0.9);
scatter3(ax3,WP(:,1),WP(:,2),WP(:,3),60,'w','filled','MarkerEdgeColor',[0.5,0.6,0.7]);
for k = 1:nWP
    text(ax3,WP(k,1)+0.4,WP(k,2)+0.4,WP(k,3)+0.5,sprintf('WP%d',k),'Color',[0.6,0.7,0.8],'FontSize',7);
end
h_trail = animatedline(ax3,'Color',[0.2,0.85,1.0],'LineWidth',1.8,'MaximumNumPoints',4000);

% Drone visual parts
SC = 0.55;   arm_r = L*SC;
ARM_DIRS = [1,0,0; 0,-1,0; -1,0,0; 0,1,0];
nac_offsets = (ARM_DIRS*arm_r)';

bv = make_box_verts(0.16*SC,0.055*SC,0.05*SC);
h_body = patch(ax3,'Vertices',bv,'Faces',box_faces(),'FaceColor',[0.18,0.22,0.28],...
               'EdgeColor',[0.35,0.45,0.55],'FaceLighting','gouraud',...
               'AmbientStrength',0.4,'DiffuseStrength',0.75,'SpecularStrength',0.5);

ARM_COLS = [1,0.3,0.3; 0.3,0.6,1; 1,0.3,0.3; 0.3,0.6,1];
arm_v0 = cell(4,1); h_arm = gobjects(4,1);
for k = 1:4
    arm_v0{k} = make_arm_verts(ARM_DIRS(k,:), arm_r, 0.018*SC);
    h_arm(k) = patch(ax3,'Vertices',arm_v0{k},'Faces',box_faces(),'FaceColor',ARM_COLS(k,:),...
                     'EdgeColor','none','FaceLighting','gouraud','AmbientStrength',0.35,'DiffuseStrength',0.8);
end

nac_r = 0.055*SC; nac_h = 0.04*SC; nseg_c = 14;
nac_v0 = cell(4,1); h_nac = gobjects(4,1);
for k = 1:4
    [nac_v0{k}, nf] = make_cylinder(nac_offsets(:,k), nac_r, nac_h, nseg_c);
    h_nac(k) = patch(ax3,'Vertices',nac_v0{k},'Faces',nf,'FaceColor',[0.55,0.60,0.65],...
                     'EdgeColor','none','FaceLighting','gouraud','AmbientStrength',0.45);
end

rot_r = 0.22*SC; nseg_r = 28;
rotor_v0 = cell(4,1); h_rot = gobjects(4,1); rphase = zeros(4,1);
ROTOR_COLS = [1,0.85,0.25; 0.3,1,0.45; 1,0.85,0.25; 0.3,1,0.45];
for k = 1:4
    ctr_r = nac_offsets(:,k) + [0;0;nac_h*0.5+0.005];
    [rotor_v0{k}, rf] = make_disc(ctr_r, rot_r, nseg_r);
    h_rot(k) = patch(ax3,'Vertices',rotor_v0{k},'Faces',rf,'FaceColor',ROTOR_COLS(k,:),...
                     'EdgeColor','none','FaceAlpha',0.45,'FaceLighting','gouraud');
end

LEG_DIR = [1,1; -1,1; -1,-1; 1,-1]/sqrt(2);
leg_v0 = cell(4,1); h_leg = gobjects(4,1);
for k = 1:4
    lbase = [LEG_DIR(k,1); LEG_DIR(k,2); 0]*arm_r*0.55;
    ltip = lbase + [0;0;-0.08*SC];
    leg_v0{k} = make_leg_verts(lbase,ltip,0.008*SC);
    h_leg(k) = patch(ax3,'Vertices',leg_v0{k},'Faces',box_faces(),'FaceColor',[0.28,0.32,0.38],...
                     'EdgeColor','none','FaceLighting','gouraud');
end

h_shadow = patch(ax3,[0,0,0,0],[0,0,0,0],[0.001,0.001,0.001,0.001],[0.1,0.1,0.15],...
                 'EdgeColor','none','FaceAlpha',0.5);
h_vvec = quiver3(ax3,0,0,0,0,0,0,'Color',[0.25,1,0.5],'LineWidth',2,'MaxHeadSize',0.7,'AutoScale','off');
h_hud = annotation(fig,'textbox',[0.01,0.01,0.27,0.135],'String','','Color',[0.3,1,0.6],...
                   'BackgroundColor',[0,0,0,0.65],'EdgeColor',[0.2,0.4,0.3],'FontName','Courier New',...
                   'FontSize',8.5,'VerticalAlignment','top','Interpreter','none');

% Telemetry panels
GX0 = 0.585; GW = 0.40; GH = 0.19; GAP = 0.055;
tops = [0.76, 0.76-GH-GAP, 0.76-2*(GH+GAP), 0.76-3*(GH+GAP)];
axP = axes('Parent',fig,'Position',[GX0,tops(1),GW,GH]);
dark_ax(axP,'Position (solid) vs Target (dash)','','m'); hold(axP,'on');
hPx = plot(axP,nan,nan,'r-','LineWidth',1.1);
hPy = plot(axP,nan,nan,'Color',[0.2,0.9,0.3],'LineWidth',1.1);
hPz = plot(axP,nan,nan,'b-','LineWidth',1.1);
hTx = plot(axP,nan,nan,'r:','LineWidth',0.75);
hTy = plot(axP,nan,nan,'Color',[0.2,0.9,0.3],'LineStyle',':','LineWidth',0.75);
hTz = plot(axP,nan,nan,'b:','LineWidth',0.75);
legend(axP,'X','Y','Z','TextColor','w','Color',BG,'FontSize',7,'Location','northwest');

axV = axes('Parent',fig,'Position',[GX0,tops(2),GW,GH]);
dark_ax(axV,'Velocity','','m/s'); hold(axV,'on');
hVx = plot(axV,nan,nan,'r-','LineWidth',1.1);
hVy = plot(axV,nan,nan,'Color',[0.2,0.9,0.3],'LineWidth',1.1);
hVz = plot(axV,nan,nan,'b-','LineWidth',1.1);
legend(axV,'Vx','Vy','Vz','TextColor','w','Color',BG,'FontSize',7,'Location','northwest');

axA = axes('Parent',fig,'Position',[GX0,tops(3),GW,GH]);
dark_ax(axA,'Euler Angles','','deg'); hold(axA,'on');
hAr = plot(axA,nan,nan,'r-','LineWidth',1.1);
hAp = plot(axA,nan,nan,'Color',[0.2,0.9,0.3],'LineWidth',1.1);
hAy = plot(axA,nan,nan,'b-','LineWidth',1.1);
legend(axA,'Roll','Pitch','Yaw','TextColor','w','Color',BG,'FontSize',7,'Location','northwest');

axM = axes('Parent',fig,'Position',[GX0,tops(4),GW,GH]);
dark_ax(axM,'Motor RPM','Time [s]','RPM'); hold(axM,'on');
hM1 = plot(axM,nan,nan,'r-','LineWidth',1.1);
hM2 = plot(axM,nan,nan,'Color',[0.2,0.9,0.3],'LineWidth',1.1);
hM3 = plot(axM,nan,nan,'b-','LineWidth',1.1);
hM4 = plot(axM,nan,nan,'m-','LineWidth',1.1);
legend(axM,'M1 F','M2 R','M3 B','M4 L','TextColor','w','Color',BG,'FontSize',7,'Location','northwest');
for axh = [axP,axV,axA,axM]; xlabel(axh,'Time [s]','Color',[0.5,0.6,0.7]); end

% Main loop
SKIP = 20;
fprintf('\nSimulation started (%d steps, dt=%.3f s)\n\n',N,dt);

for i = 1:N
    t = time(i);
    phi = ang(1); theta = ang(2); psi = ang(3);

    % Waypoint state machine
    if norm(pos-tgt_pos) < 0.45 && wp_idx < nWP
        wp_timer = wp_timer + dt;
        if wp_timer >= WP(wp_idx,5)
            wp_idx = wp_idx+1; wp_timer = 0;
            tgt_pos = WP(wp_idx,1:3)'; tgt_yaw = WP(wp_idx,4);
            fprintf('  t = %5.1f s  ->  WP%d  (%.0f, %.0f, %.0f m)\n', t, wp_idx, tgt_pos);
        end
    end

    % Wind disturbance
    wind = [0.9*sin(0.11*t)+0.28*randn; 0.6*cos(0.14*t)+0.18*randn; 0.12*sin(0.08*t)];

    % Smooth reference trajectory
    ref_err = tgt_pos - ref_pos;
    ref_spd = [MAX_VEL_H; MAX_VEL_H; MAX_VEL_V] * 0.85;
    ref_pos = ref_pos + clamp3(ref_err, ref_spd*dt);
    ref_yaw = ref_yaw + clamp1(wrap_pi(tgt_yaw-ref_yaw), MAX_YAW_R*dt);

    % Position PID -> desired velocity
    ep = ref_pos - pos; ip = clamp3(ip+ep*dt, IW); dep = (ep-ep0)/dt; ep0 = ep;
    dv = Kp_pos.*ep + Ki_pos.*ip + Kd_pos.*dep;
    dv(1:2) = clamp_n(dv(1:2), MAX_VEL_H); dv(3) = clamp1(dv(3), MAX_VEL_V);

    % Velocity PID -> desired acceleration & thrust
    ev = dv - vel; iv = clamp3(iv+ev*dt, IW); ev0 = ev;
    da = Kp_vel.*ev + Ki_vel.*iv;   % no D term
    da = clamp3(da, 14.0);
    T_des = max(min(m*(da(3)+g), T_max), T_min);
    axb = da(1)*cos(psi) + da(2)*sin(psi);
    ayb = -da(1)*sin(psi) + da(2)*cos(psi);
    d_pitch = clamp1(atan2(axb, g), MAX_TILT);
    d_roll  = clamp1(atan2(-ayb, g), MAX_TILT);

    % Attitude PID -> desired body rates
    ea = [d_roll-phi; d_pitch-theta; wrap_pi(ref_yaw-psi)];
    ia = clamp3(ia+ea*dt, IW); dea = (ea-ea0)/dt; ea0 = ea;
    dr = Kp_att.*ea + Ki_att.*ia + Kd_att.*dea;
    dr(3) = clamp1(dr(3), MAX_YAW_R);

    % Rate PID -> torque commands
    er = dr - omg; ir = clamp3(ir+er*dt, IW); der = (er-er0)/dt; er0 = er;
    tau_c = Kp_rate.*er + Ki_rate.*ir + Kd_rate.*der;

    % Motor mixing
    A_mix = [kf, kf, kf, kf;
             0, -kf*L, 0, kf*L;
             kf*L, 0, -kf*L, 0;
             -km, km, -km, km];
    wsq_d = max(min(A_mix\[T_des; tau_c], w_max^2), w_min^2);
    w_des = sqrt(wsq_d);
    w_mot = w_mot + (w_des - w_mot)*(dt/tau_mot);
    w_mot = max(min(w_mot, w_max), w_min);
    wsq = w_mot.^2;

    % Physics update
    R = Rmat(phi, theta, psi);
    T_act = kf*sum(wsq);
    tau_b = [kf*L*(-wsq(2)+wsq(4)); kf*L*(wsq(1)-wsq(3)); km*(-wsq(1)+wsq(2)-wsq(3)+wsq(4))];
    F_net = R*[0;0;T_act] + [0;0;-m*g] - Cd*vel + 0.038*m*wind;
    vel = vel + F_net/m*dt; pos = pos + vel*dt;
    if pos(3) < 0 && vel(3) < 0; pos(3) = 0; vel(3) = 0; end
    gyro = [(Iyy-Izz)*omg(2)*omg(3)/Ixx; (Izz-Ixx)*omg(1)*omg(3)/Iyy; (Ixx-Iyy)*omg(1)*omg(2)/Izz];
    omg = omg + (tau_b./[Ixx;Iyy;Izz] - gyro)*dt;
    ct = cos(theta);
    if abs(ct) < 0.015; ct = sign(ct+1e-9)*0.015; end   % avoid singularities
    sp = sin(phi); cp = cos(phi);
    Teu = [1, sp*tan(theta), cp*tan(theta); 0, cp, -sp; 0, sp/ct, cp/ct];
    ang = ang + Teu*omg*dt;
    ang(3) = wrap_pi(ang(3));

    % Logging
    LP(:,i) = pos; LV(:,i) = vel; LA(:,i) = ang;
    LR(:,i) = w_mot*60/(2*pi); LTP(:,i) = tgt_pos;

    % Rendering (every SKIP steps)
    if mod(i, SKIP) == 0
        ii = 1:i;
        Rs = R*SC;

        % Update body
        set(h_body, 'Vertices', xform(bv, Rs, pos));
        for k = 1:4
            set(h_arm(k), 'Vertices', xform(arm_v0{k}, Rs, pos));
            set(h_nac(k), 'Vertices', xform(nac_v0{k}, Rs, pos));
            rphase(k) = rphase(k) + w_mot(k)*dt*SKIP*3;
            rv_spin = spin_disc(rotor_v0{k}, nac_offsets(:,k), rphase(k));
            set(h_rot(k), 'Vertices', xform(rv_spin, Rs, pos));
            set(h_leg(k), 'Vertices', xform(leg_v0{k}, Rs, pos));
        end

        % Shadow
        bv_w = xform(bv, Rs, pos);
        alt = max(pos(3),0.02);
        sx = pos(1) + (bv_w(:,1)-pos(1))/alt*alt;
        sy = pos(2) + (bv_w(:,2)-pos(2))/alt*alt;
        set(h_shadow, 'XData', sx, 'YData', sy, 'ZData', repmat(0.001,8,1));

        % Velocity arrow
        vs = 0.45;
        set(h_vvec, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
            'UData', vs*vel(1), 'VData', vs*vel(2), 'WData', vs*vel(3));

        addpoints(h_trail, pos(1), pos(2), pos(3));
        title(ax3, sprintf('  t = %.1f s   |   WP %d/%d   |   Alt = %.1f m  ', t, wp_idx, nWP, pos(3)), ...
              'Color',[0.7,0.85,1],'FontSize',10);

        % HUD text
        set(h_hud, 'String', sprintf(...
            '  TIME    %6.1f s\n  ALT     %6.2f m\n  SPEED   %6.2f m/s\n  DIST WP %6.2f m\n  ROLL    %6.1f deg\n  PITCH   %6.1f deg\n  YAW     %6.1f deg\n  THRUST  %6.1f N',...
            t, pos(3), norm(vel), norm(tgt_pos-pos), rad2deg(ang(1)), rad2deg(ang(2)), rad2deg(ang(3)), T_act));

        % Update graphs
        set(hPx, 'XData', time(ii), 'YData', LP(1,ii));
        set(hPy, 'XData', time(ii), 'YData', LP(2,ii));
        set(hPz, 'XData', time(ii), 'YData', LP(3,ii));
        set(hTx, 'XData', time(ii), 'YData', LTP(1,ii));
        set(hTy, 'XData', time(ii), 'YData', LTP(2,ii));
        set(hTz, 'XData', time(ii), 'YData', LTP(3,ii));
        set(hVx, 'XData', time(ii), 'YData', LV(1,ii));
        set(hVy, 'XData', time(ii), 'YData', LV(2,ii));
        set(hVz, 'XData', time(ii), 'YData', LV(3,ii));
        set(hAr, 'XData', time(ii), 'YData', rad2deg(LA(1,ii)));
        set(hAp, 'XData', time(ii), 'YData', rad2deg(LA(2,ii)));
        set(hAy, 'XData', time(ii), 'YData', rad2deg(LA(3,ii)));
        set(hM1, 'XData', time(ii), 'YData', LR(1,ii));
        set(hM2, 'XData', time(ii), 'YData', LR(2,ii));
        set(hM3, 'XData', time(ii), 'YData', LR(3,ii));
        set(hM4, 'XData', time(ii), 'YData', LR(4,ii));

        drawnow limitrate;
    end
end

fprintf('\nDone. Final position = (%.2f, %.2f, %.2f) m\n', pos);


% let now call Helper functions

function Vout = xform(V0, Rs, p)
    Vout = V0 * Rs' + p';
end

function V = make_box_verts(hw, hs, hh)
    xs = [-1,1,1,-1,-1,1,1,-1]'*hw;
    ys = [-1,-1,1,1,-1,-1,1,1]'*hs;
    zs = [-1,-1,-1,-1,1,1,1,1]'*hh;
    V = [xs,ys,zs];
end

function F = box_faces()
    F = [1,2,3,4; 5,6,7,8; 1,2,6,5; 3,4,8,7; 2,3,7,6; 4,1,5,8];
end

function V = make_arm_verts(dir, len, r)
    d = dir(:);
    perp = null(d');
    p1 = perp(:,1)*r; p2 = perp(:,2)*r;
    tip = d*len;
    V = [p1+p2, p1-p2, -p1-p2, -p1+p2, tip+p1+p2, tip+p1-p2, tip-p1-p2, tip-p1+p2]';
end

function [V,F] = make_cylinder(ctr, r, h, ns)
    th = linspace(0,2*pi,ns+1); th = th(1:end-1);
    xc = r*cos(th(:)); yc = r*sin(th(:));
    Vb = [xc+ctr(1), yc+ctr(2), zeros(ns,1)+ctr(3)];
    Vt = [xc+ctr(1), yc+ctr(2), zeros(ns,1)+ctr(3)+h];
    bc = [ctr(1),ctr(2),ctr(3)];
    tc = [ctr(1),ctr(2),ctr(3)+h];
    V = [Vb; Vt; bc; tc];
    F = [];
    for k = 1:ns
        kn = mod(k,ns)+1;
        F = [F; k, kn, ns+kn, ns+k];
        F = [F; k, kn, 2*ns+1, 2*ns+1];
        F = [F; ns+k, ns+kn, 2*ns+2, 2*ns+2];
    end
end

function [V,F] = make_disc(ctr, r, ns)
    th = linspace(0,2*pi,ns+1); th = th(1:end-1);
    V = [[r*cos(th(:)), r*sin(th(:)), zeros(ns,1)] + ctr'; ctr'];
    nc = size(V,1);
    F = zeros(ns,3);
    for k = 1:ns
        F(k,:) = [k, mod(k,ns)+1, nc];
    end
end

function V = spin_disc(V0, ctr, phase)
    c = cos(phase); s = sin(phase);
    Rz = [c,-s,0; s,c,0; 0,0,1];
    V = (V0 - ctr') * Rz' + ctr';
end

function V = make_leg_verts(base, tip, r)
    ax = (tip-base)/norm(tip-base);
    perp = null(ax');
    p1 = perp(:,1)*r; p2 = perp(:,2)*r;
    V = [base+p1+p2, base+p1-p2, base-p1-p2, base-p1+p2, ...
         tip+p1+p2,  tip+p1-p2,  tip-p1-p2,  tip-p1+p2]';
end

function R = Rmat(ph,th,ps)
    Rz = [cos(ps),-sin(ps),0; sin(ps),cos(ps),0; 0,0,1];
    Ry = [cos(th),0,sin(th); 0,1,0; -sin(th),0,cos(th)];
    Rx = [1,0,0; 0,cos(ph),-sin(ph); 0,sin(ph),cos(ph)];
    R = Rz*Ry*Rx;
end

function a = wrap_pi(a)
    a = mod(a+pi,2*pi) - pi;
end

function v = clamp1(v, lim)
    v = max(min(v,lim), -lim);
end

function v = clamp3(v, lim)
    v = max(min(v,lim), -lim);
end

function v = clamp_n(v, mx)
    n = norm(v);
    if n > mx
        v = v * (mx/n);
    end
end

function dark_ax(ax, ttl, xl, yl)
    set(ax,'Color',[0.03,0.03,0.05],'XColor',[0.5,0.6,0.7],'YColor',[0.5,0.6,0.7],...
           'GridColor',[0.10,0.12,0.16],'GridAlpha',1,'FontSize',7.5);
    title(ax,ttl,'Color','w','FontSize',8.5);
    if ~isempty(xl); xlabel(ax,xl,'Color',[0.5,0.6,0.7]); end
    if ~isempty(yl); ylabel(ax,yl,'Color',[0.5,0.6,0.7]); end
    grid(ax,'on');
end
