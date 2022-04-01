clearvars -except Streaming Normalized stairAscentTrialData R01 rawR01 estimatedAngles;

close all
addpath("Utility Functions")

if ~exist('Normalized')
    load '../Data/Normalized.mat'
end
if exist('estimatedAngles')
    load estimatedAngles.mat
end

sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

% percentGait=linspace(0,1,150);

prompt = 'Leave out 20^o(1), 25^o(2), 30^o(3), 35^o(4), -20^o(5), -25^o(6), -30^o(7), -35^o(8): ';
leftOutIncline = input(prompt);

% leftOutIncline = 8;

trial={'s3'};

incline={'i20'};

[thigh_i20, knee_i20, ankle_i20, thigh_i20sd, knee_i20sd, ankle_i20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i25'};

[thigh_i25, knee_i25, ankle_i25, thigh_i25sd, knee_i25sd, ankle_i25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i30'};

[thigh_i30, knee_i30, ankle_i30, thigh_i30sd, knee_i30sd, ankle_i30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'i35'};

[thigh_i35, knee_i35, ankle_i35, thigh_i35sd, knee_i35sd, ankle_i35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in20'};

[thigh_d20, knee_d20, ankle_d20, thigh_d20sd, knee_d20sd, ankle_d20sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in25'};

[thigh_d25, knee_d25, ankle_d25, thigh_d25sd, knee_d25sd, ankle_d25sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in30'};

[thigh_d30, knee_d30, ankle_d30, thigh_d30sd, knee_d30sd, ankle_d30sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);

incline={'in35'};

[thigh_d35, knee_d35, ankle_d35, thigh_d35sd, knee_d35sd, ankle_d35sd] = averageJointKinematics_Stair(Normalized,sub,trial,incline);




L = length(thigh_i20);

% Load Walking Trajectories
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};
trial={'s1'};
incline={'i0'};
[thigh_0, knee_0, ankle_0, thigh_0sd, knee_0sd, ankle_0sd] = averageJointKinematics_Walk(Normalized,sub,trial,incline);

t = linspace(0,1,length(thigh_i20));
figure
plot(t,thigh_i30)
hold on
plot(t,thigh_i35)


thigh_0_shift = circshift(thigh_0,-find(thigh_0 == max(thigh_0)));
thigh_i20_shift = circshift(thigh_i20,-find(thigh_i20 == max(thigh_i20)));
thigh_i25_shift = circshift(thigh_i25,-find(thigh_i25 == max(thigh_i25)));
thigh_i30_shift = circshift(thigh_i30,-find(thigh_i30 == max(thigh_i30)));
thigh_i35_shift = circshift(thigh_i35,-find(thigh_i35 == max(thigh_i35)));
thigh_d20_shift = circshift(thigh_d20,-find(thigh_d20 == max(thigh_d20)));
thigh_d25_shift = circshift(thigh_d25,-find(thigh_d25 == max(thigh_d25)));
thigh_d30_shift = circshift(thigh_d30,-find(thigh_d30 == max(thigh_d30)));
thigh_d35_shift = circshift(thigh_d35,-find(thigh_d35 == max(thigh_d35)));

figure
plot(t, thigh_i20_shift,t,thigh_i25_shift,t,thigh_i30_shift,t,thigh_i35_shift)
hold on
plot(t, thigh_d20_shift,t,thigh_d25_shift,t,thigh_d30_shift,t,thigh_d35_shift)

c_0 = t(find(thigh_0_shift == min(thigh_0_shift)));
c_i20 = t(find(thigh_i20_shift == min(thigh_i20_shift)));
c_i25 = t(find(thigh_i25_shift == min(thigh_i25_shift)));
c_i30 = t(find(thigh_i30_shift == min(thigh_i30_shift)));
c_i35 = t(find(thigh_i35_shift == min(thigh_i35_shift)));
c_d20 = t(find(thigh_d20_shift == min(thigh_d20_shift)));
c_d25 = t(find(thigh_d25_shift == min(thigh_d25_shift)));
c_d30 = t(find(thigh_d30_shift == min(thigh_d30_shift)));
c_d35 = t(find(thigh_d35_shift == min(thigh_d35_shift)));

c_avg = mode([c_i20 c_i25 c_i30 c_i35 c_d20 c_d25 c_d30 c_d35]);

%% Calculate Phase Variable
figure
for ind = 1:9
    
    switch ind
        case 1
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_0;%circshift(thigh_0,-find(thigh_0 == max(thigh_0)));
            knee_mean = knee_0;
            ankle_mean = ankle_0;
            current_incline = '0^o'
            pv_swing_thresh = .65;
            c = c_0;
            q_po = -10;
        case 2
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_i20;
            knee_mean = knee_i20;
            ankle_mean = ankle_i20;
            current_incline = '20^o'
            pv_swing_thresh = .65;
            c= c_i20;
            q_po = -4;
        case 3
            %             load '../Data/filtStairTraj_i25'
            thigh_mean = thigh_i25;
            knee_mean = knee_i25;
            ankle_mean = ankle_i25;
            current_incline = '25^o'
            pv_swing_thresh = .65;
            c= c_i25;
            q_po = -4;
        case 4
            %             load '../Data/filtStairTraj_i30'
            thigh_mean = thigh_i30;
            knee_mean = knee_i30;
            ankle_mean = ankle_i30;
            current_incline = '30^o'
            pv_swing_thresh = .65;
            c= c_i30;
            q_po = -4;
        case 5
            %             load '../Data/filtStairTraj_i35'
            thigh_mean = thigh_i35;
            knee_mean = knee_i35;
            ankle_mean = ankle_i35;
            current_incline = '35^o';
            pv_swing_thresh = .65;
            c= c_i35;
            q_po = -4;
        case 6
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_d20;
            knee_mean = knee_d20;
            ankle_mean = ankle_d20;
            current_incline = '-20^o'
            q_po = 7;
            pv_swing_thresh = .96;
            c= c_d20;
        case 7
            %             load '../Data/filtStairTraj_i25'
            thigh_mean = thigh_d25;
            knee_mean = knee_d25;
            ankle_mean = ankle_d25;
            current_incline = '-25^o'
            q_po = 7;
            pv_swing_thresh = .96;
            c= c_d25;
        case 8
            %             load '../Data/filtStairTraj_i30'
            thigh_mean = thigh_d30;
            knee_mean = knee_d30;
            ankle_mean = ankle_d30;
            current_incline = '-30^o'
            q_po = 7;
            pv_swing_thresh = .96;
            c= c_d30;
        case 9
            %             load '../Data/filtStairTraj_i35'
            thigh_mean = thigh_d35;
            knee_mean = knee_d35;
            ankle_mean = ankle_d35;
            current_incline = '-35^o'
            q_po = 7;
            pv_swing_thresh = .96;
            c= c_d35;
        otherwise
            %             load '../Data/filtStairTraj_i20'
            thigh_mean = thigh_i20;
            knee_mean = knee_i20;
            ankle_mean = ankle_i20;
            current_incline = '20^o';
            pv_swing_thresh = .96;
            c= c_i20;
    end
    
    %     knee_mean = knee_mean;
    %     thigh_mean = thigh_mean;
    %     ankle_mean = ankle_mean;
    thighd_mean = ddt(thigh_mean);
    prevPV = 0;
    prevState = 1;
    sm = 0;
    qhm = 0;
    qh_max = max(thigh_mean);
    qh_min = min(thigh_mean);
%     c = t(find(thigh_mean== qh_min))
    
    
    %peak detection parameters
    %threshold and min distance
    thresh = .00001;
    minpeakh = 15;
    minpeakd = 50;
    mhf = 0;
    temp_traj = [];
    ypeak = [];
    xpeak = [];
    
    
    %peak detection parameters
    %threshold and min distance
    thresh = .0000001;
    minpeakh_mhe = 6;
    minpeakd_mhe = 6;
    thigh_start_mhe = 7;
    mhe = 0;
    temp_traj_MHE = [];
%     pv_swing_thresh = .96;
    ytrough = [];
    xtrough = [];
    
    
    for i = 1:length(t)
        
        thigh = thigh_mean(i);
        thighd = thighd_mean(i);
        
        
        
        if prevState == 2|| prevState == 3
            temp_traj_MHE = [temp_traj_MHE -thigh];
            if length(temp_traj_MHE) > 3
                [pks_MHE,locs_MHE] = findpeaks(temp_traj_MHE,'threshold', thresh,'MinPeakHeight',-minpeakh_mhe);
                if ~isempty(pks_MHE)
                    temp_traj_MHE = [];
                    ytrough = -[ytrough pks_MHE];
                    xtrough = [xtrough i-1];
                    mhe = 1;
                    
                end
            end
            
            
        end
        if prevState == 3 || prevState == 4
            if thigh > 15 %&& i-p > minpeakd
                temp_traj = [temp_traj thigh];
                if length(temp_traj) > 3
                    temp_traj;
                    [pks,locs] = findpeaks(temp_traj,'threshold', thresh,'MinPeakHeight',minpeakh);
                    if ~isempty(pks)
                        p = i-1;
                        temp_traj = [];
                        ypeak = [ypeak pks];
                        xpeak = [xpeak p];
                        temp_max = qh_max;
                        qh_max = mean([ypeak max(thigh_mean)]);
                        mhf = 1;
                    end
                end
                
            end
        end

        
        [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
        %
        
        
        pv(i) = currPV;
        prevState = currState;
        prevPV = currPV;
        
    end
    
    
%     if strcmp(current_incline, '0^o')
% %         pv_shift = circshift(pv, -find(thigh_mean ==max(thigh_mean)));
%         pv = unique(pv,'stable');
%         pv = smooth(interp1(1:length(pv), pv, 1:length(pv)/151:length(pv)))';
%         pv = circshift(pv, find(thigh_0 == max(thigh_0)));
%     end
    
    switch ind
        case 1
            %             load '../Data/filtStairTraj_i20'
%             thigh_mean = thigh_0;%circshift(thigh_0,-find(thigh_0 == max(thigh_0)));
%             knee_mean = knee_0;
%             ankle_mean = ankle_0;
%             current_incline = '0^o'
%             pv_swing_thresh = .65;
%             c = c_0;
%             q_po = -10;

            estimatedAngles.phase.i0 = pv;
        case 2
%             %             load '../Data/filtStairTraj_i20'
%             thigh_mean = thigh_i20;
%             knee_mean = knee_i20;
%             ankle_mean = ankle_i20;
%             current_incline = '20^o'
%             pv_swing_thresh = .65;
%             c= c_i20;
%             q_po = -4;
            estimatedAngles.phase.i20 = pv;
              
        case 3
%             %             load '../Data/filtStairTraj_i25'
%             thigh_mean = thigh_i25;
%             knee_mean = knee_i25;
%             ankle_mean = ankle_i25;
%             current_incline = '25^o'
%             pv_swing_thresh = .65;
%             c= c_i25;
%             q_po = -4;
            estimatedAngles.phase.i25 = pv;
        case 4
%             %             load '../Data/filtStairTraj_i30'
%             thigh_mean = thigh_i30;
%             knee_mean = knee_i30;
%             ankle_mean = ankle_i30;
%             current_incline = '30^o'
%             pv_swing_thresh = .65;
%             c= c_i30;
%             q_po = -4;
            estimatedAngles.phase.i30 = pv;
        case 5
%             %             load '../Data/filtStairTraj_i35'
%             thigh_mean = thigh_i35;
%             knee_mean = knee_i35;
%             ankle_mean = ankle_i35;
%             current_incline = '35^o';
%             pv_swing_thresh = .65;
%             c= c_i35;
%             q_po = -4;
            estimatedAngles.phase.i35 = pv;
        case 6
            %             load '../Data/filtStairTraj_i20'
%             thigh_mean = thigh_d20;
%             knee_mean = knee_d20;
%             ankle_mean = ankle_d20;
%             current_incline = '-20^o'
%             q_po = 7;
%             pv_swing_thresh = .96;
%             c= c_d20;
            estimatedAngles.phase.in20 = pv;
        case 7
            %             load '../Data/filtStairTraj_i25'
%             thigh_mean = thigh_d25;
%             knee_mean = knee_d25;
%             ankle_mean = ankle_d25;
%             current_incline = '-25^o'
%             q_po = 7;
%             pv_swing_thresh = .96;
%             c= c_d25;
            estimatedAngles.phase.in25 = pv;
        case 8
            %             load '../Data/filtStairTraj_i30'
%             thigh_mean = thigh_d30;
%             knee_mean = knee_d30;
%             ankle_mean = ankle_d30;
%             current_incline = '-30^o'
%             q_po = 7;
%             pv_swing_thresh = .96;
%             c= c_d30;
            estimatedAngles.phase.in30 = pv;
        case 9
            %             load '../Data/filtStairTraj_i35'
%             thigh_mean = thigh_d35;
%             knee_mean = knee_d35;
%             ankle_mean = ankle_d35;
%             current_incline = '-35^o'
%             q_po = 7;
%             pv_swing_thresh = .96;
%             c= c_d35;
            estimatedAngles.phase.in35 = pv;
        otherwise
            %             load '../Data/filtStairTraj_i20'
%             thigh_mean = thigh_i20;
%             knee_mean = knee_i20;
%             ankle_mean = ankle_i20;
%             current_incline = '20^o';
%             pv_swing_thresh = .96;
%             c= c_i20;
            estimatedAngles.phase.i0 = pv;
    end





    plot(t,pv)
    
    hold on
    T(ind,:) = interp1(pv,t,pv,'linear','extrap');
    knee_interp(ind,:) = interp1(pv,knee_mean,T(ind,:),'linear','extrap');
    ankle_interp(ind,:) = interp1(pv, ankle_mean, T(ind,:),'linear','extrap');
    knee_traj(ind,:) = knee_mean;
    ankle_traj(ind,:) = ankle_mean;
    
    thigh_ROM(ind,:) = qh_max-qh_min;
end

legend('0^o','20^o','25^o','30^o','35^o','-20^o','-25^o','-30^o','-35^o')

% figure
% plot([0,20,25,30,35,-20,-25,-30,-35]',thigh_ROM ,'*')
% %legend('0^o','20^o','25^o','30^o','35^o','-20^o','-25^o','-30^o','-35^o')

knee_0_incvc = knee_interp(1,:);
knee_20_incvc = knee_interp(2,:);
knee_25_incvc = knee_interp(3,:);
knee_30_incvc = knee_interp(4,:);
knee_35_incvc = knee_interp(5,:);

knee_20_decvc = knee_interp(6,:);
knee_25_decvc = knee_interp(7,:);
knee_30_decvc = knee_interp(8,:);
knee_35_decvc = knee_interp(9,:);



ankle_0_incvc = ankle_interp(1,:);
ankle_20_incvc = ankle_interp(2,:);
ankle_25_incvc = ankle_interp(3,:);
ankle_30_incvc = ankle_interp(4,:);
ankle_35_incvc = ankle_interp(5,:);

ankle_20_decvc = ankle_interp(6,:);
ankle_25_decvc = ankle_interp(7,:);
ankle_30_decvc = ankle_interp(8,:);
ankle_35_decvc = ankle_interp(9,:);


stairVCTrajectories.knee.i0 = knee_0_incvc;
stairVCTrajectories.knee.i20 = knee_20_incvc;
stairVCTrajectories.knee.i25 = knee_25_incvc;
stairVCTrajectories.knee.i30 = knee_30_incvc;
stairVCTrajectories.knee.i35 = knee_35_incvc;

stairVCTrajectories.knee.in20 = knee_20_decvc;
stairVCTrajectories.knee.in25 = knee_25_decvc;
stairVCTrajectories.knee.in30 = knee_30_decvc;
stairVCTrajectories.knee.in35 = knee_35_decvc;

stairVCTrajectories.ankle.i0 = ankle_0_incvc;
stairVCTrajectories.ankle.i20 = ankle_20_incvc;
stairVCTrajectories.ankle.i25 = ankle_25_incvc;
stairVCTrajectories.ankle.i30 = ankle_30_incvc;
stairVCTrajectories.ankle.i35 = ankle_35_incvc;

stairVCTrajectories.ankle.in20 = ankle_20_decvc;
stairVCTrajectories.ankle.in25 = ankle_25_decvc;
stairVCTrajectories.ankle.in30 = ankle_30_decvc;
stairVCTrajectories.ankle.in35 = ankle_35_decvc;

save stairVCTrajectories.mat stairVCTrajectories
% stairVCTrajectories.knee.i35 = knee_interp(5,:);
% stairVCTrajectories.knee.i30 = knee_interp(4,:);
% stairVCTrajectories.knee.i25 = knee_interp(3,:);
% stairVCTrajectories.knee.i20 = knee_interp(2,:);
% stairVCTrajectories.knee.i0 = knee_interp(1,:);



%Knee Coefficients
pk_real20_incvc = real(fft(knee_20_incvc)/(L/2));
pk_real25_incvc = real(fft(knee_25_incvc)/(L/2));
pk_real30_incvc = real(fft(knee_30_incvc)/(L/2));
pk_real35_incvc = real(fft(knee_35_incvc)/(L/2));

pk_im20_incvc = imag(fft(knee_20_incvc)/(L/2));
pk_im25_incvc = imag(fft(knee_25_incvc)/(L/2));
pk_im30_incvc = imag(fft(knee_30_incvc)/(L/2));
pk_im35_incvc = imag(fft(knee_35_incvc)/(L/2));


pk_real20_decvc = real(fft(knee_20_decvc)/(L/2));
pk_real25_decvc = real(fft(knee_25_decvc)/(L/2));
pk_real30_decvc = real(fft(knee_30_decvc)/(L/2));
pk_real35_decvc = real(fft(knee_35_decvc)/(L/2));

pk_im20_decvc = imag(fft(knee_20_decvc)/(L/2));
pk_im25_decvc = imag(fft(knee_25_decvc)/(L/2));
pk_im30_decvc = imag(fft(knee_30_decvc)/(L/2));
pk_im35_decvc = imag(fft(knee_35_decvc)/(L/2));

%Ankle Coefficients
pa_real20_incvc = real(fft(ankle_20_incvc)/(L/2));
pa_real25_incvc = real(fft(ankle_25_incvc)/(L/2));
pa_real30_incvc = real(fft(ankle_30_incvc)/(L/2));
pa_real35_incvc = real(fft(ankle_35_incvc)/(L/2));

pa_im20_incvc = imag(fft(ankle_20_incvc)/(L/2));
pa_im25_incvc = imag(fft(ankle_25_incvc)/(L/2));
pa_im30_incvc = imag(fft(ankle_30_incvc)/(L/2));
pa_im35_incvc = imag(fft(ankle_35_incvc)/(L/2));


pa_real20_decvc = real(fft(ankle_20_decvc)/(L/2));
pa_real25_decvc = real(fft(ankle_25_decvc)/(L/2));
pa_real30_decvc = real(fft(ankle_30_decvc)/(L/2));
pa_real35_decvc = real(fft(ankle_35_decvc)/(L/2));

pa_im20_decvc = imag(fft(ankle_20_decvc)/(L/2));
pa_im25_decvc = imag(fft(ankle_25_decvc)/(L/2));
pa_im30_decvc = imag(fft(ankle_30_decvc)/(L/2));
pa_im35_decvc = imag(fft(ankle_35_decvc)/(L/2));

inclines = [0,20,25,30,35,-20,-25,-30,-35];
x = [thigh_ROM(1:leftOutIncline)' thigh_ROM(leftOutIncline+2:end)'];

% 
figure
plot(inclines,thigh_ROM ,'*')
hold on
plot([inclines(1:leftOutIncline) inclines(leftOutIncline+2:end)],x,'o')
%legend('0^o','20^o','25^o','30^o','35^o','-20^o','-25^o','-30^o','-35^o')

pk_real0_incvc = real(fft(knee_0_incvc)/(L/2));
pk_im0_incvc = imag(fft(knee_0_incvc)/(L/2));

pa_real0_incvc = real(fft(ankle_0_incvc)/(L/2));
pa_im0_incvc = imag(fft(ankle_0_incvc)/(L/2));

%% Get Fourier Coefficients

N = length(t)
for i = 1:N/2
    switch leftOutIncline
        case 1 %20
            vkreal_vc(i,:) = [pk_real0_incvc(i) pk_real25_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i) pk_real20_decvc(i) pk_real25_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_vc(i,:) = [pk_im0_incvc(i) pk_im25_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i) pk_im20_decvc(i) pk_im25_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_vc(i,:) = [pa_real0_incvc(i) pa_real25_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i) pa_real20_decvc(i) pa_real25_decvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_vc(i,:) = [pa_im0_incvc(i) pa_im25_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i) pa_im20_decvc(i) pa_im25_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
            
        case 2 %25
            vkreal_vc(i,:) = [pk_real0_incvc(i) pk_real20_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i) pk_real20_decvc(i) pk_real25_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_vc(i,:) = [pk_im0_incvc(i) pk_im20_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i) pk_im20_decvc(i) pk_im25_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_vc(i,:) = [pa_real0_incvc(i) pa_real20_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i) pa_real20_decvc(i) pa_real25_decvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_vc(i,:) = [pa_im0_incvc(i) pa_im20_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i) pa_im20_decvc(i) pa_im25_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
        case 3 %30
            vkreal_vc(i,:) = [pk_real0_incvc(i) pk_real20_incvc(i) pk_real25_incvc(i) pk_real35_incvc(i) pk_real20_decvc(i) pk_real25_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_vc(i,:) = [pk_im0_incvc(i) pk_im20_incvc(i) pk_im25_incvc(i) pk_im35_incvc(i) pk_im20_decvc(i) pk_im25_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_vc(i,:) = [pa_real0_incvc(i) pa_real20_incvc(i) pa_real25_incvc(i) pa_real35_incvc(i) pa_real20_decvc(i) pa_real25_decvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_vc(i,:) = [pa_im0_incvc(i) pa_im20_incvc(i) pa_im25_incvc(i) pa_im35_incvc(i) pa_im20_decvc(i) pa_im25_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
        case 4 %35
            vkreal_vc(i,:) = [pk_real0_incvc(i) pk_real20_incvc(i) pk_real25_incvc(i) pk_real30_incvc(i) pk_real20_decvc(i) pk_real25_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_vc(i,:) = [pk_im0_incvc(i) pk_im20_incvc(i) pk_im25_incvc(i) pk_im30_incvc(i) pk_im20_decvc(i) pk_im25_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_vc(i,:) = [pa_real0_incvc(i) pa_real20_incvc(i) pa_real25_incvc(i) pa_real30_incvc(i) pa_real20_decvc(i) pa_real25_decvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_vc(i,:) = [pa_im0_incvc(i) pa_im20_incvc(i) pa_im25_incvc(i) pa_im30_incvc(i) pa_im20_decvc(i) pa_im25_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
        case 5 %-20
            vkreal_vc(i,:) = [pk_real0_incvc(i) pk_real20_incvc(i) pk_real25_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i) pk_real25_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_vc(i,:) = [pk_im0_incvc(i) pk_im20_incvc(i) pk_im25_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i) pk_im25_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_vc(i,:) = [pa_real0_incvc(i) pa_real20_incvc(i) pa_real25_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i) pa_real25_decvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_vc(i,:) = [pa_im0_incvc(i) pa_im20_incvc(i) pa_im25_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i) pa_im25_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
        case 6 %-25
            vkreal_vc(i,:) = [pk_real0_incvc(i) pk_real20_incvc(i) pk_real25_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i) pk_real20_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_vc(i,:) = [pk_im0_incvc(i) pk_im20_incvc(i) pk_im25_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i) pk_im20_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_vc(i,:) = [pa_real0_incvc(i) pa_real20_incvc(i) pa_real25_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i) pa_real20_decvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_vc(i,:) = [pa_im0_incvc(i) pa_im20_incvc(i) pa_im25_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i) pa_im20_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
        case 7 %-30
            vkreal_vc(i,:) = [pk_real0_incvc(i) pk_real20_incvc(i) pk_real25_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i) pk_real20_decvc(i) pk_real25_decvc(i) pk_real35_decvc(i)];
            vkim_vc(i,:) = [pk_im0_incvc(i) pk_im20_incvc(i) pk_im25_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i) pk_im20_decvc(i) pk_im25_decvc(i) pk_im35_decvc(i)];
            vareal_vc(i,:) = [pa_real0_incvc(i) pa_real20_incvc(i) pa_real25_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i) pa_real20_decvc(i) pa_real25_decvc(i) pa_real35_decvc(i)];
            vaim_vc(i,:) = [pa_im0_incvc(i) pa_im20_incvc(i) pa_im25_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i) pa_im20_decvc(i) pa_im25_decvc(i) pa_im35_decvc(i)];
        case 8 %-35
            vkreal_vc(i,:) = [pk_real0_incvc(i) pk_real20_incvc(i) pk_real25_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i) pk_real20_decvc(i) pk_real25_decvc(i) pk_real30_decvc(i)];
            vkim_vc(i,:) = [pk_im0_incvc(i) pk_im20_incvc(i) pk_im25_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i) pk_im20_decvc(i) pk_im25_decvc(i) pk_im30_decvc(i)];
            vareal_vc(i,:) = [pa_real0_incvc(i) pa_real20_incvc(i) pa_real25_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i) pa_real20_decvc(i) pa_real25_decvc(i) pa_real30_decvc(i)];
            vaim_vc(i,:) = [pa_im0_incvc(i) pa_im20_incvc(i) pa_im25_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i) pa_im20_decvc(i) pa_im25_decvc(i) pa_im30_decvc(i)];
        otherwise
            vkreal_vc(i,:) = [pk_real0_incvc(i) pk_real20_incvc(i) pk_real25_incvc(i) pk_real30_incvc(i) pk_real35_incvc(i) pk_real20_decvc(i) pk_real25_decvc(i) pk_real30_decvc(i) pk_real35_decvc(i)];
            vkim_vc(i,:) = [pk_im0_incvc(i) pk_im20_incvc(i) pk_im25_incvc(i) pk_im30_incvc(i) pk_im35_incvc(i) pk_im20_decvc(i) pk_im25_decvc(i) pk_im30_decvc(i) pk_im35_decvc(i)];
            vareal_vc(i,:) = [pa_real0_incvc(i) pa_real20_incvc(i) pa_real25_incvc(i) pa_real30_incvc(i) pa_real35_incvc(i) pa_real20_decvc(i) pa_real25_decvc(i) pa_real30_decvc(i) pa_real35_decvc(i)];
            vaim_vc(i,:) = [pa_im0_incvc(i) pa_im20_incvc(i) pa_im25_incvc(i) pa_im30_incvc(i) pa_im35_incvc(i) pa_im20_decvc(i) pa_im25_decvc(i) pa_im30_decvc(i) pa_im35_decvc(i)];
    end
end

%% Leave one out plots

for i = 1:N/2
    switch leftOutIncline
        case 1 %20
            thigh_ref = thigh_i20;
            knee_ref = knee_i20;
            ankle_ref = ankle_i20;
            
            thigh_sd =thigh_i20sd;
            ankle_sd =ankle_i20sd;
            knee_sd =knee_i20sd;
            
            thigh_traj = [thigh_0; thigh_i25;thigh_i30;thigh_i35; thigh_d20; thigh_d25; thigh_d30; thigh_d35];
            incline={'i20'};
            leg = ["0^o","25^o", "30^o", "35^o","-20^o","-25^o", "-30^o", "-35^o", "Unknown (20^o)"];
            strIncline = "20^o";
            q_po = -4;
            
            c = c_i20;
        case 2 %25
            thigh_ref = thigh_i25;
            knee_ref = knee_i25;
            ankle_ref = ankle_i25;
            
            thigh_sd =thigh_i25sd;
            ankle_sd =ankle_i25sd;
            knee_sd =knee_i25sd;
            
            thigh_traj = [thigh_0; thigh_i20;thigh_i30;thigh_i35; thigh_d20; thigh_d25; thigh_d30; thigh_d35];
            incline={'i25'};
            leg = ["0^o","20^o", "30^o", "35^o","-20^o","-25^o", "-30^o", "-35^o", "Unknown (25^o)"];
            strIncline = "25^o";
            q_po = -4;
            c = c_i25;
        case 3 %30
            thigh_ref = thigh_i30;
            knee_ref = knee_i30;
            ankle_ref = ankle_i30;
            
            thigh_sd =thigh_i30sd;
            ankle_sd =ankle_i30sd;
            knee_sd =knee_i30sd;
            
            thigh_traj = [thigh_0; thigh_i20;thigh_i25;thigh_i35; thigh_d20; thigh_d25; thigh_d30; thigh_d35];
            incline={'i30'};
            leg = ["0^o","20^o", "25^o", "35^o","-20^o","-25^o", "-30^o", "-35^o", "Unknown (30^o)"];
            strIncline = "30^o";
            q_po = -4;
            c = c_i30;
        case 4 %35
            thigh_ref = thigh_i35;
            knee_ref = knee_i35;
            ankle_ref = ankle_i35;
            
            thigh_sd =thigh_i35sd;
            ankle_sd =ankle_i35sd;
            knee_sd =knee_i35sd;
            
            thigh_traj = [thigh_0; thigh_i20;thigh_i25;thigh_i30; thigh_d20; thigh_d25; thigh_d30; thigh_d35];
            incline={'i35'};
            leg = ["0^o","20^o", "25^o", "30^o","-20^o","-25^o", "-30^o", "-35^o", "Unknown (35^o)"];
            strIncline = "35^o";
            q_po = -4;
            c = c_i35;
        case 5 %-20
            thigh_ref = thigh_d20;
            knee_ref = knee_d20;
            ankle_ref = ankle_d20;
            
            thigh_sd =thigh_d20sd;
            ankle_sd =ankle_d20sd;
            knee_sd =knee_d20sd;
            
            thigh_traj = [thigh_0; thigh_i20;thigh_i25;thigh_i30; thigh_i35; thigh_d25; thigh_d30; thigh_d35];
            incline={'in20'};
            leg = ["0^o","20^o", "25^o", "30^o","35^o","-25^o", "-30^o", "-35^o", "Unknown (-20^o)"];
            strIncline = "-20^o";
            q_po = 7;
            c = c_d20;
        case 6 %-25
            thigh_ref = thigh_d25;
            knee_ref = knee_d25;
            ankle_ref = ankle_d25;
            
            thigh_sd =thigh_d25sd;
            ankle_sd =ankle_d25sd;
            knee_sd =knee_d25sd;
            
            thigh_traj = [thigh_0; thigh_i20;thigh_i25;thigh_i30; thigh_i35; thigh_d20; thigh_d30; thigh_d35];
            incline={'in25'};
            leg = ["0^o","20^o", "25^o", "30^o","35^o","-20^o", "-30^o", "-35^o", "Unknown (-25^o)"];
            strIncline = "-25^o";
            q_po = 7;
            c = c_d25;
        case 7 %-30
            thigh_ref = thigh_d30;
            knee_ref = knee_d30;
            ankle_ref = ankle_d30;
            
            thigh_sd =thigh_d30sd;
            ankle_sd =ankle_d30sd;
            knee_sd =knee_d30sd;
            
            thigh_traj = [thigh_0; thigh_i20;thigh_i25;thigh_i30; thigh_i35; thigh_d20; thigh_d25; thigh_d35];
            incline={'in30'};
            leg = ["0^o","20^o", "25^o", "30^o","35^o","-20^o", "-25^o", "-35^o", "Unknown (-30^o)"];
            strIncline = "-30^o";
            q_po = 7;
            c = c_d30;
        case 8 %-35
            thigh_ref = thigh_d35;
            knee_ref = knee_d35;
            ankle_ref = ankle_d35;
            
            thigh_sd =thigh_d35sd;
            ankle_sd =ankle_d35sd;
            knee_sd =knee_d35sd;
            
            thigh_traj = [thigh_0; thigh_i20;thigh_i25;thigh_i30; thigh_i35; thigh_d20; thigh_d25; thigh_d30];
            incline={'in35'};
            leg = ["0^o","20^o", "25^o", "30^o","35^o","-20^o", "-25^o", "-30^o", "Unknown (-35^o)"];
            strIncline = "-35^o";
            q_po = 7;
            c = c_d35;
        otherwise
            thigh_ref = thigh_d30;
            knee_ref = knee_d30;
            ankle_ref = ankle_d30;
            
            thigh_sd =thigh_d30sd;
            ankle_sd =ankle_d30sd;
            knee_sd =knee_d30sd;
            
            thigh_traj = [thigh_0; thigh_i20;thigh_i25;thigh_i30; thigh_i35; thigh_d20; thigh_d25; thigh_d35];
            incline={'in30'};
            leg = ["0^o","20^o", "25^o", "30^o","35^o","-20^o", "-25^o", "-35^o", "Unknown (-30^o)"];
            strIncline = "-30^o";
            
            q_po = 7;
            c = c_i30;
    end
end

% calculate knee
Y_k = fft(knee_ref);
L = length(Y_k);
pk_real = real(Y_k/(L/2));
pk_im = imag(Y_k/(L/2));


% calculate ankle
Y_a = fft(ankle_ref);
L = length(Y_a);
f = 116*(0:(L/2))/L;
pa_real = real(Y_a/(L/2));
pa_im = imag(Y_a/(L/2));

thigh_stream = thigh_ref;%thigh_s3'% smooth([thigh_s1 thigh_s3],10)';
knee_stream = knee_ref;%knee_s3' % smooth([knee_s1 knee_s3],10)';
ankle_stream = ankle_ref; % smooth([ankle_s1 ankle_s3],10)';

%PV params
prevPV = 0;
prevState = 1;
sm = 0;
qhm = 0;
thighd_stream = ddt(thigh_stream);
t = linspace(0,1,length(thigh_mean));

qh_max = max(thigh_stream);
qh_min = min(thigh_stream);



%peak detection parameters
%threshold and min distance
% thresh = .00001;
minpeakh = 15;
minpeakd = 50;
mhf = 0;
temp_traj = [];
ypeak = [];
xpeak = [];


%peak detection parameters
%threshold and min distance
thresh = .0000001;
minpeakh_mhe = 6;
minpeakd_mhe = 6;
thigh_start_mhe = 7;
mhe = 0;
temp_traj_MHE = [];
pv_swing_thresh = .65;
ytrough = [];
xtrough = [];
current_min = 0;

xq = qh_min;
syms sh

for i = 1:length(t)
    
    thigh = thigh_stream(i);
    thighd = thighd_stream(i);
    
    
    if current_min ~= qh_min
        
        
    end
    
    
    
    if prevState == 2|| prevState == 3
        temp_traj_MHE = [temp_traj_MHE -thigh];
        if length(temp_traj_MHE) > 3
            [pks_MHE,locs_MHE] = findpeaks(temp_traj_MHE,'threshold', thresh,'MinPeakHeight',-minpeakh_mhe);
            if ~isempty(pks_MHE)
                temp_traj_MHE = [];
                ytrough = -[ytrough pks_MHE];
                xtrough = [xtrough i-1];
                qh_min = mean(ytrough);
                mhe = 1;
                
            end
        end
        
        
    end
    if prevState == 3 || prevState == 4
        if thigh > 15 %&& i-p > minpeakd
            temp_traj = [temp_traj thigh];
            if length(temp_traj) > 3
                temp_traj;
                [pks,locs] = findpeaks(temp_traj,'threshold', thresh,'MinPeakHeight',minpeakh);
                if ~isempty(pks)
                    p = i-1;
                    temp_traj = [];
                    ypeak = [ypeak pks];
                    xpeak = [xpeak p];
                    temp_max = qh_max;
                    qh_max = mean([ypeak max(thigh_stream)]);
                    mhf = 1
                end
            end
            
        end
    end
    %
    [currPV,currState,sm,qhm] = calculatePhaseVariable_Stair(thigh, thighd, qh_min, qh_max,q_po, c, prevState,prevPV, sm, qhm,mhf, pv_swing_thresh);
    %
    
    
    s(i) = currPV;
    prevState = currState;
    state(i) = currState;
    prevPV = currPV;
    
    
%     
%     knee_dec_est(i) = double(subs(hk_dec, sh, s(i)));
%     ankle_dec_est(i) = double(subs(ha_dec, sh, s(i)));
    
end

xq = abs(qh_max - qh_min);
N_k = 14;
N_a = 14;
hk_romvc = .5*interp1(x,vkreal_vc(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_vc(N_k/2+1,:),xq,'linear','extrap')*cos(pi*N_k*sh);
ha_romvc = .5*interp1(x,vareal_vc(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_vc(N_a/2+1,:),xq,'linear','extrap')*cos(pi*N_a*sh);

% hk_dec = .5*interp1(x,vkreal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vkim_dec(N_k/2+1,:),xq,'linear','extrap')*cos(pi*N_k*sh);
% ha_dec = .5*interp1(x,vareal_dec(1,:),xq,'linear','extrap')+.5*interp1(x,vaim_dec(N_a/2+1,:),xq,'linear','extrap')*cos(pi*N_a*sh);
% 
% hk_vc = .5*pk_real_vc(1) + .5*pk_real_vc(N_k/2)*cos(pi*N_k*sh);
% ha_vc = .5*pa_real_vc(1) + .5*pa_real_vc(N_a/2)*cos(pi*N_a*sh);
% 
% hk = .5*pk_real(1) + .5*pk_real(N_k/2)*cos(pi*N_k*sh);
% ha = .5*pa_real(1) + .5*pa_real(N_a/2)*cos(pi*N_a*sh);

for k = 1:N_k/2-1
    
     hk_romvc = hk_romvc + interp1(x,vkreal_vc(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_vc(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
%     hk = hk + pk_real(k+1)*cos(2*pi*(k)*sh)-pk_im(k+1)*sin(2*pi*(k)*sh);
%     hk_vc = hk_vc + pk_real_vc(k+1)*cos(2*pi*(k)*sh)-pk_im_vc(k+1)*sin(2*pi*(k)*sh);
%     hk_dec = hk_dec + interp1(x,vkreal_dec(k+1,:),xq,'linear','extrap')*cos(2*pi*(k)*sh)-interp1(x,vkim_dec(k+1,:),xq,'linear','extrap')*sin(2*pi*(k)*sh);
end

for a = 1:N_a/2-1
    
    ha_romvc = ha_romvc + interp1(x,vareal_vc(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_vc(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
    
%     ha_dec = ha_dec + interp1(x,vareal_dec(a+1,:),xq,'linear','extrap')*cos(2*pi*(a)*sh)-interp1(x,vaim_dec(a+1,:),xq,'linear','extrap')*sin(2*pi*(a)*sh);
%     ha = ha + pa_real(a+1)*cos(2*pi*(a)*sh)-pa_im(a+1)*sin(2*pi*(a)*sh);
%     ha_vc = ha_vc + pa_real_vc(a+1)*cos(2*pi*(a)*sh)-pa_im_vc(a+1)*sin(2*pi*(a)*sh);
end

knee_romvc_est = double(subs(hk_romvc, sh, s));
ankle_romvc_est = double(subs(ha_romvc, sh, s));
% current_min = xq;

estimatedAngles.knee.(incline{1}) = knee_romvc_est;
estimatedAngles.ankle.(incline{1}) = ankle_romvc_est;

gc = linspace(0,100,length(t));
figure
sgtitle(strcat("ROM Stair Walking Cross Validation: Leave Out ",num2str(strIncline)))
subplot(311)
plot(gc,thigh_traj(1,:),'linewidth',2)
hold on
plot(gc,thigh_traj(2,:),'linewidth',2)
plot(gc,thigh_traj(3,:),'linewidth',2)
plot(gc,thigh_traj(4,:),'linewidth',2)
plot(gc,thigh_traj(5,:),'linewidth',2)
plot(gc,thigh_traj(6,:),'linewidth',2)
plot(gc,thigh_traj(7,:),'linewidth',2)
plot(gc,thigh_traj(8,:),'linewidth',2)
plot(gc,thigh_stream,'--','linewidth',2)
grid on
legend(leg)
ylabel('Thigh Angle (^o)')
title('Thigh Joint Position')

% subplot(412)
% yyaxis left
% plot(gc,s,'k')
% ylabel('Phase Variable')
% yyaxis right
% plot(gc,state)
% ylabel('State')
% title('Phase/State')
% grid on


subplot(312)


k1 = knee_stream+knee_sd;
k2 = knee_stream-knee_sd;

gc2 = [gc, fliplr(gc)];
inBetween = [k1,fliplr(k2)];
pk1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
hold on
pk2 = plot(gc, knee_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
pk3 = plot(gc, knee_romvc_est, '--','linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
% pk4 = plot(gc, knee_dec_est, ':','color','black','linewidth',2);
% xlabel('samples')
ylabel('Knee Angle (^o)')
title('Knee Joint Position')
legend([pk2,pk3],'Raw','Est(ROM and VC interp)','Location','Southeast')
% legend([pk2,pk3,pk4],'Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
grid on
kneeRMSE_romvc = RMSE(knee_romvc_est,knee_stream)
% kneeRMSE_inc = RMSE(knee_dec_est,knee_stream)

subplot(313)

a1 = ankle_stream+ankle_sd;
a2 = ankle_stream-ankle_sd;

gc2 = [gc, fliplr(gc)];
inBetween = [a1,fliplr(a2)];
pa1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
hold on
pa2 = plot(gc, ankle_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
pa3 = plot(gc, ankle_romvc_est,'--','linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
% pa4 = plot(gc, ankle_dec_est,':','color','black','linewidth',2);
legend([pa2,pa3],'Raw','Est(ROM and VC interp)','Location','Southeast')
% legend([pa2,pa3,pa4],'Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
ylabel('Ankle Angle (^o)')
xlabel('Gait Cycle (%)','FontWeight','bold')
title('Ankle Joint Position')
ankleRMSE_romvc = RMSE(ankle_romvc_est,ankle_stream)
% ankleRMSE_inc = RMSE(ankle_dec_est,ankle_stream)
grid on


save estimatedAngles.mat estimatedAngles
% gc = linspace(0,100,length(t));
% figure
% sgtitle(strcat("ROM Stair Walking Cross Validation: Leave Out ",num2str(strIncline)))
% subplot(411)
% plot(gc,thigh_traj(1,:),'linewidth',2)
% hold on
% plot(gc,thigh_traj(2,:),'linewidth',2)
% plot(gc,thigh_traj(3,:),'linewidth',2)
% plot(gc,thigh_traj(4,:),'linewidth',2)
% plot(gc,thigh_traj(5,:),'linewidth',2)
% plot(gc,thigh_traj(6,:),'linewidth',2)
% plot(gc,thigh_traj(7,:),'linewidth',2)
% plot(gc,thigh_traj(8,:),'linewidth',2)
% plot(gc,thigh_stream,'--','linewidth',2)
% grid on
% legend(leg)
% ylabel('Thigh Angle (^o)')
% title('Thigh Joint Position')
% 
% subplot(412)
% yyaxis left
% plot(gc,s,'k')
% ylabel('Phase Variable')
% yyaxis right
% plot(gc,state)
% ylabel('State')
% title('Phase/State')
% grid on
% 
% 
% subplot(413)
% 
% 
% k1 = knee_stream+knee_sd;
% k2 = knee_stream-knee_sd;
% 
% gc2 = [gc, fliplr(gc)];
% inBetween = [k1,fliplr(k2)];
% pk1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
% hold on
% pk2 = plot(gc, knee_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
% pk3 = plot(gc, knee_romvc_est, '--','linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
% % pk4 = plot(gc, knee_dec_est, ':','color','black','linewidth',2);
% % xlabel('samples')
% ylabel('Knee Angle (^o)')
% title('Knee Joint Position')
% legend([pk2,pk3],'Raw','Est(Inc and VC interp)','Location','Southeast')
% % legend([pk2,pk3,pk4],'Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
% grid on
% kneeRMSE_romvc = RMSE(knee_romvc_est,knee_stream)
% % kneeRMSE_inc = RMSE(knee_dec_est,knee_stream)
% 
% subplot(414)
% 
% a1 = ankle_stream+ankle_sd;
% a2 = ankle_stream-ankle_sd;
% 
% gc2 = [gc, fliplr(gc)];
% inBetween = [a1,fliplr(a2)];
% pa1 = fill(gc2,inBetween,[0.3010, 0.7450, 0.9330], 'linestyle','None');
% hold on
% pa2 = plot(gc, ankle_stream,'linewidth',2,'color',[0, 0.4470, 0.7410]);
% pa3 = plot(gc, ankle_romvc_est,'--','linewidth',2, 'color',[0.8500, 0.3250, 0.0980]);
% % pa4 = plot(gc, ankle_dec_est,':','color','black','linewidth',2);
% legend([pa2,pa3],'Raw','Est(Inc and VC interp)','Location','Southeast')
% % legend([pa2,pa3,pa4],'Raw','Est(Inc and VC interp)','Est(Inc interp Only)','Location','Southeast')
% ylabel('Ankle Angle (^o)')
% xlabel('Gait Cycle (%)','FontWeight','bold')
% title('Ankle Joint Position')
% ankleRMSE_romvc = RMSE(ankle_romvc_est,ankle_stream)
% % ankleRMSE_inc = RMSE(ankle_dec_est,ankle_stream)
% grid on

        