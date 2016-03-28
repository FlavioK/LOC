clear all; close all; fclose all; clc;
%% Script for iSDK
%
%
% -------------------------------------------------------------------------
% Version | Comment                    |   Date   |            | Author 
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
% v1.0    | Initial Version            | 11.04.14 |            | Tim
% -------------------------------------------------------------------------
%
%
% -------------------------------------------------------------------------
% (c) iMAR Navigation | http://www.imar-navigation.de
%
%% Settings
FileNameHs           = 'circle/20160316_204950_uVRU_NAV_HS.log';
FileNameLs           = 'circle/20160316_204950_uVRU_NAV_LS.log';
rateHS               = 0.005;   % 200Hz
rateLS               = 0.2;     %   5Hz
figureFlag           = 0;
SaveFigures          = 0;
FigPath              = 'Fig';
addtit               = '';
ext                  = '.fig';
R2D                  = 180/pi;
D2R                  = pi/180;

%% Read Data
DataHs = load(FileNameHs);
DataLs = load(FileNameLs);
   
% High Speed Package
vru.headerHS.FrameCnt = DataHs(:,2);
vru.headerHS.Week     = DataHs(:,5);
vru.headerHS.TOW      = DataHs(:,6);
vru.headerHS.TimeStamp= DataHs(:,7);
vru.dataHS.accx       = DataHs(:,8);
vru.dataHS.accy       = DataHs(:,9);
vru.dataHS.accz       = DataHs(:,10);
vru.dataHS.omgx       = DataHs(:,11);
vru.dataHS.omgy       = DataHs(:,12);
vru.dataHS.omgz       = DataHs(:,13);
vru.dataHS.rpyx       = DataHs(:,14);
vru.dataHS.rpyy       = DataHs(:,15);
vru.dataHS.rpyz       = DataHs(:,16);
vru.dataHS.ImuTemp    = DataHs(:,17);
vru.dataHS.vn         = DataHs(:,18);
vru.dataHS.ve         = DataHs(:,19);
vru.dataHS.vd         = DataHs(:,20);
vru.dataHS.lon        = DataHs(:,21);
vru.dataHS.lat        = DataHs(:,22);
vru.dataHS.alt        = DataHs(:,23);
vru.dataHS.magHdg     = DataHs(:,24);
vru.dataHS.odo        = DataHs(:,25);
vru.dataHS.baroAlt    = DataHs(:,26);
vru.statHS.SysStat    = DataHs(:,27);
vru.statHS.AddStat    = DataHs(:,28);
vru.statHS.EKF        = DataHs(:,29);
vru.dataHS.time       = 0:rateHS:(length(vru.dataHS.lon-1)-1)/(1/rateHS);
vru.headerHS.TimeComp = (vru.headerHS.TOW/1000) + (vru.headerHS.TimeStamp/1000000);
vru.headerHS.TimeCompDiff = diff(vru.headerHS.TimeComp);

vru.status.EKF.AlignComp      = bitget(vru.statHS.EKF,1);
vru.status.EKF.BiasMean       = bitget(vru.statHS.EKF,2);
vru.status.EKF.LowDxnamics    = bitget(vru.statHS.EKF,3);
vru.status.EKF.Error          = bitget(vru.statHS.EKF,4);
vru.status.EKF.LlhAiding      = bitget(vru.statHS.EKF,5);
vru.status.EKF.LatMeas        = bitget(vru.statHS.EKF,6);
vru.status.EKF.LonMeas        = bitget(vru.statHS.EKF,7);
vru.status.EKF.AltMeas        = bitget(vru.statHS.EKF,8);
vru.status.EKF.velNedAiding   = bitget(vru.statHS.EKF,9);
vru.status.EKF.VnMeas         = bitget(vru.statHS.EKF,10);
vru.status.EKF.VeMeas         = bitget(vru.statHS.EKF,11);
vru.status.EKF.VdMeas         = bitget(vru.statHS.EKF,12);
vru.status.EKF.velBdyAiding   = bitget(vru.statHS.EKF,13);
vru.status.EKF.VxMeas         = bitget(vru.statHS.EKF,14);
vru.status.EKF.VyMeas         = bitget(vru.statHS.EKF,15);
vru.status.EKF.VzMeas         = bitget(vru.statHS.EKF,16);
vru.status.EKF.MagAiding      = bitget(vru.statHS.EKF,17);
vru.status.EKF.MagMeas        = bitget(vru.statHS.EKF,18);
vru.status.EKF.CogAiding      = bitget(vru.statHS.EKF,19);
vru.status.EKF.CogMeas        = bitget(vru.statHS.EKF,20);
vru.status.EKF.TasAiding      = bitget(vru.statHS.EKF,21);
vru.status.EKF.TasMeas        = bitget(vru.statHS.EKF,22);
vru.status.EKF.BaroAltAiding  = bitget(vru.statHS.EKF,23);
vru.status.EKF.BaroAltMeas    = bitget(vru.statHS.EKF,24);
vru.status.EKF.BaroRateAiding = bitget(vru.statHS.EKF,25);
vru.status.EKF.BaroRateMeas   = bitget(vru.statHS.EKF,26);

vru.status.EKF.FSMMODE1   = bitget(vru.statHS.EKF,29);
vru.status.EKF.FSMMODE2   = bitget(vru.statHS.EKF,30);
vru.status.EKF.FSMMODE3   = bitget(vru.statHS.EKF,31);
vru.status.EKF.FSMMODE4   = bitget(vru.statHS.EKF,32);


% Low Speed Package
vru.headerLS.FrameCnt  = DataLs(:,2);
vru.headerLS.Week      = DataLs(:,5);
vru.headerLS.TimeStamp = DataLs(:,6);
vru.dataLS.TOW         = DataLs(:,7);
vru.dataLS.Week        = DataLs(:,8);
vru.dataLS.GpsFix      = DataLs(:,9);
vru.dataLS.GpsNavFlags = DataLs(:,10);
vru.dataLS.Lon         = DataLs(:,11);
vru.dataLS.Lat         = DataLs(:,12);
vru.dataLS.Alt         = DataLs(:,13);
vru.dataLS.Vn          = DataLs(:,14);
vru.dataLS.Ve          = DataLs(:,15);
vru.dataLS.Vd          = DataLs(:,16);
vru.dataLS.COG         = DataLs(:,17);
vru.dataLS.SV          = DataLs(:,18);
vru.dataLS.PDOP        = DataLs(:,20);
vru.dataLS.time        = 0:rateLS:(length(vru.dataLS.Lon-1)-1)/(1/rateLS);


%Compute bias during calib time (first 30s)     
accbix_bias = mean(vru.dataHS.accx(3000:5000))
accbiy_bias = mean(vru.dataHS.accy(3000:5000))
accbiz_bias = mean(vru.dataHS.accz(3000:5000)) + 9.81

p_bias = mean(vru.dataHS.omgx(3000:5000))
q_bias = mean(vru.dataHS.omgy(3000:5000))
r_bias = mean(vru.dataHS.omgz(3000:5000))

yaw_uIMU = vru.dataHS.rpyz ;
%compute yaw
dt = vru.dataHS.time(2)-vru.dataHS.time(1)
w_z = (vru.dataHS.omgz-r_bias)*D2R;


yaw = 1 : length(vru.dataHS.time);
yaw(1) = yaw_uIMU(1);
%Forward Rule
for i=1:length(yaw)-1
    yaw(i+1) = yaw(i)+dt*w_z(i+1);
    if(yaw(i+1)>=pi)
        yaw(i+1) = yaw(i+1) - (2 * pi);
    end
    if(yaw(i+1)<=(-pi))
      yaw(i+1) = yaw(i+1) + (2 * pi);
    end
end


figure(1)
 subplot(2, 1, 1);
 plot(vru.dataHS.time,yaw*R2D)
 title('Computed \psi');
 xlabel('Time[s]');
 ylabel('Angle [\circ]');
 xlim([30 47])
 subplot(2, 1, 2);
 plot(vru.dataHS.time,yaw_uIMU*R2D)
 title('uIMU \psi');
 xlabel('Time[s]');
 ylabel('Angle [\circ]');
 xlim([30 47])
 saveas(gcf,'Yaw_forward');
yaw_t = 1 : length(vru.dataHS.time);
%Trapez Rule
yaw_t(1) = yaw_uIMU(1);


for i=1:length(yaw_t)-1
    yaw_t(i+1) = yaw_t(i)+dt*(w_z(i)+w_z(i+1))/2;
    if(yaw_t(i+1)>=pi)
        yaw_t(i+1) = yaw_t(i+1) - (2 * pi);
    end
    if(yaw_t(i+1)<=(-pi))
      yaw_t(i+1) = yaw_t(i+1) + (2 * pi);
    end
end

final_error_trapeze = (yaw_t(1)-yaw_t(10139))*R2D
final_error_forward = (yaw(1)-yaw(10139))*R2D
final_error_uIMU = (yaw_uIMU(1)-yaw_uIMU(10139))*R2D

figure(2)
 subplot(2, 1, 1);
 plot(vru.dataHS.time,yaw_t*R2D)
 title('Computed \psi');
 xlabel('Time[s]');
 ylabel('Angle [\circ]');
 xlim([30 47])
 subplot(2, 1, 2);
 plot(vru.dataHS.time,yaw_uIMU*R2D)
 title('uIMU \psi');
 xlabel('Time[s]');
 ylabel('Angle [\circ]');
 xlim([30 47])
  saveas(gcf,'Yaw_trapeze');

 %plot all angle
 figure(3)
  plot(vru.dataHS.time,yaw_uIMU*R2D,'b')
  hold on
  plot(vru.dataHS.time,yaw_t*R2D,'r')
  plot(vru.dataHS.time,yaw*R2D,'m')
  title('All Angles');
  xlabel('Time[s]');
  ylabel('Angle [\circ]');
  legend('Yaw_uIME','Yaw_trapez','Yaw_forward')
  xlim([30 47])
hold off
  saveas(gcf,'Yaw_all');
 
  
for i=1:length(vru.dataLS.COG)
    vru.dataLS.CogScaled(i) = mod(vru.dataLS.COG(i), 360);
    if(vru.dataLS.CogScaled(i) > 360/2)
        vru.dataLS.CogScaled(i) = vru.dataLS.CogScaled(i) - 360;
    else
        if(vru.dataLS.CogScaled(i) < -360/2)
            vru.dataLS.CogScaled(i) = vru.dataLS.CogScaled(i) + 360;
        else
            vru.dataLS.CogScaled(i) = vru.dataLS.CogScaled(i);
        end
    end
end        




%% Plot Data
if(figureFlag == 1)    
     if(SaveFigures == 1)                         
         if(exist(FigPath,'dir'))
             cd(FigPath)
         else
            mkdir(FigPath) 
            cd(FigPath)
         end
     end
     
     % Acceleration
     figure('Name', 'iuVRU Accelerometers');
     a(1) = subplot(4, 1, 1);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.accx, 'r');
     title('Acceleration [m/s�]','fontsize',12);
     ylabel('AccX [m/s�]');
     grid;
     hold off;

     a(2) = subplot(4, 1, 2);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.accy, 'r')
     ylabel('AccY [m/s�]');
     grid;
     hold off;

     a(3) = subplot(4, 1, 3);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.accz, 'r')     
     ylabel('AccZ [m/s�]');
     grid;
     hold off;
     
     a(4) = subplot(4, 1, 4);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.ImuTemp, 'b')
     xlabel('Time [sec]');
     ylabel('Temperature [�C]');
     grid;
     hold off;
    
     linkaxes([a(1) a(2) a(3) a(4)],'x');  % Base Y-limits on bottom subplot
     clear a;
    
     if(SaveFigures == 1)
        saveas(gcf,['Acc',addtit,ext]);
     end
     
     % Rates
     figure('Name', 'iuVRU Gyros');
     a(1) = subplot(3, 1, 1);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.omgx, 'r');
     title('Turn Rates [deg/sec]','fontsize',12);
     ylabel('OmgX [deg/sec]');
     grid;
     hold off;

     a(2) = subplot(3, 1, 2);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.omgy, 'r')
     ylabel('OmgY [deg/sec]');
     grid;
     hold off;

     a(3) = subplot(3, 1, 3);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.omgz, 'r')
     xlabel('Time [sec]');
     ylabel('OmgZ [deg/sec]');
     grid;
     hold off;
    
     linkaxes([a(1) a(2) a(3)],'x');  % Base Y-limits on bottom subplot
     clear a;
    
     if(SaveFigures == 1)
        saveas(gcf,['Omg',addtit,ext]);
     end
     
     % Attitude
     figure('Name', 'iuVRU Attitude');
     a(1) = subplot(3, 1, 1);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.rpyx*R2D, 'r');
     title('Attitude [deg]','fontsize',12);
     ylabel('Roll [deg]');
     grid;
     hold off;

     a(2) = subplot(3, 1, 2);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.rpyy*R2D, 'r')
     ylabel('Pitch [deg]');
     grid;
     hold off;

     a(3) = subplot(3, 1, 3);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.rpyz*R2D, 'r')
     plot(vru.dataLS.time,vru.dataLS.CogScaled, 'b')
     plot(vru.dataHS.time,vru.dataHS.magHdg*R2D, 'm')
     
     xlabel('Time [sec]');
     ylabel('Yaw [deg]');
     grid;
     legend('INS', 'GNSS (COG)', 'MAG');
     hold off;
    
     linkaxes([a(1) a(2) a(3)],'x');  % Base Y-limits on bottom subplot
     clear a;
    
     if(SaveFigures == 1)
        saveas(gcf,['Rpy',addtit,ext]);
     end
     
     % Velocity
     figure('Name', 'iuVRU Velocity');
     a(1) = subplot(4, 1, 1);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.vn, 'r');
     plot(vru.dataLS.time,vru.dataLS.Vn, 'b');
     title('Velocity [m/s]','fontsize',12);
     ylabel('Vn [m/s]');
     grid;
     legend('INS', 'GNSS');
     hold off;

     a(2) = subplot(4, 1, 2);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.ve, 'r')
     plot(vru.dataLS.time,vru.dataLS.Ve, 'b');
     ylabel('Ve [m/s]');
     grid;
     legend('INS', 'GNSS');
     hold off;

     a(3) = subplot(4, 1, 3);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.vd, 'r')
     plot(vru.dataLS.time,vru.dataLS.Vd, 'b');
     ylabel('Vd [m/s]');
     grid;
     legend('INS', 'GNSS');
     hold off;
     
     a(4) = subplot(4, 1, 4);
     hold on;    
     plot(vru.dataHS.time,sqrt(vru.dataHS.vn.^2 + vru.dataHS.ve.^2 + vru.dataHS.vd.^2)*3.6, 'r')
     plot(vru.dataLS.time,sqrt(vru.dataLS.Vn.^2 + vru.dataLS.Ve.^2 + vru.dataLS.Vd.^2)*3.6, 'b')
     xlabel('Time [sec]');
     ylabel('Speed [km/h]');
     grid;
     legend('INS', 'GNSS');
     hold off;
    
     linkaxes([a(1) a(2) a(3) a(4)],'x');  % Base Y-limits on bottom subplot
     clear a;
    
     if(SaveFigures == 1)
        saveas(gcf,['Vel',addtit,ext]);
     end
     
    % Position
     figure('Name', 'iuVRU Position');
     a(1) = subplot(3, 1, 1);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.lon, 'r');
     plot(vru.dataLS.time,vru.dataLS.Lon, 'b');
     legend('INS', 'GNSS');
     title('Position','fontsize',12);
     ylabel('Longitude [deg]');
     grid;
     hold off;

     a(2) = subplot(3, 1, 2);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.lat, 'r')
     plot(vru.dataLS.time,vru.dataLS.Lat, 'b');
     legend('INS', 'GNSS');
     ylabel('Latitude [deg]');
     xlabel('Time [sec]');
     grid;
     hold off;

     a(3) = subplot(3, 1, 3);
     hold on;    
     plot(vru.dataHS.time,vru.dataHS.alt, 'r')
     plot(vru.dataLS.time,vru.dataLS.Alt, 'b')
     plot(vru.dataHS.time,vru.dataHS.baroAlt, 'm')
     legend('INS', 'GNSS', 'Baro');
     ylabel('Altitude');
     grid;
     hold off;
    
     if(SaveFigures == 1)
        saveas(gcf,['Pos',addtit,ext]);
     end
     
     figure('Name', 'iuVRU PositionPlot');     
     hold on;    
     plot(vru.dataHS.lon,vru.dataHS.lat, 'r');
     plot(vru.dataLS.Lon,vru.dataLS.Lat, 'b');
     legend('INS', 'GNSS');
     title('Position','fontsize',12);
     ylabel('Latitude [deg]');
     xlabel('Longitude [deg]');
     grid;     
     
     % GNSS
     figure('Name', 'iuVRU GNSS');
     a(1) = subplot(3, 1, 1);
     hold on;    
     plot(vru.dataLS.time,vru.dataLS.PDOP, 'r');
     title('GNSS Data','fontsize',12);
     ylabel('PDOP ');
     grid;
     hold off;

     a(2) = subplot(3, 1, 2);
     hold on;    
     plot(vru.dataLS.time,vru.dataLS.SV, 'r')
     ylabel('Sats');
     grid;
     hold off;

     a(3) = subplot(3, 1, 3);
     hold on;    
     plot(vru.dataLS.time,vru.dataLS.GpsFix, 'r')
     xlabel('Time [sec]');
     ylabel('Gps Fix');
     grid;
     hold off;
    
     linkaxes([a(1) a(2) a(3)],'x');  % Base Y-limits on bottom subplot
     clear a;
    
     if(SaveFigures == 1)
        saveas(gcf,['GNSS',addtit,ext]);
     end
     
     
     
     
    if(SaveFigures)
        cd('..');
    end
end


