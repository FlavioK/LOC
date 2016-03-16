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
FileNameHs           = 'data/20160316_203612_uVRU_NAV_HS.log';
FileNameLs           = 'data/20160316_203612_uVRU_NAV_LS.log';
rateHS               = 0.005;   % 200Hz
rateLS               = 0.2;     %   5Hz
figureFlag           = 1;
SaveFigures          = 1;
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


vru.dataLS.CogScaled(1:length(vru.dataLS.COG)) = nan;
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
    
  
%Compute roll angle:

%Offsets
accbix_bias =  0.025414
accbiy_bias =  0.090968
accbiz_bias = -0.12360
p_bias = -0.35134
q_bias =  0.097664
r_bias =  0.015934

%Build mean value
accbix = mean(vru.dataHS.accx);
accbiy = mean(vru.dataHS.accy);
accbiz = mean(vru.dataHS.accz);

roll = R2D*atan2(-(accbiy-accbiy_bias),-(accbiz-accbiz_bias))

     
