function out=EA2DCM(phi, theta, psi);

%---------------------------------------------------------------------
% Filename : EA2DCM.m
% Author   : Ch. Eck
% Date     : 23.7.97
% Revision : 19.5.98, 19.3.99
%
% Description:
% Calculate the direction cosine matrix Cbn using as input the
% Euler angles (EA).
%
% Input  : Euler angles: phi, theta, psi
%          (phi,theta,psi)  --> Cbn (3x3)
%          [phi theta psi]  --> Cbn (3x3)
%          [phi theta psi]' --> Cbn (1x9) (rows added one to another)
% Output : Direction cosine matrix DCM called Cbn (body to nav)
%---------------------------------------------------------------------

[n,m] = size(phi);

if (n~=1)  % z.B. Ausruf aus Simulink-Blockschaltbild
  theta = phi(2);
  psi   = phi(3);
  phi   = phi(1);
end;

if (n==1)
  if (m==3)
    theta = phi(1,2);
    psi   = phi(1,3);
    phi   = phi(1,1);
  end;
end;
  
cp = cos(phi);
sp = sin(phi);
ct = cos(theta);
st = sin(theta);
cw = cos(psi);
sw = sin(psi);

out = [ ct*cw -cp*sw+sp*st*cw  sp*sw+cp*st*cw;...
        ct*sw  cp*cw+sp*st*sw -sp*cw+cp*st*sw;...
       -st     sp*ct           cp*ct        ];

if (n~=1)  % z.B. Ausruf aus Simulink-Blockschaltbild
  out = [out(1,1) out(1,2) out(1,3)...
         out(2,1) out(2,2) out(2,3)...
         out(3,1) out(3,2) out(3,3)];
end;
