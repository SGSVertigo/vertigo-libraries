function [ eul ] = vtg_quat2eul( q )
%VTG_QUAT2EUL
%
% This file is part of the Vertigo project
% Jon Sowman 2017
% jon+vertigo@jonsowman.com

    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    t0 = -2 * (qy * qy + qz * qz) + 1;
    t1 = 2 * (qx * qy - qw * qz);
    t2 = -2 * (qx * qz + qw * qy);
    t3 = 2 * (qy * qz - qw * qx);
    t4 = -2 * (qx * qx + qy * qy) + 1;
    
    if t2 > 1
        t2 = 1;
    end
    
    if t2 < -1
        t2 = -1;
    end
    
    pitch = asin(t2) * 2;
    roll = atan2(t3, t4);
    yaw = atan2(t1, t0);
    
    pitch = pitch * (180.0 / pi);
	roll = roll * (180.0 / pi);
	yaw = yaw * (180.0 / pi);
	%if (pitch < 0) pitch = 360.0 + pitch; end
	%if (roll < 0) roll = 360.0 + roll; end
	%if (yaw < 0) yaw = 360.0 + yaw;	end
    
    eul = [roll pitch yaw];

end
