clear all ;
close all ;
clc ;

rhos = 5; % Radius of the Circle

waypoints = [[0, 10,0]; 
             [60, 60,45]; 
             [80, 120,30]; 
             [150, 70, -90]; 
             [100, 30,-120]; 
             [50,0,-180];]';

Heading_angles = waypoints(3,:);
i=1;
j=1;
TH = 0:0.01:2*180 ;

while i< size(waypoints,2)

    csl(:,i) = [(waypoints(1,i) + (rhos*cosd(waypoints(3,i)+90))),(waypoints(2,i) + (rhos*sind(waypoints(3,i)+90)))];
    csr(:,i) = [(waypoints(1,i) - (rhos*cosd(waypoints(3,i)+90))),(waypoints(2,i) - (rhos*sind(waypoints(3,i)+90)))];
    cfl(:,i) = [(waypoints(1,i+1) + (rhos*cosd(waypoints(3,i+1)+90))),(waypoints(2,i+1) + (rhos*sind(waypoints(3,i+1)+90)))];
    cfr(:,i) = [(waypoints(1,i+1) - (rhos*cosd(waypoints(3,i+1)+90))),(waypoints(2,i+1) - (rhos*sind(waypoints(3,i+1)+90)))];
    
    Xc1(:,i) = csl(1,i) + rhos * cosd( TH ) ;
    Yc1(:,i) = csl(2,i) + rhos * sind( TH ) ;
    Xc2(:,i) = csr(1,i) + rhos * cosd( TH ) ;
    Yc2(:,i) = csr(2,i) + rhos * sind( TH ) ;
    
%     figure(1) ;
%     plot( Xc1, Yc1, 'b', 'LineWidth', 1 ) ;
%     hold on ;
%     plot( Xc2, Yc2, 'r', 'LineWidth', 1 ) ;
%     hold on ;
%     axis([ -10 180 -10 180 ]) ;
    
    C_LineLL(i) = [sqrt( (csl(1,i)-cfl(1,i))^2 + (csl(2,i)-cfl(2,i))^2 )];
    C_LineLR(i) = [sqrt( (csl(1,i)-cfr(1,i))^2 + (csl(2,i)-cfr(2,i))^2 )];
    C_LineRL(i) = [sqrt( (csr(1,i)-cfl(1,i))^2 + (csr(2,i)-cfl(2,i))^2 )];
    C_LineRR(i) = [sqrt( (csr(1,i)-cfr(1,i))^2 + (csr(2,i)-cfr(2,i))^2 )];

    PhieLL(i) = [asind((rhos-rhos)/C_LineLL(i))];
    PhieLR(i) = [asind((rhos+rhos)/C_LineLR(i))];
    PhieRL(i) = [asind((rhos+rhos)/C_LineRL(i))];
    PhieRR(i) = [asind((rhos-rhos)/C_LineRR(i))];
    
    psiLL(i) = atan2d((cfl(2,i)-csl(2,i)),(cfl(1,i)-csl(1,i)));
    psiLR(i) = atan2d((cfr(2,i)-csl(2,i)),(cfr(1,i)-csl(1,i)));
    psiRL(i) = atan2d((cfl(2,i)-csr(2,i)),(cfl(1,i)-csr(1,i)));
    psiRR(i) = atan2d((cfr(2,i)-csr(2,i)),(cfr(1,i)-csr(1,i)));

    PhiExLL(i) = PhieLL(i)-90+psiLL(i);
    PhiExLR(i) = PhieLR(i)-90+psiLR(i);
    PhiExRL(i) = PhieRL(i)+90+psiRL(i);
    PhiExRR(i) = PhieRR(i)+90+psiRR(i);
    
    TexLL(:,i) = [(csl(1,i)+(rhos*cosd(PhiExLL(i)))), (csl(2,i)+(rhos*sind(PhiExLL(i))))];
    TexLR(:,i) = [(csl(1,i)+(rhos*cosd(PhiExLR(i)))), (csl(2,i)+(rhos*sind(PhiExLR(i))))];
    TexRL(:,i) = [(csr(1,i)+(rhos*cosd(PhiExRL(i)))), (csr(2,i)+(rhos*sind(PhiExRL(i))))];
    TexRR(:,i) = [(csr(1,i)+(rhos*cosd(PhiExRR(i)))), (csr(2,i)+(rhos*sind(PhiExRR(i))))];
    
    PhiEnLL(i) = PhieLL(i)-90+psiLL(i);
    PhiEnLR(i) = PhieLR(i)+90+psiLR(i);
    PhiEnRL(i) = PhieRL(i)-90+psiRL(i);
    PhiEnRR(i) = PhieRR(i)+90+psiRR(i);

    TenLL(:,i) = [(cfl(1,i)+(rhos*cosd(PhiEnLL(i)))), (cfl(2,i)+(rhos*sind(PhiEnLL(i))))];
    TenLR(:,i) = [(cfr(1,i)+(rhos*cosd(PhiEnLR(i)))), (cfr(2,i)+(rhos*sind(PhiEnLR(i))))];
    TenRL(:,i) = [(cfl(1,i)+(rhos*cosd(PhiEnRL(i)))), (cfl(2,i)+(rhos*sind(PhiEnRL(i))))];
    TenRR(:,i) = [(cfr(1,i)+(rhos*cosd(PhiEnRR(i)))), (cfr(2,i)+(rhos*sind(PhiEnRR(i))))];

    i=i+1;
    j=j+1;


    if(i == size(waypoints,2))
        csl(:,i) = [(waypoints(1,i) + (rhos*cosd(waypoints(3,i)+90))),(waypoints(2,i) + (rhos*sind(waypoints(3,i)+90)))];
        csr(:,i) = [(waypoints(1,i) - (rhos*cosd(waypoints(3,i)+90))),(waypoints(2,i) - (rhos*sind(waypoints(3,i)+90)))];
        
        Xc1 = csl(1,i) + rhos * cosd( TH ) ;
        Yc1 = csl(2,i) + rhos * sind( TH ) ;
        Xc2 = csr(1,i) + rhos * cosd( TH ) ;
        Yc2 = csr(2,i) + rhos * sind( TH ) ;
        
%         figure(1) ;
%         plot( Xc1, Yc1, 'b', 'LineWidth', 1 ) ;
%         hold on ;
%         plot( Xc2, Yc2, 'r', 'LineWidth', 1 ) ;
%         hold on ;
%         axis([ -10 180 -10 180 ]) ;
    end
end
   
% j=0;
% 
%     line1x = [csl(1,1),csl(2,1)];
%     line1y = [cfr(1,1),cfr(2,1)];
%     plot(line1x, line1y, 'g', 'LineWidth', 1 ) ;
%     hold on;

% for s=1:6
%     xp = [TexLL(1,s), TenLL(1,s)];
%     yp = [TexLL(2,s), TenLL(2,s)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
% end

    
%     Dist(:,1) = [sqrt( (TexLL(1,1)-TenLL(1,1))^2 + (TexLL(2,1)-TenLL(2,1))^2 )];
%     Dist(:,2) = norm(TexLL-TenLL);

%     Dist(:,3) = norm([TexLL(1,1),TexLL(2,1)]-[TenLL(1,1),TenLL(2,1)]);

%     xp = [TexLL(1,1), TenLL(1,1)];
%     yp = [TexLL(2,1), TenLL(2,1)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
% 
%     xp = [TexLL(1,2), TenLR(1,2)];
%     yp = [TexLL(2,2), TenLR(2,2)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
% 
%     xp = [TexRR(1,3), TenRR(1,3)];
%     yp = [TexRR(2,3), TenRR(2,3)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
% 
%     xp = [TexRR(1,4), TenLL(1,4)];
%     yp = [TexRR(2,4), TenLL(2,4)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
%     
%     xp = [TexRR(1,5), TenRR(1,5)];
%     yp = [TexRR(2,5), TenRR(2,5)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;

% %%%%%%%%%%%%%
% 
%     xp = [TexLL(1,1), TenLL(1,1)];
%     yp = [TexLL(2,1), TenLL(2,1)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
% 
%     xp = [TexLL(1,2), TenLL(1,2)];
%     yp = [TexLL(2,2), TenLL(2,2)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
% 
%     xp = [TexLL(1,3), TenLL(1,3)];
%     yp = [TexLL(2,3), TenLL(2,3)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
% 
%     xp = [TexLL(1,4), TenLL(1,4)];
%     yp = [TexLL(2,4), TenLL(2,4)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
%     
%     xp = [TexLL(1,5), TenLL(1,5)];
%     yp = [TexLL(2,5), TenLL(2,5)];
%     plot(xp, yp, 'g', 'LineWidth', 1);
%     hold on;
% 
% %%%%%%%%%%%%%

    for d=1:size(TenLL,2)
        DistLL(:,d) = [sqrt( (TexLL(1,d)-TenLL(1,d))^2 + (TexLL(2,d)-TenLL(2,d))^2 )];
        DistLR(:,d) = [sqrt( (TexLR(1,d)-TenLR(1,d))^2 + (TexLR(2,d)-TenLR(2,d))^2 )];
        DistRL(:,d) = [sqrt( (TexRL(1,d)-TenRL(1,d))^2 + (TexRL(2,d)-TenRL(2,d))^2 )];
        DistRR(:,d) = [sqrt( (TexRR(1,d)-TenRR(1,d))^2 + (TexRR(2,d)-TenRR(2,d))^2 )];
        

        %% TexLL

        % Define the center of the circle
        centerTexLL = [csl(1,d),csl(2,d)];
    
        % Define two points on the circle
        point1TexLL = [waypoints(1,d),waypoints(2,d)];
        point2TexLL = [TexLL(1,d),TexLL(2,d)];
        
        point11TexLL = [waypoints(1,d),TexLL(1,d)];
        point12TexLL = [waypoints(2,d),TexLL(2,d)];
        dhosTexLL = sqrt((TexLL(1,d)-waypoints(1,d))^2 + (TexLL(2,d)-waypoints(2,d))^2);
        cos_thetaTexLL = (rhos^2 + rhos^2 - dhosTexLL^2) / (2 * rhos * rhos);
        thetareqTexLL(:,d) = acosd(cos_thetaTexLL);
%         plot( point11TexLL, point12TexLL, 'y', 'LineWidth', 2 ) ;
%         hold on ;

        % Calculate the vectors from the center to the points
        vec1TexLL = point1TexLL - centerTexLL;
        vec2TexLL = point2TexLL - centerTexLL;
        
        % Calculate the cross product of the vectors
        crossTexLL = vec1TexLL(1)*vec2TexLL(2) - vec1TexLL(2)*vec2TexLL(1);
        
        % Determine whether the rotation is clockwise or counterclockwise
        if crossTexLL > 0
            t1TexLL = atan2d(waypoints(2,d),waypoints(1,d));
            t2TexLL = atan2d(TexLL(2,d),TexLL(1,d));
            alphaTexLL(:,d) = (t1TexLL-t2TexLL);
            arc_lenTexLL = abs(rhos*alphaTexLL*pi/180);
            %disp('Counterclockwise rotation');
        elseif crossTexLL < 0
            t1TexLL = atan2d(waypoints(2,d),waypoints(1,d));
            t2TexLL = atan2d(TexLL(2,d),TexLL(1,d));
            alphaTexLL(:,d) = 360-abs(t1TexLL-t2TexLL);
            arc_lenTexLL = abs(rhos*alphaTexLL*pi/180);
            %disp('Clockwise rotation');
            %plot_arc(alphaTexLL(:,d),centerTexLL);
        else
            disp('The points are collinear');
        end

        %% TexLR
        
        centerTexLR = [csl(1,d),csl(2,d)];
        point1TexLR = [waypoints(1,d),waypoints(2,d)];
        point2TexLR = [TexLR(1,d),TexLR(2,d)];
        
        vec1TexLR = point1TexLR - centerTexLR;
        vec2TexLR = point2TexLR - centerTexLR;
        
        crossTexLR = vec1TexLR(1)*vec2TexLR(2) - vec1TexLR(2)*vec2TexLR(1);
        
        point11TexLR = [waypoints(1,d),TexLR(1,d)];
        point12TexLR = [waypoints(2,d),TexLR(2,d)];
        dhosTexLR = sqrt((TexLR(1,d)-waypoints(1,d))^2 + (TexLR(2,d)-waypoints(2,d))^2);
        cos_thetaTexLR = (rhos^2 + rhos^2 - dhosTexLR^2) / (2 * rhos * rhos);
        thetareqTexLR(:,d) = acosd(cos_thetaTexLR);
%         plot( point11TexLR, point12TexLR, 'y', 'LineWidth', 2 ) ;
%         hold on ;

        if crossTexLR > 0
            t1TexLR = atan2d(waypoints(2,d),waypoints(1,d));
            t2TexLR = atan2d(TexLR(2,d),TexLR(1,d));
            alphaTexLR(:,d) = (t1TexLR-t2TexLR);
            arc_lenTexLR = abs(rhos*alphaTexLR*pi/180);
            %disp('Counterclockwise rotation');
        elseif crossTexLR < 0
            t1TexLR = atan2d(waypoints(2,d),waypoints(1,d));
            t2TexLR = atan2d(TexLR(2,d),TexLR(1,d));
            alphaTexLR(:,d) = 360-abs(t1TexLR-t2TexLR);
            arc_lenTexLR = abs(rhos*alphaTexLR*pi/180);
            %disp('Clockwise rotation');
        else
            disp('The points are collinear');
        end

        %% TexRL

        centerTexRL = [csr(1,d),csr(2,d)];
        point1TexRL = [waypoints(1,d),waypoints(2,d)];
        point2TexRL = [TexRL(1,d),TexRL(2,d)];
        
        vec1TexRL = point1TexRL - centerTexRL;
        vec2TexRL = point2TexRL - centerTexRL;
        
        crossTexRL = vec1TexRL(1)*vec2TexRL(2) - vec1TexRL(2)*vec2TexRL(1);
        
        point11TexRL = [waypoints(1,d),TexRL(1,d)];
        point12TexRL = [waypoints(2,d),TexRL(2,d)];

        dhosTexRL = sqrt((TexRL(1,d)-waypoints(1,d))^2 + (TexRL(2,d)-waypoints(2,d))^2);
        cos_thetaTexRL = (rhos^2 + rhos^2 - dhosTexRL^2) / (2 * rhos * rhos);
        thetareqTexRL(:,d) = acosd(cos_thetaTexRL);
        
        if crossTexRL < 0
            t1TexRL = atan2d(waypoints(2,d),waypoints(1,d));
            t2TexRL = atan2d(TexRL(2,d),TexRL(1,d));
            alphaTexRL(:,d) = (t1TexRL-t2TexRL);
            arc_lenTexRL = abs(rhos*alphaTexRL*pi/180);
            %disp('Counterclockwise rotation');
        elseif crossTexRL > 0
            t1TexRL = atan2d(waypoints(2,d),waypoints(1,d));
            t2TexRL = atan2d(TexRL(2,d),TexRL(1,d));
            alphaTexRL(:,d) = 360-abs(t1TexRL-t2TexRL);
            arc_lenTexRL = abs(rhos*alphaTexRL*pi/180);
            %disp('Clockwise rotation');
        else
            disp('The points are collinear');
        end

        %% TexRR

        centerTexRR = [csr(1,d),csr(2,d)];
        point1TexRR = [waypoints(1,d),waypoints(2,d)];
        point2TexRR = [TexRR(1,d),TexRR(2,d)];
        
        vec1TexRR = point1TexRR - centerTexRR;
        vec2TexRR = point2TexRR - centerTexRR;
        
        crossTexRR = vec1TexRR(1)*vec2TexRR(2) - vec1TexRR(2)*vec2TexRR(1);
        
        dhosTexRR = sqrt((TexRR(1,d)-waypoints(1,d))^2 + (TexRR(2,d)-waypoints(2,d))^2);
        cos_thetaTexRR = (rhos^2 + rhos^2 - dhosTexRR^2) / (2 * rhos * rhos);
        thetareqTexRR(:,d) = acosd(cos_thetaTexRR);
        
        if crossTexRR < 0
            t1TexRR = atan2d(waypoints(2,d),waypoints(1,d));
            t2TexRR = atan2d(TexRR(2,d),TexRR(1,d));
            alphaTexRR(:,d) = (t1TexRR-t2TexRR);
            arc_lenTexRR = abs(rhos*alphaTexRR*pi/180);
            %disp('Counterclockwise rotation');
        elseif crossTexRR > 0
            t1TexRR = atan2d(waypoints(2,d),waypoints(1,d));
            t2TexRR = atan2d(TexRR(2,d),TexRR(1,d));
            alphaTexRR(:,d) = 360-abs(t1TexRR-t2TexRR);
            arc_lenTexRR = abs(rhos*alphaTexRR*pi/180);
            %disp('Clockwise rotation');
        else
            disp('The points are collinear');
        end

        %% TenLL

        centerTenLL = [cfl(1,d),cfl(2,d)];
        point1TenLL = [waypoints(1,d+1),waypoints(2,d+1)];
        point2TenLL = [TenLL(1,d),TenLL(2,d)];
        
        vec1TenLL = point1TenLL - centerTenLL;
        vec2TenLL = point2TenLL - centerTenLL;
        
        crossTenLL = vec2TenLL(1)*vec1TenLL(2) - vec2TenLL(2)*vec1TenLL(1);
        
        dhosTenLL = sqrt((TenLL(1,d)-waypoints(1,d+1))^2 + (TenLL(2,d)-waypoints(2,d+1))^2);
        cos_thetaTenLL = (rhos^2 + rhos^2 - dhosTenLL^2) / (2 * rhos * rhos);
        thetareqTenLL(:,d) = acosd(cos_thetaTenLL);
        
        if crossTenLL > 0
            t2TenLL = atan2d(waypoints(2,d+1),waypoints(1,d+1));
            t1TenLL = atan2d(TenLL(2,d),TenLL(1,d));
            alphaTenLL(:,d) = (t1TenLL-t2TenLL);
            arc_lenTenLL = abs(rhos*alphaTenLL*pi/180);
            %disp('Counterclockwise rotation');
        elseif crossTenLL < 0
            t2TenLL = atan2d(waypoints(2,d+1),waypoints(1,d+1));
            t1TenLL = atan2d(TenLL(2,d),TenLL(1,d));
            alphaTenLL(:,d) = 360-abs(t1TenLL-t2TenLL);
            arc_lenTenLL = abs(rhos*alphaTenLL*pi/180);
            %disp('Clockwise rotation');
        else
            disp('The points are collinear');
        end

        %% TenLR
    
        centerTenLR = [cfr(1,d),cfr(2,d)];
        point1TenLR = [waypoints(1,d+1),waypoints(2,d+1)];
        point2TenLR = [TenLR(1,d),TenLR(2,d)];
        
        vec1TenLR = point1TenLR - centerTenLR;
        vec2TenLR = point2TenLR - centerTenLR;
        
        crossTenLR = vec2TenLR(1)*vec1TenLR(2) - vec2TenLR(2)*vec1TenLR(1);
        
        dhosTenLR = sqrt((TenLR(1,d)-waypoints(1,d+1))^2 + (TenLR(2,d)-waypoints(2,d+1))^2);
        cos_thetaTenLR = (rhos^2 + rhos^2 - dhosTenLR^2) / (2 * rhos * rhos);
        thetareqTenLR(:,d) = acosd(cos_thetaTenLR);

        if crossTenLR < 0
            t2TenLR = atan2d(waypoints(2,d+1),waypoints(1,d+1));
            t1TenLR = atan2d(TenLR(2,d),TenLR(1,d));
            alphaTenLR(:,d) = (t1TenLR-t2TenLR);
            arc_lenTenLR = abs(rhos*alphaTenLR*pi/180);
            %disp('Counterclockwise rotation');
            %disp(t2TenLR)
            %disp(t1TenLR)
        elseif crossTenLR > 0
            t2TenLR = atan2d(waypoints(2,d+1),waypoints(1,d+1));
            t1TenLR = atan2d(TenLR(2,d),TenLR(1,d));
            alphaTenLR(:,d) = 360-abs(t1TenLR-t2TenLR);
            arc_lenTenLR = abs(rhos*alphaTenLR*pi/180);
            %disp('Clockwise rotation');
            %disp(t2TenLR)
            %disp(t1TenLR)
        else
            disp('The points are collinear');
        end

        %% TenRL
    
        centerTenRL = [cfl(1,d),cfl(2,d)];
        point1TenRL = [waypoints(1,d+1),waypoints(2,d+1)];
        point2TenRL = [TenRL(1,d),TenRL(2,d)];
        
        vec1TenRL = point1TenRL - centerTenRL;
        vec2TenRL = point2TenRL - centerTenRL;
        
        crossTenRL = vec2TenRL(1)*vec1TenRL(2) - vec2TenRL(2)*vec1TenRL(1);
        
        dhosTenRL = sqrt((TenRL(1,d)-waypoints(1,d+1))^2 + (TenRL(2,d)-waypoints(2,d+1))^2);
        cos_thetaTenRL = (rhos^2 + rhos^2 - dhosTenRL^2) / (2 * rhos * rhos);
        thetareqTenRL(:,d) = acosd(cos_thetaTenRL);

        if crossTenRL > 0
            t2TenRL = atan2d(waypoints(2,d+1),waypoints(1,d+1));
            t1TenRL = atan2d(TenRL(2,d),TenRL(1,d));
            alphaTenRL(:,d) = (t1TenRL-t2TenRL);
            arc_lenTenRL = abs(rhos*alphaTenRL*pi/180);
            %disp('Counterclockwise rotation');
        elseif crossTenRL < 0
            t2TenRL = atan2d(waypoints(2,d+1),waypoints(1,d+1));
            t1TenRL = atan2d(TenRL(2,d),TenRL(1,d));
            alphaTenRL(:,d) = 360-abs(t1TenRL-t2TenRL);
            arc_lenTenRL = abs(rhos*alphaTenRL*pi/180);
            %disp('Clockwise rotation');
        else
            disp('The points are collinear');
        end

        %% TenRR
        
        centerTenRR = [cfr(1,d),cfr(2,d)];
        point1TenRR = [waypoints(1,d+1),waypoints(2,d+1)];
        point2TenRR = [TenRR(1,d),TenRR(2,d)];
        
        vec1TenRR = point1TenRR - centerTenRR;
        vec2TenRR = point2TenRR - centerTenRR;
        
        crossTenRR = vec2TenRR(1)*vec1TenRR(2) - vec2TenRR(2)*vec1TenRR(1);
        
        dhosTenRR = sqrt((TenRR(1,d)-waypoints(1,d+1))^2 + (TenRR(2,d)-waypoints(2,d+1))^2);
        cos_thetaTenRR = (rhos^2 + rhos^2 - dhosTenRR^2) / (2 * rhos * rhos);
        thetareqTenRR(:,d) = acosd(cos_thetaTenRR);

        if crossTenRR < 0
            t2TenRR = atan2d(waypoints(2,d+1),waypoints(1,d+1));
            t1TenRR = atan2d(TenRR(2,d),TenRR(1,d));
            alphaTenRR(:,d) = (t1TenRR-t2TenRR);
            arc_lenTenRR = abs(rhos*alphaTenRR*pi/180);
            %disp('Counterclockwise rotation');
        elseif crossTenRR > 0
            t2TenRR = atan2d(waypoints(2,d+1),waypoints(1,d+1));
            t1TenRR = atan2d(TenRR(2,d),TenRR(1,d));
            alphaTenRR(:,d) = 360-abs(t1TenRR-t2TenRR);
            arc_lenTenRR = abs(rhos*alphaTenRR*pi/180);
            %disp('Clockwise rotation');
        else
            disp('The points are collinear');
        end
    
        Len_LL(:,d) = abs(arc_lenTexLL(d))+ abs(DistLL(:,d))+ abs(arc_lenTenLL(d));
        Len_LR(:,d) = abs(arc_lenTexLR(d))+ abs(DistLR(:,d))+ abs(arc_lenTenLR(d));
        Len_RL(:,d) = abs(arc_lenTexRL(d))+ abs(DistRL(:,d))+ abs(arc_lenTenRL(d));
        Len_RR(:,d) = abs(arc_lenTexRR(d))+ abs(DistRR(:,d))+ abs(arc_lenTenRR(d));

%         Path = [Len_LL;Len_LR;Len_RL;Len_RR];
%         
%         [min_length,min_index] = min(Path);
%         Path_Length =  sum(min_length);
        

end
    
% Loop to obtain the minimum path length required to traverse for the robot along the Dubins path
Path_len =0;
for l=1:1:size(Len_LL,2)
    [min_Line(:,l), min_idx(:,l)] = min([Len_LL(1,l),Len_LR(1,l),Len_RL(1,l),Len_RR(1,l)]);
    Path_len = min_Line(:,l)+Path_len; 
end

pathstr = ["The Path Length for the given configurations are: ", Path_len];
disp(pathstr)

Cur_head_angle = waypoints(1);
xpos=waypoints(1,1);
ypos=waypoints(2,1);
Cur_pos=([xpos;ypos]);

% Loop to obtain the Combination of Dubins configuration 'LSL', 'LSR', 'RSL', 'RSR'
for c=1:1:length(min_idx)
    if min_idx(c) == 1
        Path_Combi(c,:) = ('LSL');
        exit_c(:,c) = csl(:,c);
        entry_c(:,c) = cfl(:,c);
        exit_p(:,c) = TexLL(:,c);
        entry_p(:,c) = TenLL(:,c);
        exit_arclen(:,c) = arc_lenTexLL(:,c);
        entry_arclen(:,c) = arc_lenTenLL(:,c);
        exit_angle(:,c) = thetareqTexLL(:,c);
        entry_angle(:,c) = thetareqTenLL(:,c);
%        [Cur_pos,Cur_head_angle] = CCA_Straight(exit_p(:,c),entry_p(:,c), Cur_pos, Cur_head_angle);
        
        Xc1 = csl(1,c) + rhos * cosd( TH ) ;
        Yc1 = csl(2,c) + rhos * sind( TH ) ;
        Xc2 = cfl(1,c) + rhos * cosd( TH ) ;
        Yc2 = cfl(2,c) + rhos * sind( TH ) ;
        
        figure(1) ;
        plot( Xc1, Yc1, 'b', 'LineWidth', 1 ) ;
        hold on ;
        plot( Xc2, Yc2, 'r', 'LineWidth', 1 ) ;
        hold on ;
        xp = [TexLL(1,c), TenLL(1,c)];
        yp = [TexLL(2,c), TenLL(2,c)];
        plot(xp, yp, 'g', 'LineWidth', 1);
        hold on;
        
    elseif min_idx(c) == 2
        Path_Combi(c,:) = ('LSR');
        exit_c(:,c) = csl(:,c);
        entry_c(:,c) = cfr(:,c);
        exit_p(:,c) = TexLR(:,c);
        entry_p(:,c) = TenLR(:,c);
        exit_arclen(:,c) = arc_lenTexLR(:,c);
        entry_arclen(:,c) = arc_lenTenLR(:,c);
        exit_angle(:,c) = thetareqTexLR(:,c);
        entry_angle(:,c) = thetareqTenLR(:,c);
%         [Cur_pos,Cur_head_angle] = CCA_Straight(exit_p(:,c),entry_p(:,c), Cur_pos, Cur_head_angle);
        
        Xc1 = csl(1,c) + rhos * cosd( TH ) ;
        Yc1 = csl(2,c) + rhos * sind( TH ) ;
        Xc2 = cfr(1,c) + rhos * cosd( TH ) ;
        Yc2 = cfr(2,c) + rhos * sind( TH ) ;
        
        figure(1) ;
        plot( Xc1, Yc1, 'b', 'LineWidth', 1 ) ;
        hold on ;
        plot( Xc2, Yc2, 'r', 'LineWidth', 1 ) ;
        hold on ;
        xp = [TexLR(1,c), TenLR(1,c)];
        yp = [TexLR(2,c), TenLR(2,c)];
        plot(xp, yp, 'g', 'LineWidth', 1);
        hold on;

    elseif min_idx(c) == 3
        Path_Combi(c,:) = ('RSL');
        exit_c(:,c) = csr(:,c);
        entry_c(:,c) = cfl(:,c);
        exit_p(:,c) = TexRL(:,c);
        entry_p(:,c) = TenRL(:,c);
        exit_arclen(:,c) = arc_lenTexRL(:,c);
        entry_arclen(:,c) = arc_lenTenRL(:,c);
        exit_angle(:,c) = thetareqTexRL(:,c);
        entry_angle(:,c) = thetareqTenRL(:,c);
%         [Cur_pos,Cur_head_angle] = CCA_Straight(exit_p(:,c),entry_p(:,c), Cur_pos, Cur_head_angle);
         
        Xc1 = csr(1,c) + rhos * cosd( TH ) ;
        Yc1 = csr(2,c) + rhos * sind( TH ) ;
        Xc2 = cfl(1,c) + rhos * cosd( TH ) ;
        Yc2 = cfl(2,c) + rhos * sind( TH ) ;
        
        figure(1) ;
        plot( Xc1, Yc1, 'b', 'LineWidth', 1 ) ;
        hold on ;
        plot( Xc2, Yc2, 'r', 'LineWidth', 1 ) ;
        hold on ;
        xp = [TexRL(1,c), TenRL(1,c)];
        yp = [TexRL(2,c), TenRL(2,c)];
        plot(xp, yp, 'g', 'LineWidth', 1);
        hold on;

    elseif min_idx(c) == 4
        Path_Combi(c,:) = ('RSR');
        exit_c(:,c) = csr(:,c);
        entry_c(:,c) = cfr(:,c);
        exit_p(:,c) = TexRR(:,c);
        entry_p(:,c) = TenRR(:,c);
        exit_arclen(:,c) = arc_lenTexRR(:,c);
        entry_arclen(:,c) = arc_lenTenRR(:,c);
        exit_angle(:,c) = thetareqTexRR(:,c);
        entry_angle(:,c) = thetareqTenRR(:,c);
%         [Cur_pos,Cur_head_angle] = CCA_Straight(exit_p(:,c),entry_p(:,c), Cur_pos, Cur_head_angle);
         
        Xc1 = csr(1,c) + rhos * cosd( TH ) ;
        Yc1 = csr(2,c) + rhos * sind( TH ) ;
        Xc2 = cfr(1,c) + rhos * cosd( TH ) ;
        Yc2 = cfr(2,c) + rhos * sind( TH ) ;
        
        figure(1) ;
        plot( Xc1, Yc1, 'b', 'LineWidth', 1 ) ;
        hold on ;
        plot( Xc2, Yc2, 'r', 'LineWidth', 1 ) ;
        hold on ;
        xp = [TexRR(1,c), TenRR(1,c)];
        yp = [TexRR(2,c), TenRR(2,c)];
        plot(xp, yp, 'g', 'LineWidth', 1);
        hold on;
    end
end

% for e=1:1:1 %length(Heading_angles) %size(exit_p,2)
%     [xpos,ypos,head_angle] = CCA_Straight(exit_p(:,e),entry_p(:,e), exit_p(:,e), waypoints(3,e));
%     plot( [ exit_p(1,e), entry_p(1,e) ], [ exit_p(2,e), entry_p(2,e) ], 'y', 'LineWidth', 2 ) ;
%     hold on ;
% end

plot([0,0],[-10,200],'k');
hold on;
plot([-10,200],[0,0],'k');
hold on;
