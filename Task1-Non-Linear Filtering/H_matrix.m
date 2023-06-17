% Function to obtain the H matrix

function H =H_matrix(xa,r,b,l,fg)

if fg==0
    for i=1:length(l)
        if i==1
        H_1=[-r(1,i)*cos(b(1,i)+xa(3,1))/r(1,i) ,-r(1,i)*sin(b(1,i)+xa(3,1))/r(1,i), 0;
                r(1,i)*sin(b(1,i)+xa(3,1))/(r(1,i))^2 ,-r(1,i)*cos(b(1,i)+xa(3,1))/(r(1,i))^2, -1];
        elseif i==2
        H_2=[-r(1,i)*cos(b(1,i)+xa(3,1))/r(1,i) ,-r(1,i)*sin(b(1,i)+xa(3,1))/r(1,i), 0;
                r(1,i)*sin(b(1,i)+xa(3,1))/(r(1,i))^2 ,-r(1,i)*cos(b(1,i)+xa(3,1))/(r(1,i))^2, -1];
        elseif i==3
         H_3=[-r(1,i)*cos(b(1,i)+xa(3,1))/r(1,i) ,-r(1,i)*sin(b(1,i)+xa(3,1))/r(1,i), 0;
                r(1,i)*sin(b(1,i)+xa(3,1))/(r(1,i))^2 ,-r(1,i)*cos(b(1,i)+xa(3,1))/(r(1,i))^2, -1];
        elseif i==4
         H_4=[-r(1,i)*cos(b(1,i)+xa(3,1))/r(1,i) ,-r(1,i)*sin(b(1,i)+xa(3,1))/r(1,i), 0;
                r(1,i)*sin(b(1,i)+xa(3,1))/(r(1,i))^2 ,-r(1,i)*cos(b(1,i)+xa(3,1))/(r(1,i))^2, -1];
        elseif i==5
         H_5=[-r(1,i)*cos(b(1,i)+xa(3,1))/r(1,i) ,-r(1,i)*sin(b(1,i)+xa(3,1))/r(1,i), 0;
                r(1,i)*sin(b(1,i)+xa(3,1))/(r(1,i))^2 ,-r(1,i)*cos(b(1,i)+xa(3,1))/(r(1,i))^2, -1];
        elseif i==6
         H_6=[-r(1,i)*cos(b(1,i)+xa(3,1))/r(1,i) ,-r(1,i)*sin(b(1,i)+xa(3,1))/r(1,i), 0;
                r(1,i)*sin(b(1,i)+xa(3,1))/(r(1,i))^2 ,-r(1,i)*cos(b(1,i)+xa(3,1))/(r(1,i))^2, -1];
        end

    end

    H=[H_1;H_2;H_3;H_4;H_5;H_6];
else
    H_1=[-r(1,fg)*cos(b(1,fg)+xa(3,1))/r(1,fg) ,-r(1,fg)*sin(b(1,fg)+xa(3,1))/r(1,fg), 0;
                r(1,fg)*sin(b(1,fg)+xa(3,1))/(r(1,fg))^2 ,-r(1,fg)*cos(b(1,fg)+xa(3,1))/(r(1,fg))^2, -1];
    H= H_1;
end

end