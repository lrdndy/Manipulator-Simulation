function qsolved =solveq(R,p)
    %this is ABBirb6640 Inverse Kinematics function to solve q
    abb6640def;n=size(abb6640.H,2);robdef=abb6640;radius=.1;
    [rob,colLink]=collisionBody(robdef,radius);
    ez = [0;0;1];ey = [0;1;0];ex = [1;0;0];
    q1=subprob4(-ez,ey,p-R*p6T,ey'*(p12+p23+p34));
    
    % solve for q3
    q3a=subprob3(ey,-p34,p23,norm(-p12+rotz(-q1(1))*(p-R*p6T)));
    q3b=subprob3(ey,-p34,p23,norm(-p12+rotz(-q1(2))*(p-R*p6T)));
    
    % solve for q2
    
    q2_a1=subprob1(ey,p23+roty(q3a(1))*p34,rotz(-q1(1))*(p-R*p6T)-p12);
    q2_a2=subprob1(ey,p23+roty(q3a(2))*p34,rotz(-q1(1))*(p-R*p6T)-p12);
    q2_b1=subprob1(ey,p23+roty(q3b(1))*p34,rotz(-q1(2))*(p-R*p6T)-p12);
    q2_b2=subprob1(ey,p23+roty(q3b(2))*p34,rotz(-q1(2))*(p-R*p6T)-p12);
         
    % put all 8 solutions together
    qsol=zeros(6,9);
    qsol(:,9)=zeros(6,1); % last column is the original
    qsol(1:3,1)=[q1(1);q2_a1;q3a(1)];
    qsol(1:3,2)=[q1(1);q2_a2;q3a(2)];
    qsol(1:3,3)=[q1(2);q2_b1;q3b(1)];
    qsol(1:3,4)=[q1(2);q2_b2;q3b(2)];
    
    % wrist angles
    
    for i=1:4
        qsol(1:3,i+4)=qsol(1:3,i);
        R03=rot(h1,qsol(1,i))*...
            rot(h2,qsol(2,i))*...
            rot(h3,qsol(3,i));
        %        [q4vec,q5vec]=subproblem2(h4,h5,h6,R03'*R*h6);
        [q4vec,q5vec]=subprob2(-h4,h5,R03'*R*h6,h6);
        q4a=q4vec(1);q4b=q4vec(2);  
        q5a=q5vec(1);q5b=q5vec(2);  
        qsol(4:5,i)=[q4a;q5a];
        qsol(4:5,i+4)=[q4b;q5b];  
        
        R05a=R03*rot(h4,q4a)*rot(h5,q5a);
        R05b=R03*rot(h4,q4b)*rot(h5,q5b);
        
        R56=R05a'*R;
        qsol(6,i)=atan2(.5*h6'*vee(R56-R56'),...
                        .5*(trace(R56)-1));
        R56=R05b'*R;
        qsol(6,i+4)=atan2(.5*h6'*vee(R56-R56'),...
                          .5*(trace(R56)-1));
    end
    qsolved = qsol(:,1); %only 1,4,5,8can be used in this workplace
end