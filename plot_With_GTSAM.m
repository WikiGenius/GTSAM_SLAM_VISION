function plot_With_GTSAM(AllPosesComputed, LandMarksComputed)
%PLOT_WITH_GTSAM Summary of this function goes here

figure(2);
plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3),'o');
hold on;
title('Dataset with GTSAM');
xlabel('x');ylabel('y');zlabel('z');
plot3(LandMarksComputed(:,2),LandMarksComputed(:,3), LandMarksComputed(:,4), 'r*');
plot3(LandMarksComputed(:,5),LandMarksComputed(:,6), LandMarksComputed(:,7),'b*');
plot3(LandMarksComputed(:,8),LandMarksComputed(:,9), LandMarksComputed(:,10),'green*');
plot3(LandMarksComputed(:,11),LandMarksComputed(:,12), LandMarksComputed(:,13),'black*');  
hold off;
legend('All poses camera','p1','p2','p3','p4')
end

