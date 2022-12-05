function plot_Without_GTSAM(AllPosesComputed, LandMarksComputed)

	figure(1);
	plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3),'o');
	hold on;
	title('Dataset Without GTSAM');
	xlabel('x');ylabel('y');zlabel('z');
	plot3(LandMarksComputed(:,2),LandMarksComputed(:,3), zeros(81,1), 'r*');
	plot3(LandMarksComputed(:,4),LandMarksComputed(:,5), zeros(81,1),'b*');
	plot3(LandMarksComputed(:,6),LandMarksComputed(:,7), zeros(81,1),'green*');
	plot3(LandMarksComputed(:,8),LandMarksComputed(:,9), zeros(81,1),'black*');  
	legend('All poses camera','p1','p2','p3','p4')
	hold off;
end

