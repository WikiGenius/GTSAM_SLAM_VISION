function LandMarksComputed = record_world_landmarks_frame(DetAll, Data, intrinsics, frame_no, LandMarksComputed)
%RECORD_WORLD_LANDMARKS_FRAME Summary of this function goes here
%   frame_no: frame number
R = Data.R{frame_no};
T = Data.T{frame_no};
% Record the world landmarks for each frame
curr_Fr = sortrows(DetAll{frame_no});
for k=1:size(curr_Fr, 1)
	tag = curr_Fr(k,:);
	if numel(LandMarksComputed) == 0 | ~ismember(tag(1),LandMarksComputed(:,1))
		Co_Img = [[tag(2),tag(3)];[tag(4),tag(5)];[tag(6),tag(7)];[tag(8),tag(9)]];
		LandMarks = pointsToWorld(intrinsics,R',T,Co_Img);
		LandMarksComputed = [LandMarksComputed; [tag(1), reshape(LandMarks',1,[])]];
	end
end

