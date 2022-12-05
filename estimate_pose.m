function [x, R, T] = estimate_pose(K, DetAll, TagSize, frame_no, LandMarksComputed)
%ESTIMATE_POSE Summary of this function goes here
%   Detailed explanation goes here

    if frame_no == 1
        % Initialize the first frame and tag
        first_frame = sortrows(DetAll{frame_no});
        first_tag = first_frame(1,:);

        % World Coordinates
        Co_Wd = [[0,0];[TagSize,0];[TagSize,TagSize];[0,TagSize]];
        % Image Coordinates
        Co_Img = [[first_tag(2),first_tag(3)];[first_tag(4),first_tag(5)];[first_tag(6),first_tag(7)];[first_tag(8),first_tag(9)]];
        Img_X = Co_Img(:,1); Img_Y = Co_Img(:,2); Wd_X = Co_Wd(:,1); Wd_Y = Co_Wd(:,2);
    else
        Wd_X = []; Wd_Y = []; Img_X = []; Img_Y = [];
        % last world frame
        Fr_Wd = LandMarksComputed;
        % Imgent image frame
        Fr_Img = sortrows(DetAll{frame_no});

        for Img_Tag=1:size(Fr_Img,1)
            for lastTag=1:size(Fr_Wd,1)
                if (Fr_Wd(lastTag, 1) == Fr_Img(Img_Tag, 1))
                    Wd_X = [Wd_X; Fr_Wd(lastTag, 2); Fr_Wd(lastTag, 4); Fr_Wd(lastTag, 6); Fr_Wd(lastTag, 8)];
                    Wd_Y = [Wd_Y; Fr_Wd(lastTag, 3); Fr_Wd(lastTag, 5); Fr_Wd(lastTag, 7); Fr_Wd(lastTag, 9)];
                    Img_X = [Img_X; Fr_Img(Img_Tag, 2); Fr_Img(Img_Tag, 4); Fr_Img(Img_Tag, 6); Fr_Img(Img_Tag, 8)];
                    Img_Y = [Img_Y; Fr_Img(Img_Tag, 3); Fr_Img(Img_Tag, 5); Fr_Img(Img_Tag, 7); Fr_Img(Img_Tag, 9)];  
                end
            end
        end
    end

    H_pr = inv(K) * homography(Img_X, Img_Y, Wd_X, Wd_Y);
    
    [U, ~, V] = svd([H_pr(:,1), H_pr(:,2), cross(H_pr(:,1),H_pr(:,2))]);
    R = U * [1, 0, 0; 0, 1, 0; 0, 0, det(U*V')] * V';
    T = H_pr(:,3)/(norm(H_pr(:,1)));
    x = (-R'*T)';
    
end

