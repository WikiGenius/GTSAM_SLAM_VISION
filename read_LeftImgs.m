function [LeftImgs] = read_LeftImgs(path_LeftImgs)

%% load LeftImgs
if ~isfolder(path_LeftImgs)
  errorMessage = sprintf('Error: The following folder does not exist:\n%s', path_LeftImgs);
  uiwait(warndlg(errorMessage));
  return;
end
filePattern = fullfile(path_LeftImgs, '*.jpg');
jpegFiles = dir(filePattern);
LeftImgs = cell(length(jpegFiles),1);
for k = 1:length(jpegFiles)
  baseFileName = jpegFiles(k).name;
  fullFileName = fullfile(path_LeftImgs, baseFileName);
%   fprintf(1, 'Now reading %s\n', fullFileName);
  LeftImgs{k} = imread(fullFileName);
  % imshow(imageArray);  % Display image.
  % drawnow; % Force display to update immediately.
end

end

