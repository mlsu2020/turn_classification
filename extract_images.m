bag = rosbag('C:\Users\jknaup3\Downloads\beta_autorally4_2018-03-29-15-34-10_split0.bag');
%%
left_images = select(bag, 'Topic', '/left_camera/image_color/compressed');
% [left_images_ts, left_images_cols] = timeseries(left_images);
msgs = readMessages(left_images, 'DataFormat','struct');
%%
for ii = 1:length(msgs)
    image = readImage(copyImage(msg{ii}));
    imshow(image);
    path = sprintf('images/img%d.png', ii);
    imwrite(image, path);
end

function msg = copyImage(msgStruct)
    msg = rosmessage(msgStruct.MessageType);
    fNames = fieldnames(msg);
    for k = 1:numel(fNames)
        if ~any(strcmp(fNames{k}, {'MessageType', 'Header'}))
            msg.(fNames{k}) = msgStruct.(fNames{k});
        end
    end
end