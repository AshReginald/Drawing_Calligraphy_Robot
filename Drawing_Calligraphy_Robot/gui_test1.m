clearvars; % Xóa tất cả các biến trong không gian làm việc
sim=remApi('remoteApi'); % Khởi tạo đối tượng API từ remoteApi
sim.simxFinish(-1); % Đóng tất cả các kết nối trước đó
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); % Kết nối tới máy chủ CoppeliaSim tại địa chỉ và cổng xác định
if (clientID>-1)
    disp('Đã kết nối tới máy chủ API từ xa');
    
    % Chọn hình ảnh mới
    [fileName, filePath] = uigetfile({'*.jpg;*.png;*.bmp','Các tệp ảnh (*.jpg, *.png, *.bmp)'},'Chọn hình ảnh mới');
    if isequal(fileName,0)
        disp('Không có hình ảnh nào được chọn.');
        return;
    else
        disp(['Hình ảnh được chọn: ', fullfile(filePath,fileName)]);
        J = imread(fullfile(filePath,fileName)); % Đọc hình ảnh được chọn
    end
    
    [~,dum]= sim.simxGetObjectHandle(clientID,'IRB140_target',sim.simx_opmode_blocking); % Lấy handle của đối tượng mục tiêu IRB140
    
    K = imresize(J, 0.25); % Thay đổi kích thước hình ảnh

    if size(K, 3) == 3
        I = rgb2gray(K); % Chuyển đổi hình ảnh sang thang độ xám nếu hình ảnh là màu
    else
        I = K;
    end

    sf = 3; % Hệ số phóng đại
    sz = size(I);
    xg = 1:sz(1);
    yg = 1:sz(2);
    F = griddedInterpolant({xg,yg},double(I)); % Nội suy hình ảnh
    F.Method = 'cubic';
    xq = (1:(1/sf)*(0.53):sz(1))'; % Tỷ lệ phóng đại lên theo hệ số phóng đại
    yq = (1:(1/sf)*(0.53):sz(2))';
    I_re = im2bw(uint8(F({xq,yq})),0.5); % Chuyển đổi hình ảnh sang nhị phân
    I_c = I_re; % Để kiểm soát vùng thay đổi làm mịn

    % Thuật toán lọc lặp lại bên dưới để áp dụng cho ứng dụng này
    sz = size(I_re);
    L1 = bwlabel(I_re);
    N = max(L1,[],'all');
    windowSize = 19; % Lọc, làm mờ nhẹ rồi ngưỡng lại để loại bỏ các cạnh
    for k = 1:10 % lặp lại k lần
        L1 = bwlabel(I_re);
        I_b= zeros(sz);
        % Lấy từng đặc trưng và lưu vào mảng tạm thời
        for i = 1:N
            I_temp = zeros(sz);
            I_b1 = I_temp;
            for j = 1:sz(1)
                for jj = 1:sz(2)
                    if L1(j,jj) == i
                        I_temp(j,jj) = I_re(j,jj);
                    end
                end
            end
            % Điều chỉnh kích thước cửa sổ cho các hạt nhỏ
            if sum(I_temp(:) > 0) < 1*(windowSize^2)*pi*sf^2 
                window = round(windowSize/3);
            else
                window = windowSize;
            end
            kernel = ones(window)/window^2;
            I_b1 = conv2(I_temp, kernel, 'same'); % Lọc bằng phép tích chập 2D
            % Ngăn chặn hợp nhất các hạt hoặc tới cạnh bằng cách thêm một không
            for j = 1:sz(1)
                for jj = 1:sz(2)
                    if I_b(j,jj) == 0 && I_b1(j,jj) > 0
                        I_b(j,jj) = I_b1(j,jj);
                    elseif I_b(j,jj) ~= 0 && I_b1(j,jj) ~= 0
                        I_b(j,jj) = 0; 
                    elseif j == sz(1) || jj == sz(2)
                        I_b(j,jj) = 0; 
                    end
                end
            end
        end
        I_re = I_b > 0.5;
    end
    I_2 = I_re;
    % Loại bỏ các pixel nhô ra
    I_2 = ~I_2;
    I_2 = bwmorph(I_2,'majority');
    I_2 = ~I_2;
    I_2 = bwmorph(I_2,'majority');
    [B, L] = bwboundaries(I_2);
    imshow(I_2); % Hiển thị hình ảnh đã lọc
    hold on;

    % Duyệt qua từng biên
    for k = 1:length(B)
        boundary = B{k};
        plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2); % Vẽ đường viền
    end
    
    % Phần này của mã tạo ra đường đi của cánh tay robot
    x= [];
    y= [];
    z= [];
    count =0;
    
    for k = 1:length(B)
        boundary = B{k};
        for i=1:length(boundary(:,2))
            count = count+1;
            x(count) = boundary(i,2);
            y(count) = boundary(i,1);
            z(count) = 0;
        end
        count = count -1;
        z(count)= 30; % Điều chỉnh trục Z cho mỗi biên
    end
    for m= 1:length(x)
        [returnCode]=sim.simxSetObjectPosition(clientID,dum,-1,[-0.22+(x(m)*0.0008),-0.1+(y(m)*0.0008),(z(m)*0.004)+0.515],sim.simx_opmode_blocking); % Thiết lập vị trí của đối tượng
    end
    [returnCode]=sim.simxSetObjectPosition(clientID,dum,-1,[-0.4,-0.45,0.625],sim.simx_opmode_blocking); % Di chuyển cánh tay robot ra khỏi khu vực vẽ

    % Lấy handle của camera
    [~, camHandle] = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking);

    % Lấy dữ liệu hình ảnh từ camera
    [returnCode,resolution,image]=sim.simxGetVisionSensorImage(clientID,camHandle,0,sim.simx_opmode_streaming);

    % Hiển thị hình ảnh từ camera
    if returnCode == sim.simx_return_ok
        img = reshape(image,[resolution(1),resolution(2),3]);
        figure;
        imshow(img); % Hiển thị hình ảnh từ camera
    else
        disp('Lỗi khi lấy hình ảnh từ camera.');
    end
end
sim.simxFinish(-1); % Đóng kết nối với CoppeliaSim
sim.delete(); % Xóa đối tượng API
