clearvars;
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
       disp('Connected to remote API server');
       [~,dum]= sim.simxGetObjectHandle(clientID,'IRB140_target',sim.simx_opmode_blocking);
        J = imread('Anh50.jpg'); % Replace 'test.jpg' with your image file
	K = imresize(J, 0.25);

	if size(K, 3) == 3
   		I = rgb2gray(K);
	else
    		I = K;
	end
	sf = 3; %upscaling factor
	sz = size(I);
	xg = 1:sz(1);
	yg = 1:sz(2);
	F = griddedInterpolant({xg,yg},double(I));
	F.Method = 'cubic';
	xq = (1:(1/sf)*(0.53):sz(1))'; %scale to upscaling factor*1px/nm
	yq = (1:(1/sf)*(0.53):sz(2))';
	I_re = im2bw(uint8(F({xq,yq})),0.5);
	I_c = I_re; %To control area changes smoothing

	% Below follows an iterative filtering algorithm for this application.
	sz = size(I_re);
	L1 = bwlabel(I_re);
	N = max(L1,[],'all');
	windowSize = 19; % Here some filtering, slight blurring then rethreshold to get rid of edges
	for k = 1:10 %k iterations
    		L1 = bwlabel(I_re);
    		I_b= zeros(sz);
    	% The following retrieves each feature and saves it in temp array
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
        % The following corrects windowsize for small particles
        if sum(I_temp(:) > 0) < 1*(windowSize^2)*pi*sf^2 
            window = round(windowSize/3);
        else
            window = windowSize;
        end
        kernel = ones(window)/window^2;
        I_b1 = conv2(I_temp, kernel, 'same'); % Filtering by 2dconvolution
        % The following prevents merging of particles or to the edge by adding a zero
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
	% Remove pixels sticking out
	I_2 = ~I_2;
	I_2 = bwmorph(I_2,'majority');
	I_2 = ~I_2;
	I_2 = bwmorph(I_2,'majority');
	[B, L] = bwboundaries(I_2);
	imshow(I_2);
	hold on;



	% Iterate through each boundary
	for k = 1:length(B)
    		boundary = B{k};
    		plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2);
	end
    
    %this part of code make the motion trajectory of robot arm 
    x= [];
    y= [];
    z= [];
    count =0;
    
    for k = 1:length(B)
        boundary =B{k};
        for i=1:length(boundary(:,2))
            count = count+1;
            x(count) = boundary(i,2);
            y(count) = boundary(i,1);
            z(count) = 0;
        end
        count = count -1;
        z(count)= 30;
    end
    for m= 1:length(x)
    [returnCode]=sim.simxSetObjectPosition(clientID,dum,-1,[-0.22+(x(m)*0.0008),-0.1+(y(m)*0.0008),(z(m)*0.004)+0.515],sim.simx_opmode_blocking);
    end
    [returnCode]=sim.simxSetObjectPosition(clientID,dum,-1,[-0.4,-0.45,0.625],sim.simx_opmode_blocking); %just to move robot arm away from drawing area
end
sim.simxFinish(-1);
sim.delete();