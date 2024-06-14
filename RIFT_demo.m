% This is a simplest implementation of the proposed RIFT algorithm. In this implementation,...
% rotation invariance part and corner point detection are not included.

clc;clear;close all;
warning('off')

file_image1= '.\Images\1DSMsets\pair7-2.png';
file_image2= '.\Images\1DSMsets\pair7-1.png';

%% 1 Import and display reference and image to be registered
image_1=imread(file_image1);
image_2=imread(file_image2);
[m,n,c1] = size(image_1);
[i,j,c2] = size(image_2);
if m ~= n || i~=j  || m ~= i 
    kk=max([m,n,i,j]);
    image_1 = imresize(image_1,[kk, kk]);
    image_2 = imresize(image_2,[kk, kk]);
end
save_path  = '.\outputs\1DSMsets\pair7r1.jpg';
%% -----------Additive noise--------------- % SNR=0;
% SNR=-1;
% [image_1,noise]=Additive_noise(image_1,SNR);
% save_path  = '.\outputs\1DSMsets\add_pair1+0.jpg';

%% ------------Stripe noise---------------- % sigma=0.10;
% sigma=0.12;
% image_1 = Stripe_noise(image_1,sigma);
% save_path  = '.\outputs\1DSMsets\stripe_pair1+0p10.jpg';

%% 2  Setting of initial parameters 

if size(image_1,3)==3
    im1 = rgb2gray(image_1);
else
    im1 = image_1;
end

if size(image_2,3)==3
    im2 = rgb2gray(image_2);
end

t1=clock;
%% Keypoint matching.
disp('RIFT feature detection and description')
% RIFT feature detection and description
[des_m1,des_m2] = RIFT_no_rotation_invariance(im1,im2,4,6,96);

disp('nearest matching')
% nearest matching
[indexPairs,matchmetric] = matchFeatures(des_m1.des,des_m2.des,'MaxRatio',1,'MatchThreshold', 100);
matchedPoints1 = des_m1.kps(indexPairs(:, 1), :);
matchedPoints2 = des_m2.kps(indexPairs(:, 2), :);
[matchedPoints2,IA]=unique(matchedPoints2,'rows');
matchedPoints1=matchedPoints1(IA,:);
%% Outlier removal
disp('outlier removal')
%outlier removal
% H=FSC(matchedPoints1,matchedPoints2,'affine',2);
[H,rmse]=FSC(matchedPoints1,matchedPoints2,'affine',3);
Y_=H*[matchedPoints1';ones(1,size(matchedPoints1,1))];
Y_(1,:)=Y_(1,:)./Y_(3,:);
Y_(2,:)=Y_(2,:)./Y_(3,:);
E=sqrt(sum((Y_(1:2,:)-matchedPoints2').^2));
inliersIndex=E<3;
cleanedPoints1 = matchedPoints1(inliersIndex, :);
cleanedPoints2 = matchedPoints2(inliersIndex, :);

t2=clock;
disp(['Total time cost on RIFT:',num2str(etime(t2,t1)),'s']);  
%% show results
disp('Show matches')
disp(['RMSE of Matching results: ',num2str(rmse),'  pixel']);

figure; showMatchedFeatures(image_1, image_2, cleanedPoints1, cleanedPoints2, 'montage');
correct_num=size(cleanedPoints1,1);

text(50,10,'RIFT:','horiz','center','color','r','FontSize',15);
text(60,30,['Correct num:',num2str(correct_num)],'horiz','center','color','r','FontSize',15);
text(60,50,['RMSE:',num2str(rmse)],'horiz','center','color','r','FontSize',15);
text(60,70,['RT:',num2str(etime(t2,t1)),' S'],'horiz','center','color','r','FontSize',15);

set(gcf,'color',[1,1,1]) 
f=getframe(gcf);
imwrite(f.cdata,save_path)

% disp('registration result')
% registration
% image_fusion(im2,im1,double(H));
