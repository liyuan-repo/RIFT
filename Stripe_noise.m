function x_stripe = Stripe_noise(img1,sigma)

%-------------------- ��������----------------
% img=rgb2gray(img1);
img1=double(img1);
xorig = img1/255;
% xorig=double(img1);
[m,n,c] = size(xorig);
% u = sigma^2 *randn(m,n,c);  
u = sigma^2 *randn(m,n);
uu(:,:,1)=u;
uu(:,:,2)=u;
uu(:,:,3)=u;                  
kk=zeros(m,n,c);
for i=1:c
    kk(:,:,i)=xorig(:,:,i)*uu(:,:,i);
end
% kk=bsxfun(@times,u,xorig); % ����Ԫ�س˻�
xin = xorig+kk;
% xin = xorig + u * xorig;   % u��xorig֮��ע������Ԫ����˺;�����ˣ�������˵õ�����������Ԫ����˵õ��ߵ�����
x_stripe=255*xin;
x_stripe=uint8(x_stripe);



% map = gray(256); % 256*3 �����е�ÿһ�а���һ���ض���ɫ�ĺ졢�̡���ǿ�ȡ�ǿ�Ƚ��� [0,1] ��Χ�ڣ�

% subplot(121)
% % imshow(255*xorig,map)
% imshow(uint8(255*xorig))
% title('original image')
% subplot(122)
% % imshow(x_stripe,map)
% imshow(x_stripe)
% title('noise contaminated image')