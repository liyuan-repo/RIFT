%���Ҷ�ͼ�����ָ������ȵ�����
%�ȼ����źŵĹ���,�ٻ�������Ĺ���,Ȼ������һ����׼��˹�ֲ�����ֵΪ0����׼��Ϊ1��
%���������У����źų���һ����,��ͨ��ת���õ�����������Ҫ�ĸ�˹����.
function [X1_noise,noise]=Additive_noise(X,SNR)
% X��ԭ�Ҷ�ͼ��,ʹ��imread����
% SNR��ָ���������
% X1_noise:����������ͼ��
% noise������ӵ�����
X1=im2double(X); %ֻ���ͼ�񣬿��Ի���X1=double(X)/255; 
%figure;imshow(X1)
[m,n,c]=size(X1);
noise=randn(m,n);
noise=noise-mean2(noise);  %��ֵΪ0������ӽ�1

avg1=mean2(X1);
s1=0;
for i = 1:m
    for j = 1:n
        s1 = s1+(X1(i,j)-avg1)^2;
        %s1 = s1+X(i,j)^2;  %δ����ȥ����ֵ
    end
end
signal_power=s1/(m*n);%�ź�ƽ������
noise_variance=signal_power*10^(-SNR/10);
noise=sqrt(noise_variance)/std2(noise)*noise;  %����������
X1_noise=X1+noise; %�õ�ָ������ȵĻҶ�ͼ��
%figure;imshow(X1_noise)
return


