%% �����ı��ļ��е����ݡ�
% ���ڴ������ı��ļ��������ݵĽű�:
%
%    G:\�Զ��������\ϵͳ��ʶ\6,14�����������ʶ\��ʶ\DATA(2017.6.14-19.30.26).csv
%
% Ҫ��������չ������ѡ�����ݻ������ı��ļ��������ɺ���������ű���

% �� MATLAB �Զ������� 2017/06/14 20:49:27

%% ��ʼ��������
filename = 'G:\�Զ��������\ϵͳ��ʶ\6,14�����������ʶ\��ʶ\DATA(2017.6.14-19.30.26).csv';
delimiter = ',';
startRow = 2;

%% ÿ���ı��еĸ�ʽ:
%   ��1: ˫����ֵ (%f)
%	��2: ˫����ֵ (%f)
%   ��3: ˫����ֵ (%f)
%	��4: ˫����ֵ (%f)
%   ��5: ˫����ֵ (%f)
%	��6: ˫����ֵ (%f)
%   ��7: ˫����ֵ (%f)
%	��8: ˫����ֵ (%f)
%   ��9: ˫����ֵ (%f)
%	��10: ˫����ֵ (%f)
%   ��11: ˫����ֵ (%f)
%	��12: ˫����ֵ (%f)
%   ��13: ˫����ֵ (%f)
%	��14: ˫����ֵ (%f)
%   ��15: ˫����ֵ (%f)
%	��16: ˫����ֵ (%f)
%   ��17: ˫����ֵ (%f)
%	��18: ˫����ֵ (%f)
%   ��19: ˫����ֵ (%f)
%	��20: ˫����ֵ (%f)
%   ��21: ˫����ֵ (%f)
%	��22: ˫����ֵ (%f)
%   ��23: ˫����ֵ (%f)
% �й���ϸ��Ϣ������� TEXTSCAN �ĵ���
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% ���ı��ļ���
fileID = fopen(filename,'r');

%% ���ݸ�ʽ��ȡ�����С�
% �õ��û������ɴ˴������õ��ļ��Ľṹ����������ļ����ִ����볢��ͨ�����빤���������ɴ��롣
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% �ر��ı��ļ���
fclose(fileID);

%% ���޷���������ݽ��еĺ���
% �ڵ��������δӦ���޷���������ݵĹ�����˲�����������롣Ҫ�����������޷���������ݵĴ��룬�����ļ���ѡ���޷������Ԫ����Ȼ���������ɽű���

%% ����������������б�������
vxUSER_DATA1 = dataArray{:, 1};
vyUSER_DATA2 = dataArray{:, 2};
vzUSER_DATA3 = dataArray{:, 3};
axUSER_DATA4 = dataArray{:, 4};
ayUSER_DATA5 = dataArray{:, 5};
azUSER_DATA6 = dataArray{:, 6};
USER_DATA7 = dataArray{:, 7};
USER_DATA8 = dataArray{:, 8};
USER_DATA9 = dataArray{:, 9};
USER_DATA10 = dataArray{:, 10};
USER_DATA11 = dataArray{:, 11};
USER_DATA12 = dataArray{:, 12};
USER_DATA13 = dataArray{:, 13};
USER_DATA14 = dataArray{:, 14};
USER_DATA15 = dataArray{:, 15};
USER_DATA16 = dataArray{:, 16};
USER_DATA17 = dataArray{:, 17};
USER_DATA18 = dataArray{:, 18};
USER_DATA19 = dataArray{:, 19};
USER_DATA20 = dataArray{:, 20};
Angle_ROL = dataArray{:, 21};
Angle_PIT =- dataArray{:, 22};
Angle_YAW = dataArray{:, 23};


%% �����ʱ����
clearvars filename delimiter startRow formatSpec fileID dataArray ans;