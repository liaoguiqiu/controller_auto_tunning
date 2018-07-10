%% 导入文本文件中的数据。
% 用于从以下文本文件导入数据的脚本:
%
%    G:\自动控制相关\系统辨识\6,14四轴飞行器辨识\辨识\DATA(2017.6.14-19.30.26).csv
%
% 要将代码扩展到其他选定数据或其他文本文件，请生成函数来代替脚本。

% 由 MATLAB 自动生成于 2017/06/14 20:49:27

%% 初始化变量。
filename = 'G:\自动控制相关\系统辨识\6,14四轴飞行器辨识\辨识\DATA(2017.6.14-19.30.26).csv';
delimiter = ',';
startRow = 2;

%% 每个文本行的格式:
%   列1: 双精度值 (%f)
%	列2: 双精度值 (%f)
%   列3: 双精度值 (%f)
%	列4: 双精度值 (%f)
%   列5: 双精度值 (%f)
%	列6: 双精度值 (%f)
%   列7: 双精度值 (%f)
%	列8: 双精度值 (%f)
%   列9: 双精度值 (%f)
%	列10: 双精度值 (%f)
%   列11: 双精度值 (%f)
%	列12: 双精度值 (%f)
%   列13: 双精度值 (%f)
%	列14: 双精度值 (%f)
%   列15: 双精度值 (%f)
%	列16: 双精度值 (%f)
%   列17: 双精度值 (%f)
%	列18: 双精度值 (%f)
%   列19: 双精度值 (%f)
%	列20: 双精度值 (%f)
%   列21: 双精度值 (%f)
%	列22: 双精度值 (%f)
%   列23: 双精度值 (%f)
% 有关详细信息，请参阅 TEXTSCAN 文档。
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% 打开文本文件。
fileID = fopen(filename,'r');

%% 根据格式读取数据列。
% 该调用基于生成此代码所用的文件的结构。如果其他文件出现错误，请尝试通过导入工具重新生成代码。
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% 关闭文本文件。
fclose(fileID);

%% 对无法导入的数据进行的后处理。
% 在导入过程中未应用无法导入的数据的规则，因此不包括后处理代码。要生成适用于无法导入的数据的代码，请在文件中选择无法导入的元胞，然后重新生成脚本。

%% 将导入的数组分配给列变量名称
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


%% 清除临时变量
clearvars filename delimiter startRow formatSpec fileID dataArray ans;