function varargout = aplikasiGUI(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @aplikasiGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @aplikasiGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before aplikasiGUI is made visible.
function aplikasiGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to aplikasiGUI (see VARARGIN)

% Choose default command line output for aplikasiGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes aplikasiGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = aplikasiGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure Inisialisasi
% pertama untuk panel yang diaktifkan.
varargout{1} = handles.output;
set(handles.panel2, 'visible', 'off')
set(handles.panel3, 'visible', 'off')

% Executes on button press in pushbutton1 Untuk menampilkan Panel Informasi
function pushbutton1_Callback(hObject, eventdata, handles)
set(handles.panel, 'visible', 'off')
set(handles.panel2, 'visible', 'on')
set(handles.panel3, 'visible', 'off')
set(handles.axes1, 'visible', 'off')
set(handles.axes2, 'visible', 'off')
set(handles.pushbutton1, 'visible', 'off')
set(handles.pushbutton2, 'visible', 'off')
set(handles.pushbutton3, 'visible', 'on')
set(handles.pushbutton4, 'visible', 'off')
set(handles.pushbutton5, 'visible', 'off')
set(handles.pushbutton7, 'visible', 'off')
set(handles.text2, 'visible', 'off')
set(handles.text3, 'visible', 'off')
set(handles.text4, 'visible', 'off')
set(handles.text5, 'visible', 'off')
set(handles.text6, 'visible', 'off')
set(handles.text7, 'visible', 'off')
set(handles.text8, 'visible', 'off')
set(handles.text9, 'visible', 'off')
set(handles.text10, 'visible', 'off')
set(handles.text14, 'visible', 'on')
set(handles.text15, 'visible', 'off')
set(handles.text16, 'visible', 'off')
set(handles.text17, 'visible', 'off')

% Executes on button press in pushbutton2 Untuk menampilkan panel utama
function pushbutton2_Callback(hObject, eventdata, handles)
set(handles.panel, 'visible', 'off')
set(handles.panel2, 'visible', 'off')
set(handles.panel3, 'visible', 'on')
set(handles.axes1, 'visible', 'on')
set(handles.axes2, 'visible', 'on')
set(handles.pushbutton1, 'visible', 'off')
set(handles.pushbutton2, 'visible', 'off')
set(handles.pushbutton3, 'visible', 'on')
set(handles.pushbutton4, 'visible', 'on')
set(handles.pushbutton5, 'visible', 'on')
set(handles.pushbutton7, 'visible', 'on')
set(handles.text2, 'visible', 'off')
set(handles.text3, 'visible', 'off')
set(handles.text4, 'visible', 'off')
set(handles.text5, 'visible', 'off')
set(handles.text6, 'visible', 'off')
set(handles.text7, 'visible', 'off')
set(handles.text8, 'visible', 'off')
set(handles.text9, 'visible', 'off')
set(handles.text10, 'visible', 'off')
set(handles.text14, 'visible', 'on')
set(handles.text15, 'visible', 'on')
set(handles.text16, 'visible', 'on')
set(handles.text17, 'visible', 'on')

% Executes on button press in pushbutton3 Untuk Kembali ke panel menu
function pushbutton3_Callback(hObject, eventdata, handles)
set(handles.panel, 'visible', 'on')
set(handles.panel2, 'visible', 'off')
set(handles.pushbutton1, 'visible', 'on')
set(handles.pushbutton2, 'visible', 'on')
set(handles.pushbutton3, 'visible', 'off')
set(handles.text2, 'visible', 'on')
set(handles.text3, 'visible', 'on')
set(handles.text4, 'visible', 'on')
set(handles.text5, 'visible', 'on')
set(handles.text6, 'visible', 'on')
set(handles.text7, 'visible', 'on')
set(handles.text8, 'visible', 'on')
set(handles.text9, 'visible', 'on')
set(handles.text10, 'visible', 'on')
set(handles.text14, 'visible', 'off')

% Executes on button press in pushbutton4 Untuk mengunggah gambar ke
% aplikasi
function pushbutton4_Callback(hObject, eventdata, handles)
[filename,pathname]=uigetfile('*.jpg;*.png;*.jpeg');
if ~isequal(filename,0)
    handles.data1 = imread(fullfile(pathname,filename));
    guidata(hObject,handles);
    axes(handles.axes1)
    cla reset
    imshow(handles.data1);
else
    return
end

% --- Executes on button press in pushbutton5 Untuk mendeteksi wajah ke
% aplikasi
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tic
X = handles.data1;
thresholdFace = 1;
thresholdParts = 1;
stdsize = 176;

nameDetector = {'LeftEye'; 'RightEye'; 'Mouth'; 'Nose'; }; %viola jones di sini secara kerangka kerja menggunakan deteksi pada mata, mulut dan hidung.
mins = [[12 18]; [12 18]; [15 25]; [15 18]; ];

detector.stdsize = stdsize;
detector.detector = cell(5,1);
for k=1:4
    minSize = int32([stdsize/5 stdsize/5]);
    minSize = [max(minSize(1),mins(k,1)), max(minSize(2),mins(k,2))];
    detector.detector{k} = vision.CascadeObjectDetector(char(nameDetector(k)), 'MergeThreshold', thresholdParts, 'MinSize', minSize);
end

detector.detector{5} = vision.CascadeObjectDetector('FrontalFaceCART', 'MergeThreshold', thresholdFace);

bbox = step(detector.detector{5}, X);
bbsize = size(bbox);
partsNum = zeros(size(bbox,1),1);
stdsize = detector.stdsize;

for k=1:4
    if( k == 1 )
        region = [1,int32(stdsize*2/3); 1, int32(stdsize*2/3)];
    elseif( k == 2 )
        region = [int32(stdsize/3),stdsize; 1, int32(stdsize*2/3)];
    elseif( k == 3 )
        region = [1,stdsize; int32(stdsize/3), stdsize];
    elseif( k == 4 )
        region = [int32(stdsize/5),int32(stdsize*4/5); int32(stdsize/3),stdsize];
    else
        region = [1,stdsize;1,stdsize];
    end
    
    bb = zeros(bbsize);
    for i=1:size(bbox,1)
        XX = X(bbox(i,2):bbox(i,2)+bbox(i,4)-1,bbox(i,1):bbox(i,1)+bbox(i,3)-1,:);
        XX = imresize(XX,[stdsize, stdsize]);
        XX = XX(region(2,1):region(2,2),region(1,1):region(1,2),:);
        
        b = step(detector.detector{k},XX);
        
        if( size(b,1) > 0 )
            partsNum(i) = partsNum(i) + 1;
            
            if( k == 1 )
                b = sortrows(b,1);
            elseif( k == 2 )
                b = flipud(sortrows(b,1));
            elseif( k == 3 )
                b = flipud(sortrows(b,2));
            elseif( k == 4 )
                b = flipud(sortrows(b,3));
            end
            
            ratio = double(bbox(i,3)) / double(stdsize);
            b(1,1) = int32( ( b(1,1)-1 + region(1,1)-1 ) * ratio + 0.5 ) + bbox(i,1);
            b(1,2) = int32( ( b(1,2)-1 + region(2,1)-1 ) * ratio + 0.5 ) + bbox(i,2);
            b(1,3) = int32( b(1,3) * ratio + 0.5 );
            b(1,4) = int32( b(1,4) * ratio + 0.5 );
            
            bb(i,:) = b(1,:);
        end
    end
    bbox = [bbox,bb];
end

bbox = [bbox,partsNum];
bbox(partsNum<=2,:)=[];

face =  bbox(:,1: 4);
axes(handles.axes2)
cla reset
imshow(X);
hold on

[m, ~] = size(face);
for j = 1:m
    rectangle('Position',[face(j,1),face(j,2),face(j,3),face(j,4)],'EdgeColor','y','LineWidth',1);
end
hold off
toc

% Executes on button press in pushbutton untuk menyimpan gambar dari
% hasil deteksi
function pushbutton_Callback(hObject, eventdata, handles)
img = getframe(gca);
[filename2,pathname2] = uiputfile(...
    {'*.bmp','bitmap image (*.bmp)';
    '*.jpg','jpeg image(*.bmp)';
    '*.*','All file(*.*)'},...
    'Save Image');
if ~isequal(filename2,0)
    imwrite(img.cdata,fullfile(pathname2,filename2));
else
    return
end


% --- Executes on button press in pushbutton7 Untuk kembali ke panel menu
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.panel, 'visible', 'on')
set(handles.panel2, 'visible', 'off')
set(handles.panel3, 'visible', 'off')
set(handles.axes1, 'visible', 'off')
set(handles.axes2, 'visible', 'off')
set(handles.pushbutton1, 'visible', 'on')
set(handles.pushbutton2, 'visible', 'on')
set(handles.pushbutton3, 'visible', 'off')
set(handles.pushbutton4, 'visible', 'off')
set(handles.pushbutton5, 'visible', 'off')
set(handles.pushbutton7, 'visible', 'off')
set(handles.text2, 'visible', 'on')
set(handles.text3, 'visible', 'on')
set(handles.text4, 'visible', 'on')
set(handles.text5, 'visible', 'on')
set(handles.text6, 'visible', 'on')
set(handles.text7, 'visible', 'on')
set(handles.text8, 'visible', 'on')
set(handles.text9, 'visible', 'on')
set(handles.text10, 'visible', 'on')
set(handles.text14, 'visible', 'off')
set(handles.text15, 'visible', 'off')
set(handles.text16, 'visible', 'off')
set(handles.text17, 'visible', 'off')
