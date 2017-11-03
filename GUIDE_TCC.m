function varargout = GUIDE_TCC(varargin)
% GUIDE_TCC MATLAB code for GUIDE_TCC.fig
%      GUIDE_TCC, by itself, creates a new GUIDE_TCC or raises the existing
%      singleton*.
%
%      H = GUIDE_TCC returns the handle to a new GUIDE_TCC or the handle to
%      the existing singleton*.
%
%      GUIDE_TCC('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUIDE_TCC.M with the given input arguments.
%
%      GUIDE_TCC('Property','Value',...) creates a new GUIDE_TCC or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUIDE_TCC_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUIDE_TCC_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUIDE_TCC

% Last Modified by GUIDE v2.5 09-Sep-2017 19:10:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUIDE_TCC_OpeningFcn, ...
                   'gui_OutputFcn',  @GUIDE_TCC_OutputFcn, ...
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


% --- Executes just before GUIDE_TCC is made visible.
function GUIDE_TCC_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUIDE_TCC (see VARARGIN)

% Choose default command line output for GUIDE_TCC
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

myImage = imread('titulo.jpg');
axes(handles.axes1);
imshow(myImage);


% --- Outputs from this function are returned to the command line.
function varargout = GUIDE_TCC_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_run.
function pushbutton_run_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%flag=get(hObject,'string');
%if strcmp(flag,'SIMULAR')==1
set_param(handles.modelname,'SimulationCommand','Start');
%set(hObject,'string','PARAR');
%else
%set_param(handles.modelname,'SimulationCommand','Stop');
%set(hObject,'string','SIMULAR');
%end
guidata(hObject,handles);

function edit_simfile_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit_simfile_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_simbrowse.
function pushbutton_simbrowse_Callback(hObject, eventdata, handles)
[input_file,pathname] = uigetfile( ...
       {'*mdl','mdl Files (*.mdl)';...
        '*.*', 'All Files (*.*)'}, ...
        'Select files', ... 
        'MultiSelect', 'on');
if pathname == 0
    return
end
%gets the current data file names inside the listbox
inputfile= fullfile(pathname,input_file);
current_folder=strcat(cd,'\');
mdlname=strrep(inputfile,current_folder,'');
mdlname=strrep(mdlname,'.mdl','');
%updates the gui to display all filenames in the listbox
set(handles.edit_simfile,'String',mdlname);
guidata(hObject, handles);

% --- Executes on button press in pushbutton_simremove.
function pushbutton_simremove_Callback(hObject, eventdata, handles)
modelname=get(handles.edit_simfile,'String');
save_system(modelname);
close_system(modelname);
handles=guidata(hObject);
set(handles.edit_simfile,'String','');
set(handles.edit_simfile,'BackgroundColor', [1 1 1]);
guidata(hObject, handles);


% --- Executes on button press in pushbutton_loadmodel.
function pushbutton_loadmodel_Callback(hObject, eventdata, handles)
modelname=get(handles.edit_simfile,'string');
set(handles.edit_simfile,'BackgroundColor', [1 0 0]);
if isempty(modelname)
    errordlg('Voce deve carregar o modelo!!!');
end
checkload=~isempty(find_system('type','block_diagram','name',modelname));
if checkload==0
    try
    load_system(modelname);
    catch
    end
end

block_scope_s_real = sprintf('%s/SAIDA REAL',modelname);
block_scope_s_desejada = sprintf('%s/SAIDA DESEJADA',modelname);
block_scope_erro = sprintf('%s/ERRO',modelname);
block_scope_cartesiano = sprintf('%s/CARTESIANO',modelname);
block_scope_trajetoriaxy = sprintf('%s/TRAJETORIA XY',modelname);
block_scope_realdesejada = sprintf('%s/REAL X DESEJADA',modelname);
block_scope_torque = sprintf('%s/TORQUE',modelname);

block_KI1=sprintf('%s/integrador/KI1',modelname);
block_KI2=sprintf('%s/integrador/KI2',modelname);
block_KI3=sprintf('%s/integrador/KI3',modelname);

block_KP1=sprintf('%s/proporcional/KP1',modelname);
block_KP2=sprintf('%s/proporcional/KP2',modelname);
block_KP3=sprintf('%s/proporcional/KP3',modelname);

block_KD1=sprintf('%s/derivador/KD1',modelname);
block_KD2=sprintf('%s/derivador/KD2',modelname);
block_KD3=sprintf('%s/derivador/KD3',modelname);


KP1=get_param(block_KP1,'Gain');
KP2=get_param(block_KP2,'Gain');
KP3=get_param(block_KP3,'Gain');

KI1=get_param(block_KI1,'Gain');
KI2=get_param(block_KI2,'Gain');
KI3=get_param(block_KI3,'Gain');

KD1=get_param(block_KD1,'Gain');
KD2=get_param(block_KD2,'Gain');
KD3=get_param(block_KD3,'Gain');


set(handles.slider_KP1,'value',str2double(KP1));
set(handles.slider_KP2,'value',str2double(KP2));
set(handles.slider_KP3,'value',str2double(KP3));

set(handles.slider_KI1,'value',str2double(KI1));
set(handles.slider_KI2,'value',str2double(KI2));
set(handles.slider_KI3,'value',str2double(KI3));

set(handles.slider_KD1,'value',str2double(KD1));
set(handles.slider_KD2,'value',str2double(KD2));
set(handles.slider_KD3,'value',str2double(KD3));

set(handles.edit_KP1,'string',num2str(KP1));
set(handles.edit_KP2,'string',num2str(KP2));
set(handles.edit_KP3,'string',num2str(KP3));

set(handles.edit_KI1,'string',num2str(KI1));
set(handles.edit_KI2,'string',num2str(KI2));
set(handles.edit_KI3,'string',num2str(KI3));

set(handles.edit_KD1,'string',num2str(KD1));
set(handles.edit_KD2,'string',num2str(KD2));
set(handles.edit_KD3,'string',num2str(KD3));

handles.modelname=modelname;

handles.block_scope_s_real = block_scope_s_real;
handles.block_scope_s_desejada = block_scope_s_desejada;
handles.block_scope_erro = block_scope_erro;
handles.block_scope_cartesiano = block_scope_cartesiano;
handles.block_scope_trajetoriaxy = block_scope_trajetoriaxy;
handles.block_scope_realdesejada = block_scope_realdesejada;
handles.block_scope_torque = block_scope_torque;

handles.block_KP1=block_KP1;
handles.block_KP2=block_KP2;
handles.block_KP3=block_KP3;

handles.block_KI1=block_KI1;
handles.block_KI2=block_KI2;
handles.block_KI3=block_KI3;

handles.block_KD1=block_KD1;
handles.block_KD2=block_KD2;
handles.block_KD3=block_KD3;
guidata(hObject,handles)

function edit_KD3_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
val=get(hObject,'string');
set(handles.slider_KD3,'value',str2double(val));
set_param(handles.block_KD3,'Gain',val);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_KD3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function slider_KD3_Callback(hObject, eventdata, handles)
val=get(hObject,'value');
set(handles.edit_KD3,'string',num2str(val));
set_param(handles.block_KD3,'Gain',num2str(val));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_KD3_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_KD2_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
val=get(hObject,'string');
set(handles.slider_KD2,'value',str2double(val));
set_param(handles.block_KD2,'Gain',val);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_KD2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function slider_KD2_Callback(hObject, eventdata, handles)
val=get(hObject,'value');
set(handles.edit_KD2,'string',num2str(val));
set_param(handles.block_KD2,'Gain',num2str(val));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_KD2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_KD1_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
val=get(hObject,'string');
set(handles.slider_KD1,'value',str2double(val));
set_param(handles.block_KD1,'Gain',val);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_KD1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_KD1_Callback(hObject, eventdata, handles)
val=get(hObject,'value');
set(handles.edit_KD1,'string',num2str(val));
set_param(handles.block_KD1,'Gain',num2str(val));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_KD1_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_KI3_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
val=get(hObject,'string');
set(handles.slider_KI3,'value',str2double(val));
set_param(handles.block_KI3,'Gain',val);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_KI3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_KI3_Callback(hObject, eventdata, handles)
val=get(hObject,'value');
set(handles.edit_KI3,'string',num2str(val));
set_param(handles.block_KI3,'Gain',num2str(val));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_KI3_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_KI2_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
val=get(hObject,'string');
set(handles.slider_KI2,'value',str2double(val));
set_param(handles.block_KI2,'Gain',val);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_KI2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_KI2_Callback(hObject, eventdata, handles)
val=get(hObject,'value');
set(handles.edit_KI2,'string',num2str(val));
set_param(handles.block_KI2,'Gain',num2str(val));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_KI2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_KI1_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
val=get(hObject,'string');
set(handles.slider_KI1,'value',str2double(val));
set_param(handles.block_KI1,'Gain',val);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_KI1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_KI1_Callback(hObject, eventdata, handles)
val=get(hObject,'value');
set(handles.edit_KI1,'string',num2str(val));
set_param(handles.block_KI1,'Gain',num2str(val));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_KI1_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider_KP1_Callback(hObject, eventdata, handles)
val=get(hObject,'value');
set(handles.edit_KP1,'string',num2str(val));
set_param(handles.block_KP1,'Gain',num2str(val));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_KP1_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_KP1_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
val=get(hObject,'string');
set(handles.slider_KP1,'value',str2double(val));
set_param(handles.block_KP1,'Gain',val);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_KP1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_KP2_Callback(hObject, eventdata, handles)
val=get(hObject,'value');
set(handles.edit_KP2,'string',num2str(val));
set_param(handles.block_KP2,'Gain',num2str(val));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_KP2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_KP2_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
val=get(hObject,'string');
set(handles.slider_KP2,'value',str2double(val));
set_param(handles.block_KP2,'Gain',val);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_KP2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function slider_KP3_Callback(hObject, eventdata, handles)
val=get(hObject,'value');
set(handles.edit_KP3,'string',num2str(val));
set_param(handles.block_KP3,'Gain',num2str(val));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_KP3_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_KP3_Callback(hObject, eventdata, handles)
handles=guidata(hObject);
val=get(hObject,'string');
set(handles.slider_KP3,'value',str2double(val));
set_param(handles.block_KP3,'Gain',val);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_KP3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in checkbox_S_REAL.
function checkbox_S_REAL_Callback(hObject, eventdata, handles)
a = get(hObject, 'Value');
if (a == 1)
    open_system(handles.block_scope_s_real);
else
    close_system(handles.block_scope_s_real);
end

% --- Executes on button press in checkbox_S_DESEJADA.
function checkbox_S_DESEJADA_Callback(hObject, eventdata, handles)
a = get(hObject, 'Value');
if (a == 1)
    open_system(handles.block_scope_s_desejada);
else
    close_system(handles.block_scope_s_desejada);
end

% --- Executes on button press in checkbox_ERRO.
function checkbox_ERRO_Callback(hObject, eventdata, handles)
a = get(hObject, 'Value');
if (a == 1)
    open_system(handles.block_scope_erro);
else
    close_system(handles.block_scope_erro);
end

% --- Executes on button press in pushbutton_mostra.
function pushbutton_mostra_Callback(hObject, eventdata, handles)
modelname=get(handles.edit_simfile,'String');
open_system(modelname);

% --- Executes on button press in checkbox_T_CART.
function checkbox_T_CART_Callback(hObject, eventdata, handles)
a = get(hObject, 'Value');
if (a == 1)
    open_system(handles.block_scope_cartesiano);
else
    close_system(handles.block_scope_cartesiano);
end

% --- Executes on button press in checkbox_TRAJXY.
function checkbox_TRAJXY_Callback(hObject, eventdata, handles)
a = get(hObject, 'Value');
if (a == 1)
    open_system(handles.block_scope_trajetoriaxy);
else
    close_system(handles.block_scope_trajetoriaxy);
end

% --- Executes on button press in checkbox_T_RD.
function checkbox_T_RD_Callback(hObject, eventdata, handles)
a = get(hObject, 'Value');
if (a == 1)
    open_system(handles.block_scope_realdesejada);
else
    close_system(handles.block_scope_realdesejada);
end
% Hint: get(hObject,'Value') returns toggle state of checkbox_T_RD

% --- Executes on button press in checkbox15.
function checkbox15_Callback(hObject, eventdata, handles)

% --- Executes on button press in checkbox16.
function checkbox16_Callback(hObject, eventdata, handles)

% --- Executes on button press in checkbox17.
function checkbox17_Callback(hObject, eventdata, handles)

% --- Executes on button press in pushbuttontraj.
function pushbuttontraj_Callback(hObject, eventdata, handles)

set(handles.edit29,'BackgroundColor', [1 1 1]);
global tempo_simulacao;
tempo_simulacao = get(handles.edit29,'String');
tempo_simulacao = str2double(tempo_simulacao);
assignin ('base','tempo_simulacao',tempo_simulacao);
%%
button = 1;
 %eixos
figure(2)
axis([0 0.23 -0.23 0.23])
grid on
hold on

global rx;
global ry;
global rz;

rx = []; %vetor para receber coordenadas de x
ry = []; %vetor para receber coordenadas de y
rz = []; %vetor para receber coordenadas de x

%vai fazer a trajetoria come?ar da posi?? inicial
rx(1) = 0.23;
ry(1) = 0;
rz(1) = 0.08;
i = 2;

while(button ==1) % enquantos clicamos com botao esquerdo do mause
rz(i) = 0.07;
[x,y,button] = ginput(1); %obtemos a coordenada de um ponto da tela
plot3(x,y,rz,'ro'); %mostramos o ponto
%terminamos de ler com um clique no botao direito
rx(i) = x;
ry(i) = y;
i = i+1;
end

x = rx;
y = ry;
dz = rz;

plot(x,y);

%correcao da altura em z
theta = [0 0 0];
d = [0.08 0 0];
l = [0 0.13 0.1];
alpha = [-pi/2 0 0];
t=[0:1:1]';

z = dz - d(1); 

%pontos em x espaco de trabalho
NP = length(x); %numero de pontos
DIST = get(handles.edit30,'String');
DIST = str2double(DIST); %distancia entre cada ponto

%TRAJETORIA
for i=1:1:(NP-1)
    distancia(i) = sqrt((x(i)-x(i+1))^2+(y(i)-y(i+1))^2+(z(i)-z(i+1))^2); % distancia entre cada ponto informado
    NPS(i) = distancia(i)/DIST;  %numero de pontos-1 por cada     
    NPS(i) = round(NPS(i));
    xd(i) = (x(i+1)-x(i))/(NPS(i)); % distancia entre pontos no eixo x
    yd(i) = (y(i+1)-y(i))/(NPS(i)); % distancia entre pontos no eixo y
    zd(i) = (z(i+1)-z(i))/(NPS(i)); % distancia entre pontos no eixo z
    
    
    linhasyez=length(yd); 
    pos{1,i}=[]; %cria vetor da trajetoria em x
    pos{2,i}=[]; %cria vetor da trajetoria em y
    pos{3,i}=[]; %cria vetor da trajetoria em z
	for j=1:1:NPS(i)
        pos{1,i}=horzcat(pos{1,i},((x(i)-xd(i))+xd(1,linhasyez)*j));  
        pos{2,i}=horzcat(pos{2,i},((y(i)-yd(i))+yd(1,linhasyez)*j));
        pos{3,i}=horzcat(pos{3,i},((z(i)-zd(i))+zd(1,linhasyez)*j));
    end
    
%     pos{1,i}(1,NPS(i))=x(i+1); % ultimo valor de cada trajeto recebe o valor valor final desejado
%     pos{2,i}(1,length(pos{1,i}))=y(i+1); % ultimo valor de cada trajeto recebe o valor valor final desejado
%     pos{3,i}(1,length(pos{1,i}))=z(i+1); % ultimo valor de cada trajeto recebe o valor valor final desejado
end

CP = [];
trajx = [];
trajy = [];
trajz = [];

%separa variaveis de trajetoria
for i=1:1:(NP-1)
trajx = horzcat(trajx,pos{1,i});
trajy = horzcat(trajy,pos{2,i});
trajz = horzcat(trajz,pos{3,i});
end
NPN = length(trajx);
 
rad = pi/180;
graus = 180/pi;

%cinematica
    for i=1:1:NPN
    th0(i) = atan2(trajy(i),trajx(i));
    ix(i) = (trajx(i)^2 + trajy(i)^2)^0.5;
%     #stuff for calculating th2
    r_2(i) = ix(i)^2 + (-trajz(i))^2;
    l_sq = l(2)^2 + l(3)^2;
    term2 = (r_2(i) - l_sq)/(2*l(2)*l(3));
    term1 = ((1 - term2^2)^0.5)*-1;
%     #calculate th2
    th2(i) = atan2(term1, term2);
%     #optional line. Comment this one out if you 
%     #notice any problems
%     th2(i) = -1*th2(i);
%   x =x y=-z z=y

%     #Stuff for calculating th2
    k1 = l(2) + l(3)*cos(th2(i));
    k2 = l(3)*sin(th2(i));
    r  = (k1^2 + k2^2)^0.5;
    gamma = atan2(k2,k1);
%     #calculate th1
    th1(i) = atan2(-trajz(i),ix(i)) - gamma;
    end

for i=1:1:NPN
    P{i} = [th0(i) th1(i) th2(i)]
end

CP = [];
for i=1:1:(NPN)
CP=vertcat(CP,P{i});
end

for i=1:1:(NPN-1)
    [q{i},qd{i},qdd{i}]=jtraj(P{i},P{i+1}, t); 
end

CQ = [];
for i=1:1:(NPN-1)
CQ = vertcat(CQ,q{i});
end

CQQ = [];
for i=1:1:(NPN-1)
CQQ = vertcat(CQQ,qd{i});
end

CQQQ = [];
for i=1:1:(NPN-1)
CQQQ = vertcat(CQQQ,qdd{i});
end

% figure(2)
% %unimos cada par de pontos
% robot.plot([0,0,0]);
% hold on
% plot3(rx,ry,rz);

%vetor tempo
npt = length(CQ); % numero de pontos da trajetoria
perio = tempo_simulacao/npt; % periodo de amostragem
i = 0:perio:(tempo_simulacao-perio);
assignin ('base','perio',perio);
t = i;

%adequa matriz para simulink com tempo
t = t';
CQ = horzcat(t,CQ);
CQQ = horzcat(t,CQQ);
CQQQ = horzcat(t,CQQQ);

%mandar pra works
assignin('base','CQ',CQ);
assignin('base','CQQ',CQQ);
assignin('base','CQQQ',CQQQ);
%%

function edit27_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function edit27_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_tempo_Callback(hObject, eventdata, handles)

% --- Executes on button press in pushbuttonrobo.
function pushbuttonrobo_Callback(hObject, eventdata, handles)
%%
clear all;
% close all;
clc

%constantes do compensador
conskp = 5;
conski = 2;
conskd = 1;
aggkp = 50;
aggki = 63;
aggkd = 2;

%setar parametros nos blocos PID
set_param('dinamica/integrador/KI1','Gain','2');
set_param('dinamica/integrador/KI2','Gain','2');
set_param('dinamica/integrador/KI3','Gain','2');
set_param('dinamica/proporcional/KP1','Gain','6');
set_param('dinamica/proporcional/KP2','Gain','6');
set_param('dinamica/proporcional/KP3','Gain','6');
set_param('dinamica/derivador/KD1','Gain','1');
set_param('dinamica/derivador/KD2','Gain','1');
set_param('dinamica/derivador/KD3','Gain','1');

%parametros motor
%parametros motor
motor_R = 1.5;
motor_L= 0.006;
motor_K = 22/10000;
motor_J = 2/10000000;
motor_B = 9/100000000;

%parametros DH
theta = [0 0 0];
d = [0.08 0 0];
l = [0 0.13 0.1];
alpha = [pi/2 0 0];

%criar elos
% theta | D | l | alpha | sigma | m | rx ry rz | Ixx Iyy Izz Ixy Iyz Ixz | Jm | G | B | Tc[0 0]   
 L(1)=Link([theta(1) d(1) l(1) alpha(1) 0 0.45 0.00 0.03 0.02 0 0 0 0 0 0 0 motor_J 1/50 motor_B 0 0]); 
 L(2)=Link([theta(2) d(2) l(2) alpha(2) 0 0.5 0.01 -0.03 0.00 0 0 0 0 0 0 0 motor_J 1/50 motor_B 0 0]); 
 L(3)=Link([theta(3) d(3) l(3) alpha(3) 0 0.4 0.01 0.03 0.00 0 0 0 0 0 0 0 motor_J 1/50 motor_B 0 0]); 

 %criar robo
global robot;
robot = SerialLink(L);
assignin('base','robot',robot);
t=[0:1:1]';
%%

function edit29_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function edit29_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox18.
function checkbox18_Callback(hObject, eventdata, handles)
a = get(hObject, 'Value');
% robot = evalin('base','robot');
global robot;
global rx;
global ry;
global rz;

xr = evalin('base','xr');
yr = evalin('base','yr');
zr = evalin('base','zr');
qreal = evalin('base','qreal');

if (a == 1)
    figure(1);
    realcart = horzcat(xr,yr,zr);
    for i=1:1:length(qreal)-1
    robot.plot(qreal(i,:));
    hold on;
    plot2(realcart(i:i+1,:),'r');
    end
    robot.plot(qreal(length(qreal),:));
    hold on
    figure(1);
    plot3(rx,ry,rz,'b');
else
   close(figure(2));
end


function checkbox19_Callback(hObject, eventdata, handles)

a = get(hObject, 'Value');
if (a == 1)
    open_system(handles.block_scope_torque);
else
    close_system(handles.block_scope_torque);
end


function edit30_Callback(hObject, eventdata, handles)

function edit30_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
