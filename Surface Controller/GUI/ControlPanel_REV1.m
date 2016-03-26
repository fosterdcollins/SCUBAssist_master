function varargout = ControlPanel_REV1(varargin)
global joystick1 HG VG RecordData Telem Mode 
% CONTROLPANEL_REV1 MATLAB code for ControlPanel_REV1.fig
%      CONTROLPANEL_REV1, by itself, creates a new CONTROLPANEL_REV1 or raises the existing
%      singleton*.
%
%      H = CONTROLPANEL_REV1 returns the handle to a new CONTROLPANEL_REV1 or the handle to
%      the existing singleton*.
%
%      CONTROLPANEL_REV1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONTROLPANEL_REV1.M with the given input arguments.
%
%      CONTROLPANEL_REV1('Property','Value',...) creates a new CONTROLPANEL_REV1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ControlPanel_REV1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ControlPanel_REV1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ControlPanel_REV1

% Last Modified by GUIDE v2.5 29-Nov-2015 12:45:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ControlPanel_REV1_OpeningFcn, ...
                   'gui_OutputFcn',  @ControlPanel_REV1_OutputFcn, ...
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


% --- Executes just before ControlPanel_REV1 is made visible.
function ControlPanel_REV1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ControlPanel_REV1 (see VARARGIN)

% Choose default command line output for ControlPanel_REV1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);



% UIWAIT makes ControlPanel_REV1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function varargout = update_Callback(hObject, eventdata, handles) 
disp('here')

% --- Outputs from this function are returned to the command line.
function varargout = ControlPanel_REV1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function HG_CreateFcn(hObject, eventdata, handles)
% hObject    handle to HG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function VG_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in Enable.
function Enable_Callback(hObject, eventdata, handles)
% hObject    handle to Enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Mode
Mode = 1;
disp('Enable')

% --- Executes on button press in Disable.
function Disable_Callback(hObject, eventdata, handles)
% hObject    handle to Disable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Mode
Mode = 0;
disp('disable')


% --- Executes on slider movement.
function HG_Callback(hObject, eventdata, handles)
% hObject    handle to HG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
get(handles.HG,'Value')
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function JoystickTheta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to JoystickTheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate JoystickTheta


% --- Executes during object creation, after setting all properties.
function JoystickZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to JoystickZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate JoystickZ


% --- Executes during object creation, after setting all properties.
function JoystickXY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to JoystickXY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
%


% Hint: place code in OpeningFcn to populate JoystickXY


% --- Executes on button press in DepthHold.
function DepthHold_Callback(hObject, eventdata, handles)
% hObject    handle to DepthHold (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.JoystickXY, 'XData', 0, 'YData', 0)
% Hint: get(hObject,'Value') returns toggle state of DepthHold


% --- Executes on button press in HeadingHold.
function HeadingHold_Callback(hObject, eventdata, handles)
% hObject    handle to HeadingHold (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of HeadingHold


% --- Executes on button press in RecordData.
function RecordData_Callback(hObject, eventdata, handles)
axes(handles.JoystickXY)
grid on
axis equal
axis([-1 1 -1 1])
plot(0,1,'b.')

% hObject    handle to RecordData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RecordData

function RedrawJoystick(hObject, eventdata, handles)
axes(handles.JoystickZ)
grid on
axis equal
axis([-1 1 -1 1])