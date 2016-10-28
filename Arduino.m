function varargout = Arduino(varargin)
% ARDUINO MATLAB code for Arduino.fig
%      ARDUINO, by itself, creates a new ARDUINO or raises the existing
%      singleton*.
%
%      H = ARDUINO returns the handle to a new ARDUINO or the handle to
%      the existing singleton*.
%
%      ARDUINO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ARDUINO.M with the given input arguments.
%
%      ARDUINO('Property','Value',...) creates a new ARDUINO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Arduino_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Arduino_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Arduino

% Last Modified by GUIDE v2.5 17-Oct-2016 20:44:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Arduino_OpeningFcn, ...
                   'gui_OutputFcn',  @Arduino_OutputFcn, ...
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


% --- Executes just before Arduino is made visible.
function Arduino_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Arduino (see VARARGIN)

% Choose default command line output for Arduino
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Arduino wait for user response (see UIRESUME)
% uiwait(handles.figure1);

delete(instrfind({'Port'},{'COM3'}))
clear a;
global a;
a = arduino('COM3');
configurePin(a,'D8','DigitalOutput');

% --- Outputs from this function are returned to the command line.
function varargout = Arduino_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global k a
x=0; y=0; z=0;
v=[str2num(get(handles.text,'String'))];

for k=1:1:v
    b=readVoltage(a, 'A0');
    if b<1 % Se tensão em cima do fotoresistor for menor que 1V ligar luz de emergência.
        writeDigitalPin(a,'D8',1); %Liga luz de emergência atravez do módulo relé.
    end
    if b>=1 % Se tensão em cima do fotoresistor for maior ou igual a 1V desligar luz de emergência.
        writeDigitalPin(a,'D8',0); %Desliga luz de emergência atravez do módulo relé.
    end
    x=[x,b];
    subplot(311);
    plot(x,'LineWidth',2); grid on;
    axis([0 v 0 5.5]);
    subplot(312);
    y=[y,b];
    stem(y,'LineWidth',2); grid on;
    axis([0 v 0 6]);
    z=[z,b];
    subplot(313);
    stairs(z,'LineWidth',2); grid on;
    axis([0 v 0 6]);
    pause(0.04);    
end



% --- Executes on button press in turn_on.
function turn_on_Callback(hObject, eventdata, handles)
% hObject    handle to turn_on (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a;
writeDigitalPin(a, 'D8', 1);


% --- Executes on button press in turn_off.
function turn_off_Callback(hObject, eventdata, handles)
% hObject    handle to turn_off (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a;
writeDigitalPin(a, 'D8', 0);


function text_Callback(hObject, eventdata, handles)
% hObject    handle to text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of text as text
%        str2double(get(hObject,'String')) returns contents of text as a double


% --- Executes during object creation, after setting all properties.
function text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
