function varargout = MRfin(varargin)
% MRFIN M-file for MRfin.fig
%      MRFIN, by itself, creates a new MRFIN or raises the existing
%      singleton*.
%
%      H = MRFIN returns the handle to a new MRFIN or the handle to
%      the existing singleton*.
%
%      MRFIN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MRFIN.M with the given input arguments.
%
%      MRFIN('Property','Value',...) creates a new MRFIN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MRfin_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MRfin_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MRfin

% Last Modified by GUIDE v2.5 11-May-2015 13:27:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @MRfin_OpeningFcn, ...
    'gui_OutputFcn',  @MRfin_OutputFcn, ...
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
%==============My fynctions==============================
function rad = deg2rad( deg )
rad = pi*deg/180;
%==============My fynctions==============================

% --- Executes just before MRfin is made visible.
function MRfin_OpeningFcn(hObject, eventdata, handles, varargin)

handles.C=1;   % Kontola tego czy ma wykonywac sie kod C, C=1 TAK, C=0 NIE

handles.output = hObject;
try
    handles.Ipp = evalin('base','Ipp');
    handles.Iss = evalin('base','Iss');
    handles.theta = evalin('base','theta');
    set( handles.edFrame_End,'string', num2str( size( handles.Ipp,1 ) ) );
    set( handles.edFrame_Step,'string', '100' );
catch
    s = sprintf('Ipp Iss not found \nin Base Workspace');
    he = warndlg( s );
    uiwait( he );
end

% if ~handles.C
%     if (matlabpool('size')~=4 & matlabpool('size')>0)
%         matlabpool close;
%         matlabpool 4;
%     end
%     if(matlabpool('size')==0)
%         matlabpool 4;
%     end
% end

try
    handles.setup = evalin('base','setup');
catch
    s = sprintf('Setup not found \nin Base Workspace')
    he = warndlg( s );
    uiwait( he );
end
handles.Wr.wavelength = 654.25;
handles.Wr.theta = 0;
handles.Wr.polarization = 0;

handles.Wg.wavelength = 532.07;
handles.Wg.polarization = 1;
handles.Wg.theta = pi;





    handles.Tp = atan( tan( handles.theta.mTp - pi / 2 ) *...
        str2num( get( handles.edScale,'string' ) ) ) + pi/2 +...
        deg2rad( str2num( get( handles.edShift_R,'string' ) ) );
    handles.Ts = atan( tan( handles.theta.mTs - pi / 2 ) *...
        str2num( get( handles.edScale,'string' ) ) ) + pi/2 +...
        deg2rad( str2num( get( handles.edShift_G,'string' ) ) );
    handles.rrp = ( running_radius( abs( handles.Tp - pi/2 ),...
        handles.setup.hccd_max_R, handles.setup.Diafragma, handles.Wr.wavelength ) ) .^ 2;
    handles.rrs = ( running_radius(abs(handles.Ts-pi/2),...
        handles.setup.hccd_max_G, handles.setup.Diafragma, handles.Wg.wavelength ) ).^2;

handles.mr = Calculate_m(25,handles.Wr.wavelength,'EG');
handles.mg = Calculate_m(25,handles.Wg.wavelength,'EG');
handles.r = 1e3:20:15e3;
handles.ind = 1:100:size( handles.Ipp,1 );
set(handles.te_m_red,'string',['m_r = ' num2str( handles.mr )]);
set(handles.te_m_green,'string',['m_g = ' num2str( handles.mg )]);
set(handles.uipanel1,'title',handles.setup.FileName);
if handles.C
    save_workspace;         % KOD C
end

% Update handles structure
guidata(hObject, handles);


function varargout = MRfin_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on selection change in pmRefInd.
function pmRefInd_Callback(hObject, eventdata, handles)
% Calculate refractive index
S = get( handles.pmRefInd,'String' );
Vel = get( handles.pmRefInd,'Value' );
handles.mr = Calculate_m( 25, handles.Wr.wavelength, S{Vel} ) +...
    str2num( get( handles.edShift_m, 'string' ) ) + str2num( get( handles.edShift_mred, 'string' ) );
handles.mg = Calculate_m( 25, handles.Wg.wavelength, S{Vel} ) +...
    str2num( get( handles.edShift_m, 'string' ) );
set( handles.te_m_red,'string',['m_r = ' num2str(handles.mr)]);
set( handles.te_m_green,'string',['m_g = ' num2str(handles.mg)]);
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function pmRefInd_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edRmin_Callback(hObject, eventdata, handles)
handles.r = str2num( get( handles.edRmin,'string' ) ):...
    str2num( get( handles.edRstep,'string' ) ):...
    str2num( get( handles.edRmax,'string' ) );
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edRmin_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edRmax_Callback(hObject, eventdata, handles)
handles.r = str2num( get( handles.edRmin,'string' ) ):...
    str2num( get( handles.edRstep,'string' ) ):...
    str2num( get( handles.edRmax,'string' ) );
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.

function edRmax_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edRstep_Callback(hObject, eventdata, handles)
handles.r = str2num( get( handles.edRmin,'string' ) ):...
    str2num( get( handles.edRstep,'string' ) ):...
    str2num( get( handles.edRmax,'string' ) );
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edRstep_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edScale_Callback(hObject, eventdata, handles)

    handles.Tp = atan( tan( handles.theta.mTp - pi / 2 ) *...
        str2num( get( handles.edScale,'string' ) ) ) + pi/2 +...
        deg2rad( str2num( get( handles.edShift_R,'string' ) ) );
    handles.Ts = atan( tan( handles.theta.mTs - pi / 2 ) *...
        str2num( get( handles.edScale,'string' ) ) ) + pi/2 +...
        deg2rad( str2num( get( handles.edShift_G,'string' ) ) );

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edScale_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edShift_R_Callback(hObject, eventdata, handles)

    handles.Tp = atan( tan( handles.theta.mTp - pi / 2 ) *...
        str2num( get( handles.edScale,'string' ) ) ) + pi/2 +...
        deg2rad( str2num( get( handles.edShift_R,'string' ) ) );

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edShift_R_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edShift_G_Callback(hObject, eventdata, handles)

    handles.Ts = atan( tan( handles.theta.mTs - pi / 2 ) *...
        str2num( get( handles.edScale,'string' ) ) ) + pi/2 +...
        deg2rad( str2num( get( handles.edShift_G,'string' ) ) );

guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of edShift_G as text
%        str2double(get(hObject,'String')) returns contents of edShift_G as a double


% --- Executes during object creation, after setting all properties.
function edShift_G_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edShift_m_Callback(hObject, eventdata, handles)
S = get( handles.pmRefInd,'String' );
Vel = get( handles.pmRefInd,'Value' );
handles.mr = Calculate_m( 25, handles.Wr.wavelength, S{Vel} ) +...
    str2num( get( handles.edShift_m, 'string' ) ) + str2num( get( handles.edShift_mred, 'string' ) );
handles.mg = Calculate_m( 25, handles.Wg.wavelength, S{Vel} ) +...
    str2num( get( handles.edShift_m, 'string' ) );

set( handles.te_m_red,'string',sprintf('m_r = %2.4f ',  handles.mr ));

set( handles.te_m_green,'string',sprintf('m_g = %2.4f ',  handles.mg  ) );
guidata(hObject, handles);


function edShift_m_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edShift_mred_Callback(hObject, eventdata, handles)
S = get( handles.pmRefInd,'String' );
Vel = get( handles.pmRefInd,'Value' );
handles.mr = Calculate_m( 25, handles.Wr.wavelength, S{Vel} ) +...
    str2num( get( handles.edShift_m, 'string' ) ) + str2num( get( handles.edShift_mred, 'string' ) );
set( handles.te_m_red,'string',sprintf('m_r = %2.4f ',  handles.mr ));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edShift_mred_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edFrame_Step_Callback(hObject, eventdata, handles)
handles.ind =  str2num( get( handles.edFrame_begin,'string' ) ):...
    str2num( get( handles.edFrame_Step,'string' ) ):...
    str2num( get( handles.edFrame_End,'string' ) );
if handles.C
    save_workspace;         % KOD C
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edFrame_Step_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edFrame_begin_Callback(hObject, eventdata, handles)
handles.ind =  str2num( get( handles.edFrame_begin,'string' ) ):...
    str2num( get( handles.edFrame_Step,'string' ) ):...
    str2num( get( handles.edFrame_End,'string' ) );
if handles.C
    save_workspace;         % KOD C
end
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edFrame_begin_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edFrame_End_Callback(hObject, eventdata, handles)
handles.ind =  str2num( get( handles.edFrame_begin,'string' ) ):...
    str2num( get( handles.edFrame_Step,'string' ) ):...
    str2num( get( handles.edFrame_End,'string' ) );
if handles.C
    save_workspace;         % KOD C
end
guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function edFrame_End_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbCalc.
function pbCalc_Callback(hObject, eventdata, handles)
% ======= Calc ==========================

if handles.C
    save_setup; %KOD DO C
    TT=tic;
    [status,result] = system('./client .socket','-echo'); %KOD DO C
    czas=toc(TT)
end

if ~handles.C
    TTTT=tic;
    ppp=tic;
    
    Ittp = GeneratePattern( handles.r,handles.mr,handles.Tp,handles.Wr );
    Itts = GeneratePattern( handles.r, handles.mg, handles.Ts, handles.Wg );
    
    CzasGenerate=toc(ppp)
    
    %-------------------------------------------------------------------------
    Ittp = Ittp ./ meshgrid( handles.rrp, 1 : size( Ittp, 1 ) ); % unormowane przez running radius
    Itts = Itts ./ meshgrid( handles.rrs, 1 : size( Itts, 1 ) );
    %[uv] = memory;
    %step = floor( uv.MaxPossibleArrayBytes / ( size( Ittp, 1 ) * 64 ) );
    step=500;
    
    if size( handles.Ipp( handles.ind,: ),1 ) > step
        N = floor( size( handles.Ipp( handles.ind,: ),1 ) / step );
    else
        N = 1;
        step = size( handles.Ipp( handles.ind,: ),1 );
    end
    
    res.r = zeros( 1, size( handles.Ipp( handles.ind,: ),1 ) );
    res.rr = res.r;
    res.rg = res.r;
    
    
    CCC=tic;
    for p = 0 : N - 1
        vek = ( ( p * step ) + 1 ) : ( ( p + 1 )* step );
        
        
        
        [ errp, ~] = ReferenceDistance( handles.Ipp( handles.ind( vek ), : ), Ittp );
        [~, irmp] = min( errp, [], 2 );
       
        [ errs, ~] = ReferenceDistance( handles.Iss(handles.ind( vek ),:), Itts );
        [~, irms] = min(errs, [], 2);
        
        %-------------------------------------------------------------------------
        err = ( errp.' ./ meshgrid( mean( errp, 2 ), 1 : size( errp, 2 ) ) ).* ...
            ( errs.' ./ meshgrid( mean( errs, 2 ), 1 : size( errs, 2 ) ) );
        
        [v irm] = min( err.', [], 2 ); % v to vektor warto�ci, irm to vektor indeks�w
        res.rr( vek ) = handles.r( irmp );
        res.rg( vek ) = handles.r( irms );
        res.r( vek ) = handles.r( irm  );
    end
    CzasDystansu=toc(CCC)
    
    
    if N*step < size( handles.Ipp( handles.ind,: ),1 )
        vek = N*step:size( handles.Ipp( handles.ind,: ),1 );
        [ errp, scalep ] = ReferenceDistance( handles.Ipp( handles.ind( vek ), : ), Ittp );
        [vp irmp] = min( errp, [], 2 ); % to jest podgl�d min dla pierwszej polaryzacji
        clear('scalep','vp');
        
        [ errs, scales ] = ReferenceDistance( handles.Iss(handles.ind( vek ),:), Itts );
        [vp irms] = min(errs, [], 2);
        clear('scales','vp');
        
        %-------------------------------------------------------------------------
        err = ( errp.' ./ meshgrid( median( errp, 2 ), 1 : size( errp, 2 ) ) ).* ...
            ( errs.' ./ meshgrid( median( errs, 2 ), 1 : size( errs, 2 ) ) );
        
        [v irm] = min( err.', [], 2 ); % v to vektor warto�ci, irm to vektor indeks�w
        res.rr( vek ) = handles.r( irmp );
        res.rg( vek ) = handles.r( irms );
        res.r( vek ) = handles.r( irm  );
        res.mr = handles.mr;
        res.mg = handles.mg;
        
        
    end
    
end
if handles.C
   import_results;
end


% ======= Draw ==========================
hf =  figure;
axes;
if get(handles.cbRed,'value')
    plot( res.rr ,'r.', 'MarkerSize', 5 );
    grid on;
end
%=================== fitt preview ====================================
if 1==0
    for ii = 1:size(handles.Ipp(handles.ind,:),1);
        plot( handles.Tp,handles.Ipp( ii,: ) );%,theta.mTs(nom),Iss(ii,nom));grid on;
        hold on; grid on;
        plot(handles.Tp, Ittp( irmp( ii ), : )* scalep( ii,irmp( ii ) ) ,'r');
        hold off;
        pause( 0.1 );
    end
end
%=================== fitt preview ====================================
if get(handles.cbGreen,'value')
    figure( hf );
    hold on;
    plot( res.rg, 'g.', 'MarkerSize', 5);
    grid on;
end
s = sprintf(['Scale = %s \n'...
    'Shift R = %s \n'...
    'Shift G = %s \n'...
    'Shift m = %s\n'...
    'mr = %2.4f \n'...
    'mg = %2.4f \n'...
    'r(1) = %2.1f \n'...
    'r(end) = %2.1f'],...
    get( handles.edScale,'string' ),...
    get( handles.edShift_R,'string' ),...
    get( handles.edShift_G,'string' ),...
    get( handles.edShift_m, 'string' ),...
    handles.mr, handles.mg,...
    res.rg(1) , res.rg(end) );
       % handles.r( irms(1) ),handles.r( irms(end) ) );
legend(s);
%=================== fitt preview ====================================
if 1==0
    for ii = 1:size(handles.Iss(handles.ind,:),1);
        plot( handles.Ts,handles.Iss( ii,: ) );%,theta.mTs(nom),Iss(ii,nom));grid on;
        hold on; grid on;
        plot(handles.Ts, Itts( irms( ii ), : )* scales( ii,irms( ii ) ) ,'r');
        hold off;
        pause( 0.5 );
    end
end
%=================== fitt preview ====================================
if get(handles.cbBlue,'value')
    figure( hf );
    figure(hf);
    plot( res.r,'.b' );
    grid on;
    hold off;
end
assignin('base','results',res);
if ~handles.C
    czas=toc(TTTT)
end
% --- Executes on button press in pbLoadParam.
function pbLoadParam_Callback(hObject, eventdata, handles)

P = evalin('base', 'parameters');
set( handles.edShift_mred,'string',P.Angle_Lim );
set( handles.edShift_m,'string',P.Shift_m );
set( handles.edShift_G,'string',P.Shift_G );
set( handles.edShift_R,'string',P.Shift_R );
set( handles.edScale,'string',P.Scale );
set( handles.edRstep,'string',P.Rstep );
set( handles.edRmax,'string',P.Rmax );
set( handles.edRmin,'string',P.Rmin );
set( handles.edFrame_Step,'string',P.Frame_Step );
set( handles.edFrame_End,'string',P.Frame_End );
set( handles.edFrame_begin,'string',P.Frame_begin );
set( handles.pmRefInd,'Value', P.Vel );
set(handles.uipanel1,'title',handles.setup.FileName);


    handles.Tp = atan( tan( handles.theta.mTp - pi / 2 ) *...
        str2num( get( handles.edScale,'string' ) ) ) + pi/2 +...
        deg2rad( str2num( get( handles.edShift_R,'string' ) ) );
    handles.Ts = atan( tan( handles.theta.mTs - pi / 2 ) *...
        str2num( get( handles.edScale,'string' ) ) ) + pi/2 +...
        deg2rad( str2num( get( handles.edShift_G,'string' ) ) );
S = get( handles.pmRefInd,'String' );
Vel = get( handles.pmRefInd,'Value' );

handles.mr = Calculate_m( 25, handles.Wr.wavelength, S{Vel} ) +...
    str2num( get( handles.edShift_m, 'string' ) ) + str2num( get( handles.edShift_mred, 'string' ) );
handles.mg = Calculate_m( 25, handles.Wg.wavelength, S{Vel} ) +...
    str2num( get( handles.edShift_m, 'string' ) );


set( handles.te_m_red,'string',sprintf('m_r = %2.4f ',  handles.mr ));

set( handles.te_m_green,'string',sprintf('m_g = %2.4f ',  handles.mg  ) );

handles.r = str2num( get( handles.edRmin,'string' ) ):...
    str2num( get( handles.edRstep,'string' ) ):...
    str2num( get( handles.edRmax,'string' ) );
handles.ind =  str2num( get( handles.edFrame_begin,'string' ) ):...
    str2num( get( handles.edFrame_Step,'string' ) ):...
    str2num( get( handles.edFrame_End,'string' ) );
handles.rrp = ( running_radius( abs( handles.Tp - pi/2 ),...
    handles.setup.hccd_max_R, handles.setup.Diafragma, handles.Wr.wavelength ) ) .^ 2;
handles.rrs = ( running_radius(abs(handles.Ts-pi/2),...
    handles.setup.hccd_max_G, handles.setup.Diafragma, handles.Wg.wavelength ) ) .^ 2;
if handles.C
    save_workspace;         % KOD C
end
guidata(hObject, handles);

% --- Executes on button press in pbSaveParam.
function pbSaveParam_Callback(hObject, eventdata, handles)

P.Angle_Lim =  get( handles.edShift_mred,'string') ;
P.Shift_m =  get(handles.edShift_m,'string') ;
P.Shift_G =  get(handles.edShift_G,'string') ;
P.Shift_R =  get(handles.edShift_R,'string') ;
P.Scale =  get(handles.edScale,'string') ;
P.Rstep =  get(handles.edRstep,'string');
P.Rmax = get(handles.edRmax,'string') ;
P.Rmin =  get(handles.edRmin,'string') ;
P.Frame_Step =  get(handles.edFrame_Step,'string') ;
P.Frame_End =  get(handles.edFrame_End,'string') ;
P.Frame_begin =  get(handles.edFrame_begin,'string') ;
P.Vel = get( handles.pmRefInd,'Value' );
assignin('base', 'parameters',P);


% --- Executes on button press in pbView.
function pbView_Callback(hObject, eventdata, handles)
figure;
[AX,H1,H2] = plotyy(handles.Tp,handles.Ipp(1,:),handles.Ts,handles.Iss(1,:));
grid on;
title('First frame.');
set(H1,'color','r');
set(H2,'color','g');
% set(AX(1),'color','r');
% set(AX(2),'color','g');
figure;
[AX,H1,H2] = plotyy(handles.Tp,handles.Ipp(end,:),handles.Ts,handles.Iss(end,:));
grid on;
title('Last frame.');
set(H1,'color','r');
set(H2,'color','g');

% --- Executes on button press in cbRed.
function cbRed_Callback(hObject, eventdata, handles)
% hObject    handle to cbRed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbRed


% --- Executes on button press in cbBlue.
function cbBlue_Callback(hObject, eventdata, handles)
% hObject    handle to cbBlue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbBlue


% --- Executes on button press in cbGreen.
function cbGreen_Callback(hObject, eventdata, handles)
% hObject    handle to cbGreen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbGreen


% --- Executes during object creation, after setting all properties.
function uipanel1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
