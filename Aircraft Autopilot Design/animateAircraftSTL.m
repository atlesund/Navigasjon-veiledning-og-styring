function animateAircraftSTL(simdata, stlFile, Vg, record, videoFile, third_person_view, view_scale, animationStep, playbackSpeed, show_body_axes, show_yaw_vector)
% animateAircraftSTL
% Animates an STL aircraft using lateral aircraft simulation data.
%
% The aircraft moves horizontally according to course chi and a constant
% ground speed. The X and Y axis limits follow the aircraft, creating a
% zoomed-in moving-camera effect.
%
% Only the animation axes and a title are shown. The title is fixed to the
% top of the figure window, independent of the 3D camera view.
%
% Inputs:
%   simdata           - Simulation data matrix from closedLoop.m
%   stlFile           - Name or path of the aircraft STL file
%   Vg                - Ground speed [m/s]
%   record            - true to save an MP4 video, otherwise false
%   videoFile         - Optional video filename
%   third_person_view - true for a chase camera that follows the course
%   view_scale        - Scale factor for the visible area around the aircraft
%   animationStep     - Number of samples to advance per frame
%   playbackSpeed     - Playback speed multiplier (1 = real time)
%   show_body_axes    - Optional, true (default) to show the red/green/blue
%                       body-axis arrows on the aircraft, false to hide them
%   show_yaw_vector   - Optional, true (default) to show the dark red yaw
%                       (heading) arrow, false to hide it
%
% Expected simdata columns:
%    1: time, t
%    2: yaw angle, psi
%    3: course angle, chi
%    4: sideslip angle, beta
%    5: roll angle, phi
%   (columns 6-13 are accepted but not displayed)
%
% Coordinate convention used for the plot:
%   X-axis: north
%   Y-axis: east
%   Z-axis: down
%
% The lateral aircraft model has no pitch or altitude dynamics.
% Therefore, pitch and vertical position are set to zero.

    %% Optional inputs

    if nargin < 11 || isempty(show_yaw_vector)
        show_yaw_vector = true;
    end

    if nargin < 10 || isempty(show_body_axes)
        show_body_axes = true;
    end

    if nargin < 5 || isempty(videoFile)
        videoFile = "F16_animation.mp4";
    end

    if nargin < 4 || isempty(record)
        record = false;
    end

    %% Visualization settings

    % Compress the physical flight path for visualization.
    % With positionScale = 0.002: 1000 physical metres become 2 plot units.
    positionScale = 0.002;

    % Displayed size of the aircraft.
    aircraftDisplayLength = 2.0;

    % Half-width of the moving X and Y axis limits.
    viewHalfWidth = 4.0*view_scale;

    % Half-height of the Z-axis limits.
    viewHalfHeight = 3.0*view_scale;

    % Move the viewing-window center slightly ahead of the aircraft.
    lookAheadDistance = 0.8;

    % Yaw and course arrow length.
    vectorLength = 1.5;

    % Maximum number of displayed trail points.
    maximumTrailPoints = 300;

    % Rotate the camera with the aircraft course.
    useChaseCamera = third_person_view;

    % Fixed camera elevation.
    cameraElevation = 20;

    % Set true to show course vector
    show_course_vector = false;

    % Height of the title strip at the top of the window (normalized).
    titleHeight = 0.10;

    %% Validate inputs

    if ~isfile(stlFile)
        error('The STL file "%s" could not be found.', stlFile);
    end

    if size(simdata, 2) < 13
        error([ ...
            'simdata must contain at least 13 columns. ' ...
            'The supplied matrix contains %d columns.'], ...
            size(simdata, 2));
    end

    if size(simdata, 1) < 2
        error('simdata must contain at least two time samples.');
    end

    if any(~isfinite(simdata(:)))
        error('simdata contains NaN or Inf values.');
    end

    %% Extract simulation data

    t    = simdata(:,1);
    psi  = simdata(:,2);
    chi  = simdata(:,3);
    beta = simdata(:,4);
    phi  = simdata(:,5);

    % The lateral model does not contain pitch angle.
    theta = zeros(size(t));

    if any(diff(t) <= 0)
        error('The simulation time vector must be strictly increasing.');
    end

    %% Calculate horizontal position

    % chi = 0 deg: north, chi = 90 deg: east.
    velocityEast  = Vg .* sin(chi);
    velocityNorth = Vg .* cos(chi);

    eastPosition  = cumtrapz(t, velocityEast);
    northPosition = cumtrapz(t, velocityNorth);
    downPosition  = zeros(size(t));

    eastDisplay  = positionScale .* eastPosition;
    northDisplay = positionScale .* northPosition;
    downDisplay  = positionScale .* downPosition;

    %% Read STL model

    try
        stlModel = stlread(stlFile);
    catch stlError
        error('Unable to read the STL file:\n%s', stlError.message);
    end

    if isa(stlModel, 'triangulation')
        faces = stlModel.ConnectivityList;
        vertices = stlModel.Points;
    elseif isstruct(stlModel) && ...
            isfield(stlModel, 'ConnectivityList') && ...
            isfield(stlModel, 'Points')
        faces = stlModel.ConnectivityList;
        vertices = stlModel.Points;
    elseif isstruct(stlModel) && ...
            isfield(stlModel, 'faces') && ...
            isfield(stlModel, 'vertices')
        faces = stlModel.faces;
        vertices = stlModel.vertices;
    else
        error([ ...
            'stlread returned an unsupported data type. ' ...
            'Check which stlread function MATLAB is using.']);
    end

    %% Validate STL geometry

    if isempty(faces) || isempty(vertices)
        error('The STL file contains no valid geometry.');
    end

    if size(vertices,2) ~= 3
        error('The STL vertex matrix must contain three columns.');
    end

    %% Center STL around selected rotation origin

    minimumPoint = min(vertices,[],1);
    maximumPoint = max(vertices,[],1);

    modelCenter = (minimumPoint + maximumPoint) / 2;

    originOffset = [0, 0, -0.28 * (maximumPoint(3) - minimumPoint(3))];

    newOrigin = modelCenter + originOffset;
    vertices = vertices - newOrigin;

    %% Scale STL model

    modelDimensions = max(vertices,[],1) - min(vertices,[],1);
    modelSize = max(modelDimensions);

    if ~isfinite(modelSize) || modelSize <= 0
        error('The STL model has zero or invalid size.');
    end

    verticesReference = aircraftDisplayLength .* vertices ./ modelSize;

    %% Rotate STL into aircraft body coordinates (X fwd, Y right, Z down)

    R_stl = [ ...
         0, -1,  0;
         1,  0,  0;
         0,  0, -1];

    verticesReference = (R_stl * verticesReference')';

    %% Finalize animation settings

    animationStep = max(1,round(animationStep));
    playbackSpeed = max(playbackSpeed,eps);

    %% Create animation figure

    animationFigure = figure(6);
    clf(animationFigure);

    set(animationFigure, ...
        'Color','w', ...
        'Name','F-16 flight animation', ...
        'NumberTitle','off', ...
        'Units','normalized', ...
        'Position',[0.05 0.08 0.90 0.82], ...
        'Resize','on');

    %% Create animation axes (fills the window below the title strip)

    animationAxes = axes( ...
        'Parent',animationFigure, ...
        'Units','normalized', ...
        'Position',[0.05 0.05 0.90 0.95 - titleHeight - 0.02], ...
        'NextPlot','add');

    grid(animationAxes,'on');
    box(animationAxes,'on');
    axis(animationAxes,'equal');
    axis(animationAxes,'vis3d');

    set(animationAxes, ...
        'ZDir','reverse', ...
        'XLimMode','manual', ...
        'YLimMode','manual', ...
        'ZLimMode','manual');

    xlabel(animationAxes,'North, visual scale');
    ylabel(animationAxes,'East, visual scale');
    zlabel(animationAxes,'Down, visual scale');

    view(animationAxes,125,cameraElevation);

    %% Create title fixed to the top of the window

    % A figure-level annotation stays at the top of the window regardless
    % of the 3D camera orientation, unlike an axes title.
    animationTitle = annotation( ...
        animationFigure, ...
        'textbox', ...
        [0 1 - titleHeight 1 titleHeight], ...
        'String','F-16 lateral flight animation', ...
        'FontSize',14, ...
        'FontWeight','bold', ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','middle', ...
        'EdgeColor','none', ...
        'BackgroundColor','none', ...
        'Interpreter','none', ...
        'FitBoxToText','off');

    %% Create aircraft transform and patch

    aircraftTransform = hgtransform('Parent',animationAxes);

    aircraftPatch = patch( ...
        'Parent',aircraftTransform, ...
        'Faces',faces, ...
        'Vertices',verticesReference, ...
        'FaceColor',[0.70 0.70 0.70], ...
        'EdgeColor','none', ...
        'FaceLighting','gouraud', ...
        'AmbientStrength',0.35, ...
        'DiffuseStrength',0.75, ...
        'SpecularStrength',0.20);

    camlight(animationAxes,'headlight');
    material(animationAxes,'dull');

    %% Create body-axis vectors

    bodyAxisLength = 0.75 * aircraftDisplayLength;

    if show_body_axes
        bodyXAxis = quiver3(animationAxes, 0,0,0, bodyAxisLength,0,0, ...
            'Color','r', 'LineWidth',2, 'MaxHeadSize',0.35);

        bodyYAxis = quiver3(animationAxes, 0,0,0, 0,bodyAxisLength,0, ...
            'Color','g', 'LineWidth',2, 'MaxHeadSize',0.35);

        bodyZAxis = quiver3(animationAxes, 0,0,0, 0,0,bodyAxisLength, ...
            'Color','b', 'LineWidth',2, 'MaxHeadSize',0.35);
    end

    %% Create yaw vector

    if show_yaw_vector
        yawVector = quiver3(animationAxes, ...
            northDisplay(1), eastDisplay(1), downDisplay(1), ...
            0,vectorLength,0, ...
            'Color',[0.85 0.10 0.10], 'LineWidth',2.5, 'MaxHeadSize',0.35);
    end

    %% Create course vector

    if show_course_vector
        courseVector = quiver3(animationAxes, ...
            northDisplay(1), eastDisplay(1), downDisplay(1), ...
            0,vectorLength,0, ...
            'Color',[0.00 0.65 0.15], 'LineWidth',2.5, 'MaxHeadSize',0.35);
    end

    %% Create flight trail

    traveledPath = animatedline(animationAxes, ...
        'Color',[0.00 0.45 0.90], ...
        'LineWidth',2, ...
        'MaximumNumPoints',maximumTrailPoints);

    %% Set initial moving limits

    initialViewCenterEast  = eastDisplay(1)  + lookAheadDistance * sin(chi(1));
    initialViewCenterNorth = northDisplay(1) + lookAheadDistance * cos(chi(1));

    xlim(animationAxes, initialViewCenterNorth + [-viewHalfWidth viewHalfWidth]);
    ylim(animationAxes, initialViewCenterEast  + [-viewHalfWidth viewHalfWidth]);
    zlim(animationAxes, downDisplay(1) + [-viewHalfHeight viewHalfHeight]);

    %% Create video file

    if record
        videoObject = VideoWriter(videoFile, 'MPEG-4');
        videoObject.FrameRate = 30;
        videoObject.Quality = 95;
        open(videoObject);
    end

    %% Force figure to fully render before starting animation

    drawnow;
    pause(0.1);
    drawnow;

    while strcmp(animationFigure.Visible,'off')
        drawnow;
    end

    %% Animation loop

    try
        for k = 1:animationStep:length(t)

            % Stop if the figure has been closed.
            if ~isgraphics(animationFigure) || ~isgraphics(aircraftPatch)
                break;
            end

            %% Attitude rotation

            Rx = [ ...
                1,           0,            0;
                0, cos(phi(k)), -sin(phi(k));
                0, sin(phi(k)),  cos(phi(k))];

            Ry = [ ...
                 cos(theta(k)), 0, sin(theta(k));
                 0,             1,             0;
                -sin(theta(k)), 0, cos(theta(k))];

            Rz = [ ...
                cos(psi(k)), -sin(psi(k)), 0;
                sin(psi(k)),  cos(psi(k)), 0;
                0,            0,           1];

            RbodyToPlot = Rz * Ry * Rx;

            %% Translate aircraft

            aircraftPosition = [northDisplay(k); eastDisplay(k); downDisplay(k)];

            transformMatrix = eye(4);
            transformMatrix(1:3,1:3) = RbodyToPlot;
            transformMatrix(1:3,4) = aircraftPosition;

            set(aircraftTransform,'Matrix',transformMatrix);

            %% Update body-axis vectors

            if show_body_axes
                bodyXDirection = RbodyToPlot * [1;0;0];
                bodyYDirection = RbodyToPlot * [0;1;0];
                bodyZDirection = RbodyToPlot * [0;0;1];

                set(bodyXAxis, ...
                    'XData',aircraftPosition(1), 'YData',aircraftPosition(2), 'ZData',aircraftPosition(3), ...
                    'UData',bodyAxisLength * bodyXDirection(1), ...
                    'VData',bodyAxisLength * bodyXDirection(2), ...
                    'WData',bodyAxisLength * bodyXDirection(3));

                set(bodyYAxis, ...
                    'XData',aircraftPosition(1), 'YData',aircraftPosition(2), 'ZData',aircraftPosition(3), ...
                    'UData',bodyAxisLength * bodyYDirection(1), ...
                    'VData',bodyAxisLength * bodyYDirection(2), ...
                    'WData',bodyAxisLength * bodyYDirection(3));

                set(bodyZAxis, ...
                    'XData',aircraftPosition(1), 'YData',aircraftPosition(2), 'ZData',aircraftPosition(3), ...
                    'UData',bodyAxisLength * bodyZDirection(1), ...
                    'VData',bodyAxisLength * bodyZDirection(2), ...
                    'WData',bodyAxisLength * bodyZDirection(3));
            end

            %% Update yaw vector

            if show_yaw_vector
                set(yawVector, ...
                    'XData',northDisplay(k), 'YData',eastDisplay(k), 'ZData',downDisplay(k), ...
                    'UData',vectorLength * cos(psi(k)), ...
                    'VData',vectorLength * sin(psi(k)), ...
                    'WData',0);
            end

            %% Update course vector

            courseEast  = sin(chi(k));
            courseNorth = cos(chi(k));

            if show_course_vector
                set(courseVector, ...
                    'XData',northDisplay(k), 'YData',eastDisplay(k), 'ZData',downDisplay(k), ...
                    'UData',vectorLength * courseNorth, ...
                    'VData',vectorLength * courseEast, ...
                    'WData',0);
            end

            %% Add current position to flight trail

            addpoints(traveledPath, northDisplay(k), eastDisplay(k), downDisplay(k));

            %% Move axis limits with the aircraft

            viewCenterEast  = eastDisplay(k)  + lookAheadDistance * courseEast;
            viewCenterNorth = northDisplay(k) + lookAheadDistance * courseNorth;

            xlim(animationAxes, viewCenterNorth + [-viewHalfWidth viewHalfWidth]);
            ylim(animationAxes, viewCenterEast  + [-viewHalfWidth viewHalfWidth]);
            zlim(animationAxes, downDisplay(k) + [-viewHalfHeight viewHalfHeight]);

            %% Optional rotating chase camera

            if useChaseCamera
                cameraAzimuth = 270 + rad2deg(chi(k));
                view(animationAxes, cameraAzimuth, 30);
            end

            %% Update title

            titleText = sprintf( ...
                ['F-16 flight,  t = %6.1f s\n' ...
                 'Roll = %6.1f deg   Yaw = %6.1f deg   ' ...
                 'Course = %6.1f deg   Sideslip = %5.2f deg'], ...
                t(k), ...
                rad2deg(phi(k)), ...
                rad2deg(psi(k)), ...
                rad2deg(chi(k)), ...
                rad2deg(beta(k)));

            set(animationTitle,'String',titleText);

            %% Render frame

            drawnow limitrate;

            %% Record video

            if record
                drawnow;
                videoFrame = getframe(animationFigure);
                writeVideo(videoObject, videoFrame);
            end

            %% Control playback speed

            if ~record
                nextIndex = k + animationStep;
                if nextIndex <= length(t)
                    frameTime = t(nextIndex) - t(k);
                    pause(frameTime / playbackSpeed);
                end
            end
        end

    catch animationError

        if record
            close(videoObject);
        end

        rethrow(animationError);
    end

    %% Close video file

    if record
        close(videoObject);
        fprintf('Video saved as: %s\n', videoFile);
    end
end