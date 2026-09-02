function animateSatelliteSTL(t, phi, theta, psi, stlFile, record)
% You do not need to change any code. 
% animateSatelliteSTL animates an STL model using estimated Euler angles.
% 
% Inputs:
%   t       - Simulation time vector [s]
%   phi     - Estimated roll angle [rad]
%   theta   - Estimated pitch angle [rad]
%   psi     - Estimated yaw angle [rad]
%   stlFile - Name or path of the STL model

    %% Check inputs

    if ~isfile(stlFile)
        error('The STL file "%s" could not be found.', stlFile);
    end

    if length(t) ~= length(phi) || ...
       length(t) ~= length(theta) || ...
       length(t) ~= length(psi)

        error('t, phi, theta and psi must have the same length.');
    end

    %% Read STL model

    % MATLAB's built-in stlread returns a triangulation object.
    stlModel = stlread(stlFile);
    
    if isa(stlModel, 'triangulation')
        faces = stlModel.ConnectivityList;
        vertices = stlModel.Points;
    else
        error(['stlread did not return a triangulation object. ' ...
               'Check which stlread function MATLAB is using.']);
    end

    %% Check STL data

    if isempty(faces) || isempty(vertices)
        error('The STL file does not contain valid faces and vertices.');
    end

    if size(vertices, 2) ~= 3
        error('The STL vertex matrix must have three columns.');
    end

    %% Center and scale the STL model

    % Move the geometric center of the model to the origin.
    modelCenter = mean(vertices, 1);
    vertices = vertices - modelCenter;

    % Find the largest dimension of the STL model.
    modelDimensions = max(vertices, [], 1) - min(vertices, [], 1);
    modelSize = max(modelDimensions);

    if modelSize <= 0
        error('The STL model has zero or invalid size.');
    end

    % Scale the largest dimension to approximately two plot units.
    verticesReference = 2 * vertices / modelSize;
    
    % Fixed alignment between the STL coordinate system and body axes.
    % Rotate the STL model 180 degrees about its z-axis.
    R_stl = [0  1  0;
        -1 0  0;
        0  0  -1];

    verticesReference = (R_stl * verticesReference')';

    %% Animation settings

    % Display every fifth simulation sample.
    % Increase this number if the animation is too slow.
    animationStep = 5;

    % Playback speed relative to simulation time.
    % For example, 20 means 20 times faster than real time.
    playbackSpeed = 60;

    %% Create the animation figure
    
    animationFigure = figure(6);
    clf(animationFigure);


    set(animationFigure, ...
        'Color', 'w', ...
        'Name', 'Satellite attitude animation', ...
        'NumberTitle', 'off');

    animationAxes = axes('Parent', animationFigure);

    satellitePatch = patch(animationAxes, ...
        'Faces', faces, ...
        'Vertices', verticesReference, ...
        'FaceColor',[0.70 0.70 0.70],...%%'FaceColor', [0.35 0.38 0.42], ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud');

    hold(animationAxes, 'on');


    %% Draw the fixed reference axis
    ref_axis_length = 1.2;
    
    quiver3(animationAxes, 0, 0, 0, ref_axis_length, 0, 0, ...
        'Color', 'r', ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.25);

    quiver3(animationAxes, 0, 0, 0, 0, ref_axis_length, 0, ...
        'Color', 'g', ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.25);

    quiver3(animationAxes, 0, 0, 0, 0, 0, ref_axis_length, ...
        'Color', 'b', ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.25);

    text(animationAxes, ref_axis_length, 0, 0, 'x', ...
        'Color', 'r', ...
        'FontSize', 12, ...
        'FontWeight', 'bold');

    text(animationAxes, 0, ref_axis_length, 0, 'y', ...
        'Color', 'g', ...
        'FontSize', 12, ...
        'FontWeight', 'bold');

    text(animationAxes, 0, 0, ref_axis_length, 'z', ...
        'Color', 'b', ...
        'FontSize', 12, ...
        'FontWeight', 'bold');

    hold(animationAxes, 'off');

    %% Format the animation axes

    axis(animationAxes, 'equal');
    axis_lengths = 1.3;
    axis(animationAxes, [-axis_lengths axis_lengths -axis_lengths axis_lengths -axis_lengths axis_lengths]);
    set(animationAxes, 'ZDir', 'reverse');
    set(animationAxes, 'YDir', 'reverse');
    
    
    grid(animationAxes, 'on');
    box(animationAxes, 'on');

    xlabel(animationAxes, 'x');
    ylabel(animationAxes, 'y');
    zlabel(animationAxes, 'z');

    view(animationAxes, 35+90, 25);

    camlight(animationAxes, 'headlight');
    material(animationAxes, 'dull');

    animationTitle = title(animationAxes, ...
        'Satellite attitude animation');
    
    %% Create video file

    if record
        videoObject = VideoWriter(videoFile, 'MPEG-4');
        videoObject.FrameRate = 30;
        videoObject.Quality = 95;
        open(videoObject);
    end
    %% Animation loop

    for k = 1:animationStep:length(t)

        % Stop cleanly if the animation figure has been closed.
        if ~isgraphics(animationFigure) || ...
           ~isgraphics(satellitePatch)
            break;
        end
        
        % Roll rotation about the x-axis.
        Rx = [1,           0,            0;
              0, cos(phi(k)), -sin(phi(k));
              0, sin(phi(k)),  cos(phi(k))];

        % Pitch rotation about the y-axis.
        Ry = [ cos(theta(k)), 0, sin(theta(k));
                            0, 1,             0;
             -sin(theta(k)), 0, cos(theta(k))];

        % Yaw rotation about the z-axis.
        Rz = [cos(psi(k)), -sin(psi(k)), 0;
              sin(psi(k)),  cos(psi(k)), 0;
                        0,            0, 1];

        % Body-to-inertial rotation matrix.
        R = Rz * Ry * Rx;

        % Rotate all model vertices.
        rotatedVertices = (R * verticesReference')';

        % Update the STL model.
        set(satellitePatch, ...
            'Vertices', rotatedVertices);

        % Update the title.
        titleText = sprintf( ...
            ['Satellite attitude, t = %6.1f s\n' ...
             '\\phi = %6.1f deg,  ' ...
             '\\theta = %6.1f deg,  ' ...
             '\\psi = %6.1f deg'], ...
            t(k), ...
            rad2deg(phi(k)), ...
            rad2deg(theta(k)), ...
            rad2deg(psi(k)));

        set(animationTitle, ...
            'String', titleText, ...
            'FontSize', 13);

        drawnow;
        
        if record
            videoFrame = getframe(animationFigure);
            writeVideo(videoObject, videoFrame);
        end
        % Set animation playback rate.
        nextIndex = k + animationStep;

        if nextIndex <= length(t)
            frameTime = t(nextIndex) - t(k);
            pause(frameTime / playbackSpeed);
        end
    end
    if record
        close(videoObject);
        fprintf('Video saved as: %s\n', videoFile);
    end
end