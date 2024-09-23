%****************************************************************************************************************************
% Discription:  Initialize basic Dubins path structure based on starting and ending information
% input:        start_info              Starting point information
% input:        finish_info             Ending point information
% output:       dubibs_info             Basic Dubins path information
%****************************************************************************************************************************

function dubins_info = Dubins_Init(start_info,finish_info)

dubins_info.traj.type=0;                                    % Type of Dubins path
dubins_info.traj.erro=0;                                    % Error marker
dubins_info.traj.length=0;                                  % Path length
dubins_info.traj.flag=0;                                    % 0: Dubins path
                                                            % 1: Tangent parh
%% Starting information
dubins_info.start.x=start_info(1);                          % Starting point x coordinate
dubins_info.start.y=start_info(2);                          % Starting point y coordinate
dubins_info.start.phi=start_info(3);                        % Starting heading angle
dubins_info.start.R=start_info(4);                          % Starting arc radius
dubins_info.start.phi_c=0;                                  % Starting point azimuth angle
dubins_info.start.xc=0;                                     % Sarting arc center x coordinate
dubins_info.start.yc=0;                                     % Sarting arc center y coordinate
dubins_info.start.phi_ex=0;                                 % Cutting out azimuth angle
dubins_info.start.x_ex=0;                                   % Cutting out point x coordinate
dubins_info.start.y_ex=0;                                   % Cutting out point y coordinate
dubins_info.start.psi=0;                                    % Travel angles on the starting arc

%% Ending information
dubins_info.finish.x=finish_info(1);                        % Ending point x coordinate
dubins_info.finish.y=finish_info(2);                        % Ending point y coordinate
dubins_info.finish.phi=finish_info(3);                      % Ending heading angle
dubins_info.finish.R=finish_info(4);                        % Ending arc radius
dubins_info.finish.phi_c=0;                                 % Ending point azimuth angle
dubins_info.finish.xc=0;                                    % Ending arc center x coordinate
dubins_info.finish.yc=0;                                    % Ending arc center y coordinate
dubins_info.finish.phi_en=0;                                % Cutting in azimuth angle
dubins_info.finish.phi_en1=0;                               % Cutting in heading angle
dubins_info.finish.x_en=0;                                  % Cutting in point x coordinate
dubins_info.finish.y_en=0;                                  % Cutting in point y coordinate
dubins_info.finish.psi=0;                                   % Travel angles on the Ending arc
end

