% Nuclear_Launch_Scenarios.m
% Reference file for PONI Space-Based Interceptor Simulation
%
% INSTRUCTIONS:
% Copy the desired 'silo_fields' array from below and paste it into
% Section 2 of your Main_Simulation.m file to replace the default array.
%
% Format: [Latitude (deg), Longitude (deg), Base Heading (deg)]
% Note: Headings generally point North (0 to 360) to simulate optimal
% over-the-pole trajectories between superpowers.

%% ========================================================================
% SCENARIO 1: RUSSIAN FEDERATION (Massive Retaliatory Strike)
% Targets: United States / Europe
% Includes: Western, Central, and Siberian ICBM fields.
% ========================================================================
silo_fields_Russia = [
53.9,   35.7,  350.0;  % Kozelsk (Western Russia) - Yars/UR-100N
51.0,   59.8,  355.0;  % Dombarovsky (Southern Urals) - Sarmat/Avangard
55.3,   89.8,    5.0;  % Uzhur (Siberia) - Sarmat
52.3,  104.3,   15.0;  % Irkutsk (Siberia) - Mobile Yars (approximated)
60.8,   50.0,  340.0;  % Plesetsk region
];

%% ========================================================================
% SCENARIO 2: UNITED STATES (Minuteman III Arsenal)
% Targets: Russia / China
% Includes: The three major USAF Global Strike Command missile wings.
% ========================================================================
silo_fields_USA = [
47.5, -111.1,   15.0;  % Malmstrom AFB, Montana (341st Missile Wing)
48.4, -101.3,  355.0;  % Minot AFB, North Dakota (91st Missile Wing)
41.1, -104.8,    0.0;  % F.E. Warren AFB, Wyoming (90th Missile Wing)
];

%% ========================================================================
% SCENARIO 3: PEOPLE'S REPUBLIC OF CHINA (Modernised Arsenal)
% Targets: United States
% Includes: The recently discovered massive solid-fuel ICBM fields.
% ========================================================================
silo_fields_China = [
40.2,   97.0,   25.0;  % Yumen Silo Field (approx 120 silos)
42.2,   92.8,   20.0;  % Hami Silo Field (approx 110 silos)
39.8,  108.3,   30.0;  % Hanggin Banner Silo Field (approx 90 silos)
34.0,  109.0,   35.0;  % Central Mountains (DF-5 / DF-41 older bases)
];

%% ========================================================================
% SCENARIO 4: UNITED KINGDOM & FRANCE (European CASD)
% Targets: Russia
% Includes: Entirely submarine-based deterrence (Vanguard & Triomphant class)
% generates random locations within the North Atlantic / Norwegian Sea bastions.
% ========================================================================
% Generate 3 random submarine patrol locations in the Atlantic/Arctic
% Latitude: 50N to 70N | Longitude: 30W to 10E
uk_fr_lats = 50 + (rand(3,1) * 20);
uk_fr_lons = -30 + (rand(3,1) * 40);
uk_fr_headings = 45 + (randn(3,1) * 15); % Aiming East/North-East towards Russia

silo_fields_UK_France = [uk_fr_lats, uk_fr_lons, uk_fr_headings];

%% ========================================================================
% SCENARIO 5: GLOBAL CONTINUOUS AT-SEA DETERRENT (CASD) 'POP-UP' STRIKE
% Represents SSBNs from any nation firing from unpredictable global oceans.
% ========================================================================
num_subs = 4;

% Generate truly random global coordinates
% (Note: In a pure random generator, some might spawn on land, but for
% the kinematic orbital simulation, the origin point math works exactly the same).
% Latitude: -60 to +60 | Longitude: -180 to +180
casd_lats = (rand(num_subs,1) * 120) - 60;
casd_lons = (rand(num_subs,1) * 360) - 180;
casd_headings = rand(num_subs,1) * 360; % Can be fired in any direction

silo_fields_CASD = [casd_lats, casd_lons, casd_headings];

%% ========================================================================
% SCENARIO 6: MAXIMUM SATURATION (ALL ACTORS SIMULTANEOUSLY)
% A "Doomsday" scenario combining US, Russian, Chinese land silos,
% plus 4 random global submarines.
% ========================================================================
silo_fields_Doomsday = [
silo_fields_Russia;
silo_fields_USA;
silo_fields_China;
silo_fields_CASD
];

% % --- HOW TO USE ---
% To use one of these in your Main_Simulation.m, simply copy the array
% and replace the 'silo_fields' variable. For example:
%
% % In Main_Simulation.m, Section 2:
% silo_fields = [
%     40.2,   97.0,   25.0;  % Yumen Silo Field
%     42.2,   92.8,   20.0;  % Hami Silo Field
%     39.8,  108.3,   30.0;  % Hanggin Banner Silo Field
% ];
% num_threats = 35; % Increase threat count to simulate full stockpile launch