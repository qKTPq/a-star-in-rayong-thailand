%% A* ROUTE OPTIMIZATION: RAYONG PROVINCE NETWORK
% ========================================================================
% Computes optimal routes from power plants to cultivation sites using A*
% pathfinding on OpenStreetMap road networks.
%
% KEY FEATURES:
%   • Loads OSM road shapefile and builds directed graph
%   • Implements A* with Haversine heuristic
%   • Routes 3 power plants → 3 cultivation sites
%   • Visualizes all 9 routes on satellite basemap
%   • Logs distances and timing
%
% REQUIREMENTS:
%   • Mapping Toolbox
%   • OSM Shapefile folder: thailand-251204-free.shp/
%     Using: gis_osm_roads_free_1.shp (+ .cpg, .dbf, .prj, .shx)
%
% OSM DATASETS IN FOLDER:
%   ✓ gis_osm_roads_free_*.shp       ← ROAD NETWORK (used)
%   - gis_osm_building_*.shp         (buildings)
%   - gis_osm_landuse_*.shp          (land use areas)
%   - gis_osm_natural_*.shp          (natural features)
%   - gis_osm_places_*.shp           (cities, towns)
%   - gis_osm_pofw_*.shp             (power stations)
%   - gis_osm_pois_*.shp             (points of interest)
%   - gis_osm_railways_*.shp         (railways)
%   - gis_osm_traffic_*.shp          (traffic features)
%   - gis_osm_transport_*.shp        (transport)
%   - gis_osm_waterways_*.shp        (rivers, canals)
%
% AUTHOR: Kantaphan Punnaanan
% DATE: December 2025
% ========================================================================

clearvars; clc; close all
tStart = tic;

% ========================================================================
% 1. DEFINE LOCATIONS (Power Plants & Cultivation Sites)
% ========================================================================

% Power plants (CO2 sources)
sources = struct( ...
    'name', {'BKK', 'GTS3', 'GPD'}, ...
    'lat', [12.706482500229685, 13.06626401132807, 12.954136439352459], ...
    'lon', [101.15138692811698, 101.18613186127817, 101.15442655235202]);

% Cultivation sites (Ulva targets)
targets = struct( ...
    'name', {'Ulva A', 'Ulva B', 'Ulva C'}, ...
    'lat', [12.6701795973468, 12.672814573318323, 12.6358352176851], ...
    'lon', [101.19427641886872, 101.06148329142223, 101.33290374052143]);

fprintf('=== A* ROUTE OPTIMIZATION ===\n');
fprintf('Sources: %d | Targets: %d | Possible routes: %d\n', ...
    numel(sources), numel(targets), numel(sources) * numel(targets));
fprintf('\n');

% ========================================================================
% 2. LOAD ROAD NETWORK SHAPEFILE (OSM)
% ========================================================================

shpFolder = 'thailand-251204-free.shp';
shpFile = fullfile(shpFolder, 'gis_osm_roads_free_1.shp');
fprintf('Loading OSM road shapefile: %s\n', shpFile);

% Check if folder and file exist
if ~isdir(shpFolder)
    error('Shapefile folder not found: %s\n\nExpected structure:\n  thailand-251204-free.shp/\n    ├─ gis_osm_roads_free_1.shp\n    ├─ gis_osm_roads_free_1.cpg\n    ├─ gis_osm_roads_free_1.dbf\n    ├─ gis_osm_roads_free_1.prj\n    ├─ gis_osm_roads_free_1.shx\n    └─ [other OSM layers...]', ...
        shpFolder);
end

if ~isfile(shpFile)
    % Try to find alternative road file names
    roadPattern = dir(fullfile(shpFolder, 'gis_osm_roads*.shp'));
    if isempty(roadPattern)
        error('Road shapefile not found: %s\n\nSearched for: gis_osm_roads*.shp\nVerify the OSM roads layer exists in the folder.', shpFile);
    else
        % Use first matching file
        shpFile = fullfile(shpFolder, roadPattern(1).name);
        fprintf('Found alternative: %s\n', roadPattern(1).name);
    end
end

try
    roads = shaperead(shpFile, 'UseGeoCoords', true);
    fprintf('✓ Loaded %d road polylines\n\n', numel(roads));
catch ME
    error('Failed to load road shapefile from: %s\n\nCheck:\n  1. Mapping Toolbox is installed\n  2. All shapefile components exist (.shp, .shx, .dbf, .prj, .cpg)\n  3. File permissions are readable\n\nError: %s', ...
        shpFile, ME.message);
end

% ========================================================================
% 3. BUILD ROAD GRAPH
% ========================================================================

fprintf('Building road network graph...\n');
[nodeMap, latList, lonList, adjacency] = buildRoadGraph(roads);

nNodes = numel(latList);
nEdges = sum(cellfun(@(x) size(x,1), adjacency)) / 2;
fprintf('✓ Graph constructed: %d nodes | %.0f edges\n\n', nNodes, nEdges);

% ========================================================================
% 4. SNAP LOCATIONS TO NEAREST ROAD NODES
% ========================================================================

fprintf('Snapping locations to nearest road nodes...\n');

% Snap sources
for i = 1:numel(sources)
    distances = haversineDistance(latList, lonList, sources(i).lat, sources(i).lon);
    [~, idx] = min(distances);
    sources(i).nodeIdx = idx;
    sources(i).snapLat = latList(idx);
    sources(i).snapLon = lonList(idx);
    fprintf('  %s → node %d (snap: %.4f°, %.4f°)\n', ...
        sources(i).name, idx, sources(i).snapLat, sources(i).snapLon);
end

% Snap targets
for j = 1:numel(targets)
    distances = haversineDistance(latList, lonList, targets(j).lat, targets(j).lon);
    [~, idx] = min(distances);
    targets(j).nodeIdx = idx;
    targets(j).snapLat = latList(idx);
    targets(j).snapLon = lonList(idx);
    fprintf('  %s → node %d (snap: %.4f°, %.4f°)\n', ...
        targets(j).name, idx, targets(j).snapLat, targets(j).snapLon);
end
fprintf('\n');

% ========================================================================
% 5. RUN A* PATHFINDING FOR ALL SOURCE-TARGET PAIRS
% ========================================================================

fprintf('Computing optimal routes using A*...\n');
fprintf('%-8s %-12s %-12s %12s\n', 'Route#', 'From', 'To', 'Distance (km)');
fprintf('%s\n', repmat('-', 1, 50));

% Pre-allocate storage
routeData = cell(numel(sources), numel(targets));
pathCosts = zeros(numel(sources), numel(targets));

routeIdx = 1;
for i = 1:numel(sources)
    for j = 1:numel(targets)
        % Run A* algorithm
        [pathNodeIdx, cost] = aStarSearch( ...
            sources(i).nodeIdx, targets(j).nodeIdx, ...
            latList, lonList, adjacency);
        
        % Store route
        routeData{i,j}.pathLat = latList(pathNodeIdx);
        routeData{i,j}.pathLon = lonList(pathNodeIdx);
        pathCosts(i,j) = cost;
        
        % Log result
        fprintf('%3d     %-12s → %-12s %12.2f\n', ...
            routeIdx, sources(i).name, targets(j).name, cost / 1000);
        
        routeIdx = routeIdx + 1;
    end
end
fprintf('\n');

% ========================================================================
% 6. CREATE VISUALIZATION
% ========================================================================

fprintf('Creating geographic visualization...\n');

figure('Name', 'A* Route Optimization - Rayong Province', ...
       'NumberTitle', 'off', ...
       'Position', [100 100 1400 850]);

ax = geoaxes;
hold(ax, 'on');
geobasemap(ax, 'satellite');
title(ax, 'A* Optimal Routes: Power Plants → Cultivation Sites', ...
      'FontSize', 16, 'FontWeight', 'bold', 'Margin', 12);

% Define color palettes
colorPalette = getColorPalette();

% Plot power plants (triangles)
for i = 1:numel(sources)
    geoscatter(ax, sources(i).snapLat, sources(i).snapLon, 250, ...
        colorPalette.sources(i,:), '^', 'filled', ...
        'DisplayName', sprintf('Power: %s', sources(i).name), ...
        'MarkerEdgeColor', 'w', 'LineWidth', 2.5, ...
        'Tag', sprintf('source_%d', i));
end

% Plot cultivation sites (circles)
for j = 1:numel(targets)
    geoscatter(ax, targets(j).snapLat, targets(j).snapLon, 200, ...
        colorPalette.targets(j,:), 'o', 'filled', ...
        'DisplayName', sprintf('Site: %s', targets(j).name), ...
        'MarkerEdgeColor', 'w', 'LineWidth', 2, ...
        'Tag', sprintf('target_%d', j));
end

% Plot routes with labels
routeIdx = 1;
for i = 1:numel(sources)
    for j = 1:numel(targets)
        if ~isempty(routeData{i,j}.pathLat)
            % Plot route line
            geoplot(ax, routeData{i,j}.pathLat, routeData{i,j}.pathLon, ...
                'Color', colorPalette.routes(routeIdx,:), ...
                'LineWidth', 2.5, ...
                'DisplayName', sprintf('%s → %s', sources(i).name, targets(j).name), ...
                'LineStyle', '-', ...
                'Tag', sprintf('route_%d', routeIdx));
            
            % Add distance annotation at midpoint
            midIdx = round(numel(routeData{i,j}.pathLat) / 2);
            distanceKm = pathCosts(i,j) / 1000;
            
            text(ax, routeData{i,j}.pathLon(midIdx), routeData{i,j}.pathLat(midIdx), ...
                sprintf('%.1f km', distanceKm), ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'Color', colorPalette.routes(routeIdx,:), ...
                'BackgroundColor', 'w', ...
                'EdgeColor', colorPalette.routes(routeIdx,:), ...
                'Margin', 3, 'LineWidth', 1.5, ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle');
            
            routeIdx = routeIdx + 1;
        end
    end
end

hold(ax, 'off');
lgd = legend(ax, 'Location', 'eastoutside', 'FontSize', 9, ...
    'Interpreter', 'none', 'Box', 'off', 'EdgeColor', 'none');
lgd.Title.String = 'Locations & Routes';

fprintf('✓ Map complete\n\n');

% ========================================================================
% 7. SUMMARY STATISTICS
% ========================================================================

fprintf('=== ROUTE SUMMARY ===\n');
fprintf('Total routes computed: %d\n', numel(sources) * numel(targets));
fprintf('Average distance: %.1f km\n', mean(pathCosts(:)) / 1000);
fprintf('Shortest route: %.1f km\n', min(pathCosts(:)) / 1000);
fprintf('Longest route: %.1f km\n', max(pathCosts(:)) / 1000);
fprintf('\nTotal execution time: %.2f seconds\n', toc(tStart));

% ========================================================================
% HELPER FUNCTIONS
% ========================================================================

function [nodeMap, latList, lonList, adjacency] = buildRoadGraph(roads)
    % Build adjacency list graph from shapefile polylines
    %
    % INPUT:
    %   roads    - struct array from shaperead
    %
    % OUTPUT:
    %   nodeMap  - containers.Map for node ID lookup
    %   latList  - latitude of each node
    %   lonList  - longitude of each node
    %   adjacency - cell array of neighbor lists [nodeID, distance]
    
    nodeMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
    latList = [];
    lonList = [];
    adjacency = {};
    
    for k = 1:numel(roads)
        lat = roads(k).Lat(:);
        lon = roads(k).Lon(:);
        
        % Remove NaN separators
        valid = ~isnan(lat) & ~isnan(lon);
        lat = lat(valid);
        lon = lon(valid);
        
        if numel(lat) < 2, continue; end
        
        % Add first node
        [id1, nodeMap, latList, lonList, adjacency] = ...
            getOrCreateNode(lat(1), lon(1), nodeMap, latList, lonList, adjacency);
        
        % Add edges between consecutive points
        for s = 1:numel(lat) - 1
            [id2, nodeMap, latList, lonList, adjacency] = ...
                getOrCreateNode(lat(s+1), lon(s+1), nodeMap, latList, lonList, adjacency);
            
            distance = haversineDistance(lat(s), lon(s), lat(s+1), lon(s+1));
            
            % Add bidirectional edges
            adjacency{id1} = [adjacency{id1}; id2, distance];
            adjacency{id2} = [adjacency{id2}; id1, distance];
            
            id1 = id2;
        end
    end
end

function [id, nodeMap, latList, lonList, adjacency] = ...
    getOrCreateNode(lat, lon, nodeMap, latList, lonList, adjacency)
    % Get existing node ID or create new one
    
    key = sprintf('%.6f,%.6f', lat, lon);
    
    if isKey(nodeMap, key)
        id = nodeMap(key);
    else
        id = numel(latList) + 1;
        nodeMap(key) = id;
        latList(id, 1) = lat;
        lonList(id, 1) = lon;
        adjacency{id, 1} = zeros(0, 2);
    end
end

function dist = haversineDistance(lat1, lon1, lat2, lon2)
    % Compute Haversine distance in meters (vectorized)
    %
    % Uses WGS84 Earth radius (6,371,000 m)
    
    R = 6371000;  % Earth radius in meters
    
    lat1_rad = deg2rad(lat1);
    lat2_rad = deg2rad(lat2);
    dLat = deg2rad(lat2 - lat1);
    dLon = deg2rad(lon2 - lon1);
    
    a = sin(dLat/2).^2 + cos(lat1_rad) .* cos(lat2_rad) .* sin(dLon/2).^2;
    c = 2 * asin(sqrt(a));
    dist = R * c;
end

function [pathNodeIdx, totalCost] = aStarSearch(startNode, goalNode, lat, lon, adjacency)
    % A* pathfinding algorithm with Haversine heuristic
    %
    % INPUT:
    %   startNode    - starting node ID
    %   goalNode     - goal node ID
    %   lat, lon     - node coordinates
    %   adjacency    - adjacency list [nodeID, distance]
    %
    % OUTPUT:
    %   pathNodeIdx  - node IDs from start to goal
    %   totalCost    - total path distance in meters
    
    nNodes = numel(lat);
    
    % Initialize g-cost and h-estimate
    gCost = inf(nNodes, 1);
    gCost(startNode) = 0;
    hEstimate = haversineDistance(lat, lon, lat(goalNode), lon(goalNode));
    
    % Track parent nodes for path reconstruction
    parent = zeros(nNodes, 1, 'uint32');
    
    % Open set: nodes to be evaluated
    openSet = containers.Map('KeyType', 'uint32', 'ValueType', 'double');
    openSet(uint32(startNode)) = 0;
    
    % Main A* loop
    while ~isempty(openSet)
        % Find node with smallest f-cost
        openKeys = cell2mat(openSet.keys);
        fCosts = gCost(openKeys) + hEstimate(openKeys);
        [~, minIdx] = min(fCosts);
        currentNode = openKeys(minIdx);
        
        if currentNode == goalNode
            break;
        end
        
        remove(openSet, uint32(currentNode));
        
        % Evaluate neighbors
        neighbors = adjacency{currentNode};
        for n = 1:size(neighbors, 1)
            neighborNode = neighbors(n, 1);
            edgeDistance = neighbors(n, 2);
            
            tentativeGCost = gCost(currentNode) + edgeDistance;
            
            if tentativeGCost < gCost(neighborNode)
                parent(neighborNode) = currentNode;
                gCost(neighborNode) = tentativeGCost;
                openSet(uint32(neighborNode)) = tentativeGCost;
            end
        end
    end
    
    % Reconstruct path
    pathNodeIdx = goalNode;
    while pathNodeIdx(1) ~= startNode
        pathNodeIdx = [parent(pathNodeIdx(1)); pathNodeIdx];
    end
    
    totalCost = gCost(goalNode);
end

function colorPalette = getColorPalette()
    % Define color scheme for visualization
    %
    % Returns struct with RGB color arrays for sources, targets, routes
    
    % Source power plant colors (3×3 RGB)
    colorPalette.sources = [
        0.20 0.20 0.20;  % BKK: dark gray
        0.00 0.45 0.74;  % GTS3: blue
        0.85 0.33 0.10;  % GPD: red-orange
    ];
    
    % Target cultivation site colors (3×3 RGB)
    colorPalette.targets = [
        0.47 0.67 0.19;  % Ulva A: green
        0.93 0.69 0.13;  % Ulva B: gold
        0.49 0.18 0.56;  % Ulva C: purple
    ];
    
    % Route colors (9×3 RGB) - unique color per source-target pair
    colorPalette.routes = [
        0.85 0.33 0.10;  % BKK → Ulva A
        0.93 0.69 0.13;  % BKK → Ulva B
        0.49 0.18 0.56;  % BKK → Ulva C
        0.00 0.45 0.74;  % GTS3 → Ulva A
        0.47 0.67 0.19;  % GTS3 → Ulva B
        0.30 0.75 0.93;  % GTS3 → Ulva C
        0.64 0.08 0.18;  % GPD → Ulva A
        0.00 0.60 0.50;  % GPD → Ulva B
        0.80 0.36 0.36;  % GPD → Ulva C
    ];
end
