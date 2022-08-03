
--/ This script was originally created by Minhseu123 & MinhXD#9978
--/ This pathfinder uses A Star algorithm. You can learn it from here https://youtu.be/-L-WgKMFuhE

local Config

local pathFinder = {};
local Nodes = {};
local openNodes = {};
local closedNodes = {};
local cameFrom = {};
local gScore = {};
local hScore = {};
local fScore = {};
local newVec3 = Vector3.new;
local newVec2 = Vector2.new;
local insert = table.insert;
local floor = math.floor;

--/ Physics

function Distance(startPos, goalPos)
    return (startPos - goalPos).Magnitude;
end

--/ Node

function Nodes:New(Pos)

    Pos = newVec3(floor(Pos.X), floor(Pos.Y), floor(Pos.Z));
    
    if Nodes[Pos.X] == nil then
        Nodes[Pos.X] = {};
    end
    if Nodes[Pos.X][Pos.Y] == nil then
        Nodes[Pos.X][Pos.Y] = {};
    end

    local newNode = {};
    
    newNode.Position = Pos;
    
    gScore[newNode] = 1/0;
    fScore[newNode] = 1/0;
    
    if Config["showNodes"] then
        newNode.Instance = Instance.new("Part", workspace.Nodes);
        newNode.Instance.Position = newNode.Position
        newNode.Instance.Anchored = true;
        newNode.Instance.Size = newVec3(2, 2, 2);
        newNode.Instance.CanCollide = false;
    end
    
    Nodes[Pos.X][Pos.Y][Pos.Z] = newNode;
    return Nodes[Pos.X][Pos.Y][Pos.Z];
end

function getNeighbors(node, startPos, endPos, spacing)
    local neighbors = {};
    
    --/ Reduce for loop lags
    if Config["dimension"]:lower() == "3d" then
        for x = -spacing, spacing, spacing do
          for y = -spacing, spacing, spacing do
            for z = -spacing, spacing, spacing do
                local vec = node.Position + newVec3(x, y, z);
                local existingNeighbor = nil;
            
                if Config["groundlevel"] then
                    if not workspace:FindPartOnRayWithIgnoreList(Ray.new(vec, newVec3(0, -spacing, 0)), Config["blacklistparts"]) then
                        --/ In air
                        continue;
                    end
                end
                if workspace:FindPartOnRayWithIgnoreList(Ray.new(node.Position, newVec3(x, y, z)), Config["blacklistparts"]) then
                    --/ Blocked
                    continue;
                end

                if Nodes[vec.X] and Nodes[vec.X][vec.Y] and Nodes[vec.X][vec.Y][vec.Z] then
                    existingNeighbor = Nodes[vec.X][vec.Y][vec.Z];
                end

                if existingNeighbor ~= nil then
                    insert(neighbors, existingNeighbor);
                else
                    local neighbor = Nodes:New(vec);
                    cameFrom[neighbor] = node;
                    insert(neighbors, neighbor);
                end
            end
          end
        end
    else
        for x = -spacing, spacing, spacing do
            for z = -spacing, spacing, spacing do
                local vec = node.Position + newVec3(x, 0, z);
                local existingNeighbor = nil;
            
                if Config["groundlevel"] then
                    if not workspace:FindPartOnRayWithIgnoreList(Ray.new(vec, newVec3(0, -spacing, 0)), Config["blacklistparts"]) then
                        --/ In air
                        continue;
                    end
                end
                if workspace:FindPartOnRayWithIgnoreList(Ray.new(node.Position, newVec3(x, 0, z)), Config["blacklistparts"]) then
                    --/ Blocked
                    continue;
                end

                if Nodes[vec.X] and Nodes[vec.X][vec.Y] and Nodes[vec.X][vec.Y][vec.Z] then
                    existingNeighbor = Nodes[vec.X][vec.Y][vec.Z];
                end

                if existingNeighbor ~= nil then
                    insert(neighbors, existingNeighbor);
                else
                    local neighbor = Nodes:New(vec);
                    cameFrom[neighbor] = node;
                    insert(neighbors, neighbor);
                end
            end
        end
    end
    
    return neighbors;
end

--/ Implementation

function reconstructPath(currentNode)
    local Path = {};
    local tempCurrent = currentNode
    
    while tempCurrent do
        insert(Path, 1, tempCurrent.Position);
        tempCurrent = cameFrom[tempCurrent];
    end

    return Path;
end

function pathFinder:SetConfiguration(config)
    assert(type(config) == "table", ("[MinhXD Pathfinder Beta]: Cannot set a configuration. table expected got %s"):format(type(config)))
    
    Config = config
end

function pathFinder:FindPath(startPos, endPos)
    assert(Config and Config["timeout"] ~= nil and Config["blacklistparts"] ~= nil and Config["dimension"] ~= nil and Config["groundlevel"] ~= nil and Config["instant"] ~= nil and Config["spacing"] ~= nil and Config["showNodes"] ~= nil, "[MinhXD Pathfinder Beta]: Configuration with timeout, blacklistparts, dimension, groundlevel, instant, spacing, showNodes expected. Please use :SetConfiguration to set a configuration")
    
    if Config["showNodes"] then
        Instance.new("Folder", workspace).Name = "Nodes";
        insert(Config["blacklistparts"], workspace.Nodes);
    end
    
    local start = tick()
    local startNode = Nodes:New(startPos, startPos, endPos);
    local endNode = Nodes:New(endPos, startPos, endPos)
    
    gScore[startNode] = 0;
    fScore[startNode] = 0;
    
    insert(openNodes, startNode);
    
    while #openNodes > 0 do
      for increasement = 1,10 do
        if tick() - start > Config["timeout"] then
            error("[MinhXD Pathfinder Beta][Fail safe]: Timed out")
        end

        local nodeWithLowestFCost = table.remove(openNodes, 1);
        
        insert(closedNodes, nodeWithLowestFCost);
        
        if Distance(nodeWithLowestFCost.Position, endNode.Position) < Config["spacing"] then
            if not workspace:FindPartOnRayWithIgnoreList(Ray.new(nodeWithLowestFCost.Position, endNode.Position - nodeWithLowestFCost.Position), Config["blacklistparts"]) then
            --/ Current node is at end node & Path have been found
                return reconstructPath(nodeWithLowestFCost);
            end
        end
        
        local Neighbors = getNeighbors(nodeWithLowestFCost, startNode.Position, endNode.Position, Config["spacing"])
        
        for i = 1, #Neighbors do
            local neighbor = Neighbors[i];
            local tempG = gScore[nodeWithLowestFCost] + Distance(nodeWithLowestFCost.Position, neighbor.Position) - 1;

            if not table.find(closedNodes, neighbor) then
                if tempG < gScore[neighbor] or not table.find(openNodes, neighbor) then
                    gScore[neighbor] = tempG;
                    hScore[neighbor] = Distance(neighbor.Position, endNode.Position);
                    fScore[neighbor] = gScore[neighbor] + hScore[neighbor];
                    cameFrom[neighbor] = nodeWithLowestFCost;
                    if not table.find(openNodes, neighbor) then
                        insert(openNodes, neighbor);
                    end
                end
            end
        end
        
        table.sort(openNodes, function(a,b)
            return fScore[a] < fScore[b];
        end)
      end
      if Config["instant"] == false then
          task.wait();
      end
    end
end

function pathFinder:SmoothPath(path)
    local current = path[1]
    local smoothedPath = {}
    
    table.insert(smoothedPath, current)
    
    for i = 1, #path do
        if i == #path or workspace:FindPartOnRayWithIgnoreList(Ray.new(current, path[i] - current), Config["blacklistparts"]) then
            if i == #path then i = #path + 2 end
            for i2 = 1, math.floor(Distance(path[i - 2], current) / Config["spacing"]) do
                table.insert(smoothedPath, current + (path[i - 2] - current).Unit * (i2 * Config["spacing"]))
            end
            current = path[i - 2]
        end
    end

    for i = 1, #path do
        path[i] = nil
    end
    
    for i = 1, #smoothedPath do
        table.insert(path, smoothedPath[i])
    end

    return smoothedPath
end

return pathFinder
