clc; 
clear all;
GRID=int8(zeros(128,140));
GRID(1:64,1)=1;
GRID(120,3:100)=1;
GRID(125:128,40:60)=1;
GRID(120:128,100:120)=1;
GRID(126,100:118)=0;
GRID(120:126,118)=0;
GRID(100:120,100)=1;
GRID(114:124,112:118)=0;
GRID(1,1:128)=1;
GRID(128,1:128)=1;
GRID(100,1:130)=1;
GRID(50,28:128)=1;
GRID(20:30,50)=1;
GRID(1:128,1)=1;
GRID(1:65,128)=1;
GRID(1,1:128)=1;
GRID(128,1:128)=1;
GRID(10,1:50)=1;
GRID(25,1:50)=1;
GRID(40,40:50)=1;
GRID(40,40:45)=1;
GRID(80,20:40)=1;
GRID(80:100,40)=1;
GRID(80:100,120)=1;
GRID(120:122,120:122)=1;
GRID(120:122,20:25)=1;
GRID(120:122,10:11)=1;
GRID(125:128,10:11)=1;
GRID(100:110,30:40)=1;
GRID(1:20,100:128)=1;
GRID(10:20,80:128)=1;
GRID(20:40,80:90)=1;
GRID(1:40,90:90)=1;
GRID(100:105,70:80)=1;
%Start Positions
StartX=15;
StartY=15;
%Generating goal nodes, which is represented by a matrix. Several goals can be speciefied, in which case the pathfinder will find the closest goal. 
%a cell with the value 1 represent a goal cell
GoalRegister=int8(zeros(128,140));
GoalRegister(110 ,80)=1;
%Number of Neighboors one wants to investigate from each cell. A larger
%number of nodes means that the path can be alligned in more directions. 
%Connecting_Distance=1-> Path can  be alligned along 8 different direction.
%Connecting_Distance=2-> Path can be alligned along 16 different direction.
%Connecting_Distance=3-> Path can be alligned along 32 different direction.
%Connecting_Distance=4-> Path can be alligned along 56 different direction.
%ETC......
Connecting_Distance=8; %Avoid to high values Connecting_Distances for reasonable runtimes. 
% Running PathFinder
OptimalPath=ASTARPATH(StartX,StartY,GRID,GoalRegister,Connecting_Distance)
% End. 
if size(OptimalPath,2)>1
figure(10)
imagesc((GRID))
    colormap(flipud(gray));
hold on
plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
plot(OptimalPath(:,2),OptimalPath(:,1),'r')
legend('Goal','Start','Path')
else 
     pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
 end
showNeighboors=1; 
function OptimalPath=ASTARPATH(StartX,StartY,GRID,GoalRegister,Connecting_Distance)
%Version 1.0
% By Einar Ueland 2nd of May, 2016
%FINDING ASTAR PATH IN AN OCCUPANCY GRID
%nNeighboor=3;
% Preallocation of Matrices
[Height,Width]=size(GRID); %Height and width of matrix
GScore=zeros(Height,Width);           %Matrix keeping track of G-scores 
FScore=single(inf(Height,Width));     %Matrix keeping track of F-scores (only open list) 
Hn=single(zeros(Height,Width));       %Heuristic matrix
OpenMAT=int8(zeros(Height,Width));    %Matrix keeping of open grid cells
ClosedMAT=int8(zeros(Height,Width));  %Matrix keeping track of closed grid cells
ClosedMAT(GRID==1)=1;                  %Adding object-cells to closed matrix
ParentX=int16(zeros(Height,Width));   %Matrix keeping track of X position of parent
ParentY=int16(zeros(Height,Width));   %Matrix keeping track of Y position of parent
%%% Setting up matrices representing neighboors to be investigated
NeighboorCheck=ones(2*Connecting_Distance+1);
Dummy=2*Connecting_Distance+2;
Mid=Connecting_Distance+1;
for i=1:Connecting_Distance-1
NeighboorCheck(i,i)=0;
NeighboorCheck(Dummy-i,i)=0;
NeighboorCheck(i,Dummy-i)=0;
NeighboorCheck(Dummy-i,Dummy-i)=0;
NeighboorCheck(Mid,i)=0;
NeighboorCheck(Mid,Dummy-i)=0;
NeighboorCheck(i,Mid)=0;
NeighboorCheck(Dummy-i,Mid)=0;
end
NeighboorCheck(Mid,Mid)=0;
[row, col]=find(NeighboorCheck==1);
Neighboors=[row col]-(Connecting_Distance+1);
N_Neighboors=size(col,1);
%%% End of setting up matrices representing neighboors to be investigated
%%%%%%%%% Creating Heuristic-matrix based on distance to nearest  goal node
[col, row]=find(GoalRegister==1);
RegisteredGoals=[row col];
Nodesfound=size(RegisteredGoals,1);
for k=1:size(GoalRegister,1)
    for j=1:size(GoalRegister,2)
        if GRID(k,j)==0
            Mat=RegisteredGoals-(repmat([j k],(Nodesfound),1));
            Distance=(min(sqrt(sum(abs(Mat).^2,2))));
            Hn(k,j)=Distance;
        end
    end
end
%End of creating Heuristic-matrix. 
%Note: If Hn values is set to zero the method will reduce to the Dijkstras method.
%Initializign start node with FValue and opening first node.
FScore(StartY,StartX)=Hn(StartY,StartX);         
OpenMAT(StartY,StartX)=1;   
while 1==1 %Code will break when path found or when no path exist
    MINopenFSCORE=min(min(FScore));
    if MINopenFSCORE==inf;
    %Failuere!
    OptimalPath=[inf];
    RECONSTRUCTPATH=0;
     break
    end
    [CurrentY,CurrentX]=find(FScore==MINopenFSCORE);
    CurrentY=CurrentY(1);
    CurrentX=CurrentX(1);
    if GoalRegister(CurrentY,CurrentX)==1
    %GOAL!!
        RECONSTRUCTPATH=1;
        break
    end
    
  %Remobing node from OpenList to ClosedList  
    OpenMAT(CurrentY,CurrentX)=0;
    FScore(CurrentY,CurrentX)=inf;
    ClosedMAT(CurrentY,CurrentX)=1;
    for p=1:N_Neighboors
        i=Neighboors(p,1); %Y
        j=Neighboors(p,2); %X
        if CurrentY+i<1||CurrentY+i>Height||CurrentX+j<1||CurrentX+j>Width
            continue
        end
        Flag=1;
        if(ClosedMAT(CurrentY+i,CurrentX+j)==0) %Neiboor is open;
            if (abs(i)>1||abs(j)>1);   
                % Need to check that the path does not pass an object
                JumpCells=2*max(abs(i),abs(j))-1;
                for K=1:JumpCells;
                    YPOS=round(K*i/JumpCells);
                    XPOS=round(K*j/JumpCells);
            
                    if (GRID(CurrentY+YPOS,CurrentX+XPOS)==1)
                        Flag=0;
                    end
                end
            end
             %End of  checking that the path does not pass an object
            if Flag==1;           
                tentative_gScore = GScore(CurrentY,CurrentX) + sqrt(i^2+j^2);
                if OpenMAT(CurrentY+i,CurrentX+j)==0
                    OpenMAT(CurrentY+i,CurrentX+j)=1;                    
                elseif tentative_gScore >= GScore(CurrentY+i,CurrentX+j)
                    continue
                end
                ParentX(CurrentY+i,CurrentX+j)=CurrentX;
                ParentY(CurrentY+i,CurrentX+j)=CurrentY;
                GScore(CurrentY+i,CurrentX+j)=tentative_gScore;
                FScore(CurrentY+i,CurrentX+j)= tentative_gScore+Hn(CurrentY+i,CurrentX+j);
            end
        end
    end
end
k=2;
if RECONSTRUCTPATH
    OptimalPath(1,:)=[CurrentY CurrentX];
    while RECONSTRUCTPATH
        CurrentXDummy=ParentX(CurrentY,CurrentX);
        CurrentY=ParentY(CurrentY,CurrentX);
        CurrentX=CurrentXDummy;
        OptimalPath(k,:)=[CurrentY CurrentX];
        k=k+1;
        if (((CurrentX== StartX)) &&(CurrentY==StartY))
            break
        end
    end
end
end
