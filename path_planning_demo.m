function path_planning_demo
% PATH_PLANNING_DEMO Compare A*, Dijkstra and BFS on one map.
% Run: matlab -batch "path_planning_demo" from this directory.
close all; clc;
rows=10; cols=14;
blocked=false(rows,cols);
blocked(1:8,4)=true; blocked(3,4)=false; blocked(8,4)=false;
blocked(3:10,8)=true; blocked(5,8)=false; blocked(9,8)=false;
blocked(1:7,11)=true; blocked(2,11)=false; blocked(6,11)=false;
start=sub2ind([rows cols],2,2); goal=sub2ind([rows cols],9,13);
[tA,pA,cA,gA,hA]=search_grid(blocked,start,goal,true);
[tD,pD,cD,gD,hD]=search_grid(blocked,start,goal,false);
[tB,pB,cB,gB]=bfs_grid(blocked,start,goal);
if isempty(pA)||isempty(pD)||isempty(pB), error('A planner did not find a route.'); end
if numel(pA)~=numel(pD) || numel(pA)~=numel(pB), error('The three algorithms disagree on shortest distance.'); end
names={'A*','Dijkstra','BFS'}; traces={tA,tD,tB}; paths={pA,pD,pB}; costs=[cA cD cB]; gs={gA,gD,gB}; hs={hA,hD,zeros(size(gB))};
fig=figure('Name','Path planning comparison','Color','white','Position',[100 100 1440 650], 'Visible','off');
layout=tiledlayout(fig,1,3,'TileSpacing','compact','Padding','compact');
ax=gobjects(1,3); explored=gobjects(1,3); current=gobjects(1,3); finalpath=gobjects(1,3); vehicle=gobjects(1,3); labels=gobjects(1,3);
for k=1:3
    ax(k)=nexttile(layout,k); imagesc(ax(k),1:cols,1:rows,blocked); colormap(ax(k),[.95 .97 .99;.17 .22 .29]); caxis(ax(k),[0 1]);
    set(ax(k),'YDir','reverse','XTick',1:cols,'YTick',1:rows,'XGrid','on','YGrid','on','GridAlpha',.2,'FontSize',8);
    axis(ax(k),'equal'); xlim(ax(k),[.5 cols+.5]);ylim(ax(k),[.5 rows+.5]);hold(ax(k),'on');
    [sr,sc]=ind2sub([rows cols],start);[gr,gc]=ind2sub([rows cols],goal);
    scatter(ax(k),sc,sr,90,[.1 .7 .25],'filled','MarkerEdgeColor','w');scatter(ax(k),gc,gr,90,[.85 .15 .15],'filled','MarkerEdgeColor','w');
    text(ax(k),sc+.2,sr-.25,'START','FontWeight','bold','FontSize',8);text(ax(k),gc-1.7,gr-.25,'GOAL','FontWeight','bold','FontSize',8);
    explored(k)=scatter(ax(k),nan,nan,22,[1 .65 .08],'filled','MarkerFaceAlpha',.65);
    current(k)=scatter(ax(k),nan,nan,65,[.7 .15 .8],'filled','MarkerEdgeColor','w');
    finalpath(k)=plot(ax(k),nan,nan,'-','Color',[.08 .35 .9],'LineWidth',2.7);
    vehicle(k)=scatter(ax(k),nan,nan,80,[.08 .35 .9],'filled','MarkerEdgeColor','white');
    title(ax(k),names{k},'FontSize',14);labels(k)=xlabel(ax(k),'','FontSize',9);
end
sgtitle(layout,'Same map, start and goal | Orange: explored  Purple: current  Blue: final path','FontSize',14,'FontWeight','bold');
video=VideoWriter('path_planning_comparison.mp4','MPEG-4');video.FrameRate=12;video.Quality=95;open(video);
steps=max(cellfun(@numel,traces));
for frame=1:steps
    for k=1:3
        n=min(frame,numel(traces{k}));[rr,cc]=ind2sub([rows cols],traces{k}(1:n));
        set(explored(k),'XData',cc,'YData',rr);set(current(k),'XData',cc(end),'YData',rr(end));
        if k==1, detail=sprintf('g=%d, h=%d, f=%d',gs{k}(n),hs{k}(n),gs{k}(n)+hs{k}(n));
        elseif k==2, detail=sprintf('g=%d (distance from start)',gs{k}(n));
        else, detail=sprintf('depth=%d (moves from start)',gs{k}(n));end
        set(labels(k),'String',{sprintf('Step %d / %d | Node (%d,%d)',n,numel(traces{k}),cc(end),rr(end)),detail,'Each road edge weight = 1'});
    end
    drawnow; writeVideo(video,getframe(fig));
end
edgeLabels=cell(1,3);
for k=1:3
    [pr,pc]=ind2sub([rows cols],paths{k});
    edgeLabels{k}=gobjects(1,max(0,numel(pr)-1));
    for j=1:numel(pr)-1
        edgeLabels{k}(j)=text(ax(k),(pc(j)+pc(j+1))/2,(pr(j)+pr(j+1))/2,'1', ...
            'FontSize',7,'FontWeight','bold','Color',[.55 .1 .1],'BackgroundColor','white', ...
            'HorizontalAlignment','center','Visible','off');
    end
end
for frame=1:max(cellfun(@numel,paths))
    for k=1:3
        n=min(frame,numel(paths{k}));[rr,cc]=ind2sub([rows cols],paths{k}(1:n));
        set(finalpath(k),'XData',cc,'YData',rr);set(vehicle(k),'XData',cc(end),'YData',rr(end));set(current(k),'XData',nan,'YData',nan);
        if n>1,set(edgeLabels{k}(1:n-1),'Visible','on');end
        label='Shortest on this grid';
        set(labels(k),'String',{sprintf('%s: %d moves',label,numel(paths{k})-1),'Each blue road edge has weight 1'});
    end
    drawnow;writeVideo(video,getframe(fig));
end
for j=1:24, writeVideo(video,getframe(fig)); end
close(video); exportgraphics(fig,'comparison_final.png','Resolution',160);
Algorithm=names'; Moves=cellfun(@(p)numel(p)-1,paths)'; ExploredNodes=cellfun(@numel,traces)'; RuntimeSeconds=costs';
writetable(table(Algorithm,Moves,ExploredNodes,RuntimeSeconds),'comparison_results.csv');
disp(table(Algorithm,Moves,ExploredNodes,RuntimeSeconds));
fprintf('Saved path_planning_comparison.mp4, comparison_final.png and comparison_results.csv\n');
end

function [trace,path,elapsed,traceG,traceH]=search_grid(blocked,start,goal,useHeuristic)
% Array-based min selection (not a priority queue); unit edge weights.
tic; [rows,cols]=size(blocked); V=numel(blocked); dist=inf(V,1);dist(start)=0;
prev=zeros(V,1);open=false(V,1);open(start)=true;closed=false(V,1);trace=[];traceG=[];traceH=[];
[gr,gc]=ind2sub([rows cols],goal);
while any(open)
    candidates=find(open); [r,c]=ind2sub([rows cols],candidates);
    if useHeuristic, h=abs(r-gr)+abs(c-gc); else,h=zeros(size(r));end
    [~,i]=min(dist(candidates)+h);u=candidates(i);open(u)=false;
    if closed(u),continue;end
    closed(u)=true;trace(end+1)=u;traceG(end+1)=dist(u);[ur,uc]=ind2sub([rows cols],u);traceH(end+1)=abs(ur-gr)+abs(uc-gc); %#ok<AGROW>
    if u==goal,break;end
    for v=neighbors(blocked,u)
        if closed(v),continue;end
        if dist(u)+1<dist(v),dist(v)=dist(u)+1;prev(v)=u;open(v)=true;end
    end
end
path=extract_path(prev,start,goal);elapsed=toc;
end

function [trace,path,elapsed,traceG]=bfs_grid(blocked,start,goal)
% FIFO queue and mark on discovery: shortest number of moves on unit edges.
tic; V=numel(blocked); queue=zeros(V,1);head=1;tail=1;queue(1)=start;
discovered=false(V,1);discovered(start)=true;prev=zeros(V,1);depth=zeros(V,1);trace=[];traceG=[];
while head<=tail
    u=queue(head);head=head+1;trace(end+1)=u;traceG(end+1)=depth(u); %#ok<AGROW>
    if u==goal,break;end
    for v=neighbors(blocked,u)
        if ~discovered(v)
            discovered(v)=true;prev(v)=u;depth(v)=depth(u)+1;tail=tail+1;queue(tail)=v;
        end
    end
end
path=extract_path(prev,start,goal);elapsed=toc;
end

function out=neighbors(blocked,u)
[rows,cols]=size(blocked);[r,c]=ind2sub([rows cols],u);rc=[r-1 c;r+1 c;r c-1;r c+1];
valid=rc(:,1)>=1 & rc(:,1)<=rows & rc(:,2)>=1 & rc(:,2)<=cols;rc=rc(valid,:);
out=sub2ind([rows cols],rc(:,1),rc(:,2));out=out(~blocked(out))';
end

function path=extract_path(prev,start,goal)
path=[];if goal~=start && prev(goal)==0,return;end
u=goal;path=u;while u~=start,u=prev(u);path=[u path];end %#ok<AGROW>
end
